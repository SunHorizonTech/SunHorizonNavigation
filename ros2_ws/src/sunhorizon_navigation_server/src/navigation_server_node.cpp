#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sunhorizon_navigation_core/navigation_state_machine.hpp>
#include <sunhorizon_navigation_interfaces/action/navigate_to.hpp>
#include <sunhorizon_navigation_interfaces/msg/localization_status.hpp>
#include <sunhorizon_navigation_interfaces/msg/navigation_status.hpp>
#include <sunhorizon_navigation_interfaces/msg/velocity_supervisor_status.hpp>
#include <sunhorizon_navigation_interfaces/srv/cancel.hpp>
#include <sunhorizon_navigation_interfaces/srv/get_status.hpp>
#include <sunhorizon_navigation_interfaces/srv/load_map.hpp>
#include <sunhorizon_navigation_interfaces/srv/localize.hpp>
#include <sunhorizon_navigation_interfaces/srv/plan_to.hpp>
#include <sunhorizon_navigation_interfaces/srv/save_map.hpp>
#include <sunhorizon_navigation_interfaces/srv/start_mapping.hpp>
#include <sunhorizon_navigation_interfaces/srv/stop_mapping.hpp>
#include <sunhorizon_navigation_interfaces/srv/validate_map.hpp>

#include "sunhorizon_navigation_server/map_bundle_validator.hpp"
#include "sunhorizon_navigation_server/localization_health_gate.hpp"
#include "sunhorizon_navigation_server/first_click_path_geometry.hpp"
#include "sunhorizon_navigation_server/cancel_goal_if_known.hpp"
#include "sunhorizon_navigation_server/terminal_zero_guard.hpp"

namespace {

using NavigateTo = sunhorizon_navigation_interfaces::action::NavigateTo;
using ComputePath = nav2_msgs::action::ComputePathToPose;
using Nav2Navigate = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateTo>;
using OperatorGoalHandle = rclcpp_action::ClientGoalHandle<NavigateTo>;
using ComputePathGoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;
using Nav2GoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;
using NavigationStatus = sunhorizon_navigation_interfaces::msg::NavigationStatus;
using LocalizationStatus = sunhorizon_navigation_interfaces::msg::LocalizationStatus;
using SupervisorStatus = sunhorizon_navigation_interfaces::msg::VelocitySupervisorStatus;
using StartMapping = sunhorizon_navigation_interfaces::srv::StartMapping;
using StopMapping = sunhorizon_navigation_interfaces::srv::StopMapping;
using SaveMap = sunhorizon_navigation_interfaces::srv::SaveMap;
using ValidateMap = sunhorizon_navigation_interfaces::srv::ValidateMap;
using LoadMap = sunhorizon_navigation_interfaces::srv::LoadMap;
using Localize = sunhorizon_navigation_interfaces::srv::Localize;
using PlanTo = sunhorizon_navigation_interfaces::srv::PlanTo;
using Cancel = sunhorizon_navigation_interfaces::srv::Cancel;
using GetStatus = sunhorizon_navigation_interfaces::srv::GetStatus;
using sunhorizon_navigation_core::NavigationState;
using sunhorizon_navigation_core::NavigationStateMachine;

constexpr int32_t kOk = 0;
constexpr int32_t kInvalidState = 409;
constexpr int32_t kUnavailable = 503;
constexpr int32_t kInvalidMap = 422;
constexpr char kNavigateToAction[] = "/sunhorizon/navigation/navigate_to";
constexpr auto kZeroServiceDiscoveryBudget = std::chrono::milliseconds(100);
constexpr auto kZeroAcknowledgementBudget = std::chrono::milliseconds(300);

struct ZeroLatchResult {
  bool latched{false};
  std::string reason{"zero_velocity_latch_unavailable"};
};

std::string uuid_string(const rclcpp_action::GoalUUID& uuid) {
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < uuid.size(); ++index) {
    output << std::setw(2) << static_cast<unsigned int>(uuid[index]);
  }
  return output.str();
}

double path_length(const nav_msgs::msg::Path& path) {
  double length = 0.0;
  for (std::size_t index = 1; index < path.poses.size(); ++index) {
    const auto& previous = path.poses[index - 1U].pose.position;
    const auto& current = path.poses[index].pose.position;
    length += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return length;
}

bool pose_yaw(const geometry_msgs::msg::Pose& pose, double* yaw) {
  const auto& q = pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm < 1e-6) return false;
  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  *yaw = std::atan2(2.0 * (w * z + x * y),
      1.0 - 2.0 * (y * y + z * z));
  return std::isfinite(*yaw);
}

std::string provider_profile(const std::string& provider_prefix) {
  std::string normalized = provider_prefix;
  while (normalized.size() > 1U && normalized.back() == '/') normalized.pop_back();
  const auto separator = normalized.find_last_of('/');
  const std::string profile = separator == std::string::npos ?
      normalized : normalized.substr(separator + 1U);
  return profile == "visual" || profile == "lidar" ? profile : "";
}

class NavigationServer final : public rclcpp::Node {
 public:
  NavigationServer() : Node("sunhorizon_navigation_server") {
    map_root_ = declare_parameter("map_root", "/data/navigation/maps");
    // Robot A's cold-start map provider can legitimately need slightly more
    // than two seconds while the rest of Nav2 is being configured.  Keep the
    // request fail-closed, but leave enough time for the provider's definitive
    // response instead of faulting on the timeout boundary.
    provider_timeout_sec_ = declare_parameter("provider_timeout_sec", 5.0);
    post_seed_healthy_stability_sec_ = declare_parameter(
        "post_seed_healthy_stability_sec", 1.0);
    auto provider_prefix = declare_parameter(
        "provider_prefix", "/sunhorizon/providers/lidar");
    while (provider_prefix.size() > 1U && provider_prefix.back() == '/') {
      provider_prefix.pop_back();
    }
    const auto nav2_action = declare_parameter("nav2_action", "/navigate_to_pose");
    const auto nav2_plan_action = declare_parameter(
        "nav2_compute_path_action", "/compute_path_to_pose");
    nav2_planner_id_ = declare_parameter("nav2_planner_id", "GridBased");
    nav2_plan_timeout_sec_ = declare_parameter("nav2_plan_timeout_sec", 2.0);
    nominal_navigation_speed_mps_ = declare_parameter(
        "nominal_navigation_speed_mps", 0.10);
    first_click_gate_enabled_ = declare_parameter("first_click_gate_enabled", false);
    first_click_min_distance_m_ = declare_parameter(
        "first_click_min_distance_m", 0.45);
    first_click_max_distance_m_ = declare_parameter(
        "first_click_max_distance_m", 0.60);
    first_click_max_path_length_m_ = declare_parameter(
        "first_click_max_path_length_m", 0.70);
    first_click_max_heading_error_rad_ = declare_parameter(
        "first_click_max_heading_error_rad", 0.08726646259971647);
    first_click_max_goal_yaw_delta_rad_ = declare_parameter(
        "first_click_max_goal_yaw_delta_rad", 0.08726646259971647);
    first_click_max_pose_age_sec_ = declare_parameter(
        "first_click_max_pose_age_sec", 0.85);
    operator_goal_adapter_enabled_ = declare_parameter(
        "operator_goal_adapter_enabled", false);
    operator_goal_topic_ = declare_parameter(
        "operator_goal_topic", "/sunhorizon/navigation/operator_goal");
    operator_goal_timeout_sec_ = declare_parameter(
        "operator_goal_timeout_sec", 120.0);
    operator_initial_pose_adapter_enabled_ = declare_parameter(
        "operator_initial_pose_adapter_enabled", false);
    operator_initial_pose_topic_ = declare_parameter(
        "operator_initial_pose_topic", "/initialpose");
    startup_map_id_ = declare_parameter("startup_map_id", "");
    provider_profile_ = provider_profile(provider_prefix);
    if (provider_profile_.empty()) {
      throw std::invalid_argument(
          "provider_prefix_must_end_with_visual_or_lidar:" + provider_prefix);
    }
    if (nav2_plan_timeout_sec_ <= 0.0 || nominal_navigation_speed_mps_ <= 0.0) {
      throw std::invalid_argument("navigation_planning_parameters_must_be_positive");
    }
    if (!std::isfinite(post_seed_healthy_stability_sec_) ||
        post_seed_healthy_stability_sec_ < 0.5 ||
        post_seed_healthy_stability_sec_ > 10.0) {
      throw std::invalid_argument(
          "post_seed_healthy_stability_sec_must_be_between_0_5_and_10");
    }
    localization_health_gate_ =
        sunhorizon_navigation_server::LocalizationHealthGate(
            static_cast<std::int64_t>(post_seed_healthy_stability_sec_ * 1e9));
    if (!std::isfinite(first_click_max_pose_age_sec_) || first_click_max_pose_age_sec_ <= 0.0) {
      throw std::invalid_argument("first_click_max_pose_age_sec_must_be_finite_positive");
    }
    if (first_click_gate_enabled_ &&
        (first_click_min_distance_m_ <= 0.0 ||
         first_click_max_distance_m_ < first_click_min_distance_m_ ||
         first_click_max_path_length_m_ < first_click_max_distance_m_ ||
         first_click_max_heading_error_rad_ <= 0.0 ||
         first_click_max_goal_yaw_delta_rad_ <= 0.0)) {
      throw std::invalid_argument("first_click_gate_parameters_invalid");
    }
    if (operator_goal_adapter_enabled_ &&
        (operator_goal_topic_.empty() || operator_goal_timeout_sec_ <= 0.0)) {
      throw std::invalid_argument("operator_goal_parameters_must_be_valid_when_enabled");
    }
    if (operator_initial_pose_adapter_enabled_ &&
        (operator_initial_pose_topic_.empty() || startup_map_id_.empty() ||
        operator_initial_pose_topic_ == "/sunhorizon/localization/initial_guess")) {
      throw std::invalid_argument(
          "operator_initial_pose_requires_distinct_topic_and_startup_map");
    }
    const auto zero_service = declare_parameter(
        "zero_velocity_latch_service", "/sunhorizon/navigation/latch_zero_velocity");
    const auto reset_service = declare_parameter(
        "reset_velocity_latch_service", "/sunhorizon/navigation/reset_velocity_latch");
    machine_.configure();

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    if (first_click_gate_enabled_) {
      first_click_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      first_click_tf_listener_ =
          std::make_shared<tf2_ros::TransformListener>(*first_click_tf_buffer_);
    }
    status_pub_ = create_publisher<NavigationStatus>(
        "/sunhorizon/navigation/status", rclcpp::QoS(10).transient_local());
    localization_sub_ = create_subscription<LocalizationStatus>(
        "/sunhorizon/localization/status", 10,
        std::bind(&NavigationServer::on_localization, this, std::placeholders::_1));
    supervisor_sub_ = create_subscription<SupervisorStatus>(
        "/sunhorizon/navigation/velocity_supervisor_status", 10,
        std::bind(&NavigationServer::on_supervisor, this, std::placeholders::_1));

    start_provider_ = create_client<sunhorizon_navigation_interfaces::srv::StartMapping>(
        provider_prefix + "/start_mapping", rmw_qos_profile_services_default, callback_group_);
    stop_provider_ = create_client<sunhorizon_navigation_interfaces::srv::StopMapping>(
        provider_prefix + "/stop_mapping", rmw_qos_profile_services_default, callback_group_);
    save_provider_ = create_client<sunhorizon_navigation_interfaces::srv::SaveMap>(
        provider_prefix + "/save_map", rmw_qos_profile_services_default, callback_group_);
    load_provider_ = create_client<sunhorizon_navigation_interfaces::srv::LoadMap>(
        provider_prefix + "/load_map", rmw_qos_profile_services_default, callback_group_);
    localize_provider_ = create_client<sunhorizon_navigation_interfaces::srv::Localize>(
        provider_prefix + "/localize", rmw_qos_profile_services_default, callback_group_);
    zero_client_ = create_client<std_srvs::srv::Trigger>(
        zero_service, rmw_qos_profile_services_default, callback_group_);
    reset_client_ = create_client<std_srvs::srv::Trigger>(
        reset_service, rmw_qos_profile_services_default, callback_group_);
    compute_path_client_ = rclcpp_action::create_client<ComputePath>(
        this, nav2_plan_action, callback_group_);
    nav2_client_ = rclcpp_action::create_client<Nav2Navigate>(this, nav2_action, callback_group_);

    start_service_ = create_service<sunhorizon_navigation_interfaces::srv::StartMapping>(
        "/sunhorizon/navigation/start_mapping",
        std::bind(&NavigationServer::start_mapping, this,
          std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);
    stop_service_ = create_service<sunhorizon_navigation_interfaces::srv::StopMapping>(
        "/sunhorizon/navigation/stop_mapping",
        std::bind(&NavigationServer::stop_mapping, this,
          std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);
    save_service_ = create_service<sunhorizon_navigation_interfaces::srv::SaveMap>(
        "/sunhorizon/navigation/save_map",
        std::bind(&NavigationServer::save_map, this,
          std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);
    validate_service_ = create_service<sunhorizon_navigation_interfaces::srv::ValidateMap>(
        "/sunhorizon/navigation/validate_map",
        std::bind(&NavigationServer::validate_map, this,
          std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);
    load_service_ = create_service<sunhorizon_navigation_interfaces::srv::LoadMap>(
        "/sunhorizon/navigation/load_map",
        std::bind(&NavigationServer::load_map, this,
          std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);
    localize_service_ = create_service<sunhorizon_navigation_interfaces::srv::Localize>(
        "/sunhorizon/navigation/localize",
        std::bind(&NavigationServer::localize, this,
          std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);
    plan_service_ = create_service<sunhorizon_navigation_interfaces::srv::PlanTo>(
        "/sunhorizon/navigation/plan_to",
        std::bind(&NavigationServer::plan_to, this,
          std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);
    cancel_service_ = create_service<sunhorizon_navigation_interfaces::srv::Cancel>(
        "/sunhorizon/navigation/cancel",
        std::bind(&NavigationServer::cancel, this,
          std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);
    operator_cancel_service_ = create_service<std_srvs::srv::Trigger>(
        "/sunhorizon/navigation/operator_cancel",
        std::bind(&NavigationServer::operator_cancel, this,
          std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);
    operator_reset_service_ = create_service<std_srvs::srv::Trigger>(
        "/sunhorizon/navigation/operator_reset",
        std::bind(&NavigationServer::operator_reset, this,
          std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);
    get_status_service_ = create_service<sunhorizon_navigation_interfaces::srv::GetStatus>(
        "/sunhorizon/navigation/get_status",
        std::bind(&NavigationServer::get_status, this,
          std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);
    action_server_ = rclcpp_action::create_server<NavigateTo>(
        this, kNavigateToAction,
        std::bind(&NavigationServer::handle_goal, this,
          std::placeholders::_1, std::placeholders::_2),
        std::bind(&NavigationServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&NavigationServer::handle_accepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options(), callback_group_);
    if (operator_goal_adapter_enabled_) {
      operator_action_client_ = rclcpp_action::create_client<NavigateTo>(
          this, kNavigateToAction, callback_group_);
      rclcpp::SubscriptionOptions subscription_options;
      subscription_options.callback_group = callback_group_;
      operator_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          operator_goal_topic_, rclcpp::QoS(1).reliable(),
          std::bind(&NavigationServer::on_operator_goal, this, std::placeholders::_1),
          subscription_options);
      RCLCPP_WARN(
          get_logger(),
          "RViz operator goal adapter enabled on %s; goals still pass through %s",
          operator_goal_topic_.c_str(), kNavigateToAction);
    }
    if (operator_initial_pose_adapter_enabled_) {
      rclcpp::SubscriptionOptions subscription_options;
      subscription_options.callback_group = callback_group_;
      operator_initial_pose_sub_ = create_subscription<
          geometry_msgs::msg::PoseWithCovarianceStamped>(
          operator_initial_pose_topic_, rclcpp::QoS(1).reliable(),
          std::bind(
              &NavigationServer::on_operator_initial_pose, this,
              std::placeholders::_1),
          subscription_options);
      RCLCPP_WARN(
          get_logger(),
          "RViz initial-pose adapter enabled on %s for startup map %s",
          operator_initial_pose_topic_.c_str(), startup_map_id_.c_str());
    }
    if (!startup_map_id_.empty()) {
      // A provider response must remain executable while startup waits for it.
      // Reentrant 500 ms ticks can otherwise occupy all four executor threads
      // waiting for operation_mutex_ behind the first in-flight load request.
      startup_map_callback_group_ = create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      startup_map_timer_ = create_wall_timer(
          std::chrono::milliseconds(500),
          std::bind(&NavigationServer::load_startup_map, this),
          startup_map_callback_group_);
    }
    timer_ = create_wall_timer(
        std::chrono::milliseconds(200), [this]() { status_pub_->publish(make_status()); },
        callback_group_);
  }

 private:
  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr call_provider(
      const typename rclcpp::Client<ServiceT>::SharedPtr& client,
      const typename ServiceT::Request::SharedPtr& request) {
    if (!client->service_is_ready()) return nullptr;
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(provider_timeout_sec_)) !=
        std::future_status::ready) return nullptr;
    return future.get();
  }

  bool valid_map_files(const std::string& map_id, std::string* hash, std::string* reason) const {
    const auto validation = sunhorizon_navigation_server::validate_map_bundle(
        map_root_, map_id, provider_profile_);
    *hash = validation.manifest_sha256;
    *reason = validation.reason;
    return validation.valid;
  }

  NavigationStatus make_status() {
    std::lock_guard<std::mutex> lock(mutex_);
    NavigationStatus output;
    output.header.stamp = get_clock()->now();
    output.state = static_cast<uint8_t>(machine_.state());
    output.map_id = machine_.map_id();
    output.task_id = machine_.task_id();
    output.localization_state = localization_state_;
    output.safety_latched = safety_latched_;
    output.reason = safety_latched_ ? safety_reason_ : machine_.reason();
    return output;
  }

  void on_localization(const LocalizationStatus::SharedPtr input) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++localization_status_epoch_;
    localization_state_ = input->state;
    const auto status_stamp_ns = rclcpp::Time(input->header.stamp).nanoseconds();
    const bool status_belongs_to_request =
        localization_requested_ && localization_health_gate_.newer_than_arm(
            localization_status_epoch_, status_stamp_ns);
    const bool stable_post_seed_health =
        machine_.state() == NavigationState::kLocalizing &&
        status_belongs_to_request && localization_health_gate_.observe(
            localization_status_epoch_, status_stamp_ns,
            input->measurement_accepted,
            input->state == LocalizationStatus::STATE_HEALTHY);
    if (machine_.state() == NavigationState::kLocalizing &&
        localization_requested_ && stable_post_seed_health) {
      machine_.localization_result(true, "");
      localization_requested_ = false;
      localization_health_gate_.disarm();
      startup_localization_failure_observed_ = false;
    } else if (machine_.state() == NavigationState::kLocalizing &&
               status_belongs_to_request &&
               input->state == LocalizationStatus::STATE_LOST) {
      // Loading a manual seed resets the localization evidence streams. A
      // transient LOST sample is therefore expected before their stable
      // synchronized HEALTHY result. The health gate already reset its dwell
      // window above. Record and fail-close once, but keep the request pending
      // so a later uninterrupted HEALTHY window can complete it. A new RViz
      // seed remains allowed while the machine stays kLocalizing.
      if (!startup_localization_failure_observed_) {
        machine_.localization_result(false, "localization_lost");
        ++lifecycle_epoch_;
        startup_localization_failure_observed_ = true;
        if (!active_goal_task_id_.empty()) active_goal_cancel_requested_ = true;
        const auto zero = latch_zero_velocity_and_wait();
        record_zero_latch(zero);
      }
    } else if ((machine_.state() == NavigationState::kReady ||
                machine_.state() == NavigationState::kNavigating) &&
               input->state == LocalizationStatus::STATE_LOST) {
      terminal_zero_guard_.note_fault("localization_lost");
      machine_.fault("localization_lost");
      ++lifecycle_epoch_;
      if (!active_goal_task_id_.empty()) active_goal_cancel_requested_ = true;
      const auto zero = latch_zero_velocity_and_wait();
      record_zero_latch(zero);
      nav2_client_->async_cancel_all_goals();
    }
  }

  void on_supervisor(const SupervisorStatus::SharedPtr input) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool expected_terminal_stop = terminal_zero_guard_.expected_stop(
        active_goal_task_id_, input->latched, input->reason,
        rclcpp::Time(input->header.stamp).nanoseconds(), now().nanoseconds(),
        steady_now_ns());
    if (input->latched && !expected_terminal_stop) {
      terminal_zero_guard_.note_fault("velocity_supervisor:" + input->reason);
    }
    const bool newly_latched = input->latched && !safety_latched_;
    safety_latched_ = input->latched;
    safety_reason_ = terminal_zero_guard_.fault().empty() ?
        input->reason : terminal_zero_guard_.fault();
    if (newly_latched) ++lifecycle_epoch_;
    // Retain the actual safety latch. Only defer faulting this terminal task;
    // finish_outer still requires the matching successful zero-service ACK.
    if (expected_terminal_stop) return;
    if (safety_latched_ && machine_.state() == NavigationState::kNavigating) {
      machine_.fault("velocity_supervisor:" + safety_reason_);
      nav2_client_->async_cancel_all_goals();
    }
    if (safety_latched_ && !active_goal_task_id_.empty()) {
      active_goal_cancel_requested_ = true;
    }
  }

  static std::int64_t steady_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  ZeroLatchResult latch_zero_velocity_and_wait(
      const std::string& terminal_task_id = {}) {
    ZeroLatchResult output;
    if (!zero_client_->wait_for_service(kZeroServiceDiscoveryBudget)) {
      return output;
    }
    if (!terminal_task_id.empty()) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_goal_task_id_ == terminal_task_id &&
          machine_.task_id() == terminal_task_id &&
          machine_.state() == NavigationState::kNavigating &&
          !active_goal_cancel_requested_ && !safety_latched_) {
        terminal_zero_guard_.begin(terminal_task_id, now().nanoseconds(),
            steady_now_ns(), std::chrono::duration_cast<std::chrono::nanoseconds>(
                kZeroAcknowledgementBudget).count());
      }
    }
    auto future = zero_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(kZeroAcknowledgementBudget) != std::future_status::ready) {
      output.reason = "zero_velocity_latch_timeout";
      return output;
    }
    const auto response = future.get();
    output.latched = response->success;
    output.reason = response->message.empty() ?
        (response->success ? "zero_velocity_latched" : "zero_velocity_latch_rejected") :
        response->message;
    return output;
  }

  void record_zero_latch(const ZeroLatchResult& zero) {
    // This is the navigation-server interlock, distinct from claiming that the
    // downstream zero service acknowledged the stop.  A missing acknowledgement
    // must also block all future goals until fresh supervisor status proves an
    // explicit reset.
    safety_latched_ = true;
    safety_reason_ = zero.latched ? zero.reason :
        "zero_velocity_latch_unconfirmed:" + zero.reason;
    if (!terminal_zero_guard_.fault().empty()) {
      safety_reason_ = terminal_zero_guard_.fault();
    }
  }

  void begin_cancellation_locked(const std::string& reason) {
    terminal_zero_guard_.note_fault(reason);
    ++cancellation_operations_in_progress_;
    ++lifecycle_epoch_;
    safety_latched_ = true;
    safety_reason_ = reason;
    if (!active_goal_task_id_.empty()) {
      active_goal_cancel_requested_ = true;
    }
  }

  void end_cancellation_locked() {
    if (cancellation_operations_in_progress_ > 0U) {
      --cancellation_operations_in_progress_;
    }
  }

  bool goal_lifecycle_busy_locked() const {
    return goal_dispatch_pending_ || !active_goal_task_id_.empty() ||
           cancellation_operations_in_progress_ > 0U;
  }

  bool goal_cancel_requested(const std::string& task_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_goal_task_id_ == task_id && active_goal_cancel_requested_;
  }

  void settle_goal_lifecycle(const std::string& task_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_task_id_ != task_id) return;
    goal_dispatch_pending_ = false;
    pending_goal_task_id_.clear();
    active_goal_cancel_requested_ = false;
    active_goal_task_id_.clear();
    terminal_zero_guard_.clear(task_id);
  }

  void clear_reset_in_progress() {
    std::lock_guard<std::mutex> lock(mutex_);
    reset_in_progress_ = false;
  }

  void fail_unknown_map_load(
      LoadMap::Response::SharedPtr response, const std::string& reason) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_load_pending_ = false;
      map_load_outcome_unknown_ = true;
      ++lifecycle_epoch_;
      machine_.fault(reason);
      safety_latched_ = true;
      safety_reason_ = reason;
    }
    const auto zero = latch_zero_velocity_and_wait();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      safety_latched_ = true;
      safety_reason_ = reason + ";" +
          (zero.latched ? zero.reason :
           "zero_velocity_latch_unconfirmed:" + zero.reason);
    }
    response->accepted = false;
    response->code = kUnavailable;
    response->reason = reason;
  }

  void clear_localization_request_locked() {
    localization_request_pending_ = false;
    localization_requested_ = false;
    localization_health_gate_.disarm();
    startup_localization_failure_observed_ = false;
  }

  void fail_unknown_localization(
      Localize::Response::SharedPtr response, const std::string& reason) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      clear_localization_request_locked();
      localization_outcome_unknown_ = true;
      ++lifecycle_epoch_;
      machine_.fault(reason);
      safety_latched_ = true;
      safety_reason_ = reason;
    }
    const auto zero = latch_zero_velocity_and_wait();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      safety_latched_ = true;
      safety_reason_ = reason + ";" +
          (zero.latched ? zero.reason :
           "zero_velocity_latch_unconfirmed:" + zero.reason);
    }
    response->accepted = false;
    response->code = kUnavailable;
    response->reason = reason;
  }

  void start_mapping(StartMapping::Request::SharedPtr request, StartMapping::Response::SharedPtr response) {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (machine_.state() != NavigationState::kIdle) {
        response->accepted = false; response->code = kInvalidState;
        response->reason = "start_mapping_invalid_state"; return;
      }
    }
    const auto provider = call_provider<StartMapping>(
        start_provider_, request);
    if (!provider) {
      response->accepted = false; response->code = kUnavailable;
      response->reason = "mapping_provider_unavailable"; return;
    }
    *response = *provider;
    if (provider->accepted) {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto transition = machine_.start_mapping(provider->session_id);
      if (!transition.accepted) {
        response->accepted = false; response->code = kInvalidState; response->reason = transition.reason;
      }
    }
  }

  void stop_mapping(StopMapping::Request::SharedPtr request, StopMapping::Response::SharedPtr response) {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (machine_.state() != NavigationState::kMapping) {
        response->accepted = false; response->code = kInvalidState;
        response->reason = "stop_mapping_invalid_state"; return;
      }
    }
    const auto provider = call_provider<StopMapping>(
        stop_provider_, request);
    if (!provider) {
      response->accepted = false; response->code = kUnavailable;
      response->reason = "mapping_provider_unavailable"; return;
    }
    *response = *provider;
    if (provider->accepted) {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto transition = machine_.stop_mapping(request->session_id);
      if (!transition.accepted) {
        response->accepted = false; response->code = kInvalidState; response->reason = transition.reason;
      }
    }
  }

  void save_map(SaveMap::Request::SharedPtr request, SaveMap::Response::SharedPtr response) {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (machine_.state() != NavigationState::kMapReady) {
        response->accepted = false; response->code = kInvalidState;
        response->reason = "save_map_invalid_state"; return;
      }
    }
    std::string manifest_hash;
    if (!valid_map_files(
            request->requested_map_id, &manifest_hash, &response->reason)) {
      response->accepted = false; response->code = kInvalidMap; return;
    }
    const auto provider = call_provider<SaveMap>(
        save_provider_, request);
    if (!provider) {
      response->accepted = false; response->code = kUnavailable;
      response->reason = "map_save_provider_unavailable"; return;
    }
    *response = *provider;
    if (provider->accepted) {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto transition = machine_.save_map(provider->map_id);
      if (!transition.accepted) {
        response->accepted = false; response->code = kInvalidState; response->reason = transition.reason;
      }
    }
  }

  void validate_map(ValidateMap::Request::SharedPtr request, ValidateMap::Response::SharedPtr response) {
    response->valid = valid_map_files(
        request->map_id, &response->manifest_sha256, &response->reason);
    response->code = response->valid ? kOk : kInvalidMap;
  }

  void load_map(LoadMap::Request::SharedPtr request, LoadMap::Response::SharedPtr response) {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (machine_.state() != NavigationState::kIdle &&
          machine_.state() != NavigationState::kReady) {
        response->accepted = false; response->code = kInvalidState;
        response->reason = "load_map_invalid_state"; return;
      }
      if (goal_lifecycle_busy_locked() || reset_in_progress_ || map_load_pending_ ||
          map_load_outcome_unknown_ || localization_request_pending_ ||
          localization_outcome_unknown_ || nav2_goal_outcome_unknown_) {
        response->accepted = false; response->code = kInvalidState;
        response->reason = "navigation_or_map_load_pending"; return;
      }
      // Reserve the state before the provider can switch its external map.
      // Goal acceptance checks this flag under the same mutex.
      map_load_pending_ = true;
    }
    std::string hash;
    if (!valid_map_files(request->map_id, &hash, &response->reason)) {
      std::lock_guard<std::mutex> lock(mutex_);
      map_load_pending_ = false;
      response->accepted = false; response->code = kInvalidMap; return;
    }
    if (!load_provider_->service_is_ready()) {
      std::lock_guard<std::mutex> lock(mutex_);
      map_load_pending_ = false;
      response->accepted = false;
      response->code = kUnavailable;
      response->reason = "map_load_provider_unavailable";
      return;
    }
    rclcpp::Client<LoadMap>::SharedFuture provider_future;
    try {
      provider_future = load_provider_->async_send_request(request);
    } catch (const std::exception& error) {
      fail_unknown_map_load(
          response, "map_load_outcome_unknown_after_send_exception:" +
              std::string(error.what()));
      return;
    }
    if (provider_future.wait_for(std::chrono::duration<double>(provider_timeout_sec_)) !=
        std::future_status::ready) {
      fail_unknown_map_load(response, "map_load_outcome_unknown_after_timeout");
      return;
    }
    LoadMap::Response::SharedPtr provider;
    try {
      provider = provider_future.get();
    } catch (const std::exception& error) {
      fail_unknown_map_load(
          response, "map_load_outcome_unknown_after_response_exception:" +
              std::string(error.what()));
      return;
    }
    *response = *provider;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_load_pending_ = false;
      if (!provider->accepted) return;
      const auto transition = machine_.load_map(request->map_id);
      if (!transition.accepted) {
        // The provider may already have switched maps. Fault instead of
        // continuing with provider/state-machine disagreement.
        machine_.fault("map_provider_state_mismatch:" + transition.reason);
        safety_latched_ = true;
        safety_reason_ = machine_.reason();
        response->accepted = false; response->code = kInvalidState;
        response->reason = machine_.reason();
      } else {
        clear_localization_request_locked();
      }
    }
  }

  void localize(Localize::Request::SharedPtr request, Localize::Response::SharedPtr response) {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    std::uint64_t request_lifecycle_epoch = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (localization_outcome_unknown_) {
        response->accepted = false;
        response->code = kUnavailable;
        response->reason = "localization_outcome_unknown_restart_required";
        return;
      }
      if (machine_.state() != NavigationState::kLocalizing || request->map_id != machine_.map_id()) {
        response->accepted = false; response->code = kInvalidState;
        response->reason = "map_not_loaded_for_localization"; return;
      }
      if (localization_request_pending_ || localization_requested_) {
        response->accepted = false;
        response->code = kInvalidState;
        response->reason = "localization_request_already_in_progress";
        return;
      }
      localization_request_pending_ = true;
      startup_localization_failure_observed_ = false;
      request_lifecycle_epoch = lifecycle_epoch_;
    }
    if (!localize_provider_->service_is_ready()) {
      std::lock_guard<std::mutex> lock(mutex_);
      clear_localization_request_locked();
      response->accepted = false;
      response->code = kUnavailable;
      response->reason = "localization_provider_unavailable";
      return;
    }
    rclcpp::Client<Localize>::SharedFuture provider_future;
    try {
      provider_future = localize_provider_->async_send_request(request);
    } catch (const std::exception& error) {
      fail_unknown_localization(
          response, "localization_outcome_unknown_after_send_exception:" +
              std::string(error.what()));
      return;
    }
    if (provider_future.wait_for(std::chrono::duration<double>(provider_timeout_sec_)) !=
        std::future_status::ready) {
      fail_unknown_localization(
          response, "localization_outcome_unknown_after_timeout");
      return;
    }
    Localize::Response::SharedPtr provider;
    try {
      provider = provider_future.get();
    } catch (const std::exception& error) {
      fail_unknown_localization(
          response, "localization_outcome_unknown_after_response_exception:" +
              std::string(error.what()));
      return;
    }
    *response = *provider;
    if (!provider->accepted) {
      std::lock_guard<std::mutex> lock(mutex_);
      clear_localization_request_locked();
      return;
    }
    bool state_changed = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_changed = !localization_request_pending_ ||
          lifecycle_epoch_ != request_lifecycle_epoch ||
          machine_.state() != NavigationState::kLocalizing ||
          request->map_id != machine_.map_id();
      if (!state_changed) {
        localization_request_pending_ = false;
        localization_requested_ = true;
        // Only evidence generated after the provider acknowledged its reset
        // may complete localization. A queued HEALTHY status from the previous
        // localization epoch is deliberately ignored.
        localization_health_gate_.arm(
            localization_status_epoch_, get_clock()->now().nanoseconds());
        localization_state_ = LocalizationStatus::STATE_INITIALIZING;
      }
    }
    if (state_changed) {
      fail_unknown_localization(
          response, "navigation_changed_during_localization_request");
    }
  }

  void plan_to(PlanTo::Request::SharedPtr request, PlanTo::Response::SharedPtr response) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (map_load_pending_ || map_load_outcome_unknown_ ||
          machine_.state() != NavigationState::kReady ||
          request->map_id != machine_.map_id()) {
        response->feasible = false; response->code = kInvalidState;
        response->reason = "navigation_not_ready"; return;
      }
    }
    if (request->goal.header.frame_id != "map") {
      response->feasible = false; response->code = kInvalidMap;
      response->reason = "goal_frame_must_be_map"; return;
    }
    if (!compute_path_client_->action_server_is_ready()) {
      response->feasible = false; response->code = kUnavailable;
      response->reason = "nav2_planner_unavailable"; return;
    }
    ComputePath::Goal goal;
    goal.goal = request->goal;
    goal.planner_id = nav2_planner_id_;
    goal.use_start = false;
    auto goal_future = compute_path_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::duration<double>(nav2_plan_timeout_sec_)) !=
        std::future_status::ready) {
      response->feasible = false; response->code = kUnavailable;
      response->reason = "nav2_plan_goal_timeout"; return;
    }
    const ComputePathGoalHandle::SharedPtr goal_handle = goal_future.get();
    if (!goal_handle) {
      response->feasible = false; response->code = kUnavailable;
      response->reason = "nav2_plan_goal_rejected"; return;
    }
    auto result_future = compute_path_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::duration<double>(nav2_plan_timeout_sec_)) !=
        std::future_status::ready) {
      sunhorizon_navigation_server::cancel_goal_if_known(*compute_path_client_, goal_handle);
      response->feasible = false; response->code = kUnavailable;
      response->reason = "nav2_plan_timeout"; return;
    }
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
        !wrapped.result || wrapped.result->path.poses.empty()) {
      response->feasible = false; response->code = kInvalidMap;
      response->reason = "nav2_no_feasible_path"; return;
    }
    response->feasible = true;
    response->code = kOk;
    response->path_id = uuid_string(goal_handle->get_goal_id());
    response->path_length_m = path_length(wrapped.result->path);
    response->estimated_duration_sec =
        response->path_length_m / nominal_navigation_speed_mps_;
    response->reason = "nav2_plan_ready";
  }

  bool first_click_measured_start(
      geometry_msgs::msg::PoseStamped* pose, std::string* reason) {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = first_click_tf_buffer_->lookupTransform(
          "map", "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
      *reason = "first_click_start_pose_unavailable";
      return false;
    }
    if (transform.header.frame_id != "map" || transform.child_frame_id != "base_footprint") {
      *reason = "first_click_start_pose_frame_invalid";
      return false;
    }
    const auto stamp_ns = rclcpp::Time(transform.header.stamp).nanoseconds();
    if (!sunhorizon_navigation_server::first_click_pose_time_valid(
            now().nanoseconds(), stamp_ns, first_click_max_pose_age_sec_)) {
      *reason = "first_click_start_pose_timestamp_invalid";
      return false;
    }
    pose->header = transform.header;
    pose->pose.position.x = transform.transform.translation.x;
    pose->pose.position.y = transform.transform.translation.y;
    pose->pose.position.z = transform.transform.translation.z;
    pose->pose.orientation = transform.transform.rotation;
    return true;
  }

  bool compute_and_validate_first_click_path(
      const geometry_msgs::msg::PoseStamped& requested_goal,
      std::string* reason) {
    if (!first_click_gate_enabled_) return true;
    if (!compute_path_client_->action_server_is_ready()) {
      *reason = "first_click_planner_unavailable";
      return false;
    }
    if (requested_goal.header.frame_id != "map") {
      *reason = "first_click_goal_frame_invalid";
      return false;
    }
    geometry_msgs::msg::PoseStamped measured_start;
    if (!first_click_measured_start(&measured_start, reason)) return false;
    ComputePath::Goal goal;
    goal.goal = requested_goal;
    goal.planner_id = nav2_planner_id_;
    goal.start = measured_start;
    goal.use_start = true;
    auto goal_future = compute_path_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::duration<double>(nav2_plan_timeout_sec_)) !=
        std::future_status::ready) {
      *reason = "first_click_plan_goal_timeout";
      return false;
    }
    const auto handle = goal_future.get();
    if (!handle) {
      *reason = "first_click_plan_rejected";
      return false;
    }
    auto result_future = compute_path_client_->async_get_result(handle);
    if (result_future.wait_for(std::chrono::duration<double>(nav2_plan_timeout_sec_)) !=
        std::future_status::ready) {
      sunhorizon_navigation_server::cancel_goal_if_known(*compute_path_client_, handle);
      *reason = "first_click_plan_timeout";
      return false;
    }
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
        !wrapped.result || wrapped.result->path.poses.size() < 2U) {
      *reason = "first_click_no_feasible_path";
      return false;
    }
    const auto& path = wrapped.result->path;
    if (path.header.frame_id != "map") {
      *reason = "first_click_path_frame_invalid";
      return false;
    }
    // Re-read after planning so a delayed result cannot authorize from a stale pose.
    if (!first_click_measured_start(&measured_start, reason)) return false;
    const auto& start = measured_start.pose;
    const auto& target = requested_goal.pose;
    double start_yaw = 0.0;
    double goal_yaw = 0.0;
    if (!pose_yaw(start, &start_yaw) || !pose_yaw(target, &goal_yaw)) {
      *reason = "first_click_orientation_invalid";
      return false;
    }
    if (!sunhorizon_navigation_server::validate_first_click_goal_geometry(
            {start.position.x, start.position.y}, start_yaw,
            {target.position.x, target.position.y}, goal_yaw,
            first_click_min_distance_m_, first_click_max_distance_m_,
            first_click_max_heading_error_rad_, first_click_max_goal_yaw_delta_rad_, reason)) {
      return false;
    }
    std::vector<sunhorizon_navigation_server::FirstClickPoint> points;
    points.reserve(path.poses.size());
    for (const auto& pose : path.poses) {
      points.push_back({pose.pose.position.x, pose.pose.position.y});
    }
    if (!sunhorizon_navigation_server::validate_first_click_path_geometry(
            points, {target.position.x, target.position.y},
            first_click_max_path_length_m_, reason)) {
      return false;
    }
    *reason = "first_click_path_accepted";
    return true;
  }

  void cancel(Cancel::Request::SharedPtr request, Cancel::Response::SharedPtr response) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (request->task_id.empty() ||
          machine_.state() != NavigationState::kNavigating ||
          request->task_id != machine_.task_id()) {
        response->accepted = false;
        response->code = kInvalidState;
        response->reason = "cancel_task_id_not_active";
        return;
      }
      begin_cancellation_locked("task_cancel_pending_zero_ack");
    }
    const auto zero = latch_zero_velocity_and_wait();
    nav2_client_->async_cancel_all_goals();
    compute_path_client_->async_cancel_all_goals();
    std::lock_guard<std::mutex> lock(mutex_);
    const auto transition = machine_.cancel(request->task_id);
    record_zero_latch(zero);
    end_cancellation_locked();
    response->accepted = zero.latched && transition.accepted;
    response->code = !zero.latched ? kUnavailable :
        (transition.accepted ? kOk : kInvalidState);
    response->reason = zero.reason + ";nav2_cancel_requested;" + transition.reason;
  }

  void operator_cancel(
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      begin_cancellation_locked("operator_cancel_pending_zero_ack");
    }
    // This shared helper has a hard 0.4 s upper bound: zero is acknowledged
    // before cancellation, while cancellation is still attempted on failure.
    const auto zero = latch_zero_velocity_and_wait();
    nav2_client_->async_cancel_all_goals();
    compute_path_client_->async_cancel_all_goals();

    std::string transition_reason = "no_active_navigation";
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (machine_.state() == NavigationState::kNavigating ||
          machine_.state() == NavigationState::kStopped) {
        transition_reason = machine_.cancel("").reason;
      }
      record_zero_latch(zero);
      end_cancellation_locked();
    }
    response->success = zero.latched;
    response->message = zero.reason + ";nav2_cancel_requested;" + transition_reason;
  }

  void operator_reset(
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
    NavigationState state;
    std::uint64_t reset_epoch = 0U;
    bool fault_cleared = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state = machine_.state();
      if (map_load_outcome_unknown_) {
        response->success = false;
        response->message = "map_load_outcome_unknown_restart_required";
        return;
      }
      if (localization_outcome_unknown_) {
        response->success = false;
        response->message = "localization_outcome_unknown_restart_required";
        return;
      }
      if (nav2_goal_outcome_unknown_) {
        response->success = false;
        response->message = "nav2_goal_outcome_unknown_restart_required";
        return;
      }
      if (map_load_pending_) {
        response->success = false;
        response->message = "map_load_in_progress";
        return;
      }
      if (goal_lifecycle_busy_locked()) {
        response->success = false;
        response->message = "goal_cancellation_not_settled";
        return;
      }
      if (reset_in_progress_) {
        response->success = false;
        response->message = "operator_reset_already_in_progress";
        return;
      }
      if (state == NavigationState::kNavigating ||
          (state != NavigationState::kReady && state != NavigationState::kStopped &&
           state != NavigationState::kFault)) {
        response->success = false;
        response->message = "navigation_state_not_resettable";
        return;
      }
      reset_epoch = lifecycle_epoch_;
      if (state == NavigationState::kFault) {
        const auto transition = machine_.clear_fault();
        if (!transition.accepted) {
          response->success = false;
          response->message = transition.reason;
          return;
        }
        startup_map_loaded_ = false;
        clear_localization_request_locked();
        fault_cleared = true;
      }
      if (!fault_cleared &&
          localization_state_ != LocalizationStatus::STATE_HEALTHY) {
        response->success = false;
        response->message = "localization_must_be_healthy_before_reset";
        return;
      }
      if (!fault_cleared) reset_in_progress_ = true;
    }
    if (fault_cleared) {
      if (startup_map_timer_) startup_map_timer_->reset();
      status_pub_->publish(make_status());
      response->success = true;
      response->message = "fault_cleared_relocalization_required";
      return;
    }
    if (!reset_client_->wait_for_service(kZeroServiceDiscoveryBudget)) {
      clear_reset_in_progress();
      response->success = false;
      response->message = "velocity_supervisor_reset_unavailable";
      return;
    }
    auto future = reset_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(kZeroAcknowledgementBudget) != std::future_status::ready) {
      clear_reset_in_progress();
      response->success = false;
      response->message = "velocity_supervisor_reset_timeout";
      return;
    }
    const auto reset = future.get();
    if (!reset->success) {
      clear_reset_in_progress();
      response->success = false;
      response->message = "velocity_supervisor_reset_rejected:" + reset->message;
      return;
    }

    bool revalidation_failed = false;
    std::string revalidation_reason;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (lifecycle_epoch_ != reset_epoch || goal_lifecycle_busy_locked() ||
          map_load_pending_ || map_load_outcome_unknown_ ||
          nav2_goal_outcome_unknown_ || machine_.state() != state ||
          localization_state_ != LocalizationStatus::STATE_HEALTHY) {
        revalidation_failed = true;
        revalidation_reason = "navigation_changed_during_operator_reset";
        safety_latched_ = true;
        safety_reason_ = revalidation_reason;
      } else {
        safety_latched_ = false;
        safety_reason_ = "operator_reset";
        if (state == NavigationState::kStopped) {
          const auto transition = machine_.acknowledge_stop();
          if (!transition.accepted) {
            revalidation_failed = true;
            revalidation_reason = transition.reason;
          }
        }
      }
      reset_in_progress_ = false;
    }
    if (revalidation_failed) {
      const auto zero = latch_zero_velocity_and_wait();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        record_zero_latch(zero);
      }
      response->success = false;
      response->message = revalidation_reason;
      return;
    }
    status_pub_->publish(make_status());
    response->success = true;
    response->message = "navigation_rearmed";
  }

  void get_status(GetStatus::Request::SharedPtr, GetStatus::Response::SharedPtr response) {
    response->status = make_status();
  }

  void on_operator_goal(const geometry_msgs::msg::PoseStamped::SharedPtr input) {
    if (input->header.frame_id != "map") {
      RCLCPP_WARN(
          get_logger(), "operator goal rejected: expected frame 'map', got '%s'",
          input->header.frame_id.c_str());
      return;
    }
    if (!operator_action_client_ || !operator_action_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), "operator goal rejected: NavigateTo action unavailable");
      return;
    }

    NavigateTo::Goal goal;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal.map_id = machine_.map_id();
    }
    goal.goal = *input;
    const double integral_seconds = std::floor(operator_goal_timeout_sec_);
    goal.timeout.sec = static_cast<int32_t>(integral_seconds);
    goal.timeout.nanosec = static_cast<uint32_t>(
        std::llround((operator_goal_timeout_sec_ - integral_seconds) * 1.0e9));
    if (goal.timeout.nanosec >= 1000000000U) {
      ++goal.timeout.sec;
      goal.timeout.nanosec -= 1000000000U;
    }

    auto options = rclcpp_action::Client<NavigateTo>::SendGoalOptions();
    options.goal_response_callback = [this](const OperatorGoalHandle::SharedPtr& handle) {
      if (!handle) {
        RCLCPP_WARN(
            get_logger(),
            "operator goal rejected by NavigateTo state, localization, safety, or backend gate");
      } else {
        RCLCPP_INFO(get_logger(), "operator goal accepted by NavigateTo action");
      }
    };
    options.result_callback = [this](const OperatorGoalHandle::WrappedResult& result) {
      if (result.result) {
        RCLCPP_INFO(
            get_logger(), "operator navigation finished: success=%s reason=%s",
            result.result->success ? "true" : "false", result.result->reason.c_str());
      }
    };
    operator_action_client_->async_send_goal(goal, options);
  }

  void load_startup_map() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (startup_map_loaded_) {
        startup_map_timer_->cancel();
        return;
      }
      if (machine_.state() != NavigationState::kIdle) return;
    }

    auto request = std::make_shared<LoadMap::Request>();
    auto response = std::make_shared<LoadMap::Response>();
    request->map_id = startup_map_id_;
    load_map(request, response);
    if (!response->accepted) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "startup map %s not loaded yet: %s",
          startup_map_id_.c_str(), response->reason.c_str());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      startup_map_loaded_ = true;
    }
    startup_map_timer_->cancel();
    RCLCPP_INFO(
        get_logger(), "startup map loaded; waiting for RViz initial pose: %s",
        startup_map_id_.c_str());
  }

  void on_operator_initial_pose(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr input) {
    if (input->header.frame_id != "map") {
      RCLCPP_WARN(
          get_logger(), "initial pose rejected: expected frame 'map', got '%s'",
          input->header.frame_id.c_str());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!startup_map_loaded_ ||
          machine_.state() != NavigationState::kLocalizing ||
          machine_.map_id() != startup_map_id_) {
        RCLCPP_WARN(
            get_logger(),
            "initial pose rejected: startup map is not waiting for localization");
        return;
      }
    }

    auto request = std::make_shared<Localize::Request>();
    auto response = std::make_shared<Localize::Response>();
    request->map_id = startup_map_id_;
    request->use_initial_pose = true;
    request->initial_pose = *input;
    localize(request, response);
    if (!response->accepted) {
      RCLCPP_WARN(
          get_logger(), "initial pose rejected by localization lifecycle: %s",
          response->reason.c_str());
      return;
    }
    RCLCPP_INFO(
        get_logger(),
        "initial pose accepted; waiting for stable HEALTHY localization");
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      const std::shared_ptr<const NavigateTo::Goal> goal) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (goal_lifecycle_busy_locked() || reset_in_progress_ || map_load_pending_ ||
        map_load_outcome_unknown_ || nav2_goal_outcome_unknown_ ||
        machine_.state() != NavigationState::kReady ||
        goal->map_id != machine_.map_id() ||
        goal->goal.header.frame_id != "map" ||
        safety_latched_ || localization_state_ != LocalizationStatus::STATE_HEALTHY ||
        !nav2_client_->action_server_is_ready()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_dispatch_pending_ = true;
    ++lifecycle_epoch_;
    active_goal_cancel_requested_ = false;
    pending_goal_task_id_ = uuid_string(uuid);
    active_goal_task_id_ = pending_goal_task_id_;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle> goal_handle) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto task_id = uuid_string(goal_handle->get_goal_id());
      begin_cancellation_locked("action_cancel_pending_zero_ack");
      if (active_goal_task_id_ == task_id) active_goal_cancel_requested_ = true;
    }
    const auto zero = latch_zero_velocity_and_wait();
    nav2_client_->async_cancel_all_goals();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      record_zero_latch(zero);
      end_cancellation_locked();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    auto self = std::static_pointer_cast<NavigationServer>(shared_from_this());
    std::thread([self, goal_handle]() { self->execute(goal_handle); }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> outer) {
    const std::string task_id = uuid_string(outer->get_goal_id());
    std::string first_click_reason;
    if (!compute_and_validate_first_click_path(
            outer->get_goal()->goal, &first_click_reason)) {
      bool dispatch_cancelled = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_goal_task_id_ == task_id) {
          dispatch_cancelled = active_goal_cancel_requested_ || outer->is_canceling();
        }
        if (goal_dispatch_pending_ && pending_goal_task_id_ == task_id) {
          goal_dispatch_pending_ = false;
          pending_goal_task_id_.clear();
        }
      }
      auto result = std::make_shared<NavigateTo::Result>();
      result->success = false;
      result->code = dispatch_cancelled ? 499 : kInvalidMap;
      result->task_id = task_id;
      result->reason = dispatch_cancelled ?
          "cancelled_before_nav2_dispatch" : first_click_reason;
      if (dispatch_cancelled && outer->is_canceling()) outer->canceled(result);
      else outer->abort(result);
      settle_goal_lifecycle(task_id);
      return;
    }
    bool navigation_started = false;
    bool dispatch_cancelled = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool owns_pending = goal_dispatch_pending_ &&
          pending_goal_task_id_ == task_id && active_goal_task_id_ == task_id;
      dispatch_cancelled = outer->is_canceling() ||
          (owns_pending && active_goal_cancel_requested_);
      const bool dispatch_valid = owns_pending && !dispatch_cancelled &&
          !map_load_pending_ && machine_.state() == NavigationState::kReady &&
          outer->get_goal()->map_id == machine_.map_id() && !safety_latched_ &&
          localization_state_ == LocalizationStatus::STATE_HEALTHY;
      if (dispatch_valid) {
        navigation_started = machine_.start_navigation(task_id).accepted;
        if (navigation_started) last_nav_pose_ = {};
      }
      if (owns_pending) {
        goal_dispatch_pending_ = false;
        pending_goal_task_id_.clear();
      }
    }
    if (!navigation_started) {
      // The action was accepted but lost the state race before dispatching a
      // Nav2 goal. Treat that abort like every other navigation terminal:
      // latch zero first so another concurrently-started path cannot retain
      // motion authority after this action reports completion.
      const auto zero = latch_zero_velocity_and_wait();
      nav2_client_->async_cancel_all_goals();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        record_zero_latch(zero);
      }
      auto result = std::make_shared<NavigateTo::Result>();
      result->success = false;
      result->code = zero.latched ?
          (dispatch_cancelled ? 499 : kInvalidState) : kUnavailable;
      result->task_id = task_id;
      result->reason = zero.latched ?
          (dispatch_cancelled ? "cancelled_before_nav2_dispatch" :
           "state_changed_before_execution") :
          "terminal_zero_latch_failed:" + zero.reason;
      if (dispatch_cancelled && outer->is_canceling()) outer->canceled(result);
      else outer->abort(result);
      settle_goal_lifecycle(task_id);
      return;
    }
    // Publish the execution lease before Nav2 can emit a non-zero command.
    status_pub_->publish(make_status());
    Nav2Navigate::Goal nav2_goal;
    nav2_goal.pose = outer->get_goal()->goal;
    auto options = rclcpp_action::Client<Nav2Navigate>::SendGoalOptions();
    const auto goal_response_timed_out = std::make_shared<std::atomic_bool>(false);
    options.goal_response_callback =
        [this, goal_response_timed_out](const Nav2GoalHandle::SharedPtr& handle) {
          // A cancel-all sent at the timeout boundary can race a goal that the
          // Nav2 action server accepts just afterwards.  Bind cleanup to the
          // late response too, so such a goal cannot remain orphaned.
          if (handle && goal_response_timed_out->load(std::memory_order_acquire)) {
            sunhorizon_navigation_server::cancel_goal_if_known(*nav2_client_, handle);
          }
        };
    options.feedback_callback = [this, outer, task_id](
        Nav2GoalHandle::SharedPtr,
        const std::shared_ptr<const Nav2Navigate::Feedback> feedback) {
      auto output = std::make_shared<NavigateTo::Feedback>();
      output->task_id = task_id;
      output->state = static_cast<uint8_t>(NavigationState::kNavigating);
      output->current_pose = feedback->current_pose;
      output->remaining_distance_m = feedback->distance_remaining;
      bool feedback_is_current = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        feedback_is_current = active_goal_task_id_ == task_id &&
            machine_.state() == NavigationState::kNavigating &&
            machine_.task_id() == task_id;
        if (!feedback_is_current) return;
        output->safety_reason = safety_reason_;
        last_nav_pose_ = feedback->current_pose;
      }
      outer->publish_feedback(output);
    };
    const auto goal_future = nav2_client_->async_send_goal(nav2_goal, options);
    if (goal_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      goal_response_timed_out->store(true, std::memory_order_release);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        nav2_goal_outcome_unknown_ = true;
        ++lifecycle_epoch_;
        safety_latched_ = true;
        safety_reason_ = "nav2_goal_response_timeout_restart_required";
      }
      const auto zero = latch_zero_velocity_and_wait();
      nav2_client_->async_cancel_all_goals();
      finish_outer(
          outer, task_id, false, kUnavailable,
          "nav2_goal_response_timeout_cancel_all_requested", zero);
      return;
    }
    const auto nav2_handle = goal_future.get();
    if (!nav2_handle) {
      const auto zero = latch_zero_velocity_and_wait();
      nav2_client_->async_cancel_all_goals();
      finish_outer(
          outer, task_id, false, kUnavailable, "nav2_goal_rejected", zero);
      return;
    }
    const auto result_future = nav2_client_->async_get_result(nav2_handle);
    const auto timeout = rclcpp::Duration(outer->get_goal()->timeout).seconds();
    const auto started = std::chrono::steady_clock::now();
    while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      const bool action_cancelled = outer->is_canceling();
      if (action_cancelled || goal_cancel_requested(task_id)) {
        const auto zero = latch_zero_velocity_and_wait();
        sunhorizon_navigation_server::cancel_goal_if_known(*nav2_client_, nav2_handle);
        finish_outer(
            outer, task_id, false, 499, "cancelled", zero, action_cancelled);
        return;
      }
      if (timeout > 0.0 &&
          std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() > timeout) {
        const auto zero = latch_zero_velocity_and_wait();
        sunhorizon_navigation_server::cancel_goal_if_known(*nav2_client_, nav2_handle);
        finish_outer(outer, task_id, false, 408, "timeout", zero);
        return;
      }
    }
    const auto wrapped = result_future.get();
    const bool action_cancelled = outer->is_canceling();
    if (action_cancelled || goal_cancel_requested(task_id)) {
      const auto zero = latch_zero_velocity_and_wait();
      // The result is already terminal and the action client has retired this
      // handle. Preserve the cancellation outcome without canceling it again.
      finish_outer(
          outer, task_id, false, 499, "cancelled_at_nav2_terminal", zero,
          action_cancelled);
      return;
    }
    const bool success = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED;
    const auto zero = latch_zero_velocity_and_wait(task_id);
    finish_outer(outer, task_id, success, success ? kOk : 500,
        success ? "navigation_complete" : "nav2_navigation_failed", zero);
  }

  void finish_outer(const std::shared_ptr<GoalHandle>& outer, const std::string& task_id,
      bool success, int32_t code, const std::string& reason,
      const ZeroLatchResult& zero, bool canceled = false) {
    bool final_success = success;
    int32_t final_code = code;
    std::string final_reason = reason;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      record_zero_latch(zero);
      if (terminal_zero_guard_.matches(task_id) &&
          !terminal_zero_guard_.fault().empty()) {
        final_success = false;
        final_code = kInvalidState;
        final_reason = "navigation_terminal_interrupted:" + terminal_zero_guard_.fault();
        if (machine_.state() == NavigationState::kNavigating) {
          machine_.fault(final_reason);
        }
      } else if (terminal_zero_guard_.matches(task_id) &&
          !terminal_zero_guard_.acknowledged(task_id, zero.latched, zero.reason)) {
        final_success = false;
        final_code = kUnavailable;
        final_reason = "terminal_zero_latch_failed:" + zero.reason;
        machine_.fault(final_reason);
      } else if (zero.latched) {
        const auto transition = machine_.finish_navigation(task_id, success, reason);
        if (!transition.accepted) {
          final_success = false;
          final_code = kInvalidState;
          final_reason =
              "navigation_terminal_state_changed:" + transition.reason +
              ";cause:" + machine_.reason();
        }
      } else {
        final_success = false;
        final_code = kUnavailable;
        final_reason = "terminal_zero_latch_failed:" + zero.reason;
        machine_.fault(final_reason);
      }
      terminal_zero_guard_.clear(task_id);
    }
    auto result = std::make_shared<NavigateTo::Result>();
    result->success = final_success;
    result->code = final_code;
    result->task_id = task_id;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      result->final_pose = last_nav_pose_;
    }
    result->reason = final_reason;
    if (final_success) outer->succeed(result);
    else if (canceled || outer->is_canceling()) outer->canceled(result);
    else outer->abort(result);
    settle_goal_lifecycle(task_id);
  }

  std::string map_root_;
  std::string provider_profile_;
  std::string nav2_planner_id_;
  double provider_timeout_sec_{2.0};
  double post_seed_healthy_stability_sec_{1.0};
  double nav2_plan_timeout_sec_{2.0};
  double nominal_navigation_speed_mps_{0.10};
  double first_click_max_pose_age_sec_{0.85};
  double first_click_min_distance_m_{0.45};
  double first_click_max_distance_m_{0.60};
  double first_click_max_path_length_m_{0.70};
  double first_click_max_heading_error_rad_{0.08726646259971647};
  double first_click_max_goal_yaw_delta_rad_{0.08726646259971647};
  double operator_goal_timeout_sec_{120.0};
  bool operator_goal_adapter_enabled_{false};
  bool operator_initial_pose_adapter_enabled_{false};
  bool first_click_gate_enabled_{false};
  bool goal_dispatch_pending_{false};
  bool active_goal_cancel_requested_{false};
  bool map_load_pending_{false};
  bool map_load_outcome_unknown_{false};
  bool localization_request_pending_{false};
  bool localization_outcome_unknown_{false};
  bool nav2_goal_outcome_unknown_{false};
  bool reset_in_progress_{false};
  bool startup_map_loaded_{false};
  bool startup_localization_failure_observed_{false};
  std::size_t cancellation_operations_in_progress_{0U};
  std::uint64_t lifecycle_epoch_{0U};
  std::uint64_t localization_status_epoch_{0U};
  std::string pending_goal_task_id_;
  std::string active_goal_task_id_;
  std::string operator_goal_topic_;
  std::string operator_initial_pose_topic_;
  std::string startup_map_id_;
  std::mutex operation_mutex_;
  std::mutex mutex_;
  NavigationStateMachine machine_;
  uint8_t localization_state_{LocalizationStatus::STATE_UNKNOWN};
  bool safety_latched_{true};
  bool localization_requested_{false};
  sunhorizon_navigation_server::LocalizationHealthGate localization_health_gate_;
  sunhorizon_navigation_server::TerminalZeroGuard terminal_zero_guard_;
  std::string safety_reason_{"waiting_for_velocity_supervisor"};
  geometry_msgs::msg::PoseStamped last_nav_pose_;
  std::shared_ptr<tf2_ros::Buffer> first_click_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> first_click_tf_listener_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<NavigationStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<LocalizationStatus>::SharedPtr localization_sub_;
  rclcpp::Subscription<SupervisorStatus>::SharedPtr supervisor_sub_;
  rclcpp::Client<sunhorizon_navigation_interfaces::srv::StartMapping>::SharedPtr start_provider_;
  rclcpp::Client<sunhorizon_navigation_interfaces::srv::StopMapping>::SharedPtr stop_provider_;
  rclcpp::Client<sunhorizon_navigation_interfaces::srv::SaveMap>::SharedPtr save_provider_;
  rclcpp::Client<sunhorizon_navigation_interfaces::srv::LoadMap>::SharedPtr load_provider_;
  rclcpp::Client<sunhorizon_navigation_interfaces::srv::Localize>::SharedPtr localize_provider_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr zero_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
  rclcpp_action::Client<ComputePath>::SharedPtr compute_path_client_;
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav2_client_;
  rclcpp_action::Client<NavigateTo>::SharedPtr operator_action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr operator_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      operator_initial_pose_sub_;
  rclcpp::Service<sunhorizon_navigation_interfaces::srv::StartMapping>::SharedPtr start_service_;
  rclcpp::Service<sunhorizon_navigation_interfaces::srv::StopMapping>::SharedPtr stop_service_;
  rclcpp::Service<sunhorizon_navigation_interfaces::srv::SaveMap>::SharedPtr save_service_;
  rclcpp::Service<sunhorizon_navigation_interfaces::srv::ValidateMap>::SharedPtr validate_service_;
  rclcpp::Service<sunhorizon_navigation_interfaces::srv::LoadMap>::SharedPtr load_service_;
  rclcpp::Service<sunhorizon_navigation_interfaces::srv::Localize>::SharedPtr localize_service_;
  rclcpp::Service<sunhorizon_navigation_interfaces::srv::PlanTo>::SharedPtr plan_service_;
  rclcpp::Service<sunhorizon_navigation_interfaces::srv::Cancel>::SharedPtr cancel_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr operator_cancel_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr operator_reset_service_;
  rclcpp::Service<sunhorizon_navigation_interfaces::srv::GetStatus>::SharedPtr get_status_service_;
  rclcpp_action::Server<NavigateTo>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr startup_map_timer_;
  rclcpp::CallbackGroup::SharedPtr startup_map_callback_group_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
