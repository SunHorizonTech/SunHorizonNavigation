#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <crb_ros_msg/srv/get_robot_mode.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sunhorizon_navigation_core/platform_evidence.hpp>
#include <sunhorizon_navigation_interfaces/msg/platform_safety_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "sunhorizon_platform_adapter_casbot02/evidence_timeout_contract.hpp"
#include "sunhorizon_platform_adapter_casbot02/joint_log_evidence.hpp"
#include "sunhorizon_platform_adapter_casbot02/mode_request_lifecycle.hpp"

namespace {

using PlatformSafetyState = sunhorizon_navigation_interfaces::msg::PlatformSafetyState;
using sunhorizon_navigation_core::JointSetEvidence;
using sunhorizon_navigation_core::MotionStateEvidence;
using sunhorizon_navigation_core::MotionStateGate;

class PlatformAdapter final : public rclcpp::Node {
 public:
  PlatformAdapter() : Node("sunhorizon_platform_adapter") {
    const auto mode_service = declare_parameter("robot_mode_service", "/get_robot_mode");
    joint_states_topic_ = declare_parameter(
        "joint_states_topic", "/sunhorizon/platform/proof/live_joint_states");
    joint_evidence_topic_ = declare_parameter(
        "joint_evidence_topic", "/sunhorizon/platform/proof/joint_info");
    const auto motion_state_topic = declare_parameter(
        "motion_robot_state_topic", "/motion/robot_state");
    const auto current_mode_topic = declare_parameter(
        "current_mode_topic", "/motion/current_mode");
    const auto ethercat_topic = declare_parameter(
        "ethercat_op_proof_topic", "/sunhorizon/platform/proof/ethercat_op");
    const auto wkc_topic = declare_parameter(
        "wkc_normal_proof_topic", "/sunhorizon/platform/proof/wkc_normal");
    const auto output_topic = declare_parameter(
        "output_topic", "/sunhorizon/platform/safety_state");
    required_mode_name_ = declare_parameter("required_mode_name", "WALK");
    if (required_mode_name_.empty()) {
      throw std::runtime_error("required_mode_name must not be empty");
    }
    waist_joint_name_ = declare_parameter("waist_joint_name", "waist_yaw");
    required_motion_state_token_ = declare_parameter(
        "required_motion_state_token", "State: STAND");
    allowed_running_motion_state_token_ = declare_parameter(
        "allowed_running_motion_state_token", "State: WALKING");
    navigation_control_token_ = declare_parameter(
        "navigation_control_token", "Mode: NAVIGATION");
    remote_control_exit_token_ = declare_parameter(
        "remote_control_exit_token", "Remote Control State: Exit");
    if (required_motion_state_token_.empty() ||
        allowed_running_motion_state_token_.empty() ||
        required_motion_state_token_ == allowed_running_motion_state_token_) {
      throw std::runtime_error("invalid motion state transition contract");
    }
    ethercat_evidence_timeout_sec_ =
        declare_parameter("ethercat_evidence_timeout_sec", 1.5);
    joint_state_timeout_sec_ = declare_parameter("joint_state_timeout_sec", 0.25);
    joint_fault_timeout_sec_ = declare_parameter("joint_fault_timeout_sec", 1.5);
    motion_state_timeout_sec_ = declare_parameter("motion_state_timeout_sec", 0.50);
    mode_timeout_sec_ = declare_parameter("mode_timeout_sec", 0.50);
    mode_request_timeout_sec_ = declare_parameter("mode_request_timeout_sec", 0.50);
    mode_request_period_sec_ = declare_parameter("mode_request_period_sec", 0.20);
    max_waist_yaw_rad_ = declare_parameter("max_waist_yaw_rad", 0.10);
    if (!sunhorizon_platform_adapter_casbot02::valid_ethercat_evidence_timeout(
            ethercat_evidence_timeout_sec_)) {
      throw std::runtime_error(
          "ethercat_evidence_timeout_sec must be finite, positive, and <= 1.5");
    }
    if (!std::isfinite(joint_state_timeout_sec_) || joint_state_timeout_sec_ <= 0.0 ||
        joint_state_timeout_sec_ > 0.25 || !std::isfinite(joint_fault_timeout_sec_) ||
        joint_fault_timeout_sec_ <= 0.0 || joint_fault_timeout_sec_ > 1.5) {
      throw std::runtime_error("joint evidence freshness limits invalid");
    }
    if (!std::isfinite(mode_timeout_sec_) || mode_timeout_sec_ <= 0.0 ||
        !std::isfinite(mode_request_timeout_sec_) || mode_request_timeout_sec_ <= 0.0 ||
        mode_request_timeout_sec_ > 5.0 || !std::isfinite(mode_request_period_sec_) ||
        mode_request_period_sec_ <= 0.0 || mode_request_period_sec_ >= mode_timeout_sec_) {
      throw std::runtime_error("mode request timing invalid");
    }
    mode_request_lifecycle_ =
        sunhorizon_platform_adapter_casbot02::ModeRequestLifecycle(
            std::chrono::duration_cast<
                sunhorizon_platform_adapter_casbot02::ModeRequestLifecycle::Duration>(
                std::chrono::duration<double>(mode_request_timeout_sec_)),
            std::chrono::duration_cast<
                sunhorizon_platform_adapter_casbot02::ModeRequestLifecycle::Duration>(
                std::chrono::duration<double>(mode_request_period_sec_)));
    joint_mapping_valid_ = declare_parameter("joint_mapping_valid", false);
    leg_joint_names_ = declare_parameter<std::vector<std::string>>(
        "leg_joint_names", std::vector<std::string>{});
    navigation_mode_id_ = declare_parameter("navigation_mode_id", -1);
    publish_base_transform_ = declare_parameter(
        "publish_base_footprint_to_base_link", false);
    base_transform_calibration_accepted_ = declare_parameter(
        "base_footprint_to_base_link_calibration_accepted", false);
    base_footprint_frame_ = declare_parameter("base_footprint_frame", "base_footprint");
    base_link_frame_ = declare_parameter("base_link_frame", "base_link");
    base_link_x_m_ = declare_parameter("base_link_x_m", 0.0);
    base_link_y_m_ = declare_parameter("base_link_y_m", 0.0);
    base_link_z_m_ = declare_parameter("base_link_z_m", 0.0);
    base_link_roll_rad_ = declare_parameter("base_link_roll_rad", 0.0);
    base_link_pitch_rad_ = declare_parameter("base_link_pitch_rad", 0.0);
    base_link_yaw_rad_ = declare_parameter("base_link_yaw_rad", 0.0);
    const auto leg_joint_ids = declare_parameter<std::vector<int64_t>>(
        "leg_joint_ids", std::vector<int64_t>{});
    const auto non_blocking_codes = declare_parameter<std::vector<int64_t>>(
        "non_blocking_leg_error_codes", std::vector<int64_t>{});
    leg_joint_ids_.reserve(leg_joint_ids.size());
    for (const auto id : leg_joint_ids) leg_joint_ids_.push_back(static_cast<int>(id));
    non_blocking_leg_error_codes_.reserve(non_blocking_codes.size());
    for (const auto code : non_blocking_codes) {
      if (code <= 0 || static_cast<uint64_t>(code) > 0xffffffffULL) {
        throw std::runtime_error("non-blocking leg error code outside supported range");
      }
      non_blocking_leg_error_codes_.push_back(static_cast<uint32_t>(code));
    }
    if (std::set<uint32_t>(
            non_blocking_leg_error_codes_.begin(),
            non_blocking_leg_error_codes_.end()).size() !=
        non_blocking_leg_error_codes_.size()) {
      throw std::runtime_error("non-blocking leg error codes must be unique");
    }
    const std::set<std::string> unique_leg_names(
        leg_joint_names_.begin(), leg_joint_names_.end());
    if ((joint_mapping_valid_ &&
         (leg_joint_names_.size() != 12U || unique_leg_names.size() != 12U ||
          unique_leg_names.count("") != 0U ||
          unique_leg_names.count(waist_joint_name_) != 0U)) ||
        (!joint_mapping_valid_ && !leg_joint_names_.empty())) {
      throw std::runtime_error("joint mapping parameters invalid");
    }

    if (publish_base_transform_) {
      const bool finite_transform = std::isfinite(base_link_x_m_) &&
          std::isfinite(base_link_y_m_) && std::isfinite(base_link_z_m_) &&
          std::isfinite(base_link_roll_rad_) && std::isfinite(base_link_pitch_rad_) &&
          std::isfinite(base_link_yaw_rad_);
      if (!base_transform_calibration_accepted_ || !finite_transform ||
          base_footprint_frame_.empty() || base_link_frame_.empty() ||
          base_footprint_frame_ == base_link_frame_) {
        throw std::runtime_error("invalid base_footprint_to_base_link parameters");
      }
      base_transform_broadcaster_ =
          std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = get_clock()->now();
      transform.header.frame_id = base_footprint_frame_;
      transform.child_frame_id = base_link_frame_;
      transform.transform.translation.x = base_link_x_m_;
      transform.transform.translation.y = base_link_y_m_;
      transform.transform.translation.z = base_link_z_m_;
      tf2::Quaternion rotation;
      rotation.setRPY(base_link_roll_rad_, base_link_pitch_rad_, base_link_yaw_rad_);
      rotation.normalize();
      transform.transform.rotation = tf2::toMsg(rotation);
      base_transform_broadcaster_->sendTransform(transform);
    }

    publisher_ = create_publisher<PlatformSafetyState>(output_topic, 10);
    mode_client_ = create_client<crb_ros_msg::srv::GetRobotMode>(mode_service);
    joint_evidence_sub_ = create_subscription<diagnostic_msgs::msg::KeyValue>(
        joint_evidence_topic_, 10,
        std::bind(&PlatformAdapter::on_joint_evidence, this, std::placeholders::_1));
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PlatformAdapter::on_joint_states, this, std::placeholders::_1));
    motion_state_sub_ = create_subscription<std_msgs::msg::String>(
        motion_state_topic, 10,
        std::bind(&PlatformAdapter::on_motion_state, this, std::placeholders::_1));
    current_mode_sub_ = create_subscription<std_msgs::msg::Int32>(
        current_mode_topic, 10,
        [this](std_msgs::msg::Int32::SharedPtr input) {
          current_navigation_mode_ = input->data;
          current_mode_stamp_ = get_clock()->now();
        });
    ethercat_sub_ = create_subscription<std_msgs::msg::Bool>(
        ethercat_topic, 10,
        [this](std_msgs::msg::Bool::SharedPtr input) {
          ethercat_op_ = input->data;
          ethercat_stamp_ = get_clock()->now();
        });
    wkc_sub_ = create_subscription<std_msgs::msg::Bool>(
        wkc_topic, 10,
        [this](std_msgs::msg::Bool::SharedPtr input) {
          wkc_normal_ = input->data;
          wkc_stamp_ = get_clock()->now();
        });
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&PlatformAdapter::on_timer, this));
  }

 private:
  bool fresh(const rclcpp::Time& stamp, double timeout) {
    const auto age = (get_clock()->now() - stamp).seconds();
    return stamp.nanoseconds() > 0 && age >= 0.0 && age <= timeout;
  }

  void on_joint_evidence(const diagnostic_msgs::msg::KeyValue::SharedPtr input) {
    if (input->key != "joint_info") return;
    joints_ = sunhorizon_navigation_core::parse_joint_info_json(
        input->value, leg_joint_ids_, non_blocking_leg_error_codes_);
    joints_receipt_stamp_ = get_clock()->now();
  }

  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr input) {
    const rclcpp::Time source_stamp(input->header.stamp, get_clock()->get_clock_type());
    const auto validation =
        sunhorizon_platform_adapter_casbot02::validate_joint_state_sample(
            input->name, input->position, leg_joint_names_, waist_joint_name_,
            source_stamp.nanoseconds(), get_clock()->now().nanoseconds(),
            static_cast<int64_t>(joint_state_timeout_sec_ * 1e9),
            joint_state_source_owner_unique());
    if (!joint_mapping_valid_ || !validation.valid) {
      waist_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      return;
    }
    waist_yaw_rad_ = validation.waist_position_rad;
    waist_stamp_ = source_stamp;
  }

  bool joint_state_source_owner_unique() const {
    const auto endpoints = get_publishers_info_by_topic(joint_states_topic_);
    if (endpoints.size() != 1U) return false;
    const auto& endpoint = endpoints.front();
    return endpoint.node_name() == "sunhorizon_live_joint_state_bridge" &&
        endpoint.node_namespace() == get_namespace() &&
        endpoint.topic_type() == "sensor_msgs/msg/JointState";
  }

  bool joint_log_source_owner_unique() const {
    const auto endpoints = get_publishers_info_by_topic(joint_evidence_topic_);
    if (endpoints.size() != 1U) return false;
    const auto& endpoint = endpoints.front();
    return endpoint.node_name() == "sunhorizon_joint_log_evidence" &&
        endpoint.node_namespace() == get_namespace() &&
        endpoint.topic_type() == "diagnostic_msgs/msg/KeyValue";
  }

  void on_motion_state(const std_msgs::msg::String::SharedPtr input) {
    motion_state_ = sunhorizon_navigation_core::parse_motion_state(
        input->data, required_motion_state_token_, navigation_control_token_,
        remote_control_exit_token_, allowed_running_motion_state_token_);
    motion_state_stamp_ = get_clock()->now();
  }

  void invalidate_mode_evidence() {
    mode_allowed_ = false;
    mode_name_ = "UNKNOWN";
    mode_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  void request_mode() {
    const auto now =
        sunhorizon_platform_adapter_casbot02::ModeRequestLifecycle::Clock::now();
    const auto expired = mode_request_lifecycle_.expire(now);
    if (expired) {
      bool removed = false;
      if (expired->request_id) {
        removed = mode_client_->remove_pending_request(*expired->request_id);
      }
      invalidate_mode_evidence();
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "get_robot_mode request timed out; pending request removed=%s",
          removed ? "true" : "false");
    }
    if (!mode_request_lifecycle_.ready(now) || !mode_client_->service_is_ready()) return;

    const auto request_token = mode_request_lifecycle_.begin(now);
    auto request = std::make_shared<crb_ros_msg::srv::GetRobotMode::Request>();
    try {
      const auto pending = mode_client_->async_send_request(
          request,
          [this, request_token](
              rclcpp::Client<crb_ros_msg::srv::GetRobotMode>::SharedFuture future) {
            if (!mode_request_lifecycle_.complete(request_token)) return;
            try {
              const auto response = future.get();
              mode_name_ = response->mode_name;
              mode_allowed_ = sunhorizon_navigation_core::robot_mode_allowed(
                  mode_name_, required_mode_name_);
              mode_stamp_ = get_clock()->now();
            } catch (const std::exception& error) {
              invalidate_mode_evidence();
              RCLCPP_ERROR(get_logger(), "get_robot_mode failed: %s", error.what());
            }
          });
      if (!mode_request_lifecycle_.bind_request_id(
              request_token, pending.request_id)) {
        mode_client_->remove_pending_request(pending.request_id);
        invalidate_mode_evidence();
        RCLCPP_ERROR(get_logger(), "get_robot_mode request tracking failed");
      }
    } catch (const std::exception& error) {
      mode_request_lifecycle_.abandon(request_token);
      invalidate_mode_evidence();
      RCLCPP_ERROR(get_logger(), "get_robot_mode request failed: %s", error.what());
    }
  }

  void on_timer() {
    request_mode();
    const auto now = get_clock()->now();
    const bool live_joint_state_fresh = joint_mapping_valid_ &&
        joint_state_source_owner_unique() && fresh(waist_stamp_, joint_state_timeout_sec_);
    const bool joint_log_fresh = joint_log_source_owner_unique() && joints_.parsed &&
        joints_.complete && joints_.source_stamp_ns > 0 &&
        now.nanoseconds() >= joints_.source_stamp_ns &&
        static_cast<double>(now.nanoseconds() - joints_.source_stamp_ns) * 1e-9 <=
            joint_fault_timeout_sec_ &&
        fresh(joints_receipt_stamp_, joint_fault_timeout_sec_);
    PlatformSafetyState output;
    output.header.stamp = now;
    output.header.frame_id = "base_link";
    output.mode_name = mode_name_;
    output.mode_allowed = mode_allowed_ && fresh(mode_stamp_, mode_timeout_sec_);
    output.ethercat_op =
        ethercat_op_ && fresh(ethercat_stamp_, ethercat_evidence_timeout_sec_);
    output.wkc_normal =
        wkc_normal_ && fresh(wkc_stamp_, ethercat_evidence_timeout_sec_);
    output.joint_feedback_fresh = live_joint_state_fresh;
    output.leg_joints_error_free = joint_mapping_valid_ && joint_log_fresh &&
        joints_.error_free;
    output.leg_joints_operation_enabled =
        joint_mapping_valid_ && joint_log_fresh && joints_.operation_enabled;
    // Hardware emergency stopping belongs to the platform. The deprecated
    // wire field estop_clear stays default false and grants no motion authority.
    const bool motion_state_fresh = fresh(
        motion_state_stamp_, motion_state_timeout_sec_);
    output.navigation_mode = current_navigation_mode_;
    const bool configured_mode_matches = navigation_mode_id_ < 0 ||
        (fresh(current_mode_stamp_, motion_state_timeout_sec_) &&
         current_navigation_mode_ == navigation_mode_id_);
    const bool waist_safe = fresh(waist_stamp_, joint_state_timeout_sec_) &&
        std::abs(waist_yaw_rad_) <= max_waist_yaw_rad_;
    const bool platform_prerequisites_safe = output.mode_allowed && output.ethercat_op &&
        output.wkc_normal && output.joint_feedback_fresh &&
        output.leg_joints_error_free && output.leg_joints_operation_enabled &&
        configured_mode_matches && waist_safe;
    const auto motion_decision = motion_state_gate_.evaluate(
        motion_state_, motion_state_fresh, platform_prerequisites_safe);
    output.motion_ready = motion_decision.ready;
    output.navigation_mode_discovered = motion_state_fresh &&
        motion_state_.navigation_control_active &&
        motion_state_.remote_control_exited && configured_mode_matches;
    output.waist_yaw_rad = waist_yaw_rad_;
    output.platform_fault = !output.mode_allowed || !output.ethercat_op || !output.wkc_normal ||
        !output.joint_feedback_fresh || !output.leg_joints_error_free ||
        !output.leg_joints_operation_enabled || !output.motion_ready ||
        std::abs(output.waist_yaw_rad) > max_waist_yaw_rad_;

    std::ostringstream reason;
    if (!joint_mapping_valid_) reason << "joint_mapping_not_accepted;";
    if (!output.mode_allowed) reason << "required_walk_mode_not_proven;";
    if (!output.ethercat_op) reason << "ethercat_op_not_proven;";
    if (!output.wkc_normal) reason << "wkc_not_proven;";
    if (!output.joint_feedback_fresh) reason << "joint_feedback_incomplete_or_stale;";
    if (!joint_log_fresh) reason << "joint_fault_evidence_incomplete_or_stale;";
    if (!output.leg_joints_error_free) reason << "leg_joint_error;";
    if (!output.leg_joints_operation_enabled) reason << "leg_joint_not_operation_enabled;";
    if (!output.motion_ready) {
      reason << "motion_state_not_ready:" << motion_decision.reason << ";";
    }
    if (!output.navigation_mode_discovered) reason << "navigation_mode_not_discovered_or_active;";
    if (!waist_safe) {
      reason << "waist_yaw_not_safe;";
    }
    output.reason = reason.str().empty() ? "ok" : reason.str();
    publisher_->publish(output);
  }

  std::vector<int> leg_joint_ids_;
  std::vector<uint32_t> non_blocking_leg_error_codes_;
  std::vector<std::string> leg_joint_names_;
  std::string joint_states_topic_;
  std::string joint_evidence_topic_;
  std::string required_mode_name_;
  std::string waist_joint_name_;
  std::string required_motion_state_token_;
  std::string allowed_running_motion_state_token_;
  std::string navigation_control_token_;
  std::string remote_control_exit_token_;
  double ethercat_evidence_timeout_sec_{1.5};
  double joint_state_timeout_sec_{0.25};
  double joint_fault_timeout_sec_{1.5};
  double motion_state_timeout_sec_{0.50};
  double mode_timeout_sec_{0.50};
  double mode_request_timeout_sec_{0.50};
  double mode_request_period_sec_{0.20};
  double max_waist_yaw_rad_{0.10};
  double base_link_x_m_{0.0};
  double base_link_y_m_{0.0};
  double base_link_z_m_{0.0};
  double base_link_roll_rad_{0.0};
  double base_link_pitch_rad_{0.0};
  double base_link_yaw_rad_{0.0};
  bool joint_mapping_valid_{false};
  bool publish_base_transform_{false};
  bool base_transform_calibration_accepted_{false};
  std::string base_footprint_frame_;
  std::string base_link_frame_;
  int navigation_mode_id_{-1};
  sunhorizon_platform_adapter_casbot02::ModeRequestLifecycle mode_request_lifecycle_{
      std::chrono::milliseconds(500), std::chrono::milliseconds(200)};
  bool mode_allowed_{false};
  std::string mode_name_{"UNKNOWN"};
  bool ethercat_op_{false};
  bool wkc_normal_{false};
  int current_navigation_mode_{-1};
  double waist_yaw_rad_{0.0};
  JointSetEvidence joints_;
  MotionStateEvidence motion_state_;
  MotionStateGate motion_state_gate_;
  rclcpp::Time mode_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time ethercat_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time wkc_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time joints_receipt_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time motion_state_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time current_mode_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time waist_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<PlatformSafetyState>::SharedPtr publisher_;
  rclcpp::Client<crb_ros_msg::srv::GetRobotMode>::SharedPtr mode_client_;
  rclcpp::Subscription<diagnostic_msgs::msg::KeyValue>::SharedPtr joint_evidence_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr current_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ethercat_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wkc_sub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> base_transform_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlatformAdapter>());
  rclcpp::shutdown();
  return 0;
}
