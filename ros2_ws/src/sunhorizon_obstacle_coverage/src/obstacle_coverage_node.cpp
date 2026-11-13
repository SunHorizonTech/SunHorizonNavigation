#include <chrono>
#include <atomic>
#include "sunhorizon_obstacle_coverage/coverage_direct_runtime.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sunhorizon_navigation_interfaces/msg/obstacle_coverage_status.hpp>
#include <sunhorizon_navigation_interfaces/msg/obstacle_coverage_decision.hpp>
#include <sunhorizon_navigation_interfaces/msg/obstacle_coverage_spatial_decision.hpp>
#include <sunhorizon_navigation_interfaces/msg/rgbd_status.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sunhorizon_obstacle_coverage/coverage_evaluator.hpp"
#include "sunhorizon_obstacle_coverage/belly_health_policy.hpp"
#include "sunhorizon_obstacle_coverage/region_support_tracker.hpp"
#include "sunhorizon_obstacle_coverage/region_upstream_deadline_policy.hpp"
#include "sunhorizon_obstacle_coverage/rgbd_upstream_diagnostics.hpp"
#include "sunhorizon_obstacle_coverage/rgbd_fault_reference.hpp"
#include "sunhorizon_obstacle_coverage/spatial_evidence_message.hpp"
#include "sunhorizon_navigation_core/latest_observer.hpp"

namespace {
namespace ud = sunhorizon_obstacle_coverage::upstream_diagnostics;

using ObstacleCoverageStatus =
    sunhorizon_navigation_interfaces::msg::ObstacleCoverageStatus;
using ObstacleCoverageDecision =
    sunhorizon_navigation_interfaces::msg::ObstacleCoverageDecision;
using RgbdStatus = sunhorizon_navigation_interfaces::msg::RgbdStatus;
using SpatialCoverageEvidence = sunhorizon_navigation_interfaces::msg::SpatialCoverageEvidence;
using ObstacleCoverageSpatialDecision =
    sunhorizon_navigation_interfaces::msg::ObstacleCoverageSpatialDecision;
using SpatialEvidenceTracker = sunhorizon_obstacle_coverage::SpatialEvidenceTracker;
using ValidatedSpatialEvidence = sunhorizon_obstacle_coverage::ValidatedSpatialEvidence;
constexpr const char* kSpatialInputTopic = "/sunhorizon/obstacle_coverage/spatial_evidence_v1";
constexpr const char* kSpatialDecisionTopic = "/sunhorizon/obstacle_coverage/spatial_decision_v2";

std::string spatial_gid_hex(const uint8_t* bytes, std::size_t size) {
  std::ostringstream out; out << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < size; ++i) out << std::setw(2) << static_cast<unsigned>(bytes[i]);
  return out.str();
}

bool valid_spatial_gid(const std::string& gid) {
  return gid.size() == 2 * RMW_GID_STORAGE_SIZE &&
      gid.find_first_not_of("0123456789abcdef") == std::string::npos &&
      gid.find_first_not_of('0') != std::string::npos;
}
using sunhorizon_obstacle_coverage::CoverageEvaluator;
using sunhorizon_obstacle_coverage::require_processed_observation;
using sunhorizon_obstacle_coverage::CoverageFreshness;
using sunhorizon_obstacle_coverage::CoverageInputs;
using sunhorizon_obstacle_coverage::CoverageProfile;
using sunhorizon_obstacle_coverage::RgbdCoverageEvidence;
using sunhorizon_obstacle_coverage::SourceObservation;
using sunhorizon_obstacle_coverage::evaluate_rgbd_coverage_health;
using sunhorizon_obstacle_coverage::oldest_source_stamp;
using sunhorizon_obstacle_coverage::parse_profile;
using sunhorizon_obstacle_coverage::profile_name;

double stamp_seconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

int64_t steady_nanoseconds() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

rclcpp::QoS latest_sensor_qos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

rclcpp::QoS latest_status_qos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

double age_seconds(double now_sec, double stamp_sec) {
  if (!std::isfinite(now_sec) || !std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return now_sec - stamp_sec;
}

void append_timing(
    std::ostringstream* output, const char* name,
    const SourceObservation& observation, double now_sec) {
  if (output->tellp() != std::streampos(0)) {
    *output << ';';
  }
  *output << name
          << "(header_age=" << age_seconds(now_sec, observation.stamp_sec)
          << ",receipt_age=" << age_seconds(now_sec, observation.receipt_stamp_sec)
          << ",queue_delay=";
  if (std::isfinite(observation.stamp_sec) && observation.stamp_sec > 0.0 &&
      std::isfinite(observation.receipt_stamp_sec) &&
      observation.receipt_stamp_sec > 0.0) {
    *output << observation.receipt_stamp_sec - observation.stamp_sec;
  } else {
    *output << std::numeric_limits<double>::infinity();
  }
  *output << ')';
}

std::string timing_diagnostics(
    CoverageProfile profile, const CoverageInputs& inputs) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(3);
  if (profile != CoverageProfile::kVisual) {
    append_timing(&output, "airy", inputs.airy, inputs.now_sec);
  }
  if (profile != CoverageProfile::kLidar) {
    append_timing(&output, "chest", inputs.chest, inputs.now_sec);
    append_timing(&output, "belly", inputs.belly, inputs.now_sec);
  }
  return output.str();
}

class ObstacleCoverageNode final : public rclcpp::Node, public sunhorizon_obstacle_coverage::CoverageDirectRuntime {
 public:
  explicit ObstacleCoverageNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
      std::optional<sunhorizon_obstacle_coverage::CoverageDirectOptions> direct = {})
      : Node("sunhorizon_obstacle_coverage", options), direct_(std::move(direct)),
        tf_buffer_(get_clock()),
        transforms_(direct_ ? direct_->transforms : std::shared_ptr<tf2_ros::Buffer>(&tf_buffer_,[](auto*){})) {
    if(direct_ && get_parameter("use_sim_time").as_bool())throw std::invalid_argument("coverage_direct_requires_realtime_clock");
    if(!transforms_)throw std::invalid_argument("coverage_direct_transform_buffer_required");
    if(!direct_) {
      // Standalone's existing executor owns TF, in a group independent of
      // status/sensor callbacks. Every lookup below is nonblocking. Avoid the
      // implicit listener thread: immediate destruction can cancel before its
      // spin() starts, then join forever after spin resets its running flag.
      tf_callback_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      auto dynamic_options=tf2_ros::detail::get_default_transform_listener_sub_options<>();
      auto static_options=tf2_ros::detail::get_default_transform_listener_static_sub_options<>();
      dynamic_options.callback_group=tf_callback_group_;
      static_options.callback_group=tf_callback_group_;
      tf_listener_=std::make_unique<tf2_ros::TransformListener>(tf_buffer_,this,false,
        tf2_ros::DynamicListenerQoS(),tf2_ros::StaticListenerQoS(),dynamic_options,static_options);
    }
    else if(!direct_->spatial_owner || !direct_->spatial_publisher || !direct_->producer_failed ||
        direct_->spatial_owner->get_node_base_interface()->get_context()!=get_node_base_interface()->get_context())
      throw std::invalid_argument("coverage_direct_actual_producer_required");
    const auto upstream_endpoint = ud::endpoint_from_environment();
    upstream_diagnostics_enabled_ = !upstream_endpoint.empty();
    if (upstream_diagnostics_enabled_) {
      upstream_receipt_queue_ = std::make_unique<ud::Queue<>>();
      upstream_decision_queue_ = std::make_unique<ud::Queue<>>();
      upstream_exporter_ = std::make_unique<ud::Exporter>(*upstream_receipt_queue_,
          *upstream_decision_queue_, upstream_endpoint, "coverage");
    }
    const CoverageProfile profile = parse_profile(declare_parameter("profile", "lidar"));
    CoverageFreshness freshness;
    freshness.airy_sec = declare_parameter("airy_freshness_sec", 0.30);
    freshness.chest_sec = declare_parameter("chest_freshness_sec", 0.30);
    freshness.belly_sec = declare_parameter("belly_freshness_sec", 0.30);
    belly_freshness_sec_ = freshness.belly_sec;
    evaluator_ = std::make_unique<CoverageEvaluator>(profile, freshness);

    frame_id_ = declare_parameter("frame_id", "base_footprint");
    const double status_rate_hz = declare_parameter("status_rate_hz", 20.0);
    if (!std::isfinite(status_rate_hz) || status_rate_hz <= 0.0) {
      throw std::runtime_error("status_rate_hz must be finite and positive");
    }

    const auto output_topic = declare_parameter(
        "output_topic", "/sunhorizon/obstacle_coverage/status");
    status_pub_ = create_publisher<ObstacleCoverageStatus>(
        output_topic, rclcpp::QoS(1).reliable().transient_local());

    // Keep the large Airy sample, the small RGB-D status, and the fail-closed
    // status timer independently schedulable. KeepLast(1) bounds DDS history,
    // but it cannot prevent a single-threaded executor from delaying a ready
    // status callback behind point-cloud work.
    airy_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rgbd_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    status_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    if (profile != CoverageProfile::kVisual) {
      const auto airy_topic = declare_parameter(
          "airy_points_topic", "/sunhorizon/airy/points");
      rclcpp::SubscriptionOptions airy_options;
      airy_options.callback_group = airy_callback_group_;
      if(!direct_)airy_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          airy_topic, latest_sensor_qos(),
          std::bind(&ObstacleCoverageNode::on_airy, this, std::placeholders::_1),
          airy_options);
    }
    if (profile != CoverageProfile::kLidar) {
      chest_frame_id_ = declare_parameter(
          "chest_frame_id", "camera_chest_color_optical_frame");
      belly_frame_id_ = declare_parameter(
          "belly_frame_id", "camera_belly_depth_optical_frame");
      if (chest_frame_id_.empty() || belly_frame_id_.empty()) {
        throw std::runtime_error("RGB-D coverage frame ids must not be empty");
      }
      const auto rgbd_status_topic = declare_parameter(
          "rgbd_status_topic", "/sunhorizon/rgbd/status");
      rclcpp::SubscriptionOptions rgbd_options;
      rgbd_options.callback_group = rgbd_callback_group_;
      if(!direct_)rgbd_sub_ = create_subscription<RgbdStatus>(
          rgbd_status_topic, latest_status_qos(),
          std::bind(&ObstacleCoverageNode::on_rgbd, this, std::placeholders::_1),
          rgbd_options);
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.read_only = true;
      const auto processed_topic = declare_parameter<std::string>(
          "belly_obstacle_points_topic", "", descriptor);
      processed_belly_freshness_sec_ = declare_parameter<double>(
          "belly_obstacle_freshness_sec", .30, descriptor);
      if (!std::isfinite(processed_belly_freshness_sec_) || processed_belly_freshness_sec_ <= 0.) {
        throw std::runtime_error("belly_obstacle_freshness_sec must be finite and positive");
      }
      require_processed_belly_ = !processed_topic.empty();
      if (require_processed_belly_) {
        rclcpp::SubscriptionOptions options;
        options.callback_group = airy_callback_group_;
        if(!direct_)belly_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            processed_topic, latest_sensor_qos(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
              auto observation = observe_cloud(*m);
              observation.structurally_healthy = observation.structurally_healthy &&
                  observation.frame_id == belly_frame_id_;
              const std::lock_guard<std::mutex> lock(observations_mutex_);
              belly_cloud_ = std::move(observation);
            }, options);
      }
    }

    rcl_interfaces::msg::ParameterDescriptor region_descriptor;
    region_descriptor.read_only = true;
    const bool region_required = declare_parameter<bool>(
        "region_support_required", false, region_descriptor);
    const bool typed_required = declare_parameter<bool>(
        "region_typed_evaluation_required", false, region_descriptor);
    const bool spatial_required = declare_parameter<bool>(
        "spatial_visibility_required", false, region_descriptor);
    belly_health_policy_ = sunhorizon_obstacle_coverage::BellyHealthPolicy(bool(direct_),spatial_required);
    const auto upstream_deadline_profile = declare_parameter<std::string>(
        "region_upstream_status_deadline_profile",
        sunhorizon_obstacle_coverage::kStrictRegionUpstreamDeadlineProfile,
        region_descriptor);
    if (typed_required && (!region_required || !require_processed_belly_ ||
        profile != CoverageProfile::kFusionShadow)) {
      throw std::runtime_error(
          "typed region evidence requires region support, processed belly and fusion_shadow");
    }
    if (spatial_required && (region_required || typed_required || !require_processed_belly_ ||
        (profile != CoverageProfile::kFusionShadow && profile != CoverageProfile::kVisual))) {
      throw std::runtime_error(
          "spatial visibility requires fusion_shadow or visual and processed belly");
    }
    sunhorizon_obstacle_coverage::validate_region_upstream_status_deadlines(
        upstream_deadline_profile, profile, region_required || spatial_required,
        typed_required || spatial_required,
        require_processed_belly_, belly_freshness_sec_, processed_belly_freshness_sec_,spatial_required);
    if (spatial_required) {
      sunhorizon_obstacle_coverage::SpatialEvidenceTrackerConfig config;
      const auto camera_profile=declare_parameter<std::string>("camera_freshness_profile",
        sunhorizon_obstacle_coverage::kStrictCameraFreshness,region_descriptor);
      config.maximum_age_ns=sunhorizon_obstacle_coverage::camera_freshness_age_ns(camera_profile);
      if ((config.maximum_age_ns>sunhorizon_obstacle_coverage::kDefaultCameraAgeNs ||
           upstream_deadline_profile==sunhorizon_obstacle_coverage::kLowSpeedCameraFreshness ||
           upstream_deadline_profile==sunhorizon_obstacle_coverage::kOneSecondCameraFreshness) &&
          upstream_deadline_profile!=camera_profile)
        throw std::invalid_argument("coverage_camera_freshness_profile_mismatch");
      config.owner_epoch = spatial_owner_gid_ = declare_parameter<std::string>(
          "spatial_producer_gid", "", region_descriptor);
      const auto identity_mode = declare_parameter<std::string>(
          "spatial_producer_identity_mode", "exact-gid-v1", region_descriptor);
      spatial_startup_pin_ = identity_mode == "startup-pin-once-v1";
      if ((identity_mode != "exact-gid-v1" && !spatial_startup_pin_) ||
          (spatial_startup_pin_ ? !spatial_owner_gid_.empty() : !valid_spatial_gid(spatial_owner_gid_))) {
        throw std::runtime_error("spatial producer identity mode/GID mismatch");
      }
      spatial_owner_node_ = declare_parameter<std::string>(
          "spatial_producer_node", "", region_descriptor);
      config.contract = declare_parameter<std::string>("spatial_contract", "", region_descriptor);
      config.query_digest = declare_parameter<std::string>("spatial_query_digest", "", region_descriptor);
      config.producer_epoch = declare_parameter<std::string>("spatial_producer_epoch", "", region_descriptor);
      config.clock_boot_id = declare_parameter<std::string>("spatial_clock_boot_id", "", region_descriptor);
      config.frame_id = frame_id_;
      const auto ids = declare_parameter<std::vector<int64_t>>("spatial_source_ids", {}, region_descriptor);
      const auto epochs = declare_parameter<std::vector<std::string>>("spatial_source_epochs", {}, region_descriptor);
      const auto frames = declare_parameter<std::vector<std::string>>("spatial_source_frames", {}, region_descriptor);
      if (spatial_owner_node_.empty() ||
          spatial_owner_node_.size() > 256 || spatial_owner_node_.front() != '/' ||
          ids.empty() || ids.size() > 3 || ids.size() != epochs.size() || ids.size() != frames.size()) {
        throw std::runtime_error("spatial explicit producer/source identities required");
      }
      for (std::size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] < 0 || ids[i] > 2) throw std::runtime_error("spatial source id invalid");
        config.source_identities.push_back({static_cast<uint8_t>(ids[i]), epochs[i], frames[i]});
      }
      if(direct_){
        const auto producer_camera_profile=direct_->spatial_owner->get_parameter("camera_freshness_profile").as_string();
        if(producer_camera_profile!=camera_profile)
          throw std::invalid_argument("coverage_direct_camera_freshness_profile_mismatch");
        const auto actual=direct_endpoint();
        if(!actual.first.empty() || (!spatial_startup_pin_ && spatial_owner_gid_!=actual.second) ||
            direct_->spatial_owner->get_parameter("producer_epoch").as_string()!=config.producer_epoch ||
            direct_->spatial_owner->get_parameter("clock_boot_id").as_string()!=config.clock_boot_id)
          throw std::invalid_argument("coverage_direct_producer_contract_mismatch");
        spatial_owner_gid_=config.owner_epoch=actual.second;
        spatial_startup_pin_=false; // Actual immutable owner is bound before acquisition.
      }
      spatial_tracker_config_ = config;
      // No observation can enter this placeholder tracker. Its missing result
      // stays hard until one authenticated callback freezes the real endpoint.
      if (spatial_startup_pin_) config.owner_epoch = "startup-unbound";
      spatial_tracker_ = std::make_unique<SpatialEvidenceTracker>(std::move(config),
          sunhorizon_obstacle_coverage::SpatialRecoveryPolicy::CoverageObservations);
      if (std::string(status_pub_->get_topic_name()) == kSpatialDecisionTopic ||
          std::string(status_pub_->get_topic_name()) == kSpatialInputTopic) {
        throw std::runtime_error("spatial topics must remain separate from legacy status");
      }
      spatial_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions options; options.callback_group = spatial_callback_group_;
      if(!direct_)spatial_sub_ = create_subscription<SpatialCoverageEvidence>(kSpatialInputTopic, latest_status_qos(),
          std::bind(&ObstacleCoverageNode::on_spatial, this, std::placeholders::_1, std::placeholders::_2), options);
      spatial_decision_pub_ = create_publisher<ObstacleCoverageSpatialDecision>(
          kSpatialDecisionTopic, rclcpp::QoS(1).reliable().durability_volatile());
      if ((!direct_ && std::string(spatial_sub_->get_topic_name()) != kSpatialInputTopic) ||
          std::string(spatial_decision_pub_->get_topic_name()) != kSpatialDecisionTopic) {
        throw std::runtime_error("spatial topics are fixed and cannot be remapped");
      }
    }
    if (region_required) {
      sunhorizon_obstacle_coverage::RegionSupportConfig config;
      region_status_name_ = config.status_name = declare_parameter<std::string>(
          "region_support_status_name", "", region_descriptor);
      config.region_id = declare_parameter<std::string>(
          "region_support_id", "", region_descriptor);
      config.base_frame_id = frame_id_;
      config.image_frame_id = belly_frame_id_;
      config.contract = declare_parameter<std::string>(
          "region_support_contract", "", region_descriptor);
      config.typed_evaluation_required = typed_required;
      region_contract_ = config.contract;
      const auto topic = declare_parameter<std::string>(
          "region_support_diagnostics_topic", "", region_descriptor);
      if (topic.empty()) throw std::runtime_error("region diagnostics topic is empty");
      region_tracker_ = std::make_unique<
          sunhorizon_obstacle_coverage::RegionSupportTracker>(std::move(config));
      region_callback_group_ = create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions options;
      options.callback_group = region_callback_group_;
      if(!direct_)region_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
          topic, latest_sensor_qos(),
          std::bind(&ObstacleCoverageNode::on_region, this, std::placeholders::_1), options);
    }

    if (typed_required) {
      constexpr const char* decision_topic = "/sunhorizon/obstacle_coverage/decision_v1";
      // Reject the legacy topic collision before DDS sees two message types
      // on one topic, so invalid configuration has a deterministic failure.
      if (std::string(status_pub_->get_topic_name()) == decision_topic) {
        throw std::runtime_error("typed decision topic must remain fixed and separate from status");
      }
      decision_pub_ = create_publisher<ObstacleCoverageDecision>(
          decision_topic, rclcpp::QoS(1).reliable().durability_volatile());
      if (std::string(decision_pub_->get_topic_name()) != decision_topic) {
        throw std::runtime_error("typed decision topic must remain fixed and separate from status");
      }
    }

    const auto period = std::chrono::duration<double>(1.0 / status_rate_hz);
    status_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ObstacleCoverageNode::publish_status, this),
        status_callback_group_);
    if (direct_ && spatial_decision_pub_) {
      // A fresh proof must not wait for the next health heartbeat. This lane
      // only wakes the existing publisher; it never changes source deadlines.
      // Keep DDS/TF/publication off the perception delivery callback.
      spatial_publication_ = std::make_unique<SpatialPublication>(
          [this](auto) { publish_status(); },
          [this](std::exception_ptr) { revoke("spatial_publication_failed"); });
    }
    RCLCPP_WARN(
        get_logger(), "Obstacle coverage profile '%s' active; rear coverage is disabled",
        profile_name(profile).c_str());
  }

  ~ObstacleCoverageNode() override { stop(); }

  rclcpp::Node::SharedPtr node() override {return shared_from_this();}
  void receive_airy(sensor_msgs::msg::PointCloud2::ConstSharedPtr input,int64_t receipt) override {
    require_direct();if(!input || receipt<=0)throw std::invalid_argument("coverage_direct_airy_input_missing");
    auto value=observe_cloud(*input);value.receipt_stamp_sec=double(receipt)/1e9;
    const std::lock_guard<std::mutex> lock(observations_mutex_);airy_cloud_=std::move(value);
  }
  void receive_belly(sensor_msgs::msg::PointCloud2::ConstSharedPtr input,int64_t receipt) override {
    require_direct();if(!input || receipt<=0)throw std::invalid_argument("coverage_direct_belly_input_missing");
    auto value=observe_cloud(*input);value.receipt_stamp_sec=double(receipt)/1e9;
    value.structurally_healthy=value.structurally_healthy && value.frame_id==belly_frame_id_;
    const std::lock_guard<std::mutex> lock(observations_mutex_);belly_cloud_=std::move(value);
  }
  void receive_rgbd(RgbdStatus::ConstSharedPtr input) override {
    require_direct();if(!input)throw std::invalid_argument("coverage_direct_rgbd_missing");on_rgbd(input);
  }
  void receive_region(diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr input) override {
    require_direct();if(region_tracker_){if(!input)throw std::invalid_argument("coverage_direct_region_missing");on_region(input);}
  }
  sunhorizon_obstacle_coverage::SpatialDeliveryResult receive_spatial(SpatialCoverageEvidence::ConstSharedPtr input) override {
    require_direct();
    using State=sunhorizon_obstacle_coverage::SpatialDeliveryState;
    if(!input || !spatial_tracker_)return {State::InternalFault,"spatial_direct_input_or_tracker_missing"};
    const auto endpoint=direct_endpoint();
    accept_spatial(input,endpoint.second,endpoint);
    sunhorizon_obstacle_coverage::SpatialDeliveryResult result;
    {
      const std::lock_guard<std::mutex> lock(observations_mutex_);
      result=spatial_tracker_->delivery_result();
      if(spatial_binding_terminal_ || direct_revoked_.load()) result.state=State::InternalFault;
    }
    if (spatial_publication_) spatial_publication_->enqueue(publication_wakeup_);
    return result;
  }
  void revoke(const std::string& reason) override {
    if(direct_revoked_.exchange(true))return;
    const std::lock_guard<std::mutex> lock(observations_mutex_);
    if(!direct_failure_.empty())return;
    direct_failure_=reason.substr(0,512);
    if(direct_failure_.empty())direct_failure_="unspecified_fault";
    if(spatial_tracker_){spatial_tracker_->invalidate(direct_failure_);spatial_binding_terminal_=true;}
    if(region_tracker_)region_tracker_->invalidate(direct_failure_);
  }
  void stop() override {
    direct_stopped_.store(true);
    if(status_timer_)status_timer_->cancel();
    // Callers/executor are quiescent at external stop; join before destroying
    // any tracker, transform buffer, publisher or test observer used by work.
    spatial_publication_.reset();
  }

 private:
  void require_direct() const {if(!direct_ || direct_stopped_.load())throw std::logic_error("coverage_direct_owner_not_running");}
  std::pair<std::string,std::string> direct_endpoint(){
    if(direct_->producer_failed())return {"spatial_direct_producer_revoked",{}};
    if(direct_->spatial_owner->get_fully_qualified_name()!=spatial_owner_node_ ||
        std::string(direct_->spatial_publisher->get_topic_name())!=kSpatialInputTopic)
      return {"spatial_producer_identity_changed",{}};
    const auto gid=direct_->spatial_publisher->get_gid();
    const auto value=spatial_gid_hex(gid.data,RMW_GID_STORAGE_SIZE);
    if(!valid_spatial_gid(value))return {"spatial_producer_identity_changed",{}};
    return {{},value};
  }
  std::pair<std::string, std::string> spatial_endpoint() {
    if(direct_)return direct_endpoint();
    const auto peers =
#ifdef SUNHORIZON_COVERAGE_UPSTREAM_TEST
        spatial_test_endpoints_ ? spatial_test_endpoints_() :
#endif
        get_publishers_info_by_topic(kSpatialInputTopic);
    if (peers.empty()) return {"spatial_producer_missing", {}};
    if (peers.size() != 1) return {"spatial_producer_not_unique", {}};
    const auto& peer = peers.front();
    const auto ns = peer.node_namespace();
    const auto gid = spatial_gid_hex(peer.endpoint_gid().data(), peer.endpoint_gid().size());
    if (!valid_spatial_gid(gid)) return {"spatial_producer_identity_changed", {}};
    // The observed Fast DDS placeholder is incomplete discovery, not an owner.
    // Only this exact pair with an already known correct type may wait; empty
    // fields, known wrong FQNs/types, and invalid GIDs remain rejected.
    if (peer.node_name() == "_NODE_NAME_UNKNOWN_" &&
        (ns == "_NODE_NAMESPACE_UNKNOWN_" || ns == "/_NODE_NAMESPACE_UNKNOWN_") &&
        peer.topic_type() == "sunhorizon_navigation_interfaces/msg/SpatialCoverageEvidence") {
      return {"spatial_producer_discovery_pending", gid};
    }
    const auto node = (ns.empty() || ns == "/" ? "/" : ns + "/") + peer.node_name();
    if (node != spatial_owner_node_ ||
        peer.topic_type() != "sunhorizon_navigation_interfaces/msg/SpatialCoverageEvidence") {
      return {"spatial_producer_identity_changed", {}};
    }
    return {{}, gid};
  }

  std::string spatial_endpoint_error() {
    const auto endpoint = spatial_endpoint();
    if (!endpoint.first.empty()) return endpoint.first;
    const std::lock_guard<std::mutex> lock(observations_mutex_);
    if (spatial_startup_pin_ && spatial_owner_gid_.empty()) return "spatial_producer_missing";
    if (endpoint.second != spatial_owner_gid_) return "spatial_producer_identity_changed";
    return {};
  }

  void on_spatial(const SpatialCoverageEvidence::ConstSharedPtr input, const rclcpp::MessageInfo& info) {
    const auto& gid = info.get_rmw_message_info().publisher_gid;
    const auto owner = spatial_gid_hex(gid.data, RMW_GID_STORAGE_SIZE);
    accept_spatial(input,owner,spatial_endpoint());
  }
  bool accept_spatial(const SpatialCoverageEvidence::ConstSharedPtr& input,const std::string& owner,
      const std::pair<std::string,std::string>& endpoint){
    const std::lock_guard<std::mutex> lock(observations_mutex_);
    if (spatial_binding_terminal_) return false;
    if (spatial_startup_pin_ && spatial_owner_gid_.empty() &&
        (endpoint.first == "spatial_producer_missing" ||
         (endpoint.first == "spatial_producer_discovery_pending" && owner == endpoint.second))) {
      return false;  // No evidence or identity is accepted until metadata qualifies.
    }
    if (!endpoint.first.empty() || owner != endpoint.second) {
      spatial_binding_terminal_ = true;
      spatial_tracker_->invalidate(endpoint.first.empty() ? "spatial_producer_message_gid_mismatch" : endpoint.first);
      return false;
    }
    if (spatial_startup_pin_ && spatial_owner_gid_.empty()) {
      spatial_owner_gid_ = owner;
      spatial_tracker_config_.owner_epoch = owner;
      *spatial_tracker_ = SpatialEvidenceTracker(spatial_tracker_config_,
          sunhorizon_obstacle_coverage::SpatialRecoveryPolicy::CoverageObservations);
      RCLCPP_INFO(get_logger(), "SPATIAL_OWNER_FROZEN mode=startup-pin-once-v1 node=%s gid=%s producer_epoch=%s",
          spatial_owner_node_.c_str(), owner.c_str(), spatial_tracker_config_.producer_epoch.c_str());
    }
    if (owner != spatial_owner_gid_) {
      spatial_binding_terminal_ = true;
      spatial_tracker_->invalidate("spatial_producer_identity_changed"); return false;
    }
    const auto receipt_ros = get_clock()->now().nanoseconds();
    const auto receipt_steady = steady_nanoseconds();
    if (spatial_tracker_->observe(sunhorizon_obstacle_coverage::spatial_evidence_observation(*input),
        owner, receipt_ros, receipt_steady)) {
      spatial_wire_ = *input;
      spatial_seen_ = true;return true;
    }
    return false;
  }

  void on_region(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr input) {
    const diagnostic_msgs::msg::DiagnosticStatus* selected = nullptr;
    bool duplicate = false;
    for (const auto& status : input->status) {
      if (status.name != region_status_name_) continue;
      if (selected) duplicate = true;
      selected = &status;
    }
    // Only the pinned producer status is region evidence, even if an unrelated
    // diagnostic is accidentally published on this dedicated topic.
    if (!selected) return;
    const std::lock_guard<std::mutex> lock(observations_mutex_);
    // Take both clocks after serialization with the timer. An earlier callback
    // timestamp delivered after a concurrent timer evaluation is not a clock rollback.
    const auto receipt_ros = get_clock()->now().nanoseconds();
    const auto receipt_steady = steady_nanoseconds();
    if (duplicate || input->header.stamp.sec <= 0 ||
        input->header.stamp.nanosec >= 1000000000U || selected->values.size() > 64) {
      region_tracker_->invalidate("malformed_region_diagnostic_array");
      return;
    }
    sunhorizon_obstacle_coverage::RegionSupportReport report;
    report.name = selected->name;
    report.level = selected->level;
    report.header_ros_ns = static_cast<int64_t>(input->header.stamp.sec) * 1000000000LL +
        input->header.stamp.nanosec;
    for (const auto& value : selected->values) {
      if (value.key.size() > 128 || value.value.size() > 16384) {
        region_tracker_->invalidate("oversize_region_diagnostic_value");
        return;
      }
      report.values.emplace_back(value.key, value.value);
    }
    (void)region_tracker_->observe(report, receipt_ros, receipt_steady);
  }

  void on_airy(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input) {
    auto observation = observe_cloud(*input);
    const std::lock_guard<std::mutex> lock(observations_mutex_);
    airy_cloud_ = std::move(observation);
  }

  struct CloudObservation {
    bool seen{false};
    bool structurally_healthy{false};
    double stamp_sec{0.0};
    double receipt_stamp_sec{0.0};
    std::string frame_id;
  };

  CloudObservation observe_cloud(
      const sensor_msgs::msg::PointCloud2& input) {
    CloudObservation output;
    output.seen = true;
    output.stamp_sec = stamp_seconds(input.header.stamp);
    output.receipt_stamp_sec = get_clock()->now().seconds();
    output.frame_id = input.header.frame_id;
    output.structurally_healthy =
        !output.frame_id.empty() && input.width > 0U && input.height > 0U &&
        input.point_step > 0U && input.row_step > 0U && !input.data.empty();
    return output;
  }

  bool transform_available(const CloudObservation& input) {
    if (!input.seen || input.frame_id.empty()) return false;
    return transform_available(input.frame_id);
  }

  bool transform_available(const std::string& input_frame_id) {
    if (input_frame_id.empty()) return false;
    try {
      (void)transforms_->lookupTransform(
          frame_id_, input_frame_id, tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException&) {
      return false;
    }
  }

  SourceObservation cloud_source(const CloudObservation& input) {
    return SourceObservation{
        input.seen,
        input.structurally_healthy && transform_available(input),
        input.stamp_sec,
        input.receipt_stamp_sec};
  }

  SourceObservation rgbd_source(
      const SourceObservation& status, const std::string& input_frame_id) {
    auto output = status;
    output.healthy = output.healthy && transform_available(input_frame_id);
    return output;
  }

  void on_rgbd(const RgbdStatus::ConstSharedPtr input) {
    const double status_stamp_sec = stamp_seconds(input->header.stamp);
    const auto receipt_clock = direct_ ? rclcpp::Time(input->header.stamp,get_clock()->get_clock_type()) : get_clock()->now();
    const double receipt_stamp_sec = receipt_clock.seconds();
    const auto health = evaluate_rgbd_coverage_health(RgbdCoverageEvidence{
        input->chest_pair_fresh,
        input->chest_camera_info_valid,
        input->chest_alignment_accepted,
        input->chest_depth_scale_accepted,
        input->chest_pointcloud_healthy,
        input->belly_depth_fresh,
        input->belly_camera_info_valid,
        input->belly_depth_scale_accepted,
        input->belly_pointcloud_healthy});
    const SourceObservation chest_status{
        true,
        health.chest,
        oldest_source_stamp(
            status_stamp_sec, input->chest_pair_age_sec,
            input->chest_pointcloud_age_sec),
        receipt_stamp_sec};
    const SourceObservation belly_status{
        true,
        health.belly,
        belly_health_policy_.source_stamp(
            status_stamp_sec, input->belly_depth_age_sec,
            input->belly_pointcloud_age_sec),
        receipt_stamp_sec};
    const std::lock_guard<std::mutex> lock(observations_mutex_);
    chest_status_ = chest_status;
    belly_status_ = belly_status;
    rgbd_fault_reference_ = rgbd_fault_state_.capture(*input, receipt_clock.nanoseconds());
    if (upstream_diagnostics_enabled_) {
      ud::Record receipt;
      receipt.event = ud::Event::kRgbdReceipt;
      receipt.callback_sequence = ++upstream_rgbd_sequence_;
      receipt.status_header_ns = static_cast<int64_t>(input->header.stamp.sec) * 1000000000LL + input->header.stamp.nanosec;
      receipt.receipt_ros_ns = receipt_clock.nanoseconds();
      receipt.event_steady_ns = ud::steady_ns();
      receipt.ages = ud::age_bits(*input); receipt.health = ud::health_bits(*input);
      receipt.header_ambiguous = upstream_rgbd_header_.observe(receipt.status_header_ns);
      receipt.input_seen = true;
      upstream_rgbd_reference_ = receipt;
      upstream_receipt_queue_->try_push(receipt);
    }
  }

  void publish_status() {
    // Timer and event delivery share one publication owner and cannot publish
    // an older snapshot after a newer event-driven decision.
    const std::lock_guard<std::mutex> publication_lock(publication_mutex_);
    if (direct_stopped_.load()) return;
    CloudObservation airy_cloud;
    CloudObservation belly_cloud;
    SourceObservation chest_status;
    SourceObservation belly_status;
    sunhorizon_obstacle_coverage::RgbdFaultReference fault_reference;
    SpatialCoverageEvidence spatial_wire;
    ValidatedSpatialEvidence spatial;
    // Endpoint discovery is outside the observation mutex, like TF work. The
    // authenticated message GID is also checked on every input callback.
    const auto spatial_endpoint_current = spatial_tracker_ ? spatial_endpoint() : std::pair<std::string, std::string>{};
    ud::Record upstream_reference;
    const auto snapshot_begin_ns = upstream_diagnostics_enabled_ ? ud::steady_ns() : 0;
    {
      const std::lock_guard<std::mutex> lock(observations_mutex_);
      airy_cloud = airy_cloud_;
      belly_cloud = belly_cloud_;
      chest_status = chest_status_;
      belly_status = belly_status_;
      fault_reference = rgbd_fault_reference_;
      if (spatial_tracker_) {
        auto spatial_endpoint_failure = spatial_endpoint_current.first;
        if (spatial_endpoint_failure.empty() && !spatial_owner_gid_.empty() &&
            spatial_endpoint_current.second != spatial_owner_gid_) {
          spatial_endpoint_failure = "spatial_producer_identity_changed";
        }
        // Initial absence is startup missing evidence, not auto-adoption. Once
        // observed, losing or replacing the pinned endpoint is a hard fault.
        const bool startup_pending = spatial_startup_pin_ && spatial_owner_gid_.empty() &&
            spatial_endpoint_failure == "spatial_producer_discovery_pending";
        if (!spatial_endpoint_failure.empty() && !startup_pending &&
            (spatial_seen_ || (spatial_startup_pin_ && !spatial_owner_gid_.empty()) ||
             spatial_endpoint_failure != "spatial_producer_missing")) {
          spatial_tracker_->invalidate(spatial_endpoint_failure);
          spatial_binding_terminal_ = true;
        }
        spatial = spatial_tracker_->evaluate(get_clock()->now().nanoseconds(), steady_nanoseconds());
        spatial_wire = spatial_wire_;
      }
      if (upstream_diagnostics_enabled_) upstream_reference = upstream_rgbd_reference_;
    }

    const auto snapshot_end_ns = upstream_diagnostics_enabled_ ? ud::steady_ns() : 0;
#ifdef SUNHORIZON_COVERAGE_UPSTREAM_TEST
    if (after_observation_snapshot_) after_observation_snapshot_();
#endif

    CoverageInputs inputs;
    inputs.airy = cloud_source(airy_cloud);
    inputs.chest = rgbd_source(chest_status, chest_frame_id_);
    inputs.belly = rgbd_source(belly_status, belly_frame_id_);
    const auto raw_belly = inputs.belly;  // Keep the same pre-merge snapshot for failure evidence.
    const auto processed_belly = require_processed_belly_ ? cloud_source(belly_cloud) : SourceObservation{};
    // Sample now after the snapshot and TF lookups. A subscription may update
    // its receipt timestamp while this timer is being dispatched; sampling
    // before the snapshot could classify that valid concurrent update as a
    // future receipt. Sampling before TF work could instead understate age if
    // a transform lookup is ever delayed.
    auto [now, decision, region] = [&]() {
      if (decision_pub_ || spatial_decision_pub_) {
        // Typed output is one coherent calculation: region callbacks cannot
        // replace the measurement after its shared evaluation/header clock.
        // All TF work above and both publishes below remain outside this lock.
        const std::lock_guard<std::mutex> lock(observations_mutex_);
        const auto now = get_clock()->now();
        const auto steady = steady_nanoseconds();
        inputs.now_sec = now.seconds();
        if (belly_health_policy_.require_processed(require_processed_belly_)) {
          inputs.belly = require_processed_observation(
              inputs.belly, processed_belly, inputs.now_sec, belly_freshness_sec_, processed_belly_freshness_sec_);
        }
        const auto decision = evaluator_->evaluate(inputs);
        const auto region = region_tracker_ ? region_tracker_->evaluate(now.nanoseconds(), steady) :
            sunhorizon_obstacle_coverage::RegionSupportDecision{};
        if (spatial_tracker_) {
          const auto current = spatial_tracker_->evaluate(now.nanoseconds(), steady);
          if (!current.valid) {
            spatial = current;
          } else if (spatial.valid && (now.nanoseconds() > spatial.valid_until_ros_ns ||
                                      steady > spatial.valid_until_steady_ns)) {
            // A concurrent newer input cannot renew the INITIAL snapshot used
            // for this publication after TF or executor work consumed its age.
            spatial_tracker_->invalidate_deadline("spatial_snapshot_expired_after_tf");
            spatial = spatial_tracker_->evaluate(now.nanoseconds(), steady);
          }
        }
        return std::make_tuple(now, decision, region);
      }
      // Preserve the legacy sampling order when typed output is disabled.
      const auto now = get_clock()->now();
      inputs.now_sec = now.seconds();
      if (belly_health_policy_.require_processed(require_processed_belly_)) {
        inputs.belly = require_processed_observation(
            inputs.belly, processed_belly, inputs.now_sec, belly_freshness_sec_, processed_belly_freshness_sec_);
      }
      return std::make_tuple(now, evaluator_->evaluate(inputs),
          sunhorizon_obstacle_coverage::RegionSupportDecision{});
    }();

    ObstacleCoverageStatus output;
    output.header.stamp = now;
    output.header.frame_id = frame_id_;
    output.profile = profile_name(evaluator_->profile());
    output.healthy = decision.healthy;
    output.forward_coverage = decision.forward_coverage;
    output.near_field_coverage = decision.near_field_coverage;
    output.rear_coverage = false;
    output.required_sources = decision.required_sources;
    output.healthy_sources = decision.healthy_sources;
    output.reason = decision.reason;
    {const std::lock_guard<std::mutex> lock(observations_mutex_);
      if(!direct_failure_.empty()){
        output.healthy=false;output.forward_coverage=false;output.near_field_coverage=false;
        output.reason="direct_coverage_revoked:"+direct_failure_;
      }
    }
    if (region_tracker_) {
      // Evaluate the persistent tracker under the same mutex after TF work;
      // copying it would lose its cross-timer clock rollback high-water marks.
      if (!decision_pub_) {
        const std::lock_guard<std::mutex> lock(observations_mutex_);
        region = region_tracker_->evaluate(
            get_clock()->now().nanoseconds(), steady_nanoseconds());
      }
      if (!region.healthy) {
        output.healthy = false;
        output.forward_coverage = false;
        output.near_field_coverage = false;
        output.reason += ";region_support:" + region.reason;
      }
    }
    if (spatial_tracker_ && (!spatial.valid || spatial.disposition != 1)) {
      output.healthy = false; output.forward_coverage = false; output.near_field_coverage = false;
      output.reason += ";spatial_visibility:" + spatial.reason;
    }
    status_pub_->publish(output);
    if (spatial_decision_pub_) {
      ObstacleCoverageSpatialDecision envelope;
      envelope.decision.coverage = output;
      envelope.decision.schema_version = 2;
      envelope.decision.non_region_healthy = decision.healthy && decision.forward_coverage &&
          decision.near_field_coverage && !direct_revoked_.load() && require_processed_belly_ &&
          (evaluator_->profile() == CoverageProfile::kFusionShadow || evaluator_->profile() == CoverageProfile::kVisual);
      envelope.decision.disposition = envelope.decision.non_region_healthy && spatial.valid ?
          spatial.disposition : 0;
      // Schema 2 uses the explicit spatial wire; do not masquerade cell counts
      // or input generations as legacy ray/evaluation fields.
      envelope.spatial = spatial_wire;
      spatial_decision_pub_->publish(envelope);
#ifdef SUNHORIZON_COVERAGE_UPSTREAM_TEST
      if (spatial_test_observer_) spatial_test_observer_(envelope);
#endif
    }
    if (decision_pub_) {
      using Disposition = sunhorizon_obstacle_coverage::RegionDisposition;
      using Code = sunhorizon_obstacle_coverage::RegionEvaluationCode;
      ObstacleCoverageDecision evidence;
      evidence.coverage = output;  // Exact same final snapshot and header; no second evaluation.
      evidence.schema_version = 1;
      evidence.non_region_healthy = decision.healthy && decision.forward_coverage &&
          decision.near_field_coverage && !direct_revoked_.load() && require_processed_belly_ &&
          evaluator_->profile() == CoverageProfile::kFusionShadow;
      auto disposition = Disposition::kHardFault;
      if (evidence.non_region_healthy && region.evaluation_complete) {
        if (region.healthy && region.evaluation_code == Code::kClear &&
            region.disposition == Disposition::kClear) {
          disposition = Disposition::kClear;
        } else if (!region.healthy && region.evaluation_code == Code::kIncompleteRays &&
            region.disposition == Disposition::kGeometryHold) {
          disposition = Disposition::kGeometryHold;
        }
      }
      evidence.disposition = static_cast<uint8_t>(disposition);
      evidence.region_evaluation_complete = region.evaluation_complete;
      evidence.region_evaluation_code = static_cast<uint8_t>(region.evaluation_code);
      evidence.region_disposition = static_cast<uint8_t>(region.disposition);
      evidence.region_source_stamp_ns = region.source_stamp_ns;
      evidence.region_frame_sequence = region.frame_sequence;
      evidence.region_evaluation_sequence = region.evaluation_sequence;
      evidence.region_contract = region_contract_;
      if (upstream_diagnostics_enabled_) {
        upstream_reference.sequence = ++upstream_decision_sequence_;
        upstream_reference.decision_header_ns = now.nanoseconds();
        upstream_reference.header_ambiguous = upstream_reference.header_ambiguous ||
            upstream_decision_header_.observe(now.nanoseconds());
        upstream_reference.snapshot_begin_ns = snapshot_begin_ns;
        upstream_reference.snapshot_end_ns = snapshot_end_ns;
        upstream_reference.event = ud::Event::kDecisionPublishBegin;
        upstream_reference.event_steady_ns = ud::steady_ns();
        upstream_decision_queue_->try_push(upstream_reference);
      }
      decision_pub_->publish(evidence);
      if (upstream_diagnostics_enabled_) {
        upstream_reference.event = ud::Event::kDecisionPublishReturn;
        upstream_reference.event_steady_ns = ud::steady_ns();
        upstream_decision_queue_->try_push(upstream_reference);
#ifdef SUNHORIZON_COVERAGE_UPSTREAM_TEST
        if (upstream_test_observer_) upstream_test_observer_(upstream_reference, evidence);
#endif
      }
    }

#ifdef SUNHORIZON_COVERAGE_UPSTREAM_TEST
    if (fault_test_observer_) fault_test_observer_(fault_reference, output);
#endif
    if (output.healthy) {
      if (last_reason_ != output.reason) {
        RCLCPP_INFO(get_logger(), "Obstacle coverage healthy");
      }
    } else {
      const auto timing = timing_diagnostics(evaluator_->profile(), inputs);
      // Status remains fail-closed at the configured 20 Hz. Only diagnostics
      // are rate-limited so repeated cached timestamps cannot create log load.
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Obstacle coverage unhealthy: %s; timing=%s; belly_sources=%s; %s",
          output.reason.c_str(), timing.c_str(),
          // Format only inside the existing throttled log; no new clock or TF lookup.
          sunhorizon_obstacle_coverage::processed_observation_diagnostics(
              raw_belly, processed_belly, inputs.now_sec, belly_freshness_sec_,
              processed_belly_freshness_sec_, belly_health_policy_.require_processed(require_processed_belly_)).c_str(),
          sunhorizon_obstacle_coverage::rgbd_fault_reference_diagnostics(
              fault_reference, now.nanoseconds()).c_str());
    }
    last_reason_ = output.reason;
  }

  std::string frame_id_;
  std::string chest_frame_id_;
  std::string belly_frame_id_;
  std::string last_reason_;
  std::mutex publication_mutex_;
  std::mutex observations_mutex_;
  CloudObservation airy_cloud_;
  CloudObservation belly_cloud_;
  bool require_processed_belly_{false};
  sunhorizon_obstacle_coverage::BellyHealthPolicy belly_health_policy_;
  double belly_freshness_sec_{.30};
  double processed_belly_freshness_sec_{.30};
  SourceObservation chest_status_;
  SourceObservation belly_status_;
  sunhorizon_obstacle_coverage::RgbdFaultReferenceState rgbd_fault_state_;
  sunhorizon_obstacle_coverage::RgbdFaultReference rgbd_fault_reference_;
  std::unique_ptr<CoverageEvaluator> evaluator_;
  std::string region_status_name_;
  std::string region_contract_;
  std::unique_ptr<sunhorizon_obstacle_coverage::RegionSupportTracker> region_tracker_;
  std::unique_ptr<SpatialEvidenceTracker> spatial_tracker_;
  sunhorizon_obstacle_coverage::SpatialEvidenceTrackerConfig spatial_tracker_config_;
  SpatialCoverageEvidence spatial_wire_;  // Same mutex/anchor as the tracker.
  std::string spatial_owner_gid_, spatial_owner_node_;
  bool spatial_seen_{false};
  bool spatial_startup_pin_{false}, spatial_binding_terminal_{false};
  rclcpp::Subscription<SpatialCoverageEvidence>::SharedPtr spatial_sub_;
  rclcpp::Publisher<ObstacleCoverageSpatialDecision>::SharedPtr spatial_decision_pub_;
  rclcpp::CallbackGroup::SharedPtr spatial_callback_group_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr region_sub_;
  rclcpp::CallbackGroup::SharedPtr region_callback_group_;
  rclcpp::Publisher<ObstacleCoverageStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<ObstacleCoverageDecision>::SharedPtr decision_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr airy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr belly_sub_;
  rclcpp::Subscription<RgbdStatus>::SharedPtr rgbd_sub_;
  rclcpp::CallbackGroup::SharedPtr airy_callback_group_;
  rclcpp::CallbackGroup::SharedPtr rgbd_callback_group_;
  rclcpp::CallbackGroup::SharedPtr status_callback_group_;
  std::optional<sunhorizon_obstacle_coverage::CoverageDirectOptions> direct_;
  std::atomic<bool> direct_stopped_{false},direct_revoked_{false};
  std::string direct_failure_; // observations_mutex_, terminal and never reset.
  tf2_ros::Buffer tf_buffer_; // Legacy tests/standalone keep their actual local buffer.
  std::shared_ptr<tf2_ros::Buffer> transforms_;
  rclcpp::CallbackGroup::SharedPtr tf_callback_group_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  using SpatialPublication = sunhorizon_navigation_core::LatestObserver<int>;
  const std::shared_ptr<const int> publication_wakeup_=std::make_shared<const int>(0);
  std::unique_ptr<SpatialPublication> spatial_publication_;
  bool upstream_diagnostics_enabled_{false};
  uint64_t upstream_rgbd_sequence_{0}, upstream_decision_sequence_{0};
  ud::HeaderIdentity upstream_rgbd_header_, upstream_decision_header_;
  ud::Record upstream_rgbd_reference_;  // Protected by observations_mutex_.
  std::unique_ptr<ud::Queue<>> upstream_receipt_queue_, upstream_decision_queue_;
  std::unique_ptr<ud::Exporter> upstream_exporter_;  // Destroy before queues.
#ifdef SUNHORIZON_COVERAGE_UPSTREAM_TEST
  friend struct CoverageUpstreamTestPeer;
  std::function<void()> after_observation_snapshot_;
  std::function<void(const ud::Record&, const ObstacleCoverageDecision&)> upstream_test_observer_;
  std::function<void(const sunhorizon_obstacle_coverage::RgbdFaultReference&,
      const ObstacleCoverageStatus&)> fault_test_observer_;
  std::function<void(const ObstacleCoverageSpatialDecision&)> spatial_test_observer_;
  std::function<std::vector<rclcpp::TopicEndpointInfo>()> spatial_test_endpoints_;
#endif
};

}  // namespace

namespace sunhorizon_obstacle_coverage {
std::shared_ptr<CoverageDirectRuntime> make_coverage_direct_runtime(CoverageDirectOptions options){
  const auto node_options=options.node_options;
  return std::make_shared<ObstacleCoverageNode>(node_options,std::move(options));
}
rclcpp::Node::SharedPtr make_coverage_standalone(const rclcpp::NodeOptions& options){
  return std::make_shared<ObstacleCoverageNode>(options);
}
}  // namespace sunhorizon_obstacle_coverage
