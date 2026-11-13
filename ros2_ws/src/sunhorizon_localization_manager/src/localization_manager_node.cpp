#include <algorithm>
#include <array>
#include <atomic>
#include <map>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sunhorizon_navigation_interfaces/msg/localization_comparison_status.hpp>
#include <sunhorizon_navigation_interfaces/msg/localization_status.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "sunhorizon_localization_manager/localization_manager_logic.hpp"
#include "sunhorizon_localization_manager/localization_direct_runtime.hpp"
#include "sunhorizon_navigation_core/latest_observer.hpp"
#include "sunhorizon_localization_manager/topic_contract.hpp"
#include "sunhorizon_localization_manager/delivery_timing.hpp"  // delivery timing

namespace {

using ComparisonStatus =
    sunhorizon_navigation_interfaces::msg::LocalizationComparisonStatus;
using LocalizationStatus = sunhorizon_navigation_interfaces::msg::LocalizationStatus;
using StatusMarker = visualization_msgs::msg::Marker;
using sunhorizon_localization_manager::BackendDecision;
using sunhorizon_localization_manager::BackendEvidence;
using sunhorizon_localization_manager::CorrectionGate;
using sunhorizon_localization_manager::EvidenceLimits;
using sunhorizon_localization_manager::FrameContract;
using sunhorizon_localization_manager::HealthyPoseStatusPairCache;
using sunhorizon_localization_manager::OdometryHistory;
using sunhorizon_localization_manager::OdometryHistoryConfig;
using sunhorizon_localization_manager::TrackingDecision;
using sunhorizon_localization_manager::DeliveryTiming;  // delivery timing
namespace topics = sunhorizon_localization_manager::topics;

struct AlignedEvidence {
  bool valid{false};
  std::string reason;
  BackendEvidence evidence;
};

class LocalizationManager final : public rclcpp::Node,
    public sunhorizon_localization_manager::LocalizationDirectRuntime {
  using InputStamp = sunhorizon_localization_manager::LocalizationInputStamp;
 public:
  explicit LocalizationManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
      bool direct = false, sunhorizon_localization_manager::LocalizationDirectSinks sinks = {})
      : Node("sunhorizon_localization_manager", options), direct_(direct),
        direct_sinks_(std::move(sinks)), broadcaster_(this) {
    active_backend_ = declare_parameter("active_backend", "lidar");
    livo_ndt_alignment_enabled_ = declare_parameter(
        "livo_ndt_alignment_enabled", false);
    comparison_enabled_ = declare_parameter("comparison_enabled", true);
    rcl_interfaces::msg::ParameterDescriptor frame_descriptor;
    frame_descriptor.read_only = true;
    frames_.preserve_6d_map_alignment = declare_parameter("preserve_6d_map_alignment",false,frame_descriptor);
    frames_.map = declare_parameter("map_frame", "map", frame_descriptor);
    frames_.odom = declare_parameter("odom_frame", "odom", frame_descriptor);
    frames_.base_footprint = declare_parameter(
        "base_footprint_frame", "base_footprint", frame_descriptor);
    limits_.odometry_timeout_sec = declare_parameter("odometry_timeout_sec", 0.50);
    limits_.tracking_odometry_timeout_sec = declare_parameter(
        "tracking_odometry_timeout_sec", 0.25);
    limits_.map_pose_timeout_sec = declare_parameter("map_pose_timeout_sec", 0.50);
    limits_.status_timeout_sec = declare_parameter(
        "backend_status_timeout_sec", 0.50);
    limits_.maximum_measurement_age_sec = declare_parameter(
        "maximum_measurement_age_sec", 0.50);
    limits_.evidence_sync_tolerance_sec = declare_parameter(
        "evidence_sync_tolerance_sec", 0.001);
    limits_.future_tolerance_sec = declare_parameter("future_tolerance_sec", 0.01);
    limits_.quaternion_norm_tolerance = declare_parameter(
        "quaternion_norm_tolerance", 1e-3);
    limits_.maximum_correction_translation_step_m = declare_parameter(
        "maximum_correction_translation_step_m", 0.30);
    limits_.maximum_correction_rotation_step_rad = declare_parameter(
        "maximum_correction_rotation_step_rad", 0.2617993877991494);
    limits_.maximum_comparison_translation_m = declare_parameter(
        "maximum_comparison_translation_m", 0.50);
    limits_.maximum_comparison_yaw_rad = declare_parameter(
        "maximum_comparison_yaw_rad", 0.35);
    tracking_history_duration_sec_ = declare_parameter(
        "tracking_history_duration_sec", 2.0);
    map_pose_odometry_max_time_delta_sec_ = declare_parameter(
        "map_pose_odometry_max_time_delta_sec", 0.05);
    tracking_history_max_samples_ = declare_parameter<std::int64_t>(
        "tracking_history_max_samples", 4096);
    correction_filter_time_constant_sec_ = declare_parameter(
        "correction_filter_time_constant_sec", 0.0);
    status_rate_hz_ = declare_parameter("status_rate_hz", 20.0);

    if (active_backend_ != "lidar" && active_backend_ != "visual" &&
        active_backend_ != "livo") {
      throw std::runtime_error("active_backend must be lidar, visual, or livo");
    }
    if (livo_ndt_alignment_enabled_ && active_backend_ != "livo") {
      throw std::runtime_error(
          "livo_ndt_alignment_enabled requires active_backend=livo");
    }
    if (!std::isfinite(status_rate_hz_) || status_rate_hz_ <= 0.0 ||
        !std::isfinite(limits_.odometry_timeout_sec) ||
        limits_.odometry_timeout_sec <= 0.0 ||
        !std::isfinite(limits_.tracking_odometry_timeout_sec) ||
        limits_.tracking_odometry_timeout_sec <= 0.0 ||
        !std::isfinite(limits_.map_pose_timeout_sec) ||
        limits_.map_pose_timeout_sec <= 0.0 ||
        !std::isfinite(limits_.status_timeout_sec) ||
        limits_.status_timeout_sec <= 0.0 ||
        !std::isfinite(limits_.maximum_measurement_age_sec) ||
        limits_.maximum_measurement_age_sec <= 0.0 ||
        limits_.evidence_sync_tolerance_sec < 0.0 ||
        limits_.future_tolerance_sec < 0.0 ||
        limits_.quaternion_norm_tolerance <= 0.0 ||
        limits_.maximum_correction_translation_step_m <= 0.0 ||
        limits_.maximum_correction_rotation_step_rad <= 0.0 ||
        limits_.maximum_comparison_translation_m < 0.0 ||
        limits_.maximum_comparison_yaw_rad < 0.0 ||
        tracking_history_duration_sec_ < 2.0 ||
        map_pose_odometry_max_time_delta_sec_ <= 0.0 ||
        tracking_history_max_samples_ < 2 ||
        !std::isfinite(correction_filter_time_constant_sec_) ||
        correction_filter_time_constant_sec_ < 0.0 ||
        correction_filter_time_constant_sec_ > 10.0) {
      throw std::runtime_error("invalid localization manager limits");
    }

    const OdometryHistoryConfig history_config{
        tracking_history_duration_sec_,
        map_pose_odometry_max_time_delta_sec_,
        static_cast<std::size_t>(tracking_history_max_samples_),
        limits_.quaternion_norm_tolerance};
    lidar_history_ = std::make_unique<OdometryHistory>(history_config);
    visual_history_ = std::make_unique<OdometryHistory>(history_config);
    livo_history_ = std::make_unique<OdometryHistory>(history_config);
    livo_ndt_alignment_history_ = std::make_unique<OdometryHistory>(history_config);
    correction_gate_ = std::make_unique<CorrectionGate>(
        limits_, correction_filter_time_constant_sec_);

    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        topics::kCanonicalOdometry, rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<LocalizationStatus>(
        topics::kCanonicalStatus, rclcpp::QoS(10).transient_local());
    status_marker_pub_ = create_publisher<StatusMarker>(
        topics::kStatusMarker, rclcpp::QoS(1).transient_local().reliable());
    comparison_pub_ = create_publisher<ComparisonStatus>(
        topics::kComparisonStatus, rclcpp::QoS(10).transient_local());

    if (!direct_) {
    create_backend_subscriptions(
        "lidar", topics::kLidarOdometryCandidate, topics::kLidarMapPoseCandidate,
        topics::kLidarStatus);
    create_backend_subscriptions(
        "visual", topics::kVisualOdometryCandidate, topics::kVisualMapPoseCandidate,
        topics::kVisualStatus);
    create_backend_subscriptions(
        "livo", topics::kLivoOdometryCandidate, topics::kLivoMapPoseCandidate,
        topics::kLivoStatus);

    const auto tracking_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    lidar_tracking_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        topics::kLidarTrackingOdometryCandidate, tracking_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr input) {
          DeliveryTiming timing("tracking", "lidar", std::int64_t(input->header.stamp.sec)*1000000000 + input->header.stamp.nanosec);  // delivery timing
          std::lock_guard<std::mutex> lock(mutex_);
          DeliveryTiming::mark("mutex");  // delivery timing
          record_tracking_odometry(
              "lidar", *input, get_clock()->now().seconds());
        });
    if (livo_ndt_alignment_enabled_) {
      livo_ndt_alignment_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          topics::kLivoNdtOdometryCandidate, tracking_qos,
          [this](nav_msgs::msg::Odometry::SharedPtr input) {
            DeliveryTiming timing("alignment", "livo", std::int64_t(input->header.stamp.sec)*1000000000 + input->header.stamp.nanosec);  // delivery timing
            std::lock_guard<std::mutex> lock(mutex_);
          DeliveryTiming::mark("mutex");  // delivery timing
            livo_ndt_alignment_history_->insert(
                *input, get_clock()->now().seconds());
            publish_active_if_new();
          });
    }
    }
    reset_service_ = create_service<std_srvs::srv::Trigger>(
        topics::kResetService,
        [this](
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          DeliveryTiming timing("reset", "all", 0);  // delivery timing
          std::lock_guard<std::mutex> lock(mutex_);
          DeliveryTiming::mark("mutex");  // delivery timing
          reset_localization_state();
          if (direct_) publish_canonical_status(get_clock()->now());
          response->success = true;
          response->message =
              "localization_manager_reinitialized_waiting_for_new_evidence";
        });

    const auto period = std::chrono::duration<double>(1.0 / status_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&LocalizationManager::on_timer, this));
    if (direct_) {
      if (tracking_history_max_samples_ > 16384)
        throw std::invalid_argument("direct localization history exceeds hard bound");
      if (!direct_sinks_.canonical || !direct_sinks_.status)
        throw std::invalid_argument("direct localization requires canonical and health sinks");
      const auto failure = [this](std::exception_ptr) {
        std::lock_guard<std::mutex> lock(mutex_);
        direct_fault_ = "localization_direct_observer_failed";
        direct_ros_permitted_ = false;
      };
      state_observer_ = std::make_unique<StateObserver>([this](auto state) {
        const auto now = get_clock()->now().nanoseconds();
        const auto steady = steady_ns();
        const auto correction_ros = std::max(state->map_pose.first_receipt_ros_ns,
                                              state->backend_status.first_receipt_ros_ns);
        const auto correction_steady = std::max(state->map_pose.first_receipt_steady_ns,
                                                 state->backend_status.first_receipt_steady_ns);
        if (direct_revoked_.load() || !direct_ros_permitted_.load() || state->reset_epoch != observer_reset_epoch_.load() ||
            now-correction_ros > int64_t(std::min(limits_.map_pose_timeout_sec, limits_.status_timeout_sec)*1e9) ||
            steady-correction_steady > int64_t(std::min(limits_.map_pose_timeout_sec, limits_.status_timeout_sec)*1e9) ||
            now-state->tracking.first_receipt_ros_ns > int64_t(limits_.tracking_odometry_timeout_sec*1e9) ||
            steady-state->tracking.first_receipt_steady_ns > int64_t(limits_.tracking_odometry_timeout_sec*1e9) ||
            now < state->tracking.first_receipt_ros_ns-int64_t(limits_.future_tolerance_sec*1e9) ||
            steady < state->tracking.first_receipt_steady_ns) return;
        int64_t aligned_available = 0;
        for (const auto& input : state->alignment_inputs)
          aligned_available = std::max(aligned_available, input.first_receipt_steady_ns);
        if (steady < aligned_available || steady-aligned_available >
            int64_t(limits_.odometry_timeout_sec*1e9)) return;
        odometry_pub_->publish(*state->odometry);
        broadcaster_.sendTransform(state->map_to_odom);
        broadcaster_.sendTransform(state->odom_to_base_footprint);
        if(state->base_footprint_to_base_link)broadcaster_.sendTransform(*state->base_footprint_to_base_link);
      }, failure);
      health_observer_ = std::make_unique<HealthObserver>([this](auto health) {
        if (health->reset_epoch != observer_reset_epoch_.load()) return;
        // A delayed healthy heartbeat must never overtake a newer rejection.
        if (health->status->measurement_accepted &&
            (direct_revoked_.load() || !direct_ros_permitted_.load() || steady_ns()-health->evaluated_steady_ns >
             int64_t(limits_.tracking_odometry_timeout_sec*1e9))) return;
        status_pub_->publish(*health->status);
      }, failure);
      marker_observer_ = std::make_unique<MarkerObserver>([this](auto marker) {
        status_marker_pub_->publish(*marker);
      }, failure);
      comparison_observer_ = std::make_unique<ComparisonObserver>([this](auto comparison) {
        comparison_pub_->publish(*comparison);
      }, failure);
    }
  }

  ~LocalizationManager() override {
    // Join observer workers while publishers, clocks and the gate mutex still exist.
    state_observer_.reset(); health_observer_.reset();
    marker_observer_.reset(); comparison_observer_.reset();
  }

  std::shared_ptr<rclcpp::Node> node() override { return shared_from_this(); }

  bool receive_odometry(const std::string& backend,
      nav_msgs::msg::Odometry::ConstSharedPtr input, const InputStamp& stamp) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stamp.body_velocity){
      const auto& b=*stamp.body_velocity;
      if(stamp.first_receipt_steady_ns<=pcd_reset_barrier_steady_ns_ || !b.valid ||
          b.window_begin_ns<=0 || b.window_begin_ns>=b.window_end_ns || b.window_end_ns!=stamp.source_ns ||
          b.effective_source_ns<b.window_begin_ns || b.effective_source_ns>b.window_end_ns ||
          b.sample_count<3 || !b.reset_epoch || !std::isfinite(b.variance_vx) ||
          !std::isfinite(b.variance_vy) || !std::isfinite(b.variance_wz) ||
          b.variance_vx<0. || b.variance_vy<0. || b.variance_wz<0. ||
          (b.measured_waist && (b.odom_z_origin_source_ns<=0 || b.odom_z_origin_source_ns>stamp.source_ns ||
            !b.odom_z_origin_epoch || !std::isfinite(b.odom_z_origin_m))))return false;
    }
    const auto first = admit(backend, Lane::Odometry, input, stamp);
    if (first) handle_odometry(backend, input, seconds(*first), &*first);
    publish_canonical_status(get_clock()->now());
    return first.has_value() && direct_fault_.empty();
  }
  bool receive_map_pose(const std::string& backend,
      geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr input,
      const InputStamp& stamp) override {
    const auto entered=steady_ns();
    std::lock_guard<std::mutex> lock(mutex_);
    if(backend==active_backend_){last_map_entry_ns_=entered;last_map_lock_ns_=steady_ns();last_map_input_=stamp;}
    const auto first = admit(backend, Lane::MapPose, input, stamp);
    if (first) handle_map_pose(backend, input, seconds(*first));
    publish_canonical_status(get_clock()->now());
    return first.has_value() && direct_fault_.empty();
  }
  bool receive_status(const std::string& backend, LocalizationStatus::ConstSharedPtr input,
      const InputStamp& stamp) override {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto first = admit(backend, Lane::Status, input, stamp);
    if (first) handle_status(backend, input, seconds(*first));
    publish_canonical_status(get_clock()->now());
    return first.has_value() && direct_fault_.empty();
  }
  bool receive_tracking(const std::string& backend,
      nav_msgs::msg::Odometry::ConstSharedPtr input, const InputStamp& stamp) override {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto first = admit(backend, Lane::Tracking, input, stamp);
    if (first) record_tracking_odometry(backend, *input, seconds(*first), &*first);
    publish_canonical_status(get_clock()->now());
    return first.has_value() && direct_fault_.empty();
  }
  bool receive_livo_ndt_alignment(nav_msgs::msg::Odometry::ConstSharedPtr input,
      const InputStamp& stamp) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!livo_ndt_alignment_enabled_) return false;
    const auto first = admit("livo", Lane::Alignment, input, stamp);
    if (first) {
      const auto update = livo_ndt_alignment_history_->insert(*input, seconds(*first));
      if (update.accepted) remember(alignment_receipts_, *first);
      publish_active_if_new();
    }
    publish_canonical_status(get_clock()->now());
    return first.has_value() && direct_fault_.empty();
  }
  void configure_pcd_registration_source(
      const geometry_msgs::msg::TransformStamped& source_from_base,
      const std::string& map_frame,const std::string& map_path,
      const std::string& registration_owner) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!direct_ || pcd_source_from_base_ || livo_history_->size() || !pcd_anchors_.empty() ||
        source_from_base.header.frame_id.empty() || source_from_base.child_frame_id!=frames_.base_footprint ||
        map_frame.empty() || map_path.empty() || registration_owner.empty() ||
        !valid_registration_transform(source_from_base.transform))
      throw std::invalid_argument("pcd_registration_requires_original_unstarted_source_contract");
    // Pin this node's actual clock while the configuration entry is mutable.
    // Humble's const Node::get_clock() exposes const Clock, whose now() is not
    // const; queries still read the original owner clock without a new clock.
    pcd_clock_=get_clock();
    pcd_source_from_base_=source_from_base;pcd_map_frame_=map_frame;
    pcd_map_path_=map_path;pcd_registration_owner_=registration_owner;
  }
  bool receive_registration(
      std::shared_ptr<const sunhorizon_3d_localizer::NdtDirectResult> input,
      const InputStamp& delivery) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!pcd_source_from_base_ || !pcd_fault_.empty() || direct_revoked_.load())return false;
    // An in-flight pre-reset registration cannot acquire the new epoch merely
    // because its result callback reaches this lane after reset().
    if(input && input->input.receipt_steady_ns<=pcd_reset_barrier_steady_ns_)return false;
    if(!input || !input->map_to_source ||
        input->map_to_source->header.frame_id!=pcd_map_frame_ ||
        input->map_to_source->child_frame_id!=pcd_source_from_base_->header.frame_id ||
        input->registration_map_path!=pcd_map_path_ ||
        !valid_registration_transform(input->map_to_source->transform) ||
        stamp_ns(input->map_to_source->header.stamp)!=input->input.source_stamp_ns ||
        input->input.source_stamp_ns!=delivery.source_ns ||
        input->input.source_identity.empty() || !input->input.sequence ||
        input->input.receipt_ros_ns<input->input.source_stamp_ns || input->input.receipt_steady_ns<=0 ||
        delivery.identity!=pcd_registration_owner_ || delivery.sequence!=input->input.sequence ||
        delivery.first_receipt_ros_ns!=input->result_receipt_ros_ns ||
        delivery.first_receipt_steady_ns!=input->result_receipt_steady_ns ||
        (!pcd_airy_identity_.empty() && pcd_airy_identity_!=input->input.source_identity)) {
      pcd_anchors_.clear();pcd_fault_="pcd_registration_source_or_map_identity_changed";
      std::atomic_load(&pcd_reset_revoked_)->store(true,std::memory_order_release);return false;
    }
    const auto first=admit(active_backend_,Lane::Registration,input->map_to_source,delivery);
    if(!first)return false;
    if(pcd_airy_identity_.empty())pcd_airy_identity_=input->input.source_identity;
    const auto source=input->input.source_stamp_ns;
    pcd_anchors_.try_emplace(source,PcdAnchor{std::move(input),*first,direct_reset_epoch_,
      correction_gate_->accepted_stamp_ns()==source});
    const auto newest=pcd_anchors_.rbegin()->first;
    while(pcd_anchors_.size()>1 && (pcd_anchors_.size()>static_cast<std::size_t>(tracking_history_max_samples_) ||
        newest-pcd_anchors_.begin()->first>int64_t(tracking_history_duration_sec_*1.e9)))
      pcd_anchors_.erase(pcd_anchors_.begin());
    return true;
  }
  sunhorizon_localization_manager::PcdGeometryLookup lookup_pcd_from_airy(int64_t source,
      const std::optional<sunhorizon_3d_localizer::NdtDirectStamp>& scan) const override {
    auto batch=lookup_pcd_from_airy_times({source},scan);
    return {batch.state,std::move(batch.reason),batch.snapshots.empty()?nullptr:std::move(batch.snapshots.front())};
  }
  sunhorizon_localization_manager::PcdGeometryBatchLookup lookup_pcd_from_airy_times(
      const std::vector<int64_t>& sources,
      const std::optional<sunhorizon_3d_localizer::NdtDirectStamp>& scan) const override {
    return lookup_pcd_batch<false>(sources,scan);
  }
  sunhorizon_localization_manager::PcdGeometryCompactBatchLookup lookup_pcd_from_airy_geometry_times(
      const std::vector<int64_t>& sources,
      const std::optional<sunhorizon_3d_localizer::NdtDirectStamp>& scan) const override {
    return lookup_pcd_batch<true>(sources,scan);
  }
  template<bool Compact>
  std::conditional_t<Compact,sunhorizon_localization_manager::PcdGeometryCompactBatchLookup,
    sunhorizon_localization_manager::PcdGeometryBatchLookup> lookup_pcd_batch(
      const std::vector<int64_t>& sources,
      const std::optional<sunhorizon_3d_localizer::NdtDirectStamp>& scan) const {
    namespace manager=sunhorizon_localization_manager;
    using State=manager::PcdGeometryState;
    using Output=std::conditional_t<Compact,manager::PcdGeometryCompactBatchLookup,manager::PcdGeometryBatchLookup>;
    const auto unavailable=[](State state,const std::string& reason,
        std::shared_ptr<const std::atomic<bool>> token={},int64_t barrier=0) {
      Output output;output.state=state;output.reason=reason;
      output.reset_revoked=std::move(token);output.reset_barrier_steady_ns=barrier;
      return output;
    };
    if(sources.empty() || std::any_of(sources.begin(),sources.end(),[](auto source){return source<=0;}))
      return unavailable(State::Unavailable,"pcd_query_source_invalid");
    const auto first_source=*std::min_element(sources.begin(),sources.end());
    const auto reference_source=scan?scan->source_stamp_ns:first_source;
    PcdAnchor anchor;geometry_msgs::msg::TransformStamped source_from_base;
    std::optional<OdometryHistory> history;ReceiptHistory receipts;
    std::shared_ptr<const std::atomic<bool>> reset_revoked;
    int64_t reset_barrier=0,valid_until_ros=0,valid_until_steady=0,query_ros=0,query_steady=0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(!pcd_source_from_base_)return unavailable(State::NotConfigured,"pcd_registration_not_configured");
      const auto fault=reinitialization_failure();
      if(!fault.empty() || !pcd_fault_.empty())return unavailable(State::Fault,pcd_fault_.empty()?fault:pcd_fault_);
      if(scan && (reference_source<=0 || first_source<reference_source || !scan->sequence ||
          scan->source_identity.empty() || scan->source_identity!=pcd_airy_identity_ ||
          scan->receipt_ros_ns<reference_source || scan->receipt_steady_ns<=pcd_reset_barrier_steady_ns_))
        return unavailable(State::IdentityMismatch,"pcd_query_scan_identity_mismatch");
      if(scan){
        const auto exact=pcd_anchors_.find(reference_source);
        if(exact!=pcd_anchors_.end()){
          const auto& original=exact->second.result->input;
          if(scan->sequence!=original.sequence || scan->source_identity!=original.source_identity ||
             scan->receipt_ros_ns!=original.receipt_ros_ns || scan->receipt_steady_ns!=original.receipt_steady_ns)
            return unavailable(State::IdentityMismatch,"pcd_query_original_scan_metadata_changed");
          if(!exact->second.manager_accepted)
            return unavailable(State::WaitingForManager,"pcd_registration_exact_scan_not_manager_accepted");
        }
      }
      const auto now=pcd_clock_->now();const auto current_steady=steady_ns();
      const auto active=evaluate_active_decision(now.seconds());
      if(!active.valid)return unavailable(State::Unavailable,active.reason);
      if(pcd_anchors_.empty())return unavailable(State::Unavailable,"pcd_registration_anchor_missing");
      // Freeze one latest manager-accepted anchor for the entire batch. The
      // original LIVO history transports it to either earlier or later source
      // times; this never substitutes a latest TF for the query measurement.
      auto selected=pcd_anchors_.end();
      bool pending=false;
      while(selected!=pcd_anchors_.begin()){
        --selected;
        if(selected->second.manager_accepted)break;
        pending=true;
      }
      if(selected==pcd_anchors_.end() || !selected->second.manager_accepted)
        return unavailable(pending?State::WaitingForManager:State::Unavailable,"pcd_registration_anchor_not_manager_accepted");
      anchor=selected->second;const auto& registered=*anchor.result;
      if(anchor.reset_epoch!=direct_reset_epoch_)return unavailable(State::Fault,"pcd_registration_reset_epoch_mismatch");
      const auto current_ros=now.nanoseconds();const auto age_limit=int64_t(limits_.maximum_measurement_age_sec*1.e9);
      const auto receipt_limit=int64_t(limits_.map_pose_timeout_sec*1.e9);
      if(current_ros<reference_source ||
         current_ros<registered.input.source_stamp_ns || current_ros<registered.input.receipt_ros_ns ||
         current_ros<anchor.delivery.first_receipt_ros_ns || current_steady<registered.input.receipt_steady_ns ||
         current_steady<anchor.delivery.first_receipt_steady_ns)
        return unavailable(State::Unavailable,"pcd_registration_clock_regressed");
      // The previous anchor<=query selection implicitly bounded query age.
      // Keep that original source-time limit explicit when transporting from
      // a newer anchor; new localization evidence cannot renew an old scan.
      if(current_ros-reference_source>age_limit)
        return unavailable(State::AnchorExpired,"pcd_query_original_measurement_expired");
      if(current_ros-registered.input.source_stamp_ns>age_limit ||
         current_ros-registered.input.receipt_ros_ns>receipt_limit ||
         current_ros-anchor.delivery.first_receipt_ros_ns>receipt_limit ||
         current_steady-registered.input.receipt_steady_ns>receipt_limit ||
         current_steady-anchor.delivery.first_receipt_steady_ns>receipt_limit)
        return unavailable(State::AnchorExpired,"pcd_registration_original_evidence_expired");
      valid_until_ros=std::min({reference_source+age_limit,registered.input.source_stamp_ns+age_limit,
        registered.input.receipt_ros_ns+receipt_limit,anchor.delivery.first_receipt_ros_ns+receipt_limit});
      valid_until_steady=std::min(registered.input.receipt_steady_ns,anchor.delivery.first_receipt_steady_ns)+receipt_limit;
      query_ros=current_ros;query_steady=current_steady;
      source_from_base=*pcd_source_from_base_;reset_barrier=pcd_reset_barrier_steady_ns_;
      reset_revoked=std::atomic_load(&pcd_reset_revoked_);
      // The only history owner remains livo_history_. This bounded value copy
      // freezes its complete 6D samples and the corresponding original receipts
      // once; no scan group reenters or blocks the manager's input lane.
      if(std::any_of(sources.begin(),sources.end(),[&](auto source){return source!=registered.input.source_stamp_ns;})){
        history.emplace(*livo_history_);receipts=tracking_receipts_[backend_index("livo")];
        // History samples prove the source-time transport, not that its live
        // publisher is still running. Bound that stream by its latest original
        // receipt, independently of the historical interpolation receipts.
        const auto& live=direct_tracking_[backend_index("livo")];
        if(!live)return unavailable(State::Unavailable,"pcd_registration_live_tracking_missing");
        if(current_ros<live->first_receipt_ros_ns || current_steady<live->first_receipt_steady_ns)
          return unavailable(State::Unavailable,"pcd_registration_live_tracking_clock_regressed");
        const auto tracking_limit=int64_t(limits_.tracking_odometry_timeout_sec*1.e9);
        valid_until_ros=std::min(valid_until_ros,live->first_receipt_ros_ns+tracking_limit);
        valid_until_steady=std::min(valid_until_steady,live->first_receipt_steady_ns+tracking_limit);
      }
    }
    // Borrow at most the two exact original receipts from this immutable
    // history copy. Validation does not allocate/copy identity strings for
    // each scan time; only the complete diagnostic API materializes lineage.
    using Lineage=std::array<const InputStamp*,2>;
    const auto lineage_at=[&](int64_t source)->Lineage{
      const auto after=receipts.lower_bound(source);
      if(after==receipts.end())return {};
      if(after->first==source)return {&after->second,nullptr};
      if(after==receipts.begin())return {};
      return {&std::prev(after)->second,&after->second};
    };
    const auto copy_lineage=[](const Lineage& lineage,std::vector<InputStamp>& target){
      target.reserve(lineage.back()?2:(lineage.front()?1:0));
      for(const auto* receipt:lineage)if(receipt)target.push_back(*receipt);
    };
    const auto& registered=*anchor.result;
    sunhorizon_localization_manager::OdometryLookupDecision anchor_history;
    Lineage anchor_lineage{};
    if(history){
      anchor_history=history->lookup(registered.input.source_stamp_ns);
      if(!anchor_history.valid)return unavailable(State::HistoryExpired,anchor_history.reason,reset_revoked,reset_barrier);
      anchor_lineage=lineage_at(registered.input.source_stamp_ns);
      if(!anchor_lineage.front())return unavailable(State::Unavailable,"pcd_registration_anchor_lineage_missing",reset_revoked,reset_barrier);
    }
    auto anchor_source_from_base=source_from_base.transform;
    if(direct_sinks_.source_time_source_from_base){
      const auto dynamic=direct_sinks_.source_time_source_from_base(registered.input.source_stamp_ns,source_from_base.transform);
      if(!dynamic)return unavailable(State::Unavailable,"pcd_registration_anchor_measured_waist_unavailable",reset_revoked,reset_barrier);
      anchor_source_from_base=*dynamic;
    }
    Output output;output.state=State::Ready;output.reason="original_livo_history_registration_batch";
    output.reset_revoked=reset_revoked;output.reset_barrier_steady_ns=reset_barrier;
    if constexpr(Compact){
      auto context=std::make_shared<manager::PcdGeometryBatchContext>();
      context->anchor=anchor.result;context->source_from_nominal_base=source_from_base;
      context->registration_delivery=anchor.delivery;context->query_scan=scan;
      copy_lineage(anchor_lineage,context->anchor_tracking_inputs);
      context->registration_map_path=registered.registration_map_path;context->reset_epoch=anchor.reset_epoch;
      context->reset_revoked=reset_revoked;context->geometry_valid_until_ros_ns=valid_until_ros;
      context->geometry_valid_until_steady_ns=valid_until_steady;
      output.context=std::move(context);output.geometries.reserve(sources.size());
    }else output.snapshots.reserve(sources.size());
    for(const auto source:sources){
      manager::PcdFromAiryGeometry geometry;
      geometry.source_ns=source;geometry.pcd_from_airy=registered.map_to_source->transform;
      Lineage query_lineage{};
      if(registered.input.source_stamp_ns==source){
        geometry.same_scan=bool(scan && scan->source_stamp_ns==source);
      }else{
        const auto query_history=history->lookup(source);
        if(!query_history.valid)return unavailable(query_history.reason=="odometry_history_target_after_newest"?
          State::WaitingForHistory:State::HistoryExpired,query_history.reason,reset_revoked,reset_barrier);
        query_lineage=lineage_at(source);
        if(!query_lineage.front())return unavailable(State::Unavailable,"pcd_registration_query_lineage_missing",reset_revoked,reset_barrier);
        for(const auto* lineage:{&anchor_lineage,&query_lineage})
          for(const auto* receipt:*lineage){
            if(!receipt)continue;
            if(receipt->first_receipt_steady_ns<=reset_barrier)
              return unavailable(State::HistoryExpired,"pcd_registration_tracking_before_reset",reset_revoked,reset_barrier);
            // Historical tracking transport latency is diagnostic only.
            // lookup() enforces source-time bounds/interpolation gaps, and
            // the live stream's original receipt is checked above.
            const auto age=receipt->first_receipt_ros_ns-receipt->source_ns;
            if(age < -int64_t(limits_.future_tolerance_sec*1.e9))
              return unavailable(State::HistoryExpired,"pcd_registration_tracking_from_future_at_receipt",reset_revoked,reset_barrier);
          }
        geometry.anchor_interpolated=anchor_history.interpolated;geometry.query_interpolated=query_history.interpolated;
        auto query_source_from_base=source_from_base.transform;
        if(direct_sinks_.source_time_source_from_base){
          const auto dynamic=direct_sinks_.source_time_source_from_base(source,source_from_base.transform);
          if(!dynamic)return unavailable(State::Unavailable,"pcd_registration_query_measured_waist_unavailable",reset_revoked,reset_barrier);
          query_source_from_base=*dynamic;
        }
        // PCD<-Airy(anchor) * Airy(anchor)<-base(anchor) *
        // base(anchor)<-world * world<-base(query) * base(query)<-Airy(query).
        tf2::Transform pcd_airy,anchor_base,query_base,world_anchor,world_query;
        tf2::fromMsg(registered.map_to_source->transform,pcd_airy);
        tf2::fromMsg(anchor_source_from_base,anchor_base);tf2::fromMsg(query_source_from_base,query_base);
        const auto convert=[](const auto& p){return tf2::Transform(tf2::Quaternion(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w),
          tf2::Vector3(p.position.x,p.position.y,p.position.z));};
        world_anchor=convert(anchor_history.odometry.pose.pose);world_query=convert(query_history.odometry.pose.pose);
        geometry.pcd_from_airy=tf2::toMsg(pcd_airy*anchor_base*world_anchor.inverse()*world_query*query_base.inverse());
      }
      if(query_ros>valid_until_ros || query_steady>valid_until_steady)
        return unavailable(State::HistoryExpired,"pcd_registration_live_tracking_evidence_expired",reset_revoked,reset_barrier);
      if constexpr(Compact)output.geometries.push_back(std::move(geometry));
      else{
        auto snapshot=std::make_shared<manager::PcdFromAirySnapshot>();
        snapshot->anchor=anchor.result;snapshot->registration_delivery=anchor.delivery;
        snapshot->registration_map_path=registered.registration_map_path;snapshot->reset_epoch=anchor.reset_epoch;
        snapshot->reset_revoked=reset_revoked;snapshot->source_from_nominal_base=source_from_base;snapshot->query_scan=scan;
        snapshot->geometry_valid_until_ros_ns=valid_until_ros;
        snapshot->geometry_valid_until_steady_ns=valid_until_steady;
        snapshot->pcd_from_airy=*registered.map_to_source;
        snapshot->pcd_from_airy.header.stamp.sec=static_cast<int32_t>(source/1000000000LL);
        snapshot->pcd_from_airy.header.stamp.nanosec=static_cast<uint32_t>(source%1000000000LL);
        snapshot->pcd_from_airy.transform=std::move(geometry.pcd_from_airy);
        snapshot->same_scan=geometry.same_scan;snapshot->anchor_interpolated=geometry.anchor_interpolated;
        snapshot->query_interpolated=geometry.query_interpolated;
        if(registered.input.source_stamp_ns!=source){
          copy_lineage(anchor_lineage,snapshot->anchor_tracking_inputs);
          copy_lineage(query_lineage,snapshot->query_tracking_inputs);
        }
        output.snapshots.push_back(std::move(snapshot));
      }
    }
    if(reset_revoked->load(std::memory_order_acquire))return unavailable(State::Unavailable,"pcd_registration_epoch_revoked",reset_revoked,reset_barrier);
    return output;
  }
  void reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    reset_localization_state();
    publish_canonical_status(get_clock()->now());
  }
  void revoke(const std::string& reason) override {
    if (!direct_) return;
    // This can run from our own bounded output sink. Never reacquire mutex_.
    // The first fatal owner wins; a failing health sink cannot recurse forever.
    if (direct_revoked_.exchange(true)) return;
    std::atomic_load(&pcd_reset_revoked_)->store(true,std::memory_order_release);
    direct_ros_permitted_.store(false);
    const auto failure = reason.empty() ? std::string("unspecified_source_failure") : reason.substr(0,512);
    {std::lock_guard<std::mutex> lock(fatal_reason_mutex_); direct_fatal_reason_=failure;}
    auto status=std::make_shared<LocalizationStatus>();
    status->header.stamp=get_clock()->now();status->header.frame_id=frames_.map;
    status->backend=active_backend_;status->state=LocalizationStatus::STATE_LOST;
    status->measurement_accepted=false;
    status->reason="explicit_reinitialization_required:localization_runtime_revoked:"+failure;
    auto health=std::make_shared<sunhorizon_localization_manager::CanonicalLocalizationHealth>();
    health->status=std::move(status);health->evaluated_ros_ns=get_clock()->now().nanoseconds();
    health->evaluated_steady_ns=steady_ns();health->reset_epoch=observer_reset_epoch_.load();
    // No geometry is read or invented here. The next serialized timer report
    // retains the last immutable state with its original acquisition clocks.
    try {direct_sinks_.status(health);} catch (...) {}
    if(direct_sinks_.publish_ros_compatibility)health_observer_->enqueue(std::move(health));
  }
  bool failed() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return !reinitialization_failure().empty();
  }

 private:
  using CanonicalState = sunhorizon_localization_manager::CanonicalLocalizationState;
  using CanonicalHealth = sunhorizon_localization_manager::CanonicalLocalizationHealth;
  using StateObserver = sunhorizon_navigation_core::LatestObserver<CanonicalState>;
  using HealthObserver = sunhorizon_navigation_core::LatestObserver<CanonicalHealth>;
  using MarkerObserver = sunhorizon_navigation_core::LatestObserver<StatusMarker>;
  using ComparisonObserver = sunhorizon_navigation_core::LatestObserver<ComparisonStatus>;
  using ReceiptHistory = std::map<int64_t, InputStamp>;
  enum class Lane : size_t { Odometry, MapPose, Status, Tracking, Alignment, Registration };
  static size_t backend_index(const std::string& backend) {
    if (backend == "lidar") return 0;
    if (backend == "visual") return 1;
    if (backend == "livo") return 2;
    throw std::invalid_argument("unknown localization backend");
  }
  static int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp){
    return int64_t(stamp.sec)*1000000000LL+stamp.nanosec;
  }
  bool valid_registration_transform(const geometry_msgs::msg::Transform& value)const{
    for(const auto scalar:{value.translation.x,value.translation.y,value.translation.z,
        value.rotation.x,value.rotation.y,value.rotation.z,value.rotation.w})if(!std::isfinite(scalar))return false;
    const auto& q=value.rotation;
    return std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)<=limits_.quaternion_norm_tolerance;
  }
  static int64_t steady_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  static double seconds(const InputStamp& stamp) {
    return static_cast<double>(stamp.first_receipt_ros_ns) / 1e9;
  }
  void remember(ReceiptHistory& history, const InputStamp& stamp) {
    history.try_emplace(stamp.source_ns, stamp);
    while (history.size() > static_cast<size_t>(tracking_history_max_samples_))
      history.erase(history.begin());
  }
  template<typename Message>
  std::optional<InputStamp> admit(const std::string& backend, Lane lane,
      const std::shared_ptr<const Message>& input, const InputStamp& stamp) {
    if (!direct_ || direct_revoked_.load() || !direct_fault_.empty()) return std::nullopt;
    if (backend != "lidar" && backend != "visual" && backend != "livo") {
      direct_fault_ = "localization_direct_unknown_backend"; return std::nullopt;
    }
    const auto now = get_clock()->now().nanoseconds();
    if (!input || input->header.stamp.nanosec >= 1000000000U ||
        stamp.source_ns <= 0 || stamp.source_ns !=
          int64_t(input->header.stamp.sec)*1000000000LL + input->header.stamp.nanosec ||
        stamp.first_receipt_ros_ns <= 0 || stamp.first_receipt_steady_ns <= 0 ||
        stamp.first_receipt_ros_ns > now + int64_t(limits_.future_tolerance_sec*1e9) ||
        stamp.first_receipt_steady_ns > steady_ns() || stamp.sequence == 0 ||
        stamp.identity.empty() || stamp.identity.size() > 256) {
      direct_fault_ = "localization_direct_invalid_source_receipt"; return std::nullopt;
    }
    auto& previous = direct_last_input_[backend_index(backend)][size_t(lane)];
    if (previous) {
      if (previous->identity != stamp.identity || stamp.sequence < previous->sequence ||
          stamp.first_receipt_steady_ns < previous->first_receipt_steady_ns ||
          stamp.first_receipt_ros_ns < previous->first_receipt_ros_ns) {
        direct_fault_ = "localization_direct_input_identity_or_receipt_regression";
        return std::nullopt;
      }
      if (stamp.sequence == previous->sequence) return std::nullopt;
    }
    previous = stamp;
    auto& receipts = direct_receipts_[backend_index(backend)][size_t(lane)];
    // Delayed/duplicate source epochs keep the first ever receipt; a new
    // delivery sequence cannot renew the original evidence age.
    remember(receipts, stamp);
    const auto found = receipts.find(stamp.source_ns);
    if (found == receipts.end()) return std::nullopt;  // Older than bounded history.
    return found->second;
  }
  void commit_pair_receipts(const std::string& backend, double received_sec) {
    auto& evidence = evidence_for(backend);
    if (direct_) {
      const auto index = backend_index(backend);
      const auto source = pose_status_cache_for(backend).committed_stamp_ns();
      auto& poses = direct_receipts_[index][size_t(Lane::MapPose)];
      auto& statuses = direct_receipts_[index][size_t(Lane::Status)];
      const auto pose = poses.find(source), status = statuses.find(source);
      if (pose == poses.end() || status == statuses.end()) {
        direct_fault_ = "localization_direct_pair_receipt_missing"; return;
      }
      direct_pair_[index] = std::make_pair(pose->second, status->second);
      // Preserve the original gate's commit-time receipt semantics, but use
      // the actual second acquisition receipt, never this downstream callback.
      received_sec = std::max(seconds(pose->second), seconds(status->second));
    }
    if(backend==active_backend_)last_pair_commit_ns_=steady_ns();
    evidence.map_pose_received_sec = received_sec;
    evidence.status_received_sec = received_sec;
  }
  void handle_odometry(const std::string& backend,
      nav_msgs::msg::Odometry::ConstSharedPtr input, double received_sec,
      const InputStamp* stamp) {
    auto& evidence = evidence_for(backend);
    evidence.odometry = *input;
    evidence.odometry_received_sec = received_sec;
    if (backend == "lidar") publish_active_if_new();
    else if (stamp) record_tracking_odometry(backend, *input, received_sec, stamp);
    else record_tracking_odometry(backend, *input, received_sec);
  }
  bool defer_unaligned_correction(const std::string& backend,
      const HealthyPoseStatusPairCache& staged) {
    // A healthy registration is only a candidate until its original epoch can
    // be aligned to the measured body history. Do not install an unusable new
    // pair over a still-qualified correction. This does not recover a failed
    // owner or renew any source/receipt lifetime.
    if (!direct_ || backend != "livo" || active_backend_ != backend ||
        livo_ndt_alignment_enabled_ || !frames_.preserve_6d_map_alignment ||
        !staged.committed_map_pose() || !staged.effective_status()) return false;
    const auto& pose = *staged.committed_map_pose();
    const auto now = get_clock()->now();
    const auto index = backend_index(backend);
    const auto& poses = direct_receipts_[index][size_t(Lane::MapPose)];
    const auto& statuses = direct_receipts_[index][size_t(Lane::Status)];
    const auto acquired_pose = poses.find(staged.committed_stamp_ns());
    const auto acquired_status = statuses.find(staged.committed_stamp_ns());
    if (acquired_pose == poses.end() || acquired_status == statuses.end()) return false;
    if (!sunhorizon_localization_manager::healthy_correction_pair_input_valid(
        backend, pose, *staged.effective_status(), frames_, limits_, now.seconds(),
        seconds(acquired_pose->second), seconds(acquired_status->second)))
      return false;  // Malformed/faulted inputs keep their original fail-closed path.
    const auto steady = steady_ns();
    if (steady < acquired_pose->second.first_receipt_steady_ns ||
        steady < acquired_status->second.first_receipt_steady_ns ||
        steady-acquired_pose->second.first_receipt_steady_ns >
          int64_t(limits_.map_pose_timeout_sec*1e9) ||
        steady-acquired_status->second.first_receipt_steady_ns >
          int64_t(limits_.status_timeout_sec*1e9)) return false;
    const auto age = now.seconds() - rclcpp::Time(pose.header.stamp).seconds();
    if (!std::isfinite(age) || age < -limits_.future_tolerance_sec) return false;
    const auto& previous = pose_status_cache_for(backend);
    // Retention keeps the original source and receipt; transport age itself
    // does not revoke an otherwise live, accepted correction.
    const auto previous_age = now.seconds()-double(previous.committed_stamp_ns())*1e-9;
    if (previous_age < -limits_.future_tolerance_sec) return false;
    if (!correction_gate_->accepted_correction() ||
        correction_gate_->accepted_stamp_ns() != previous.committed_stamp_ns() ||
        !reinitialization_failure().empty() || !evaluate_active_decision(now.seconds()).valid ||
        !sunhorizon_localization_manager::evaluate_tracking_odometry(
          active_history().latest(), tracking_received_sec_for(active_backend_),
          now.seconds(), frames_, limits_).valid) return false;
    const auto aligned = alignment_history_for(backend).lookup(staged.committed_stamp_ns());
    if (aligned.valid) return false;
    // Only missing temporal support is a non-commit. A latched history fault,
    // frame/identity failure or non-finite interpolation is never suppressed.
    if (aligned.reason != "odometry_history_gap_exceeds_tolerance" &&
        aligned.reason != "odometry_history_target_after_newest") return false;
    if (last_uncommitted_correction_source_ns_ != staged.committed_stamp_ns()) {
      last_uncommitted_correction_source_ns_ = staged.committed_stamp_ns();
      RCLCPP_WARN(get_logger(),
        "correction_not_committed source_ns=%lld reason=%s retained_source_ns=%lld; original leases unchanged",
        static_cast<long long>(staged.committed_stamp_ns()), aligned.reason.c_str(),
        static_cast<long long>(previous.committed_stamp_ns()));
    }
    return true;
  }
  void handle_map_pose(const std::string& backend,
      geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr input,
      double received_sec) {
    auto staged = pose_status_cache_for(backend);
    const bool committed = staged.update_map_pose(*input);
    if (committed && defer_unaligned_correction(backend, staged)) return;
    pose_status_cache_for(backend) = std::move(staged);
    if (committed) {
      commit_pair_receipts(backend, received_sec);
    }
    refresh_paired_evidence(backend);
    publish_active_if_new();
  }
  void handle_status(const std::string& backend, LocalizationStatus::ConstSharedPtr input,
      double received_sec) {
    auto staged = pose_status_cache_for(backend);
    const bool committed = staged.update_status(*input);
    if (committed && defer_unaligned_correction(backend, staged)) return;
    pose_status_cache_for(backend) = std::move(staged);
    if (committed) {
      commit_pair_receipts(backend, received_sec);
    }
    refresh_paired_evidence(backend);
    publish_active_if_new();
  }
  std::string direct_receipt_failure() const {
    if (!direct_) return {};
    if (!direct_fault_.empty()) return direct_fault_;
    const auto index = backend_index(active_backend_);
    if (!direct_tracking_[index] || !direct_pair_[index])
      return "localization_direct_source_receipt_missing";
    const auto now = steady_ns();
    const auto& tracking = *direct_tracking_[index];
    const auto& pair = *direct_pair_[index];
    const auto pair_receipt = std::max(pair.first.first_receipt_steady_ns,
                                     pair.second.first_receipt_steady_ns);
    if (now < tracking.first_receipt_steady_ns || now < pair_receipt)
      return "localization_direct_steady_clock_regression";
    if (now-tracking.first_receipt_steady_ns > int64_t(limits_.tracking_odometry_timeout_sec*1e9))
      return "localization_direct_tracking_receipt_stale";
    if (now-pair_receipt > int64_t(std::min(limits_.map_pose_timeout_sec,
                                         limits_.status_timeout_sec)*1e9))
      return "localization_direct_correction_receipt_stale";
    if (active_evidence().map_pose) {
      const auto lineage = alignment_metadata(rclcpp::Time(
          active_evidence().map_pose->header.stamp).nanoseconds());
      if (!lineage.empty()) {
        int64_t available = 0;
        for (const auto& input : lineage)
          available = std::max(available, input.first_receipt_steady_ns);
        if (now < available || now-available > int64_t(limits_.odometry_timeout_sec*1e9)) {
          const auto source=rclcpp::Time(active_evidence().map_pose->header.stamp).nanoseconds();
          if(last_alignment_stale_source_!=source){
            last_alignment_stale_source_=source;
            // Once per failed correction epoch; original clocks only. Never renews evidence.
            RCLCPP_WARN(get_logger(),
              "ALIGNMENT_RECEIPT_CAUSAL source_ns=%lld now_steady_ns=%lld alignment_count=%zu alignment_first_source_ns=%lld alignment_first_receipt_ns=%lld alignment_last_source_ns=%lld alignment_last_receipt_ns=%lld pair_pose_receipt_ns=%lld pair_status_receipt_ns=%lld last_map_source_ns=%lld last_map_receipt_ns=%lld last_map_entry_ns=%lld last_map_lock_ns=%lld pair_commit_ns=%lld",
              (long long)source,(long long)now,lineage.size(),
              (long long)lineage.front().source_ns,(long long)lineage.front().first_receipt_steady_ns,
              (long long)lineage.back().source_ns,(long long)lineage.back().first_receipt_steady_ns,
              (long long)pair.first.first_receipt_steady_ns,(long long)pair.second.first_receipt_steady_ns,
              (long long)last_map_input_.source_ns,(long long)last_map_input_.first_receipt_steady_ns,
              (long long)last_map_entry_ns_,(long long)last_map_lock_ns_,(long long)last_pair_commit_ns_);
          }
          return "localization_direct_alignment_receipt_stale";
        }
      }
    }
    return {};
  }
  std::vector<InputStamp> alignment_metadata(int64_t source) const {
    const auto& receipts = livo_ndt_alignment_enabled_ ? alignment_receipts_ :
        tracking_receipts_[backend_index(active_backend_)];
    const auto after = receipts.lower_bound(source);
    if (after == receipts.end()) return {};
    if (after->first == source) return {after->second};
    if (after == receipts.begin()) return {};
    return {std::prev(after)->second, after->second};
  }
  BackendEvidence& evidence_for(const std::string& backend) {
    if (backend == "lidar") return lidar_;
    if (backend == "visual") return visual_;
    return livo_;
  }

  const BackendEvidence& evidence_for(const std::string& backend) const {
    if (backend == "lidar") return lidar_;
    if (backend == "visual") return visual_;
    return livo_;
  }

  OdometryHistory& history_for(const std::string& backend) {
    if (backend == "lidar") return *lidar_history_;
    if (backend == "visual") return *visual_history_;
    return *livo_history_;
  }

  const OdometryHistory& history_for(const std::string& backend) const {
    if (backend == "lidar") return *lidar_history_;
    if (backend == "visual") return *visual_history_;
    return *livo_history_;
  }

  std::optional<double>& tracking_received_sec_for(const std::string& backend) {
    if (backend == "lidar") return lidar_tracking_received_sec_;
    if (backend == "visual") return visual_tracking_received_sec_;
    return livo_tracking_received_sec_;
  }

  const std::optional<double>& tracking_received_sec_for(
      const std::string& backend) const {
    if (backend == "lidar") return lidar_tracking_received_sec_;
    if (backend == "visual") return visual_tracking_received_sec_;
    return livo_tracking_received_sec_;
  }

  const BackendEvidence& active_evidence() const {
    return evidence_for(active_backend_);
  }

  HealthyPoseStatusPairCache& pose_status_cache_for(
      const std::string& backend) {
    if (backend == "lidar") return lidar_pose_status_cache_;
    if (backend == "visual") return visual_pose_status_cache_;
    return livo_pose_status_cache_;
  }

  void refresh_paired_evidence(const std::string& backend) {
    auto& evidence = evidence_for(backend);
    const auto& cache = pose_status_cache_for(backend);
    evidence.map_pose = cache.committed_map_pose();
    evidence.status = cache.effective_status();
  }

  const OdometryHistory& active_history() const {
    return history_for(active_backend_);
  }

  const OdometryHistory& alignment_history_for(
      const std::string& backend) const {
    if (backend == "livo" && livo_ndt_alignment_enabled_) {
      return *livo_ndt_alignment_history_;
    }
    return history_for(backend);
  }

  void create_backend_subscriptions(
      const std::string& backend,
      const std::string& odometry_topic,
      const std::string& map_pose_topic,
      const std::string& status_topic) {
    const auto candidate_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    // The active LIVO tracking stream is live state. Keeping ten reliable
    // samples here replays stale poses after executor contention. The separate
    // NDT alignment subscription and in-process history retain the epochs
    // needed for correction-time interpolation.
    const auto odometry_qos = rclcpp::QoS(
        rclcpp::KeepLast(backend == "livo" ? 1U : 10U)).reliable();
    odometry_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic, odometry_qos,
        [this, backend](nav_msgs::msg::Odometry::SharedPtr input) {
          DeliveryTiming timing("odometry", backend.c_str(), std::int64_t(input->header.stamp.sec)*1000000000 + input->header.stamp.nanosec, backend == "livo" ? &last_livo_callback_entry_ : nullptr);  // delivery timing
          std::lock_guard<std::mutex> lock(mutex_);
          DeliveryTiming::mark("mutex");  // delivery timing
          handle_odometry(backend, input, get_clock()->now().seconds(), nullptr);
        }));
    map_pose_subs_.push_back(
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            map_pose_topic, candidate_qos,
            [this, backend](
                geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr input) {
              DeliveryTiming timing("map_pose", backend.c_str(), std::int64_t(input->header.stamp.sec)*1000000000 + input->header.stamp.nanosec);  // delivery timing
              std::lock_guard<std::mutex> lock(mutex_);
          DeliveryTiming::mark("mutex");  // delivery timing
              handle_map_pose(backend, input, get_clock()->now().seconds());
            }));
    status_subs_.push_back(create_subscription<LocalizationStatus>(
        status_topic, rclcpp::QoS(10).transient_local(),
        [this, backend](LocalizationStatus::SharedPtr input) {
          DeliveryTiming timing("backend_status", backend.c_str(), std::int64_t(input->header.stamp.sec)*1000000000 + input->header.stamp.nanosec);  // delivery timing
          std::lock_guard<std::mutex> lock(mutex_);
          DeliveryTiming::mark("mutex");  // delivery timing
          handle_status(backend, input, get_clock()->now().seconds());
        }));
  }

  void record_tracking_odometry(
      const std::string& backend,
      const nav_msgs::msg::Odometry& input,
      double received_sec, const InputStamp* stamp = nullptr) {
    const auto update = history_for(backend).insert(input, received_sec);
    DeliveryTiming::mark("history");  // delivery timing
    if (update.accepted) {
      tracking_received_sec_for(backend) = received_sec;
      if (direct_ && stamp) {
        const auto index = backend_index(backend);
        direct_tracking_[index] = *stamp;
        remember(tracking_receipts_[index], *stamp);
      }
    }
    publish_active_if_new();
  }

  AlignedEvidence aligned_evidence_for(const std::string& backend) const {
    AlignedEvidence output;
    output.evidence = evidence_for(backend);
    if (!output.evidence.odometry) {
      output.reason = "odometry_candidate_missing";
      return output;
    }
    if (!output.evidence.map_pose) {
      output.reason = "map_pose_candidate_missing";
      return output;
    }
    const auto target_stamp = rclcpp::Time(
        output.evidence.map_pose->header.stamp).nanoseconds();
    const auto lookup = alignment_history_for(backend).lookup(target_stamp);
    if (!lookup.valid) {
      output.reason = lookup.reason;
      return output;
    }
    if (backend == "livo" && livo_ndt_alignment_enabled_ &&
        (lookup.interpolated ||
        rclcpp::Time(lookup.odometry.header.stamp).nanoseconds() != target_stamp)) {
      output.reason = "livo_ndt_alignment_stamp_not_exact";
      return output;
    }
    output.evidence.odometry = lookup.odometry;
    output.evidence.odometry_received_sec = lookup.received_sec;
    output.valid = true;
    output.reason = "ok";
    return output;
  }

  BackendDecision evaluate_active_decision(
      double now_sec,
      std::string* alignment_reason = nullptr) const {
    const auto direct_failure = direct_receipt_failure();
    if (!direct_failure.empty()) {BackendDecision result;result.reason=direct_failure;return result;}
    const auto reported_failure =
        sunhorizon_localization_manager::reported_backend_failure(
            active_backend_, active_evidence(), frames_);
    if (reported_failure) {
      if (alignment_reason != nullptr) *alignment_reason = *reported_failure;
      BackendDecision output;
      output.reason = *reported_failure;
      return output;
    }
    const auto aligned = aligned_evidence_for(active_backend_);
    if (!aligned.valid) {
      if (alignment_reason != nullptr) *alignment_reason = aligned.reason;
      BackendDecision output;
      output.reason = aligned.reason;
      return output;
    }
    BackendEvidence lidar = lidar_;
    BackendEvidence visual = visual_;
    BackendEvidence livo = livo_;
    if (active_backend_ == "lidar") lidar = aligned.evidence;
    if (active_backend_ == "visual") visual = aligned.evidence;
    if (active_backend_ == "livo") livo = aligned.evidence;
    return sunhorizon_localization_manager::evaluate_active_backend(
        active_backend_, lidar, visual, livo, now_sec, frames_, limits_);
  }

  std::string reinitialization_failure() const {
    if (direct_revoked_.load()) {
      std::lock_guard<std::mutex> lock(fatal_reason_mutex_);
      return "localization_runtime_revoked:"+direct_fatal_reason_;
    }
    if (direct_ && !direct_fault_.empty()) return direct_fault_;
    if (active_history().requires_reinitialization()) {
      return active_history().failure_reason();
    }
    if (livo_ndt_alignment_enabled_ &&
        livo_ndt_alignment_history_->requires_reinitialization()) {
      return livo_ndt_alignment_history_->failure_reason();
    }
    if (correction_gate_->requires_reinitialization()) {
      return correction_gate_->failure_reason();
    }
    return {};
  }

  void publish_active_if_new() {
    if (!reinitialization_failure().empty()) return;
    const auto tracking_odometry = active_history().latest();
    if (!tracking_odometry) return;
    const auto now = get_clock()->now();
    const BackendDecision correction = evaluate_active_decision(now.seconds());
    DeliveryTiming::mark("evaluate_correction");  // delivery timing
    if (!correction.valid) return;
    const TrackingDecision tracking =
        sunhorizon_localization_manager::evaluate_tracking_odometry(
            tracking_odometry, tracking_received_sec_for(active_backend_),
            now.seconds(), frames_, limits_);
    if (!tracking.valid) return;

    const auto correction_stamp_ns = rclcpp::Time(
        active_evidence().map_pose->header.stamp).nanoseconds();
    if (direct_ && alignment_metadata(correction_stamp_ns).empty()) {
      direct_fault_ = "localization_direct_alignment_receipt_missing"; return;
    }
    const auto correction_gate = correction_gate_->consider(
        correction_stamp_ns, correction.map_to_odom);
    if (!correction_gate.valid || !correction_gate_->accepted_correction()) return;
    // Mark only the registration whose same-source correction passed the
    // existing manager gate; no separate correction criterion is introduced.
    const auto registration=pcd_anchors_.find(correction_gate_->accepted_stamp_ns());
    if(registration!=pcd_anchors_.end())registration->second.manager_accepted=true;

    const rclcpp::Time tracking_stamp(tracking_odometry->header.stamp);
    if (!sunhorizon_localization_manager::should_publish_tracking_stamp(
            tracking_stamp.nanoseconds(),
            last_published_tracking_stamp_.nanoseconds())) {
      return;
    }

    // This is the canonical 2D navigation boundary. The raw backend sample is
    // retained in history for alignment, while only its planar projection is
    // published and broadcast as odom -> base_footprint.
    if (!direct_) odometry_pub_->publish(tracking.canonical_odometry);
    DeliveryTiming::mark("odom_publish");  // delivery timing
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = tracking_odometry->header.stamp;
    map_to_odom.header.frame_id = frames_.map;
    map_to_odom.child_frame_id = frames_.odom;
    map_to_odom.transform = *correction_gate_->accepted_correction();
    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.stamp = tracking_odometry->header.stamp;
    odom_to_base.header.frame_id = frames_.odom;
    odom_to_base.child_frame_id = frames_.base_footprint;
    odom_to_base.transform = tracking.odom_to_base_footprint;
    if (!direct_) broadcaster_.sendTransform(map_to_odom);
    DeliveryTiming::mark("tf_map_publish");  // delivery timing
    if (!direct_) broadcaster_.sendTransform(odom_to_base);
    DeliveryTiming::mark("tf_body_publish");  // delivery timing
    last_published_tracking_stamp_ = tracking_stamp;
    if (direct_) {
      auto state = std::make_shared<sunhorizon_localization_manager::CanonicalLocalizationState>();
      state->odometry = std::make_shared<const nav_msgs::msg::Odometry>(tracking.canonical_odometry);
      state->map_to_odom = std::move(map_to_odom);
      state->odom_to_base_footprint = std::move(odom_to_base);
      const auto index = backend_index(active_backend_);
      state->tracking = *direct_tracking_[index];
      if(state->tracking.body_velocity && state->tracking.body_velocity->measured_waist){
        geometry_msgs::msg::TransformStamped residual;
        residual.header=state->odom_to_base_footprint.header;residual.header.frame_id=frames_.base_footprint;
        residual.child_frame_id="base_link";
        tf2::Transform planar,full;
        tf2::fromMsg(state->odom_to_base_footprint.transform,planar);
        const auto& pose=tracking_odometry->pose.pose;
        full.setOrigin(tf2::Vector3(pose.position.x,pose.position.y,pose.position.z));
        full.setRotation(tf2::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
        residual.transform=tf2::toMsg(planar.inverse()*full);
        state->base_footprint_to_base_link=std::move(residual);
      }
      state->map_pose = direct_pair_[index]->first;
      state->backend_status = direct_pair_[index]->second;
      state->alignment_inputs = alignment_metadata(correction_stamp_ns);
      state->sequence = ++direct_sequence_;
      state->reset_epoch = direct_reset_epoch_;
      direct_last_state_ = state;
      if (direct_revoked_.load()) return;
      direct_ros_permitted_ = true;
      try { direct_sinks_.canonical(state); }
      catch (...) { direct_fault_ = "localization_direct_canonical_sink_failed"; direct_ros_permitted_ = false; return; }
      if (direct_sinks_.publish_ros_compatibility) state_observer_->enqueue(std::move(state));
    }
  }

  void publish_canonical_status(const rclcpp::Time& now) {
    bool valid = false;
    std::string reason;
    const auto reinitialization_reason = reinitialization_failure();
    if (!reinitialization_reason.empty()) {
      reason = "explicit_reinitialization_required:" + reinitialization_reason;
    } else {
      std::string alignment_reason;
      const BackendDecision correction = evaluate_active_decision(
          now.seconds(), &alignment_reason);
      const auto tracking_odometry = active_history().latest();
      const TrackingDecision tracking =
          sunhorizon_localization_manager::evaluate_tracking_odometry(
              tracking_odometry, tracking_received_sec_for(active_backend_),
              now.seconds(), frames_, limits_);
      if (!correction.valid) {
        reason = "active_backend_rejected:" +
            (alignment_reason.empty() ? correction.reason : alignment_reason);
      } else if (!tracking.valid) {
        reason = "tracking_rejected:" + tracking.reason;
      } else if (!correction_gate_->accepted_correction()) {
        reason = "correction_rejected:correction_not_accepted";
      } else {
        const auto map_pose_stamp_ns = rclcpp::Time(
            active_evidence().map_pose->header.stamp).nanoseconds();
        if (map_pose_stamp_ns != correction_gate_->accepted_stamp_ns()) {
          reason = "correction_rejected:correction_stamp_not_accepted";
        } else {
          valid = true;
          reason = "ok";
        }
      }
    }

    LocalizationStatus output;
    output.header.stamp = now;
    output.header.frame_id = frames_.map;
    output.backend = active_backend_;
    output.measurement_accepted = valid;
    output.reason = reason;
    const auto& active = active_evidence();
    if (active.status) {
      const auto& candidate = *active.status;
      output.score = std::isfinite(candidate.score) ? candidate.score : 0.0;
      output.iterations = candidate.iterations;
      output.position_covariance = std::isfinite(candidate.position_covariance) ?
          candidate.position_covariance : 0.0;
      output.yaw_covariance = std::isfinite(candidate.yaw_covariance) ?
          candidate.yaw_covariance : 0.0;
    }
    if (active.map_pose) {
      const double age = now.seconds() -
          rclcpp::Time(active.map_pose->header.stamp).seconds();
      output.measurement_age_sec = std::isfinite(age) ? std::max(0.0, age) : 0.0;
    }
    if (valid) {
      output.state = LocalizationStatus::STATE_HEALTHY;
    } else if (!active.odometry && !active.map_pose && !active.status &&
        active_history().size() == 0) {
      output.state = LocalizationStatus::STATE_INITIALIZING;
    } else {
      output.state = LocalizationStatus::STATE_LOST;
    }
    if (direct_) {
      direct_ros_permitted_ = valid;
      auto health = std::make_shared<sunhorizon_localization_manager::CanonicalLocalizationHealth>();
      health->status = std::make_shared<const LocalizationStatus>(output);
      health->last_state = direct_last_state_;
      health->evaluated_ros_ns = now.nanoseconds();
      health->evaluated_steady_ns = steady_ns();
      health->reset_epoch = direct_reset_epoch_;
      try { direct_sinks_.status(health); }
      catch (...) { direct_fault_ = "localization_direct_health_sink_failed"; direct_ros_permitted_ = false; return; }
      if (direct_sinks_.publish_ros_compatibility) health_observer_->enqueue(std::move(health));
    } else status_pub_->publish(output);
    DeliveryTiming::mark("status_publish");  // delivery timing
    publish_status_marker(output);
  }

  void publish_status_marker(const LocalizationStatus& status) {
    StatusMarker marker;
    marker.header = status.header;
    marker.ns = "sunhorizon_localization";
    marker.id = 0;
    marker.type = StatusMarker::TEXT_VIEW_FACING;
    marker.action = StatusMarker::ADD;
    const auto& active = active_evidence();
    if (active.map_pose) {
      marker.pose.position = active.map_pose->pose.pose.position;
    }
    marker.pose.position.z = 1.20;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.24;
    marker.color.a = 1.0F;
    std::string state_name = "UNKNOWN";
    if (status.state == LocalizationStatus::STATE_HEALTHY) {
      state_name = "HEALTHY";
      marker.color.g = 1.0F;
    } else if (status.state == LocalizationStatus::STATE_INITIALIZING) {
      state_name = "INITIALIZING";
      marker.color.r = 1.0F;
      marker.color.g = 0.85F;
    } else if (status.state == LocalizationStatus::STATE_DEGRADED) {
      state_name = "DEGRADED";
      marker.color.r = 1.0F;
      marker.color.g = 0.45F;
    } else if (status.state == LocalizationStatus::STATE_LOST) {
      state_name = "LOST";
      marker.color.r = 1.0F;
    } else {
      marker.color.r = 0.65F;
      marker.color.g = 0.65F;
      marker.color.b = 0.65F;
    }
    marker.text = status.backend + " " + state_name + " | " + status.reason;
    if (direct_) {
      if (direct_sinks_.publish_ros_compatibility)
        marker_observer_->enqueue(std::make_shared<const StatusMarker>(std::move(marker)));
    } else status_marker_pub_->publish(marker);
    DeliveryTiming::mark("marker_publish");  // delivery timing
  }

  void publish_comparison(const rclcpp::Time& now) {
    ComparisonStatus output;
    output.header.stamp = now;
    output.header.frame_id = frames_.map;
    if (!comparison_enabled_) {
      output.reason = "comparison_disabled";
      if (direct_) {
        if (direct_sinks_.publish_ros_compatibility)
          comparison_observer_->enqueue(std::make_shared<const ComparisonStatus>(std::move(output)));
      } else comparison_pub_->publish(output);
    DeliveryTiming::mark("comparison_publish");  // delivery timing
      return;
    }
    BackendEvidence lidar = lidar_;
    BackendEvidence visual = visual_;
    const auto aligned_lidar = aligned_evidence_for("lidar");
    const auto aligned_visual = aligned_evidence_for("visual");
    if (aligned_lidar.valid) lidar = aligned_lidar.evidence;
    if (aligned_visual.valid) visual = aligned_visual.evidence;
    const auto decision = sunhorizon_localization_manager::compare_backends(
        lidar, visual, now.seconds(), frames_, limits_);
    output.available = decision.available;
    output.lidar_healthy = decision.lidar_healthy;
    output.visual_healthy = decision.visual_healthy;
    output.translation_delta_m = decision.translation_delta_m;
    output.yaw_delta_rad = decision.yaw_delta_rad;
    output.within_limits = decision.within_limits;
    output.reason = decision.reason;
    if (direct_) {
      if (direct_sinks_.publish_ros_compatibility)
        comparison_observer_->enqueue(std::make_shared<const ComparisonStatus>(std::move(output)));
    } else comparison_pub_->publish(output);
    DeliveryTiming::mark("comparison_publish");  // delivery timing
  }

  void reset_localization_state() {
    lidar_ = {};
    visual_ = {};
    livo_ = {};
    lidar_pose_status_cache_.reset();
    visual_pose_status_cache_.reset();
    livo_pose_status_cache_.reset();
    last_uncommitted_correction_source_ns_ = 0;
    lidar_history_->reset();
    visual_history_->reset();
    livo_history_->reset();
    livo_ndt_alignment_history_->reset();
    lidar_tracking_received_sec_.reset();
    visual_tracking_received_sec_.reset();
    livo_tracking_received_sec_.reset();
    correction_gate_->reinitialize();
    std::atomic_load(&pcd_reset_revoked_)->store(true,std::memory_order_release);
    pcd_anchors_.clear();pcd_fault_.clear();pcd_reset_barrier_steady_ns_=steady_ns();
    if (direct_) {
      if (direct_sinks_.tracking_reset) direct_sinks_.tracking_reset(pcd_reset_barrier_steady_ns_);
      for (auto& lanes : direct_receipts_) for (auto& receipts : lanes) receipts.clear();
      for (auto& receipts : tracking_receipts_) receipts.clear();
      alignment_receipts_.clear();
      direct_tracking_ = {}; direct_pair_ = {};
      direct_last_state_.reset(); direct_fault_.clear(); ++direct_reset_epoch_;
      auto reset_revoked=std::make_shared<std::atomic<bool>>(false);
      std::atomic_store(&pcd_reset_revoked_,reset_revoked);
      // If revoke raced the reset, neither the old nor new epoch can revive.
      if(direct_revoked_.load())reset_revoked->store(true,std::memory_order_release);
      observer_reset_epoch_ = direct_reset_epoch_; direct_ros_permitted_ = false;
    }
    last_published_tracking_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void on_timer() {
    DeliveryTiming timing("status_timer", "all", 0);  // delivery timing
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = get_clock()->now();
    publish_canonical_status(now);
    publish_comparison(now);
  }

  std::int64_t last_livo_callback_entry_{0};  // delivery timing
  mutable std::mutex mutex_;
  bool direct_{false};
  sunhorizon_localization_manager::LocalizationDirectSinks direct_sinks_;
  std::string direct_fault_;
  std::array<std::array<std::optional<InputStamp>, 6>, 3> direct_last_input_{};
  std::array<std::array<ReceiptHistory, 6>, 3> direct_receipts_{};
  std::array<ReceiptHistory, 3> tracking_receipts_{};
  InputStamp last_map_input_{};
  int64_t last_map_entry_ns_{0},last_map_lock_ns_{0},last_pair_commit_ns_{0};
  mutable int64_t last_alignment_stale_source_{0};
  ReceiptHistory alignment_receipts_;
  std::array<std::optional<InputStamp>, 3> direct_tracking_{};
  std::array<std::optional<std::pair<InputStamp, InputStamp>>, 3> direct_pair_{};
  std::shared_ptr<const CanonicalState> direct_last_state_;
  struct PcdAnchor {
    std::shared_ptr<const sunhorizon_3d_localizer::NdtDirectResult> result;
    InputStamp delivery;
    uint64_t reset_epoch{0};
    bool manager_accepted{false};
  };
  rclcpp::Clock::SharedPtr pcd_clock_;
  std::optional<geometry_msgs::msg::TransformStamped> pcd_source_from_base_;
  std::string pcd_map_frame_,pcd_map_path_,pcd_registration_owner_,pcd_airy_identity_,pcd_fault_;
  std::map<int64_t,PcdAnchor> pcd_anchors_;
  int64_t pcd_reset_barrier_steady_ns_{0};
  std::shared_ptr<std::atomic<bool>> pcd_reset_revoked_{std::make_shared<std::atomic<bool>>(false)};
  uint64_t direct_sequence_{0}, direct_reset_epoch_{0};
  std::atomic<uint64_t> observer_reset_epoch_{0};
  std::atomic<bool> direct_ros_permitted_{false};
  std::atomic<bool> direct_revoked_{false};
  std::string direct_fatal_reason_;
  mutable std::mutex fatal_reason_mutex_;
  std::unique_ptr<StateObserver> state_observer_;
  std::unique_ptr<HealthObserver> health_observer_;
  std::unique_ptr<MarkerObserver> marker_observer_;
  std::unique_ptr<ComparisonObserver> comparison_observer_;
  std::string active_backend_;
  bool livo_ndt_alignment_enabled_{false};
  bool comparison_enabled_{true};
  double status_rate_hz_{20.0};
  double tracking_history_duration_sec_{2.0};
  double map_pose_odometry_max_time_delta_sec_{0.05};
  double correction_filter_time_constant_sec_{0.0};
  std::int64_t tracking_history_max_samples_{4096};
  FrameContract frames_;
  EvidenceLimits limits_;
  BackendEvidence lidar_;
  BackendEvidence visual_;
  BackendEvidence livo_;
  HealthyPoseStatusPairCache lidar_pose_status_cache_;
  HealthyPoseStatusPairCache visual_pose_status_cache_;
  HealthyPoseStatusPairCache livo_pose_status_cache_;
  std::int64_t last_uncommitted_correction_source_ns_{0};
  std::unique_ptr<OdometryHistory> lidar_history_;
  std::unique_ptr<OdometryHistory> visual_history_;
  std::unique_ptr<OdometryHistory> livo_history_;
  std::unique_ptr<OdometryHistory> livo_ndt_alignment_history_;
  std::optional<double> lidar_tracking_received_sec_;
  std::optional<double> visual_tracking_received_sec_;
  std::optional<double> livo_tracking_received_sec_;
  std::unique_ptr<CorrectionGate> correction_gate_;
  rclcpp::Time last_published_tracking_stamp_{0, 0, RCL_ROS_TIME};
  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<StatusMarker>::SharedPtr status_marker_pub_;
  rclcpp::Publisher<ComparisonStatus>::SharedPtr comparison_pub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odometry_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr>
      map_pose_subs_;
  std::vector<rclcpp::Subscription<LocalizationStatus>::SharedPtr> status_subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_tracking_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr livo_ndt_alignment_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

namespace sunhorizon_localization_manager {
std::shared_ptr<LocalizationDirectRuntime> make_localization_direct_runtime(
    const rclcpp::NodeOptions& options, LocalizationDirectSinks sinks) {
  return std::make_shared<LocalizationManager>(options, true, std::move(sinks));
}
std::shared_ptr<rclcpp::Node> make_localization_ros_node(const rclcpp::NodeOptions& options) {
  return std::make_shared<LocalizationManager>(options);
}
}  // namespace sunhorizon_localization_manager
