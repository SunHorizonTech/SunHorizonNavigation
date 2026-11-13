#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <limits>
#include <std_msgs/msg/header.hpp>

#include "sunhorizon_rgbd_adapter/depth_components.hpp"
#include "sunhorizon_camera_runtime/depth_measurement_quality.hpp"
#include "sunhorizon_rgbd_adapter/fused_depth_self_filter.hpp"
#include "sunhorizon_obstacle_coverage/spatial_visibility_fusion.hpp"
#include "sunhorizon_perception_runtime/classified_obstacle_frame.hpp"
#include "sunhorizon_perception_runtime/ground_reference_owner.hpp"
#include "sunhorizon_perception_runtime/ground_input_health.hpp"
#include "sunhorizon_perception_runtime/registration_ground_geometry.hpp"

namespace sunhorizon_perception_runtime {
namespace sv = sunhorizon_obstacle_coverage::spatial_visibility;
namespace dg = sunhorizon_obstacle_coverage::dynamic_self_geometry;

struct FrameIdentity {
  std::string source_id, epoch, calibration_id;
  uint64_t sequence{0};
  int64_t source_stamp_ros_ns{0}, receipt_ros_ns{0}, receipt_steady_ns{0};
  int64_t expiry_budget_ns{sv::kMaximumEvidenceAgeNs};
  bool required_for_evaluation{false};
};

struct EngineDuration { double cpu_ms{0.}, wall_ms{0.}; };
// Immutable source-preparation token; mask_prepared=false preserves body-only
// filtering independence for sources whose raw terminal never uses that mask.
struct PreparedQualityMask {
  std::shared_ptr<const sensor_msgs::msg::Image> image;
  std::shared_ptr<const sensor_msgs::msg::CameraInfo> info;
  std::optional<std_msgs::msg::Header> raw_projection_header;
  sunhorizon_camera_runtime::DepthMeasurementQuality::ConstSharedPtr measurement_quality;
  FrameIdentity identity;
  bool mask_prepared{false}, raw_enabled{false}, raw_use_mask{false};
  uint32_t body_stride{0}, raw_stride{0};
  int64_t raw_pair_delta_ns{0}, raw_expiry_ns{0};
  double component_maximum{0.};
  EngineDuration preparation;
  double scale{0.}, minimum{0.}, maximum{0.}, neighbor_delta{0.}, near_maximum{0.};
  std::size_t minimum_pixels{0}, near_minimum_pixels{0};
  sunhorizon_rgbd_adapter::DepthComponentMask mask;
};
struct BodyModelDiagnostic {
  FrameIdentity identity;
  std::string source_frame;
  double self_tolerance_m{0.};
  dg::QueryBudget query_budget;
  std::vector<dg::PreparedModel::DiagnosticLink> links;
};
struct FrameInput {
  bool capture_body_model_diagnostic{false};
  bool capture_body_replay_diagnostic{false};
  struct GroundMeasurementBounds {
    std::string provenance_id;
    double point_m{std::numeric_limits<double>::quiet_NaN()};
    double transform_translation_m{std::numeric_limits<double>::quiet_NaN()};
    double transform_angle_rad{std::numeric_limits<double>::quiet_NaN()};
  };
  std::shared_ptr<const sensor_msgs::msg::Image> image;
  std::shared_ptr<const sensor_msgs::msg::CameraInfo> info;
  FrameIdentity identity;
  // Original RGB-D chest normalization uses the paired color header. Its
  // bounded pair gate is still required; SDK depth source is retained below.
  std::optional<std_msgs::msg::Header> raw_projection_header;
  sunhorizon_camera_runtime::DepthMeasurementQuality::ConstSharedPtr measurement_quality;
  uint64_t sdk_depth_device_us{0},sdk_depth_global_us{0},sdk_depth_frame_index{0};
  double sdk_depth_scale_m{0.};
  std::shared_ptr<const PreparedQualityMask> prepared_quality;
  // Qualified source(t_source) -> common(t_query), including motion
  // compensation when chest and belly have different source times.
  sv::RigidTransform optical_to_common;
  sunhorizon_obstacle_coverage::RegionCalibrationContract calibration;
  // Independent source-time reference only; a previous camera frame or a
  // fitted plane cannot self-authorize ground clearing.
  std::optional<ground_relative::GroundReference> ground_reference;
  std::optional<ground_relative::ClassifierConfig> ground_classifier_config;
  std::optional<GroundMeasurementBounds> ground_measurement_bounds;
  // Independent fixed-reference transform; coverage still uses its original
  // common(t_query) transform above. A failed ground lookup stays unqualified.
  std::optional<ground_relative::SourceTimeTransform> ground_source_transform;
  ground_relative::GroundReferenceOwner::ConstSharedPtr ground_owner;
  ground_relative::RegistrationMapSourceBinding ground_map_binding;
  ground_relative::SourceTransformDiagnostic ground_transform_diagnostic;
  // Called once after complete BODY projection, on this same owner. It must
  // query original source identity; it must never renew a prior geometry lease.
  std::function<std::optional<RegistrationGroundGeometryBatch>()> acquire_ground_geometry;
};

struct EngineConfig {
  // Body may retain its independent accepted projection contract while an
  // optional camera lacks spatial qualification. Such frames decode Invalid
  // coverage pixels only and can never contribute spatial evidence.
  bool coverage_enabled{true};
  bool classified_obstacles_enabled{false};
  double depth_scale_m{.001}, minimum_depth_m{.1}, maximum_depth_m{8.};
  uint32_t body_stride{1};
  bool raw_projection_enabled{false}, raw_projection_use_component_mask{false};
  bool body_stereo_occlusion_enabled{false};
  bool map_body_proximity_filter_enabled{false};  // Legacy map-only profile.
  uint32_t raw_projection_stride{2};
  bool body_proximity_filter_enabled{false};
  int64_t raw_projection_max_pair_delta_ns{5000000};
  int64_t raw_projection_expiry_budget_ns{300000000};
  double self_tolerance_m{dg::kSelfSurfaceToleranceM}, clear_margin_m{.02};
  // Zero disables connected-component filtering. A finite cutoff preserves
  // distant thin returns exactly as the existing body projection does.
  std::size_t component_minimum_pixels{0}, near_component_minimum_pixels{0};
  double component_neighbor_delta_m{.05}, component_maximum_depth_m{8.};
  double near_component_maximum_depth_m{1.5};
  sunhorizon_rgbd_adapter::DynamicFilterBudget body_budget;
  std::size_t coverage_max_node_visits{sv::kMaximumChecks};
  std::size_t coverage_max_triangle_tests{sv::kMaximumChecks};
};

// Diagnostic-only immutable ownership. Image/Info are CameraFrame's owning const
// ROS messages (never borrowed SDK buffers); keep/IDs/support copy mutable scratch.
struct BodyReplayDiagnostic {
  double stereo_baseline_m{0.};
  sunhorizon_camera_runtime::DepthMeasurementQuality::ConstSharedPtr measurement_quality;
  EngineConfig config;
  std::shared_ptr<const BodyModelDiagnostic> model;
  sensor_msgs::msg::Image::ConstSharedPtr image;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
  int64_t query_ns{0};
  sunhorizon_obstacle_coverage::RegionCalibrationContract calibration;
  bool cuda_backend{false}, parallel_cpu{false}, keep_present{false};
  std::vector<uint8_t> keep;
  std::vector<uint64_t> retained_sample_ids;
  std::vector<sunhorizon_rgbd_adapter::FusedDepthSelfFilterResult::SelfSupportSample> self_support;
};

struct EngineTimings {
  EngineDuration total, source_prepare, model_prepare, quality_decode, body_filter, coverage_classify, raw_projection, obstacle_classify, near_ground_clearance;
  double body_geometry_helper_cpu_ms{0.};
};
struct EngineStats {
  double stereo_body_baseline_m{0.};
  std::string stereo_body_geometry_reason{"disabled"};
  std::size_t model_preparations{0}, image_pixels{0}, body_projected_pixels{0}, quality_mask_reuses{0};
  std::size_t component_removed_pixels{0}, component_removed_components{0};
  std::size_t coverage_required_pixels{0}, quality_rejected_pixels{0};
  std::size_t coverage_queries{0}, coverage_node_visits{0}, coverage_triangle_tests{0};
  std::size_t measurement_unreliable_pixels{0},measurement_unassessed_pixels{0};
};

// Owns all raster backing storage. No cached sv::Source points into a vector,
// so copying/moving this object cannot leave an internal dangling pointer.
class DecodedSource {
 public:
  // Borrowed view: keep this DecodedSource alive and immutable during use.
  sv::Source source() const;
  const FrameIdentity& identity() const noexcept { return identity_; }
  const std::vector<float>& depth() const noexcept { return depth_; }
  const std::vector<sv::ReturnKind>& kinds() const noexcept { return kinds_; }
 private:
  friend class PerceptionEngine;
  FrameIdentity identity_;
  std::string frame_id_;
  sv::DepthRaster raster_;  // Pointer fields stay null; source() binds a view.
  std::vector<float> depth_;
  std::vector<sv::ReturnKind> kinds_;
};

struct FrameResult {
  bool ground_query_cancelled{false};
  bool apply_near_body_mask_to_collision{false};
  // Original classified cloud indices, in ascending order.
  std::vector<std::size_t> near_body_indices;
  GroundInputHealth obstacle_input_health;
  std::shared_ptr<const BodyModelDiagnostic> body_model_diagnostic;
  std::shared_ptr<const BodyReplayDiagnostic> body_replay_diagnostic;
  sensor_msgs::msg::PointCloud2 body_cloud;
  ground_relative::ClassifiedObstacleFrame::ConstSharedPtr classified_obstacles;
  near_ground_clearance::Snapshot::ConstSharedPtr near_ground_clearance;
  // Independent original RGB-D terminal source, never the post-self body.
  // Chest source is its original paired color header; raw_source retains depth.
  sensor_msgs::msg::PointCloud2::ConstSharedPtr raw_projection_cloud;
  bool raw_projection_complete{false};
  int64_t raw_projection_source_ns{0}, raw_projection_raw_source_ns{0};
  int64_t raw_projection_expiry_budget_ns{0};
  std::shared_ptr<const PreparedQualityMask> prepared_quality;
  sunhorizon_rgbd_adapter::DynamicFilterResult body_filter;
  DecodedSource decoded;
  EngineStats stats;
  EngineTimings timings;
};
struct EvaluationResult { sv::Result visibility; EngineDuration timing; };

// Synchronous computation only: no subscriptions, TF lookups, clocks used for
// evidence freshness, or publication. One owner/thread per engine (scratch is
// reused). Input/model storage must stay immutable until process() returns.
// Acquisition, ordering/epoch/clock high-water checks and upstream health are
// separate owner responsibilities; a fresh optional source never renews the
// required source's age. Invalid contracts/model/budgets throw and emit no frame.
class PerceptionEngine {
 public:
  explicit PerceptionEngine(EngineConfig config);
  // Runtime opt-in only; one sole body owner and immutable frame per engine.
  void enable_parallel_body_geometry(std::function<void()> helper_owner_check);
  void enable_cuda_body_geometry();
  void request_stop_body_geometry() noexcept;
  // Mandatory original metadata/layout/pair validation, plus only the shared
  // raw/body component mask. No XYZ, FK, query or mutable scratch escapes.
  std::shared_ptr<const PreparedQualityMask> prepare(const FrameInput&);
  // Independent terminal projection, with strict immutable preparation reuse.
  // Legacy callers without preparation retain an internal fallback.
  FrameResult project_raw(const FrameInput&);
  FrameResult process(const FrameInput&, const std::vector<dg::LinkObservation>&,
                      const sv::Query&);
  // The caller attests that this PreparedModel contains the complete twelve
  // qualified source-time link observations. No model is prepared again.
  FrameResult process(const FrameInput&, const dg::PreparedModel&, const sv::Query&);
  static EvaluationResult evaluate(const sv::Query&,
      const std::vector<const DecodedSource*>&, int64_t now_ros_ns, int64_t now_steady_ns);
  const EngineConfig& config() const noexcept { return config_; }
 private:
  std::shared_ptr<PreparedQualityMask> make_preparation(const FrameInput&);
  EngineConfig config_;
  std::unique_ptr<sunhorizon_rgbd_adapter::BodyGeometryWorker> body_geometry_worker_;
  std::unique_ptr<sunhorizon_rgbd_adapter::BodyGeometryCuda> body_geometry_cuda_;
  sunhorizon_rgbd_adapter::DepthComponentWorkspace component_workspace_;
};
}  // namespace sunhorizon_perception_runtime
