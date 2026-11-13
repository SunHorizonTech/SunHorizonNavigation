#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace sunhorizon_3d_localizer {

struct NdtAuditPose {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw_rad{0.0};
};

// Observation only, bound to the submitted cloud's original source/sequence.
// -1 means that this stage was not reached or direct input was not used.
struct NdtQueueTiming {
  std::int64_t submit_steady_ns{-1}, admitted_steady_ns{-1};
  std::int64_t dequeue_steady_ns{-1};
  std::int64_t core_lock_enter_steady_ns{-1}, core_lock_acquired_steady_ns{-1};
};

struct NdtStageSchedulingAudit {
  bool available{false};
  int status{0};
  std::int64_t owner_tid{-1}, begin_steady_ns{-1}, end_steady_ns{-1};
  std::int64_t sampling_wall_ns{-1}, running_ns{-1}, runqueue_wait_ns{-1};
};

struct NdtMatchAudit {
  std::string registration_backend{"pcl"};
  std::string compute_backend{"cpu"};
  std::uint32_t registration_threads{1};
  std::string registration_neighborhood{"KDTREE"};
  std::int64_t sensor_stamp_ns{0};
  std::size_t source_raw_points{0};
  std::size_t source_range_voxel_points{0};
  bool source_bounds_filter_enabled{false};
  std::size_t source_bounds_retained_points{0};
  std::size_t source_bounds_rejected_points{0};
  double source_bounds_retained_fraction{
      std::numeric_limits<double>::quiet_NaN()};
  // Final source cardinality passed to NDT. Kept as a compatibility field;
  // source_range_voxel_points is the pre-bounds population.
  std::size_t source_filtered_points{0};
  NdtAuditPose initial;
  NdtAuditPose result;
  double raw_map_to_base_z_m{std::numeric_limits<double>::quiet_NaN()};
  double raw_map_to_base_roll_rad{std::numeric_limits<double>::quiet_NaN()};
  double raw_map_to_base_pitch_rad{std::numeric_limits<double>::quiet_NaN()};
  double raw_map_to_base_x_m{std::numeric_limits<double>::quiet_NaN()};
  double raw_map_to_base_y_m{std::numeric_limits<double>::quiet_NaN()};
  double raw_map_to_base_yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  double initial_to_result_translation_m{0.0};
  double initial_to_result_yaw_rad{0.0};
  bool correction_step_applicable{false};
  std::string correction_step_reason{"not_evaluated"};
  bool correction_step_has_baseline{false};
  bool correction_step_establishes_baseline{false};
  bool correction_step_gap_recovered{false};
  double correction_step_dt_sec{std::numeric_limits<double>::quiet_NaN()};
  double correction_translation_step_m{
      std::numeric_limits<double>::quiet_NaN()};
  double correction_yaw_step_rad{std::numeric_limits<double>::quiet_NaN()};
  double maximum_correction_step_gap_sec{
      std::numeric_limits<double>::quiet_NaN()};
  double maximum_correction_translation_step_m{
      std::numeric_limits<double>::quiet_NaN()};
  double maximum_correction_yaw_step_rad{
      std::numeric_limits<double>::quiet_NaN()};
  // PCL getFitnessScore() over the source actually supplied to alignment.
  double alignment_source_fitness_m2{
      std::numeric_limits<double>::quiet_NaN()};
  // Independent nearest-neighbor MSE after transforming the complete
  // range+voxel source by the NDT result and comparing it with the full map.
  double pre_bounds_all_point_fitness_m2{
      std::numeric_limits<double>::quiet_NaN()};
  bool extended_quality_diagnostics_enabled{true};
  double inlier_max_distance_m{std::numeric_limits<double>::quiet_NaN()};
  std::size_t diagnostic_target_points{0};
  std::size_t diagnostic_source_points{0};
  std::size_t inlier_points{0};
  double source_inlier_fraction{std::numeric_limits<double>::quiet_NaN()};
  double source_to_target_inlier_rmse_m{
      std::numeric_limits<double>::quiet_NaN()};
  std::size_t pre_bounds_full_target_points{0};
  std::size_t pre_bounds_source_points{0};
  std::size_t pre_bounds_inlier_points{0};
  double pre_bounds_source_inlier_fraction{
      std::numeric_limits<double>::quiet_NaN()};
  double pre_bounds_source_to_full_target_inlier_rmse_m{
      std::numeric_limits<double>::quiet_NaN()};
  std::uint32_t iterations{0};
  double alignment_runtime_ms{0.0};
  // Calling-thread CPU only: alignment excludes OpenMP worker CPU.
  // NaN means unavailable. Neither diagnostic participates in acceptance.
  double alignment_thread_cpu_ms{std::numeric_limits<double>::quiet_NaN()};
  double fitness_thread_cpu_ms{std::numeric_limits<double>::quiet_NaN()};
  // Synchronous work after align() returns. The three detailed values below
  // identify the expensive nearest-neighbor passes without changing any gate.
  double post_alignment_runtime_ms{0.0};
  double fitness_runtime_ms{0.0};
  double alignment_quality_runtime_ms{0.0};
  double pre_bounds_quality_runtime_ms{0.0};
  // End-to-end synchronous match work: alignment, result extraction, and the
  // source-to-target diagnostic KD-tree queries. This is the runtime gate value.
  double end_to_end_runtime_ms{0.0};
  // Same calling thread, sampled just outside the original end-to-end window.
  // Includes both schedstat reads; does not include OpenMP worker CPU.
  // Status codes are detail::NdtSchedstatStatus; unavailable deltas stay -1.
  std::int64_t owner_tid{-1};
  bool schedstat_available{false};
  int schedstat_status{0};
  std::int64_t schedstat_begin_steady_ns{-1}, schedstat_end_steady_ns{-1};
  std::int64_t schedstat_sampling_wall_ns{-1};
  std::int64_t schedstat_running_ns{-1}, schedstat_runqueue_wait_ns{-1};
  NdtQueueTiming queue_timing;
  std::int64_t process_enter_steady_ns{-1}, preprocess_done_steady_ns{-1};
  std::int64_t gates_done_steady_ns{-1};
  // Adjacent windows share the existing align/fitness schedstat samples.
  // Calling-thread CPU only, including sampling overhead; never a gate input.
  NdtStageSchedulingAudit preprocess_scheduling, gates_scheduling;
  bool converged{false};
  bool accepted{false};
  std::string gate_reason;
};

// Produces one stable, grep-friendly line for a completed NDT invocation.
// Formatting is deliberately independent from ROS so it can be tested exactly.
std::string format_ndt_match_audit(const NdtMatchAudit& audit);

}  // namespace sunhorizon_3d_localizer
