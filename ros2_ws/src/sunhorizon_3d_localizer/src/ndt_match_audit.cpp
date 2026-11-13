#include "sunhorizon_3d_localizer/ndt_match_audit.hpp"

#include <iomanip>
#include <sstream>

namespace sunhorizon_3d_localizer {

std::string format_ndt_match_audit(const NdtMatchAudit& audit) {
  std::ostringstream output;
  output << std::boolalpha << std::fixed << std::setprecision(6)
         << "ndt_match_audit"
         << " backend=" << audit.registration_backend
         << " threads=" << audit.registration_threads
         << " neighborhood=" << audit.registration_neighborhood
         << " compute_backend=" << audit.compute_backend
         << " sensor_stamp_ns=" << audit.sensor_stamp_ns
         << " source_raw_points=" << audit.source_raw_points
         << " source_range_voxel_points=" << audit.source_range_voxel_points
         << " source_bounds_filter_enabled="
         << audit.source_bounds_filter_enabled
         << " source_bounds_retained_points="
         << audit.source_bounds_retained_points
         << " source_bounds_rejected_points="
         << audit.source_bounds_rejected_points
         << " source_bounds_retained_fraction="
         << audit.source_bounds_retained_fraction
         << " source_filtered_points=" << audit.source_filtered_points
         << " initial_x_m=" << audit.initial.x
         << " initial_y_m=" << audit.initial.y
         << " initial_z_m=" << audit.initial.z
         << " initial_yaw_rad=" << audit.initial.yaw_rad
         << " result_x_m=" << audit.result.x
         << " result_y_m=" << audit.result.y
         << " result_z_m=" << audit.result.z
         << " result_yaw_rad=" << audit.result.yaw_rad
         << " raw_map_to_base_z_m=" << audit.raw_map_to_base_z_m
         << " raw_map_to_base_roll_rad=" << audit.raw_map_to_base_roll_rad
         << " raw_map_to_base_pitch_rad=" << audit.raw_map_to_base_pitch_rad
         << " initial_to_result_translation_m="
         << audit.initial_to_result_translation_m
         << " initial_to_result_yaw_rad=" << audit.initial_to_result_yaw_rad
         << " correction_step_applicable=" << audit.correction_step_applicable
         << " correction_step_reason=" << audit.correction_step_reason
         << " correction_step_has_baseline=" << audit.correction_step_has_baseline
         << " correction_step_establishes_baseline="
         << audit.correction_step_establishes_baseline
         << " correction_step_gap_recovered="
         << audit.correction_step_gap_recovered
         << " correction_step_dt_sec=" << audit.correction_step_dt_sec
         << " correction_translation_step_m="
         << audit.correction_translation_step_m
         << " correction_yaw_step_rad=" << audit.correction_yaw_step_rad
         << " maximum_correction_step_gap_sec="
         << audit.maximum_correction_step_gap_sec
         << " maximum_correction_translation_step_m="
         << audit.maximum_correction_translation_step_m
         << " maximum_correction_yaw_step_rad="
         << audit.maximum_correction_yaw_step_rad
         << " alignment_source_fitness_m2="
         << audit.alignment_source_fitness_m2
         << " fitness_gate_scope=alignment_source_points_mean_squared_nearest_neighbor"
         << " pre_bounds_all_point_fitness_m2="
         << audit.pre_bounds_all_point_fitness_m2
         << " pre_bounds_fitness_scope=range_voxel_source_against_full_target"
         << " extended_quality_diagnostics_enabled="
         << audit.extended_quality_diagnostics_enabled
         << " inlier_max_distance_m=" << audit.inlier_max_distance_m
         << " diagnostic_target_points=" << audit.diagnostic_target_points
         << " diagnostic_source_points=" << audit.diagnostic_source_points
         << " inlier_points=" << audit.inlier_points
         << " source_inlier_fraction=" << audit.source_inlier_fraction
         << " source_to_target_inlier_rmse_m="
         << audit.source_to_target_inlier_rmse_m
         << " pre_bounds_full_target_points="
         << audit.pre_bounds_full_target_points
         << " pre_bounds_source_points=" << audit.pre_bounds_source_points
         << " pre_bounds_inlier_points=" << audit.pre_bounds_inlier_points
         << " pre_bounds_source_inlier_fraction="
         << audit.pre_bounds_source_inlier_fraction
         << " pre_bounds_source_to_full_target_inlier_rmse_m="
         << audit.pre_bounds_source_to_full_target_inlier_rmse_m
         << " iterations=" << audit.iterations
         << " alignment_runtime_ms=" << audit.alignment_runtime_ms
         << " alignment_thread_cpu_ms=" << audit.alignment_thread_cpu_ms
         << " fitness_thread_cpu_ms=" << audit.fitness_thread_cpu_ms
         << " post_alignment_runtime_ms=" << audit.post_alignment_runtime_ms
         << " fitness_runtime_ms=" << audit.fitness_runtime_ms
         << " alignment_quality_runtime_ms="
         << audit.alignment_quality_runtime_ms
         << " pre_bounds_quality_runtime_ms="
         << audit.pre_bounds_quality_runtime_ms
         << " end_to_end_runtime_ms=" << audit.end_to_end_runtime_ms
         << " owner_tid=" << audit.owner_tid
         << " schedstat_available=" << audit.schedstat_available
         << " schedstat_status=" << audit.schedstat_status
         << " schedstat_begin_steady_ns=" << audit.schedstat_begin_steady_ns
         << " schedstat_end_steady_ns=" << audit.schedstat_end_steady_ns
         << " schedstat_sampling_wall_ns=" << audit.schedstat_sampling_wall_ns
         << " schedstat_running_ns=" << audit.schedstat_running_ns
         << " schedstat_runqueue_wait_ns=" << audit.schedstat_runqueue_wait_ns
         << " converged=" << audit.converged
         << " accepted=" << audit.accepted
         << " gate_reason=" << audit.gate_reason;
  return output.str();
}

}  // namespace sunhorizon_3d_localizer
