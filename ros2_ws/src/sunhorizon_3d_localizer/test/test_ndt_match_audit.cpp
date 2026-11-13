#include <gtest/gtest.h>

#include <limits>
#include <string>

#include "sunhorizon_3d_localizer/ndt_match_audit.hpp"

namespace sunhorizon_3d_localizer {
namespace {

TEST(NdtMatchAudit, FormatsAcceptedMatchAsOneStableLine) {
  NdtMatchAudit audit;
  audit.registration_backend = "ndt_omp";
  audit.compute_backend = "cuda_direct7";
  audit.registration_threads = 4;
  audit.registration_neighborhood = "DIRECT7";
  audit.sensor_stamp_ns = 1234567890123;
  audit.source_raw_points = 18432;
  audit.source_range_voxel_points = 2718;
  audit.source_bounds_filter_enabled = true;
  audit.source_bounds_retained_points = 2038;
  audit.source_bounds_rejected_points = 680;
  audit.source_bounds_retained_fraction = 0.749816;
  audit.source_filtered_points = 2038;
  audit.initial = {0.430001, -0.260002, 0.003, 0.0213};
  audit.result = {0.612345, -0.310123, 0.004, -0.0125};
  audit.raw_map_to_base_z_m = -0.281234;
  audit.raw_map_to_base_roll_rad = 0.003456;
  audit.raw_map_to_base_pitch_rad = -0.007890;
  audit.initial_to_result_translation_m = 0.189876;
  audit.initial_to_result_yaw_rad = 0.0338;
  audit.correction_step_applicable = true;
  audit.correction_step_reason = "ok_after_result_gap_with_bounded_correction";
  audit.correction_step_has_baseline = true;
  audit.correction_step_establishes_baseline = false;
  audit.correction_step_gap_recovered = true;
  audit.correction_step_dt_sec = 0.599914;
  audit.correction_translation_step_m = 0.048765;
  audit.correction_yaw_step_rad = 0.012345;
  audit.maximum_correction_step_gap_sec = 0.5;
  audit.maximum_correction_translation_step_m = 0.05;
  audit.maximum_correction_yaw_step_rad = 0.03490658503988659;
  audit.alignment_source_fitness_m2 = 0.714285;
  audit.pre_bounds_all_point_fitness_m2 = 4.714285;
  audit.extended_quality_diagnostics_enabled = true;
  audit.inlier_max_distance_m = 1.0;
  audit.diagnostic_target_points = 12500;
  audit.diagnostic_source_points = 2038;
  audit.inlier_points = 1528;
  audit.source_inlier_fraction = 0.749755;
  audit.source_to_target_inlier_rmse_m = 0.182345;
  audit.pre_bounds_full_target_points = 18000;
  audit.pre_bounds_source_points = 2718;
  audit.pre_bounds_inlier_points = 2038;
  audit.pre_bounds_source_inlier_fraction = 0.749816;
  audit.pre_bounds_source_to_full_target_inlier_rmse_m = 0.182345;
  audit.iterations = 17;
  audit.alignment_runtime_ms = 170.123456;
  audit.alignment_thread_cpu_ms = 21.123456;
  audit.fitness_thread_cpu_ms = 8.123456;
  audit.post_alignment_runtime_ms = 28.641976;
  audit.fitness_runtime_ms = 9.123456;
  audit.alignment_quality_runtime_ms = 8.234567;
  audit.pre_bounds_quality_runtime_ms = 10.345678;
  audit.end_to_end_runtime_ms = 198.765432;
  audit.owner_tid = 123;
  audit.schedstat_available = true;
  audit.schedstat_status = 1;
  audit.schedstat_begin_steady_ns = 1000;
  audit.schedstat_end_steady_ns = 2000;
  audit.schedstat_sampling_wall_ns = 20;
  audit.schedstat_running_ns = 200;
  audit.schedstat_runqueue_wait_ns = 700;
  audit.converged = true;
  audit.accepted = true;
  audit.gate_reason = "ok";

  EXPECT_EQ(
      format_ndt_match_audit(audit),
      "ndt_match_audit backend=ndt_omp threads=4 neighborhood=DIRECT7 compute_backend=cuda_direct7"
      " sensor_stamp_ns=1234567890123"
      " source_raw_points=18432 source_range_voxel_points=2718"
      " source_bounds_filter_enabled=true source_bounds_retained_points=2038"
      " source_bounds_rejected_points=680"
      " source_bounds_retained_fraction=0.749816 source_filtered_points=2038"
      " initial_x_m=0.430001 initial_y_m=-0.260002 initial_z_m=0.003000"
      " initial_yaw_rad=0.021300 result_x_m=0.612345 result_y_m=-0.310123"
      " result_z_m=0.004000 result_yaw_rad=-0.012500"
      " raw_map_to_base_z_m=-0.281234 raw_map_to_base_roll_rad=0.003456"
      " raw_map_to_base_pitch_rad=-0.007890"
      " initial_to_result_translation_m=0.189876"
      " initial_to_result_yaw_rad=0.033800"
      " correction_step_applicable=true"
      " correction_step_reason=ok_after_result_gap_with_bounded_correction"
      " correction_step_has_baseline=true"
      " correction_step_establishes_baseline=false"
      " correction_step_gap_recovered=true"
      " correction_step_dt_sec=0.599914"
      " correction_translation_step_m=0.048765"
      " correction_yaw_step_rad=0.012345"
      " maximum_correction_step_gap_sec=0.500000"
      " maximum_correction_translation_step_m=0.050000"
      " maximum_correction_yaw_step_rad=0.034907"
      " alignment_source_fitness_m2=0.714285"
      " fitness_gate_scope=alignment_source_points_mean_squared_nearest_neighbor"
      " pre_bounds_all_point_fitness_m2=4.714285"
      " pre_bounds_fitness_scope=range_voxel_source_against_full_target"
      " extended_quality_diagnostics_enabled=true"
      " inlier_max_distance_m=1.000000"
      " diagnostic_target_points=12500 diagnostic_source_points=2038"
      " inlier_points=1528 source_inlier_fraction=0.749755"
      " source_to_target_inlier_rmse_m=0.182345"
      " pre_bounds_full_target_points=18000 pre_bounds_source_points=2718"
      " pre_bounds_inlier_points=2038"
      " pre_bounds_source_inlier_fraction=0.749816"
      " pre_bounds_source_to_full_target_inlier_rmse_m=0.182345"
      " iterations=17"
      " alignment_runtime_ms=170.123456"
      " alignment_thread_cpu_ms=21.123456"
      " fitness_thread_cpu_ms=8.123456"
      " post_alignment_runtime_ms=28.641976"
      " fitness_runtime_ms=9.123456"
      " alignment_quality_runtime_ms=8.234567"
      " pre_bounds_quality_runtime_ms=10.345678"
      " end_to_end_runtime_ms=198.765432"
      " owner_tid=123 schedstat_available=true schedstat_status=1"
      " schedstat_begin_steady_ns=1000 schedstat_end_steady_ns=2000"
      " schedstat_sampling_wall_ns=20 schedstat_running_ns=200 schedstat_runqueue_wait_ns=700"
      " converged=true accepted=true gate_reason=ok");
}

TEST(NdtMatchAudit, PreservesRejectedGateAndNonFiniteEvidence) {
  NdtMatchAudit audit;
  audit.sensor_stamp_ns = 77;
  audit.source_raw_points = 1000;
  audit.source_filtered_points = 500;
  audit.result.x = std::numeric_limits<double>::quiet_NaN();
  audit.alignment_source_fitness_m2 = std::numeric_limits<double>::infinity();
  audit.pre_bounds_all_point_fitness_m2 =
      std::numeric_limits<double>::infinity();
  audit.iterations = 30;
  audit.alignment_runtime_ms = 225.0;
  audit.post_alignment_runtime_ms = 25.0;
  audit.end_to_end_runtime_ms = 250.0;
  audit.converged = false;
  audit.accepted = false;
  audit.gate_reason = "ndt_result_non_finite";

  const std::string line = format_ndt_match_audit(audit);
  EXPECT_EQ(line.find('\n'), std::string::npos);
  EXPECT_NE(line.find("result_x_m=nan"), std::string::npos);
  EXPECT_NE(line.find("raw_map_to_base_z_m=nan"), std::string::npos);
  EXPECT_NE(line.find("correction_step_dt_sec=nan"), std::string::npos);
  EXPECT_NE(line.find("correction_translation_step_m=nan"), std::string::npos);
  EXPECT_NE(line.find("alignment_source_fitness_m2=inf"), std::string::npos);
  EXPECT_NE(
      line.find("pre_bounds_all_point_fitness_m2=inf"), std::string::npos);
  EXPECT_NE(
      line.find(
          "fitness_gate_scope=alignment_source_points_mean_squared_nearest_neighbor"),
      std::string::npos);
  EXPECT_NE(line.find("source_inlier_fraction=nan"), std::string::npos);
  EXPECT_NE(
      line.find("source_to_target_inlier_rmse_m=nan"), std::string::npos);
  EXPECT_NE(line.find("end_to_end_runtime_ms=250.000000"), std::string::npos);
  EXPECT_NE(line.find("post_alignment_runtime_ms=25.000000"), std::string::npos);
  EXPECT_NE(line.find("converged=false accepted=false"), std::string::npos);
  EXPECT_NE(line.find("gate_reason=ndt_result_non_finite"), std::string::npos);
  EXPECT_NE(line.find("compute_backend=cpu"), std::string::npos);
  EXPECT_NE(
      line.find("backend=pcl threads=1 neighborhood=KDTREE compute_backend=cpu"),
      std::string::npos);
}

TEST(NdtMatchAudit, MissingCpuTimingIsNotZeroOrAcceptanceEvidence) {
  NdtMatchAudit audit;
  audit.gate_reason = "ndt_input_measurement_stale";
  const auto line = format_ndt_match_audit(audit);
  EXPECT_NE(line.find("alignment_thread_cpu_ms=nan"), std::string::npos);
  EXPECT_NE(line.find("fitness_thread_cpu_ms=nan"), std::string::npos);
  EXPECT_NE(line.find("owner_tid=-1 schedstat_available=false schedstat_status=0"), std::string::npos);
  EXPECT_NE(line.find("schedstat_running_ns=-1 schedstat_runqueue_wait_ns=-1"), std::string::npos);
  EXPECT_NE(line.find("accepted=false gate_reason=ndt_input_measurement_stale"), std::string::npos);
}

}  // namespace
}  // namespace sunhorizon_3d_localizer
