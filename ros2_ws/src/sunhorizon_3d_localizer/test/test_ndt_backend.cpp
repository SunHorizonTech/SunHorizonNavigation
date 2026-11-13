#include <gtest/gtest.h>

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "sunhorizon_3d_localizer/ndt_backend.hpp"

#ifdef SUNHORIZON_TEST_NDT_OMP
#include <pclomp/voxel_grid_covariance_omp.h>
#endif

namespace sunhorizon_3d_localizer {
namespace {

TEST(NdtBackend, ComputeSelectionIsExplicitAndSupported) {
  EXPECT_EQ(resolve_ndt_backend_options("ndt_omp", 4, "DIRECT7").compute_backend, "cpu");
  EXPECT_EQ(resolve_ndt_backend_options("ndt_omp", 4, "DIRECT7", "cuda_direct7").compute_backend, "cuda_direct7");
  EXPECT_THROW(resolve_ndt_backend_options("pcl", 1, "KDTREE", "cuda_direct7"), std::runtime_error);
  EXPECT_THROW(resolve_ndt_backend_options("ndt_omp", 4, "DIRECT1", "cuda_direct7"), std::runtime_error);
  EXPECT_THROW(resolve_ndt_backend_options("ndt_omp", 4, "DIRECT7", "auto"), std::runtime_error);
  if (!ndt_cuda_compiled()) {
    EXPECT_THROW(NdtRegistration(resolve_ndt_backend_options("ndt_omp", 4, "DIRECT7", "cuda_direct7")), std::runtime_error);
  }
  NdtRegistration cpu(resolve_ndt_backend_options("pcl", 1, "KDTREE"));
  EXPECT_EQ(cpu.compute_backend(), "cpu");
}

TEST(NdtBackend, PreservesPclDefaults) {
  const auto options = resolve_ndt_backend_options(
      "pcl", std::nullopt, std::nullopt);
  EXPECT_EQ(options.backend, NdtBackendKind::kPcl);
  EXPECT_EQ(options.threads, 1);
  EXPECT_EQ(options.neighborhood, NdtNeighborhood::kKdTree);
  EXPECT_EQ(to_string(options.backend), "pcl");
  EXPECT_EQ(to_string(options.neighborhood), "KDTREE");
  EXPECT_NO_THROW({
    NdtRegistration registration(options);
    EXPECT_EQ(registration.options().backend, NdtBackendKind::kPcl);
  });
}

TEST(NdtBackend, RejectsSettingsPclCannotHonor) {
  EXPECT_THROW(
      resolve_ndt_backend_options("pcl", 4, "KDTREE"), std::runtime_error);
  EXPECT_THROW(
      resolve_ndt_backend_options("pcl", 1, "DIRECT7"), std::runtime_error);
  NdtRegistration registration(resolve_ndt_backend_options("pcl", 1, "KDTREE"));
  EXPECT_NO_THROW(registration.set_team_cpu_affinity({}));
  EXPECT_THROW(registration.set_team_cpu_affinity({0}), std::invalid_argument);
}

TEST(NdtBackend, GivesNdtOmpCandidateDefaultsOnlyToNdtOmp) {
  const auto options = resolve_ndt_backend_options(
      "ndt_omp", std::nullopt, std::nullopt);
  EXPECT_EQ(options.backend, NdtBackendKind::kNdtOmp);
  EXPECT_EQ(options.threads, 4);
  EXPECT_EQ(options.neighborhood, NdtNeighborhood::kDirect7);
}

TEST(NdtBackend, AcceptsEverySupportedNdtOmpNeighborhood) {
  EXPECT_EQ(
      resolve_ndt_backend_options("ndt_omp", 2, "KDTREE").neighborhood,
      NdtNeighborhood::kKdTree);
  EXPECT_EQ(
      resolve_ndt_backend_options("ndt_omp", 3, "direct1").neighborhood,
      NdtNeighborhood::kDirect1);
  EXPECT_EQ(
      resolve_ndt_backend_options("NDT_OMP", 8, "DIRECT7").neighborhood,
      NdtNeighborhood::kDirect7);
}

TEST(NdtBackend, RejectsInvalidBackendNeighborhoodAndThreadCount) {
  EXPECT_THROW(
      resolve_ndt_backend_options("unknown", std::nullopt, std::nullopt),
      std::runtime_error);
  EXPECT_THROW(
      resolve_ndt_backend_options("ndt_omp", 0, "DIRECT7"),
      std::runtime_error);
  EXPECT_THROW(
      resolve_ndt_backend_options("ndt_omp", 4, "DIRECT26"),
      std::runtime_error);
}

TEST(NdtBackend, FailsClosedWhenNdtOmpWasNotCompiled) {
  if (ndt_omp_compiled()) GTEST_SKIP() << "ndt_omp is compiled in this build";
  const auto options = resolve_ndt_backend_options(
      "ndt_omp", std::nullopt, std::nullopt);
  try {
    NdtRegistration registration(options);
    FAIL() << "ndt_omp construction unexpectedly succeeded";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("SUNHORIZON_ENABLE_NDT_OMP=OFF"),
        std::string::npos);
  }
}

#ifdef SUNHORIZON_TEST_NDT_OMP
TEST(NdtBackend, PlanarTargetCovarianceUsesCenteredSamplesAndEigenvalueFloor) {
  // Six points with analytical unbiased variances .018, .027, and zero.
  // The existing 1% eigenvalue floor must make the last value .00027;
  // an identity matrix in the accumulator incorrectly makes it .138889.
  for (const float offset : {0.0F, 20.0F}) {
    for (const bool field_filter : {false, true}) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      for (const float x : {.05F, .20F, .35F}) {
        for (const float y : {.05F, .35F}) {
          cloud->push_back(pcl::PointXYZ(x + offset, y + offset, -1.05F + offset));
        }
      }
      pclomp::VoxelGridCovariance<pcl::PointXYZ> grid;
      grid.setLeafSize(.5F, .5F, .5F);
      grid.setInputCloud(cloud);
      if (field_filter) {
        grid.setFilterFieldName("x");
        grid.setFilterLimits(offset, offset + .5F);
      }
      grid.filter(true);
      ASSERT_EQ(grid.getLeaves().size(), 1U);
      const auto& leaf = grid.getLeaves().begin()->second;
      ASSERT_EQ(leaf.getPointCount(), 6);
      const auto covariance = leaf.getCov();
      EXPECT_NEAR(covariance(0, 0), .018, 1e-6);
      EXPECT_NEAR(covariance(1, 1), .027, 1e-6);
      EXPECT_NEAR(covariance(2, 2), .00027, 1e-8);
      EXPECT_LT((covariance - covariance.transpose()).norm(), 1e-12);
      EXPECT_LT((covariance * leaf.getInverseCov() - Eigen::Matrix3d::Identity()).norm(), 1e-9);
    }
  }
}

TEST(NdtBackend, NdtOmpIterationLimitDoesNotReportConvergence) {
  // Full-rank target distributions in distinct voxels make all six pose
  // directions observable. The source has a known rigid translation and
  // starts far enough away that one bounded step cannot finish registration.
  NdtRegistration::PointCloud::Ptr target(new NdtRegistration::PointCloud);
  NdtRegistration::PointCloud::Ptr source(new NdtRegistration::PointCloud);
  const Eigen::Vector3f offset(.18F, -.12F, .14F);
  for (int x = 0; x < 3; ++x) {
    for (int y = 0; y < 2; ++y) {
      for (int z = 0; z < 2; ++z) {
        for (int dx = -2; dx <= 2; ++dx) {
          for (int dy = -2; dy <= 2; ++dy) {
            for (int dz = -2; dz <= 2; ++dz) {
              const pcl::PointXYZ point(.25F + x + .05F * dx,
                  .25F + y + .05F * dy, .25F + z + .05F * dz);
              target->push_back(point);
              source->push_back(pcl::PointXYZ(point.x + offset.x(),
                  point.y + offset.y(), point.z + offset.z()));
            }
          }
        }
      }
    }
  }
  const auto options = resolve_ndt_backend_options("ndt_omp", 1, "DIRECT7");
  const auto registration = [&](int iterations) {
    NdtRegistration result(options);
    result.set_resolution(.5F);
    result.set_step_size(.05);
    result.set_transformation_epsilon(1e-6);
    result.set_maximum_iterations(iterations);
    result.set_input_target(target);
    result.set_input_source(source);
    return result;
  };

  // Establish that this is a solvable registration that actually needs more
  // than one iteration, rather than an unsupported or degenerate point cloud.
  auto complete = registration(80);
  NdtRegistration::PointCloud complete_output;
  complete.align(complete_output, Eigen::Matrix4f::Identity());
  ASSERT_TRUE(complete.has_converged());
  ASSERT_GT(complete.final_iteration_count(), 1);
  ASSERT_TRUE(complete.final_transformation().allFinite());
  EXPECT_LT((complete.final_transformation().block<3, 1>(0, 3) + offset).norm(), .005F);

  auto limited = registration(1);
  NdtRegistration::PointCloud limited_output;
  limited.align(limited_output, Eigen::Matrix4f::Identity());
  ASSERT_EQ(limited.final_iteration_count(), 1);
  ASSERT_TRUE(limited.final_transformation().allFinite());
  // The still-large pose error independently demonstrates that stopping was
  // caused by the budget, not successful completion of this registration.
  EXPECT_GT((limited.final_transformation().block<3, 1>(0, 3) + offset).norm(), .1F);
  EXPECT_FALSE(limited.has_converged());
}

TEST(NdtBackend, CovarianceFixPreservesMinimumTargetSupport) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < 5; ++i) cloud->push_back(pcl::PointXYZ(.05F * i, .1F, -.75F));
  pclomp::VoxelGridCovariance<pcl::PointXYZ> grid;
  grid.setLeafSize(.5F, .5F, .5F);
  grid.setInputCloud(cloud);
  grid.filter(true);
  ASSERT_EQ(grid.getLeaves().size(), 1U);
  EXPECT_LT(grid.getLeaves().begin()->second.getPointCount(), grid.getMinPointPerVoxel());
  pcl::PointXYZ query(.1F, .1F, -.75F);
  std::vector<pclomp::VoxelGridCovariance<pcl::PointXYZ>::LeafConstPtr> neighbors;
  EXPECT_EQ(grid.getNeighborhoodAtPoint7(query, neighbors), 0);
}
#endif

}  // namespace
}  // namespace sunhorizon_3d_localizer
