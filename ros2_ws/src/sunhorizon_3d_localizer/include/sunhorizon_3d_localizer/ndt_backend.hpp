#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sunhorizon_3d_localizer {

enum class NdtBackendKind {
  kPcl,
  kNdtOmp,
};

enum class NdtNeighborhood {
  kKdTree,
  kDirect1,
  kDirect7,
};

struct NdtBackendOptions {
  NdtBackendKind backend{NdtBackendKind::kPcl};
  std::string compute_backend{"cpu"};
  int threads{1};
  NdtNeighborhood neighborhood{NdtNeighborhood::kKdTree};
};

// Resolves context-sensitive defaults and rejects settings that the selected
// backend cannot honor. PCL NDT is intentionally fixed to KDTREE/1; ndt_omp
// defaults to DIRECT7/4 while accepting all three explicit neighborhoods.
NdtBackendOptions resolve_ndt_backend_options(
    const std::string& backend, const std::optional<int>& threads,
    const std::optional<std::string>& neighborhood,
    const std::string& compute_backend = "cpu");

std::string to_string(NdtBackendKind backend);
std::string to_string(NdtNeighborhood neighborhood);
bool ndt_omp_compiled() noexcept;
bool ndt_cuda_compiled() noexcept;

// A small common surface used by both the online localizer and the offline
// evaluator. Constructing an ndt_omp instance in a PCL-only build throws a
// clear exception before any registration or output can occur.
class NdtRegistration final {
 public:
  using Point = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<Point>;

  explicit NdtRegistration(const NdtBackendOptions& options);
  ~NdtRegistration();

  NdtRegistration(const NdtRegistration&) = delete;
  NdtRegistration& operator=(const NdtRegistration&) = delete;
  NdtRegistration(NdtRegistration&&) noexcept;
  NdtRegistration& operator=(NdtRegistration&&) noexcept;

  const NdtBackendOptions& options() const noexcept;
  std::string compute_backend() const;
  // Optional rank-to-CPU placement, scoped to each NDT OpenMP region.
  // Empty retains the backend's inherited scheduling; PCL cannot honor a team.
  void set_team_cpu_affinity(const std::vector<int>& cpus);
  void set_transformation_epsilon(double epsilon);
  void set_step_size(double step_size);
  void set_resolution(float resolution);
  void set_maximum_iterations(int iterations);
  void set_input_target(const PointCloud::ConstPtr& target);
  void set_input_source(const PointCloud::ConstPtr& source);
  void align(PointCloud& output, const Eigen::Matrix4f& initial_guess);
  Eigen::Matrix4f final_transformation();
  double fitness_score();
  int final_iteration_count();
  bool has_converged();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sunhorizon_3d_localizer
