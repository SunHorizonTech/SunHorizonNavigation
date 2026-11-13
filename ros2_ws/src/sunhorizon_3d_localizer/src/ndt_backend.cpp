#include "sunhorizon_3d_localizer/ndt_backend.hpp"

#include <pcl/registration/ndt.h>

#ifdef SUNHORIZON_ENABLE_NDT_OMP
#include <pclomp/ndt_omp.h>
#endif

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <utility>

namespace sunhorizon_3d_localizer {
namespace {

std::string uppercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
      [](unsigned char character) {
        return static_cast<char>(std::toupper(character));
      });
  return value;
}

std::string lowercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
      [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
      });
  return value;
}

NdtNeighborhood parse_neighborhood(const std::string& value) {
  const std::string normalized = uppercase(value);
  if (normalized == "KDTREE") return NdtNeighborhood::kKdTree;
  if (normalized == "DIRECT1") return NdtNeighborhood::kDirect1;
  if (normalized == "DIRECT7") return NdtNeighborhood::kDirect7;
  throw std::runtime_error(
      "invalid NDT neighborhood '" + value +
      "'; expected KDTREE, DIRECT1, or DIRECT7");
}

}  // namespace

NdtBackendOptions resolve_ndt_backend_options(
    const std::string& backend, const std::optional<int>& threads,
    const std::optional<std::string>& neighborhood,
    const std::string& compute_backend) {
  const std::string normalized = lowercase(backend);
  NdtBackendOptions output;
  output.compute_backend = lowercase(compute_backend);
  if (output.compute_backend != "cpu" && output.compute_backend != "cuda_direct7")
    throw std::runtime_error("invalid NDT compute_backend; expected cpu or cuda_direct7");
  if (output.compute_backend != "cpu" && normalized != "ndt_omp")
    throw std::runtime_error("cuda_direct7 requires ndt_omp");
  if (normalized == "pcl") {
    output.backend = NdtBackendKind::kPcl;
    output.threads = threads.value_or(1);
    output.neighborhood = neighborhood
        ? parse_neighborhood(*neighborhood) : NdtNeighborhood::kKdTree;
    if (output.threads != 1 || output.neighborhood != NdtNeighborhood::kKdTree) {
      throw std::runtime_error(
          "PCL NDT only supports threads=1 and neighborhood=KDTREE");
    }
    return output;
  }
  if (normalized == "ndt_omp") {
    output.backend = NdtBackendKind::kNdtOmp;
    output.threads = threads.value_or(4);
    output.neighborhood = neighborhood
        ? parse_neighborhood(*neighborhood) : NdtNeighborhood::kDirect7;
    if (output.compute_backend == "cuda_direct7" && output.neighborhood != NdtNeighborhood::kDirect7)
      throw std::runtime_error("cuda_direct7 requires DIRECT7");
    if (output.threads < 1 || output.threads > 256) {
      throw std::runtime_error("ndt_omp threads must be in [1, 256]");
    }
    return output;
  }
  throw std::runtime_error(
      "invalid NDT backend '" + backend + "'; expected pcl or ndt_omp");
}

std::string to_string(NdtBackendKind backend) {
  switch (backend) {
    case NdtBackendKind::kPcl:
      return "pcl";
    case NdtBackendKind::kNdtOmp:
      return "ndt_omp";
  }
  throw std::runtime_error("unhandled NDT backend");
}

std::string to_string(NdtNeighborhood neighborhood) {
  switch (neighborhood) {
    case NdtNeighborhood::kKdTree:
      return "KDTREE";
    case NdtNeighborhood::kDirect1:
      return "DIRECT1";
    case NdtNeighborhood::kDirect7:
      return "DIRECT7";
  }
  throw std::runtime_error("unhandled NDT neighborhood");
}

bool ndt_omp_compiled() noexcept {
#ifdef SUNHORIZON_ENABLE_NDT_OMP
  return true;
#else
  return false;
#endif
}

bool ndt_cuda_compiled() noexcept {
#if defined(SUNHORIZON_ENABLE_NDT_OMP) && defined(SUNHORIZON_ENABLE_NDT_CUDA)
  return pclomp::offline_cuda::compiled();
#else
  return false;
#endif
}

class NdtRegistration::Impl {
 public:
  explicit Impl(const NdtBackendOptions& requested) : options(resolve_ndt_backend_options(
      to_string(requested.backend), requested.threads, to_string(requested.neighborhood),
      requested.compute_backend)) {
    if (options.compute_backend == "cuda_direct7" && !ndt_cuda_compiled())
      throw std::runtime_error("cuda_direct7 requested but SUNHORIZON_ENABLE_NDT_CUDA=OFF or dependency CUDA unavailable");
    if (options.backend == NdtBackendKind::kPcl) {
      pcl_ndt = std::make_unique<
          pcl::NormalDistributionsTransform<Point, Point>>();
      return;
    }
#ifdef SUNHORIZON_ENABLE_NDT_OMP
    omp_ndt = std::make_unique<
        pclomp::NormalDistributionsTransform<Point, Point>>();
#ifdef SUNHORIZON_ENABLE_NDT_CUDA
    omp_ndt->setCudaEnabled(options.compute_backend == "cuda_direct7");
#endif
    omp_ndt->setNumThreads(options.threads);
    switch (options.neighborhood) {
      case NdtNeighborhood::kKdTree:
        omp_ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
        break;
      case NdtNeighborhood::kDirect1:
        omp_ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
        break;
      case NdtNeighborhood::kDirect7:
        omp_ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        break;
    }
#else
    throw std::runtime_error(
        "ndt_omp backend requested, but sunhorizon_3d_localizer was built "
        "with SUNHORIZON_ENABLE_NDT_OMP=OFF");
#endif
  }

  template <typename Operation>
  decltype(auto) apply(Operation&& operation) {
    if (pcl_ndt) return operation(*pcl_ndt);
#ifdef SUNHORIZON_ENABLE_NDT_OMP
    return operation(*omp_ndt);
#else
    throw std::logic_error("NDT backend is unavailable");
#endif
  }

  NdtBackendOptions options;
  std::unique_ptr<pcl::NormalDistributionsTransform<Point, Point>> pcl_ndt;
#ifdef SUNHORIZON_ENABLE_NDT_OMP
  std::unique_ptr<pclomp::NormalDistributionsTransform<Point, Point>> omp_ndt;
#endif
};

NdtRegistration::NdtRegistration(const NdtBackendOptions& options)
    : impl_(std::make_unique<Impl>(options)) {}

NdtRegistration::~NdtRegistration() = default;
NdtRegistration::NdtRegistration(NdtRegistration&&) noexcept = default;
NdtRegistration& NdtRegistration::operator=(NdtRegistration&&) noexcept = default;

const NdtBackendOptions& NdtRegistration::options() const noexcept {
  return impl_->options;
}

std::string NdtRegistration::compute_backend() const {
#ifdef SUNHORIZON_ENABLE_NDT_CUDA
  if (impl_->omp_ndt) return impl_->omp_ndt->getComputeBackend();
#endif
  return "cpu";
}

void NdtRegistration::set_team_cpu_affinity(const std::vector<int>& cpus) {
  if (impl_->pcl_ndt) {
    if (!cpus.empty()) throw std::invalid_argument("ndt_team_cpu_affinity_requires_ndt_omp");
    return;
  }
#ifdef SUNHORIZON_ENABLE_NDT_OMP
  impl_->omp_ndt->setTeamCpuAffinity(cpus);
#else
  throw std::logic_error("NDT backend is unavailable");
#endif
}

void NdtRegistration::set_transformation_epsilon(double epsilon) {
  impl_->apply([epsilon](auto& ndt) { ndt.setTransformationEpsilon(epsilon); });
}

void NdtRegistration::set_step_size(double step_size) {
  impl_->apply([step_size](auto& ndt) { ndt.setStepSize(step_size); });
}

void NdtRegistration::set_resolution(float resolution) {
  impl_->apply([resolution](auto& ndt) { ndt.setResolution(resolution); });
}

void NdtRegistration::set_maximum_iterations(int iterations) {
  impl_->apply([iterations](auto& ndt) { ndt.setMaximumIterations(iterations); });
}

void NdtRegistration::set_input_target(const PointCloud::ConstPtr& target) {
  impl_->apply([&target](auto& ndt) { ndt.setInputTarget(target); });
}

void NdtRegistration::set_input_source(const PointCloud::ConstPtr& source) {
  impl_->apply([&source](auto& ndt) { ndt.setInputSource(source); });
}

void NdtRegistration::align(
    PointCloud& output, const Eigen::Matrix4f& initial_guess) {
  impl_->apply([&output, &initial_guess](auto& ndt) {
    ndt.align(output, initial_guess);
  });
}

Eigen::Matrix4f NdtRegistration::final_transformation() {
  return impl_->apply([](auto& ndt) {
    return ndt.getFinalTransformation();
  });
}

double NdtRegistration::fitness_score() {
  return impl_->apply([](auto& ndt) { return ndt.getFitnessScore(); });
}

int NdtRegistration::final_iteration_count() {
  return impl_->apply([](auto& ndt) {
    return ndt.getFinalNumIteration();
  });
}

bool NdtRegistration::has_converged() {
  return impl_->apply([](auto& ndt) { return ndt.hasConverged(); });
}

}  // namespace sunhorizon_3d_localizer
