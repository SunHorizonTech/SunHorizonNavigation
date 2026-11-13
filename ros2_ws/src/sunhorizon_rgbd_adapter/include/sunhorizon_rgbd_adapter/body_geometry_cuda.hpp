#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <vector>
#include <sunhorizon_obstacle_coverage/dynamic_self_geometry.hpp>
namespace sunhorizon_rgbd_adapter {
// Camera BODY only. No source/TF identity or freshness can be granted here.
// The caller owns validated immutable source-time model/projection and the
// existing ordered accumulator. One instance accepts at most one call at once.
class BodyGeometryCuda {
 public:
  struct Sample {std::array<float,3> xyz;std::size_t sample_id;};
  using Result=sunhorizon_obstacle_coverage::dynamic_self_geometry::Result;
  using Model=sunhorizon_obstacle_coverage::dynamic_self_geometry::PreparedModel;
  using Pixel=sunhorizon_obstacle_coverage::dynamic_self_geometry::NativePixelSupport;
  struct Timings {
    double snapshot_ms{},cache_check_ms{},buffer_prepare_ms{},enqueue_ms{},wait_ms{},consume_ms{},total_ms{};
    float h2d_ms{},kernel_ms{},d2h_ms{};
    std::size_t mesh_bytes{},h2d_bytes{},d2h_bytes{};
    bool mesh_uploaded{};
  };
  // Profiling is opt-in for offline evidence; runtime construction leaves it off.
  explicit BodyGeometryCuda(bool profile=false);
  bool profiling() const noexcept;
  Timings last_timings() const;
  // Read-only lifecycle observation, never freshness/eligibility. True means
  // this instance's recorded completion event was observed incomplete. Safe
  // concurrently with query/stop; caller must retain the instance's lifetime.
  bool has_pending_device_work() const;
  ~BodyGeometryCuda();
  BodyGeometryCuda(const BodyGeometryCuda&)=delete;
  BodyGeometryCuda& operator=(const BodyGeometryCuda&)=delete;
  static bool compiled() noexcept;
  // Copies samples/model upload state into instance-owned pinned storage,
  // computes all queries and consumes them in original order while locked.
  // A false consume stops ordered commit at its original failure boundary.
  // Any CUDA/context/stop error throws; no CPU fallback or partial output.
  void query(const Model&,double tolerance,const Pixel*,const std::vector<Sample>&,
             const std::function<bool(std::size_t,const Result&)>& consume);
  // Stops admission/waits; already queued GPU work retains owned buffers until
  // destructor drain. It never changes device flags or resets another owner.
  void request_stop() noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
