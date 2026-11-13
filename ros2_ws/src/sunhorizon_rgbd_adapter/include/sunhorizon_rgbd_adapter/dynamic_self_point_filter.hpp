#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <exception>
#include <string>
#include <vector>
#include <functional>
#include <future>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <memory>
#include "sunhorizon_obstacle_coverage/dynamic_self_geometry.hpp"

namespace sunhorizon_rgbd_adapter {
// View over packed records; the ROS owner preserves all field/header metadata.
struct DynamicPointField { std::size_t offset{0}; std::size_t scalar_bytes{0}; };
struct DynamicCloudView {
  const std::uint8_t* data{nullptr}; std::size_t size{0};
  std::size_t width{0}, height{0}, point_step{0}, row_step{0};
  bool big_endian{false}; std::array<DynamicPointField,3> xyz;
};
struct DynamicFilterBudget {
  std::size_t max_points{307200}, max_bytes{64*1024*1024};
  std::size_t max_node_visits{32000000}, max_triangle_tests{16000000};
  sunhorizon_obstacle_coverage::dynamic_self_geometry::QueryBudget query;
};
struct DynamicFilterResult {
  bool complete{false}; std::vector<std::uint8_t> data;
  // Whole-record retained order. Model-blocked endpoints cannot train or clear
  // ground; downstream classification alone grants obstacle marking eligibility.
  // 0: eligible external; 1: original model blocked; 2: native projection edge
  // suppressed from mark/clear/protection. Every nonzero value blocks ground.
  // Value 2 retains the original audit sample and is NEVER Self denominator.
  std::vector<uint8_t> ground_blocked;
  // Reconciliation is reserved for nominal visibility disagreements. A
  // calibrated measured body surface cannot be overruled by a ground fit.
  std::vector<uint8_t> nominal_model_conflict;
  std::size_t input_count{0}, removed_count{0}, retained_count{0};
  // Complete per-ray geometric ambiguity is local evidence, never missing FK.
  std::size_t measured_reference_ambiguous_count{0};
  std::array<std::size_t,5> classifications{};
  std::size_t node_visits{0}, triangle_tests{0}; std::string reason;
};
// One persistent helper owned by one body engine, never shared between roles.
// begin_frame() leases the owner for the entire immutable source frame; every
// run() joins its bounded job before returning, including either-side errors.
class BodyGeometryWorker {
 public:
  struct Frame {
    Frame(Frame&&) = default;
    Frame(const Frame&) = delete;
   private:
    friend class BodyGeometryWorker;
    explicit Frame(std::unique_lock<std::mutex> lock):lock_(std::move(lock)){}
    std::unique_lock<std::mutex> lock_;
  };
  explicit BodyGeometryWorker(std::function<void()> owner_check);
  ~BodyGeometryWorker();
  BodyGeometryWorker(const BodyGeometryWorker&) = delete;
  Frame begin_frame();
  void run(std::function<void()> helper, const std::function<void()>& caller);
  double frame_helper_cpu_ms() const noexcept {return frame_cpu_ms_;}
 private:
  void loop();
  std::function<void()> owner_check_;
  std::mutex mutex_,frame_mutex_;
  std::condition_variable ready_;
  std::function<void()> job_;
  bool started_{false},closed_{false};
  std::exception_ptr startup_error_,worker_error_;
  double frame_cpu_ms_{0.};
  std::thread thread_;
};

// Shared bounded classification/compaction state. The model is an immutable
// same-frame snapshot and must outlive this accumulator. Query budgets must
// match the model exactly; a conflicting caller budget cannot weaken limits.
// append() borrows one original record only for the duration of the call.
// Failure clears all output bytes but preserves the legacy partial counters.
class DynamicFilterAccumulator {
 public:
  DynamicFilterAccumulator(
      const sunhorizon_obstacle_coverage::dynamic_self_geometry::PreparedModel& model,
      double tolerance_m, const DynamicFilterBudget& budget,
      std::size_t input_count, std::size_t point_step);
  bool ready() const noexcept { return active_; }
  std::size_t retained_count() const noexcept {return result_.retained_count;}
  std::size_t removed_count() const noexcept {return result_.removed_count;}
  bool append(const std::uint8_t* record, double x, double y, double z,
      const sunhorizon_obstacle_coverage::dynamic_self_geometry::NativePixelSupport* native_pixel=nullptr);
  DynamicFilterResult finish();
 private:
  friend class DynamicFilterPreparedConsumer;
  bool append_precomputed(const std::uint8_t* record,double x,double y,double z,
      const sunhorizon_obstacle_coverage::dynamic_self_geometry::Result& precomputed);
  std::size_t remaining_nodes()const{return budget_.max_node_visits-result_.node_visits;}
  std::size_t remaining_triangles()const{return budget_.max_triangle_tests-result_.triangle_tests;}
  const sunhorizon_obstacle_coverage::dynamic_self_geometry::PreparedModel& model_;
  double tolerance_;
  DynamicFilterBudget budget_;
  std::size_t point_step_;
  DynamicFilterResult result_;
  bool active_{false};
  bool fail(const char* reason);
};
inline bool DynamicFilterAccumulator::append(const std::uint8_t* p,double x,double y,double z,
    const sunhorizon_obstacle_coverage::dynamic_self_geometry::NativePixelSupport* native_pixel) {
  if(!active_)return false;
  if(!p)return fail("null_point_record");
  if(result_.removed_count+result_.retained_count>=result_.input_count)
    return fail("point_count_budget_or_empty");
  try {
    sunhorizon_obstacle_coverage::dynamic_self_geometry::Classification classification=sunhorizon_obstacle_coverage::dynamic_self_geometry::Classification::Invalid;
    bool measured_authority=false,edge_suppressed=false,body_occluded=false;
    if(std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z)&&z>0){
      const auto item=native_pixel?
        sunhorizon_obstacle_coverage::dynamic_self_geometry::classify_native_body_filter({x/z,y/z,1},z,tolerance_,model_,*native_pixel):
        sunhorizon_obstacle_coverage::dynamic_self_geometry::classify_body_surface({x/z,y/z,1},z,tolerance_,model_);
      if(!item.complete)return fail(item.reason);
      if(item.node_visits>budget_.max_node_visits-result_.node_visits||
         item.triangle_tests>budget_.max_triangle_tests-result_.triangle_tests)return fail("frame_query_budget");
      result_.node_visits+=item.node_visits;result_.triangle_tests+=item.triangle_tests;classification=item.classification;
      measured_authority=item.has_measured_surface;
      edge_suppressed=sunhorizon_obstacle_coverage::dynamic_self_geometry::projection_edge_suppressed(item);
      body_occluded=sunhorizon_obstacle_coverage::dynamic_self_geometry::measurement_body_occluded(item);
      if(item.measured_reference_ambiguous &&
          classification==sunhorizon_obstacle_coverage::dynamic_self_geometry::Classification::Unobserved)
        ++result_.measured_reference_ambiguous_count;
    }
    ++result_.classifications[static_cast<std::size_t>(classification)];
    if(classification==sunhorizon_obstacle_coverage::dynamic_self_geometry::Classification::SelfSurface)++result_.removed_count;
    else {
      result_.data.insert(result_.data.end(),p,p+point_step_);
      result_.ground_blocked.push_back(body_occluded?3:edge_suppressed?2:uint8_t(measured_authority || classification!=sunhorizon_obstacle_coverage::dynamic_self_geometry::Classification::ExternalSurface));
      result_.nominal_model_conflict.push_back(!measured_authority &&
        classification==sunhorizon_obstacle_coverage::dynamic_self_geometry::Classification::ModelConflict);
      ++result_.retained_count;
    }
    return true;
  }catch(const std::exception&){return fail("filter_allocation_failure");}
}

inline bool DynamicFilterAccumulator::append_precomputed(const std::uint8_t* p,double x,double y,double z,
    const sunhorizon_obstacle_coverage::dynamic_self_geometry::Result& precomputed) {
  if(!active_)return false;
  if(!p)return fail("null_point_record");
  if(result_.removed_count+result_.retained_count>=result_.input_count)
    return fail("point_count_budget_or_empty");
  try {
    sunhorizon_obstacle_coverage::dynamic_self_geometry::Classification classification=sunhorizon_obstacle_coverage::dynamic_self_geometry::Classification::Invalid;
    bool measured_authority=false,edge_suppressed=false,body_occluded=false;
    if(std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z)&&z>0){
      const auto& item=precomputed;
      if(!item.complete)return fail(item.reason);
      if(item.node_visits>budget_.max_node_visits-result_.node_visits||
         item.triangle_tests>budget_.max_triangle_tests-result_.triangle_tests)return fail("frame_query_budget");
      result_.node_visits+=item.node_visits;result_.triangle_tests+=item.triangle_tests;classification=item.classification;
      measured_authority=item.has_measured_surface;
      edge_suppressed=sunhorizon_obstacle_coverage::dynamic_self_geometry::projection_edge_suppressed(item);
      body_occluded=sunhorizon_obstacle_coverage::dynamic_self_geometry::measurement_body_occluded(item);
      if(item.measured_reference_ambiguous &&
          classification==sunhorizon_obstacle_coverage::dynamic_self_geometry::Classification::Unobserved)
        ++result_.measured_reference_ambiguous_count;
    }
    ++result_.classifications[static_cast<std::size_t>(classification)];
    if(classification==sunhorizon_obstacle_coverage::dynamic_self_geometry::Classification::SelfSurface)++result_.removed_count;
    else {
      result_.data.insert(result_.data.end(),p,p+point_step_);
      result_.ground_blocked.push_back(body_occluded?3:edge_suppressed?2:uint8_t(measured_authority || classification!=sunhorizon_obstacle_coverage::dynamic_self_geometry::Classification::ExternalSurface));
      result_.nominal_model_conflict.push_back(!measured_authority &&
        classification==sunhorizon_obstacle_coverage::dynamic_self_geometry::Classification::ModelConflict);
      ++result_.retained_count;
    }
    return true;
  }catch(const std::exception&){return fail("filter_allocation_failure");}
}


// These prechecks also preserve validation precedence before cloud layout and
// model work. A null result means the scalar/frame limits are well formed.
const char* dynamic_filter_budget_error(double tolerance_m, const DynamicFilterBudget& budget);

// Optical origin, positive axial Z only. Only a finite model triangle within the metric surface
// tolerance is removed. Invalid/near external/far conflict remain byte-identical.
// Any incomplete model/TF/query rejects the whole frame, including prior partial
// removals. Output compacts retained point records; row padding is not a record.
// Empty filtered records do not establish free space behind the removed body.
DynamicFilterResult filter_dynamic_self_points(const DynamicCloudView& cloud,
  const std::vector<sunhorizon_obstacle_coverage::dynamic_self_geometry::LinkObservation>& links,
  double tolerance_m, const DynamicFilterBudget& budget = {});
DynamicFilterResult filter_dynamic_self_points(const DynamicCloudView& cloud,
  const sunhorizon_obstacle_coverage::dynamic_self_geometry::PreparedModel& model,
  double tolerance_m, const DynamicFilterBudget& budget = {});
}
