#pragma once
#include <functional>
#include "sunhorizon_perception_runtime/classified_obstacle_frame.hpp"

namespace sunhorizon_perception_runtime {
// Shared registration owner and original-time FK only. A batch has one map
// alignment/reset lifetime; geometry availability supplies no physical bounds.
struct RegistrationGroundGeometryBatch {
  std::vector<ground_relative::SourceTimeTransform> transforms;
  ground_relative::RegistrationMapSourceBinding map_binding;
  ground_relative::SourceTransformDiagnostic diagnostic;
  std::shared_ptr<const std::atomic<bool>> reset_revoked{};
  int64_t reset_barrier_steady_ns{0};
};
using RegistrationGroundGeometryProvider=std::function<RegistrationGroundGeometryBatch(
  const ground_relative::SourceIdentity&,const std::vector<int64_t>&,bool airy_scan)>;
namespace detail {
// The caller supplies its original frame freshness and existing worker wake.
// No clock/receipt/lease is generated here, and only the missing future bracket
// may wait. All other provider failures retain their original disposition.
template<class Query,class Fresh,class Stopped,class Wait>
std::optional<RegistrationGroundGeometryBatch> wait_for_registration_ground_geometry(
    const ground_relative::SourceIdentity& identity,Query&& query,Fresh&& fresh,Stopped&& stopped,Wait&& wait) {
  std::shared_ptr<const std::atomic<bool>> original_reset;
  const auto cancelled=[&] {return stopped()||!fresh()||
    (original_reset&&original_reset->load(std::memory_order_acquire));};
  for(;;) {
    if(cancelled())return std::nullopt;
    auto batch=query(identity);
    if(cancelled() || (batch.reset_revoked&&batch.reset_revoked->load(std::memory_order_acquire)) ||
        identity.first_receipt_steady_ns<=batch.reset_barrier_steady_ns ||
        (original_reset&&batch.reset_revoked&&batch.reset_revoked!=original_reset))return std::nullopt;
    if(batch.diagnostic.provider_state!=3 || batch.diagnostic.reason!="odometry_history_target_after_newest") {
      if(original_reset&&batch.diagnostic.provider_state==0&&batch.reset_revoked!=original_reset)return std::nullopt;
      return batch;
    }
    if(!batch.reset_revoked)return std::nullopt; // No original epoch to wait on.
    if(!original_reset)original_reset=batch.reset_revoked;
    wait();
  }
}
} // namespace detail
} // namespace sunhorizon_perception_runtime
