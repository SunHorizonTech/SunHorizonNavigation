#pragma once

#include <nav2_util/direct_cloud.hpp>
#include <sunhorizon_perception_runtime/ground_reference_owner.hpp>
#include <algorithm>
#include <cmath>

namespace sunhorizon_navigation_runtime::detail {
inline nav2_util::DirectGroundBasis direct_ground_basis(
    const sunhorizon_perception_runtime::ground_relative::GroundQualification& ground) {
  using Basis=sunhorizon_perception_runtime::ground_relative::GroundQualification::Basis;
  if(!ground.safety_qualified)return nav2_util::DirectGroundBasis::None;
  switch(ground.basis) {
    case Basis::BoundedReference:return nav2_util::DirectGroundBasis::BoundedReference;
    case Basis::MeasuredSupport:return nav2_util::DirectGroundBasis::MeasuredSupport;
    default:return nav2_util::DirectGroundBasis::None;
  }
}
// Historical registration geometry keeps its original absolute lifetime even
// when the unchanged sensor metadata permits a longer observation age.
inline void attach_ground_validity(nav2_util::DirectCloud& packet,
    const sunhorizon_perception_runtime::ground_relative::ClassifiedObstacleFrame::ConstSharedPtr& frame) {
  namespace gr=sunhorizon_perception_runtime::ground_relative;
  if(!frame || !frame->ground().safety_qualified || !frame->reference() ||
      frame->reference()->lifetime!=gr::GroundReference::Lifetime::RegistrationMapAsset)return;
  const auto& reference=*frame->reference();
  if(!packet.ground_qualified || !gr::registration_map_binding_current(reference,frame->source().identity))
    throw std::invalid_argument("ground_validity_registration_binding_unavailable");
  const auto alignment=reference.map_binding.alignment;
  auto validity=std::make_shared<nav2_util::DirectGroundValidity>();
  validity->valid_until_ros_ns=alignment->geometry_valid_until_ros_ns();
  validity->valid_until_steady_ns=alignment->geometry_valid_until_steady_ns();
  validity->identity_current=[alignment]{return alignment->current();};
  packet.ground_validity=std::move(validity);
}
// A separate empirical correction input, never a physical bound/free volume.
// The consumer must independently exclude each historical voxel, fit TRAIN,
// validate HOLDOUT and enclose its complete raw endpoint envelope before UNKNOWN.
inline void attach_empirical_ground_support(nav2_util::DirectCloud& packet,
    const sunhorizon_perception_runtime::ground_relative::ClassifiedObstacleFrame::ConstSharedPtr& frame,
    const sunhorizon_perception_runtime::ground_relative::GroundReferenceOwner::ConstSharedPtr& owner) {
  using Basis=sunhorizon_perception_runtime::ground_relative::GroundQualification::Basis;
  if(!frame||!owner||owner->observation_only()||!frame->classification_complete()||
    !frame->ground().safety_qualified||frame->ground().basis!=Basis::MeasuredSupport)return;
  // Restrict this empirical candidate to the current map-asset route, whose
  // original absolute geometry lease was attached independently above. A timed
  // reference must not borrow the sensor freshness budget as a longer lease.
  using Lifetime=sunhorizon_perception_runtime::ground_relative::GroundReference::Lifetime;
  if(!frame->reference()||frame->reference()->lifetime!=Lifetime::RegistrationMapAsset||
    !packet.ground_validity)return;
  const auto& source=frame->source();const auto& id=source.identity;
  if(id.source_id!=packet.metadata.source_id||id.epoch!=packet.metadata.epoch||
    id.sequence!=packet.metadata.sequence||id.source_ns!=packet.metadata.source_ros_ns||
    id.first_receipt_ros_ns!=packet.metadata.first_receipt_ros_ns||
    id.first_receipt_steady_ns!=packet.metadata.first_receipt_steady_ns||
    id.source_frame!=packet.metadata.frame_id||id.calibration_id.empty())
    throw std::invalid_argument("empirical_ground_source_identity_mismatch");
  const uint64_t pixels=uint64_t(source.raster_geometry.width)*source.raster_geometry.height;
  if(!pixels||pixels>4194304)return;
  auto support=std::make_shared<nav2_util::DirectEmpiricalGroundSupport>();
  support->metadata=packet.metadata;support->calibration_id=id.calibration_id;
  const auto& reference=*frame->reference();
  if(!sunhorizon_perception_runtime::ground_relative::registration_map_binding_current(reference,id))
    throw std::invalid_argument("empirical_ground_registration_binding_unavailable");
  support->geometry_identity=nav2_util::DirectEmpiricalGeometryIdentity{
    reference.map_asset.pcd_sha256,reference.map_asset.datum_id,reference.common_frame,
    reference.map_binding.alignment->reset_epoch(),id.source_frame,id.calibration_id};
  if(!support->geometry_identity->valid())
    throw std::invalid_argument("empirical_ground_geometry_identity_incomplete");
  support->sample_ids.reserve(source.points.size());
  for(const auto& point:source.points){
    if(point.sample_id>=pixels||(!support->sample_ids.empty()&&point.sample_id<=support->sample_ids.back()))return;
    support->sample_ids.push_back(point.sample_id);
  }
  support->self_support_complete=source.self_support_complete;
  support->original_projected_samples=source.original_projected_samples;
  support->self_support_quality_rejected_samples=source.self_support_quality_rejected_samples;
  for(const auto& point:source.self_support) {
    if(point.kind!=sunhorizon_perception_runtime::ground_relative::ReturnKind::Self ||
        point.sample_id>=pixels || point.measurement_unreliable ||
        (!support->self_samples.empty()&&point.sample_id<=support->self_samples.back().sample_id) ||
        std::binary_search(support->sample_ids.begin(),support->sample_ids.end(),point.sample_id))
      throw std::invalid_argument("empirical_self_support_identity_or_layout");
    support->self_samples.push_back({{float(point.position[0]),float(point.position[1]),float(point.position[2])},point.sample_id});
  }
  packet.empirical_ground_support=std::move(support);
}

// Only independently bounded classification supplies a marking envelope or
// depth-volume evidence. Measured-support segmentation keeps ordinary endpoint
// mark/clear semantics without acquiring physical bounds or volume credit.
inline void attach_ground_clearance(nav2_util::DirectCloud& packet,
    const sunhorizon_perception_runtime::ground_relative::ClassifiedObstacleFrame::ConstSharedPtr& frame,
    const sunhorizon_perception_runtime::ground_relative::GroundReferenceOwner::ConstSharedPtr& owner,
    const sunhorizon_perception_runtime::near_ground_clearance::Snapshot::ConstSharedPtr& clearance = {}) {
  if(!frame || !owner || !frame->ground().safety_qualified || owner->observation_only())return;
  const auto& id=frame->source().identity;const auto& m=packet.metadata;
  if(!packet.ground_qualified || !frame->reference() || id.source_id!=m.source_id || id.epoch!=m.epoch ||
      id.source_frame!=m.frame_id || id.sequence!=m.sequence || id.source_ns!=m.source_ros_ns ||
      id.first_receipt_ros_ns!=m.first_receipt_ros_ns || id.first_receipt_steady_ns!=m.first_receipt_steady_ns)
    throw std::invalid_argument("ground_clearance_packet_source_mismatch");
  using Basis=sunhorizon_perception_runtime::ground_relative::GroundQualification::Basis;
  if(frame->ground().basis!=Basis::BoundedReference)return;
  // Classifier total bound includes an isotropic point/transform bound as well
  // as plane error. Taking its maximum is conservative on every Cartesian axis.
  double error=0.;
  for(const auto index:frame->protected_hit_indices()) {
    if(index>=frame->error_bounds_m().size() || !std::isfinite(frame->error_bounds_m()[index]) ||
        frame->error_bounds_m()[index]<0.)throw std::invalid_argument("ground_mark_error_missing");
    error=std::max(error,frame->error_bounds_m()[index]);
  }
  packet.source_calibration_id=id.calibration_id;
  // The error is isotropic. Retain it in the original sensor frame rather
  // than asking public planar TF to resolve the private PCD height datum.
  packet.mark_error_reference_frame=owner->registration_map_asset()?id.source_frame:id.common_frame;
  packet.mark_error_bound_m=error;packet.mark_error_bound_qualified=true;
  // The legacy volume wire type carries a timed reference and axis-aligned
  // tiles. A static PCD asset has neither those clocks nor a public TF frame.
  // Endpoint classification supplies no fabricated volume/free-space proof.
  if(owner->registration_map_asset())return;
  if(!clearance)return;
  if(clearance->ground_frame!=frame || clearance->common_frame!=id.common_frame ||
      clearance->query_stamp_ns!=frame->source().to_common.common_ns)
    throw std::invalid_argument("ground_clearance_snapshot_owner_mismatch");
  if(!clearance->ground_qualified || !clearance->evaluation_complete)return;
  auto evidence=std::make_shared<nav2_util::DirectGroundClearance>();
  evidence->metadata=m;evidence->reference_frame=clearance->common_frame;
  evidence->clock_boot_id=owner->clock_boot_id();
  const auto& ref=*frame->reference();
  evidence->reference_provenance_id=ref.provenance_id;
  evidence->reference_calibration_id=ref.calibration_id;evidence->source_calibration_id=id.calibration_id;
  evidence->source_expiry_budget_ns=id.expiry_budget_ns;
  evidence->reference_source_ns=ref.source_ns;evidence->reference_first_receipt_ros_ns=ref.first_receipt_ros_ns;
  evidence->reference_first_receipt_steady_ns=ref.first_receipt_steady_ns;
  evidence->reference_expiry_budget_ns=ref.expiry_budget_ns;
  for(const auto& tile:clearance->tiles)
    if(tile.support_qualified && tile.state==sunhorizon_obstacle_coverage::spatial_visibility::CellState::kFree)
      evidence->tiles.push_back({tile.min_xy,tile.max_xy,tile.query_z_lower_m,tile.query_z_upper_m,true});
  // An empty result cannot revoke old occupancy or refresh an old proof.
  if(!evidence->tiles.empty())packet.ground_clearance=std::move(evidence);
}
}
