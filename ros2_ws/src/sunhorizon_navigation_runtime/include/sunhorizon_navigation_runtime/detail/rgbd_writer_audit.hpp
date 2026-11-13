#pragma once
#include "sunhorizon_navigation_runtime/detail/rgbd_projection_audit.hpp"
#include "sunhorizon_navigation_runtime/detail/writer_capture_state.hpp"
#include "sunhorizon_perception_runtime/runtime.hpp"
#include <nav2_util/direct_writer_audit.hpp>
namespace sunhorizon_navigation_runtime::detail {
using WriterBody=std::shared_ptr<const sunhorizon_perception_runtime::PerceptionOutput>;
struct RgbdWriterEvent {
  std::shared_ptr<WriterCaptureRequest> request;
  std::size_t slot{0};WriterBody body;
  std::shared_ptr<const nav2_util::DirectProjectionSnapshot> projection;
  nav2_util::DirectWriterHit hit;
};
using RgbdWriterQueue=BoundedWriterAuditQueue<RgbdWriterEvent>;
// This owner outlives producer delivery, ObservationBuffer replacement and even
// assembly destruction. It retains raw primary Image/Info, quality token,
// classification/reference and source-time body model through the original Body.
class RgbdWriterOwner final:public nav2_util::DirectWriterAuditOwner {
public:
  RgbdWriterOwner(WriterBody body,nav2_util::DirectObservationMetadata metadata,
      std::shared_ptr<WriterCaptureRequest> request,std::shared_ptr<RgbdWriterQueue> queue)
    :body_(std::move(body)),metadata_(std::move(metadata)),request_(std::move(request)),queue_(std::move(queue)){}
  void actualMark(const std::shared_ptr<const nav2_util::DirectProjectionSnapshot>& projection,
      const nav2_util::DirectWriterHit& hit)const noexcept override {
    // Unexpected Uncertain writes must remain auditable; marking policy is
    // enforced at the costmap input, never by hiding actual-writer evidence.
    if(hit.after_cost!=254||!projection||!projection->cloud||!projection->classes||
      hit.point_index>=projection->classes->size()||
      ((*projection->classes)[hit.point_index]!=uint8_t(nav2_util::DirectPointClass::Obstacle)&&
       (*projection->classes)[hit.point_index]!=uint8_t(nav2_util::DirectPointClass::Uncertain))||!body_||!body_->input||
      projection->source_cloud.get()!=&body_->frame.body_cloud||
      projection->source_cloud.owner_before(body_)||body_.owner_before(projection->source_cloud)||
      !nav2_util::directMetadataEqual(projection->metadata,metadata_))return;
    const auto& frame=projection->cloud->header.frame_id;
    if((frame!="map"&&frame!="odom")||(metadata_.source_id!="belly"&&metadata_.source_id!="chest"))return;
    const auto role=metadata_.source_id=="belly"?0u:1u;
    const auto slot=role+(frame=="odom"?2u:0u);
    if(!request_->claim(slot))return;
    // No allocation, serialization, lookup or waiting. Even queue contention is
    // terminal and observable for this request/consumer, never a silent retry.
    const bool queued=queue_->tryPush(RgbdWriterEvent{request_,slot,body_,projection,hit});
    if(queued) {
      // Worker may already have finished after push released its queue lock.
      // Never overwrite Written/FailedWrite with a late Queued transition.
      auto expected=WriterCaptureState::Claimed;
      request_->states[slot].compare_exchange_strong(expected,WriterCaptureState::Queued);
    }else request_->states[slot].store(WriterCaptureState::FailedQueue);
  }
private:
  const WriterBody body_;const nav2_util::DirectObservationMetadata metadata_;
  const std::shared_ptr<WriterCaptureRequest> request_;const std::shared_ptr<RgbdWriterQueue> queue_;
};
inline RgbdProjectionAuditFrame writer_projection_frame(const RgbdWriterEvent& event) {
  if(!event.body||!event.projection||!event.body->frame.classified_obstacles)
    throw std::invalid_argument("writer_audit_missing_original_owner");
  const auto& p=*event.projection;const auto& h=event.hit;const auto& input=event.body->input;
  if(!p.cloud||!p.classes||h.point_index>=p.classes->size()||h.point_index>=std::size_t(p.cloud->width)*p.cloud->height||
     h.after_cost!=254||((*p.classes)[h.point_index]!=uint8_t(nav2_util::DirectPointClass::Obstacle)&&
                       (*p.classes)[h.point_index]!=uint8_t(nav2_util::DirectPointClass::Uncertain))||
     !event.body->frame.body_model_diagnostic||!input||!input->depth||!input->depth_info)
    throw std::invalid_argument("writer_audit_hit_or_original_owner_invalid");
  const auto xyz=rgbd_audit_xyz(*p.cloud,rgbd_audit_layout(*p.cloud),h.point_index);
  for(std::size_t axis=0;axis<3;++axis)if(h.xyz[axis]!=double(xyz[axis]))
    throw std::invalid_argument("writer_audit_actual_hit_projection_mismatch");
  for(const auto* header:{&input->depth->header,&input->depth_info->header})
    if(header->frame_id!=p.metadata.frame_id||int64_t(header->stamp.sec)*1000000000+header->stamp.nanosec!=p.metadata.source_ros_ns)
      throw std::invalid_argument("writer_audit_primary_original_identity_mismatch");
  if(input->sequence!=p.metadata.sequence||input->role!=p.metadata.source_id)
    throw std::invalid_argument("writer_audit_primary_original_sequence_mismatch");
  return {event.projection->metadata,event.projection->source_cloud,
    event.body->frame.classified_obstacles,event.projection,0,
    event.body->frame.body_model_diagnostic,event.request->token,
    event.slot<2?RgbdProjectionConsumer::Global:RgbdProjectionConsumer::Local,
    std::shared_ptr<const std::vector<std::size_t>>(event.body,&event.body->frame.near_body_indices)};
}
inline nlohmann::json writer_hit_json(const RgbdWriterEvent& event) {
  const auto& h=event.hit;const auto& p=*event.projection;const auto& m=p.metadata;
  return {{"frame",p.cloud->header.frame_id},{"source",m.source_id},{"epoch",m.epoch},
    {"seq",m.sequence},{"source_ns",m.source_ros_ns},{"receipt_ros_ns",m.first_receipt_ros_ns},
    {"receipt_steady_ns",m.first_receipt_steady_ns},{"class",(*p.classes)[h.point_index]},
    {"request",event.request->token},{"slot",event.slot},{"count",1},{"grouped",false},
    {"legacy_near_log_cycle_available",h.cycle!=0},
    {"trigger","actual_markVoxelInMap"},{"stage","immediate_after_mark_before_footprint_and_master_merge"},{"not_final_master_cost_evidence",true},{"point_index",h.point_index},{"xyz",h.xyz},
    {"voxel",h.voxel},{"before",h.before_bits},{"after",h.after_bits},
    {"layer_before",h.before_cost},{"layer_after",h.after_cost},
    {"mark_threshold_reached",h.mark_threshold_reached},{"cycle",h.cycle},{"ros_ns",h.ros_ns},
    {"measurement_support",{{"available",h.measurement_support_available},
      {"ground_obstacle_uncertain",h.ground_obstacle_uncertain},{"self_support",h.self_support},
      {"frame_supported",h.frame_supported},
      {"persistence_confirmed",h.persistence_confirmed},{"previous_distinct_sequence",h.previous_distinct_sequence},
      {"previous_distinct_source_ns",h.previous_distinct_source_ns},
      {"scope","actual aggregation used by this layer write; no source lease renewal"}}},
    {"capture_region","same-cycle rotated padded-footprint bounding box plus half cell"},
    {"selection_robot_xy_yaw",h.selection_robot_xy_yaw},{"grid_size",h.grid_size},{"obstacle_min_range",h.obstacle_min_range},{"obstacle_max_range",h.obstacle_max_range},
    {"origin",h.origin},{"resolution",h.resolution},{"z_resolution",h.z_resolution},
    {"reinforcement_of_existing_254",h.before_cost==254},{"not_first_seed_proof",true},{"raw_native_pair_embedded",false},
    {"raw_native_pair_scope","CameraFrame retains primary depth/Info and exact quality token; native/dual-IR payloads still require exact bag association."}};
}
}
