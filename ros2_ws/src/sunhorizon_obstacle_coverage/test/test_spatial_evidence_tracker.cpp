#include "sunhorizon_obstacle_coverage/spatial_evidence_tracker.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace sunhorizon_obstacle_coverage;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(std::string(__func__)+":"+std::to_string(__LINE__)+": " #x); } while (false)
namespace {
constexpr std::int64_t R = 1791958258000000000LL, S = 9000000000LL, M = 1000000LL;
SpatialEvidenceTrackerConfig config() {
  return {"authenticated_gid", "spatial-v1", "fixed_query_sha", "boot", "base_footprint", "fusion_epoch",
          {{0,"camera_epoch","belly_optical"},{1,"camera_epoch","chest_optical"},{2,"airy_epoch","airy_lidar"}}};
}
SpatialSourceObservation source(std::uint8_t id, std::int64_t offset = 0) {
  return {id, static_cast<std::uint8_t>(id == 2 ? 0 : 1), id == 2 ? "airy_epoch" : "camera_epoch",
          id == 0 ? "belly_optical" : id == 1 ? "chest_optical" : "airy_lidar", 1,
          R + offset, R + offset + 10*M, S + offset + 10*M,
          R + offset + 300*M, S + offset + 300*M, true};
}
SpatialCoverageObservation observation(std::int64_t offset = 0, std::uint64_t generation = 1) {
  SpatialCoverageObservation v;
  v.schema_version=1; v.contract="spatial-v1"; v.query_digest="fixed_query_sha";
  v.producer_epoch="fusion_epoch";v.clock_boot_id="boot";v.frame_id="base_footprint";v.reason="clear";
  v.header_ros_ns=R+offset+20*M;v.anchor_source_ros_ns=R+offset;v.evaluation_steady_ns=S+offset+20*M;
  v.anchor_frame_sequence=generation;v.evaluation_sequence=generation;v.input_generation=generation;
  v.evaluation_complete=true;v.total_cells=10;v.free_cells=10;v.sources={source(0,offset)};
  v.sources[0].frame_sequence=generation;return v;
}
bool accept(SpatialEvidenceTracker& t, const SpatialCoverageObservation& v, std::int64_t offset=0) {
  return t.observe(v,"authenticated_gid",R+offset+30*M,S+offset+30*M);
}
void clear_wait_occupied_and_incomplete() {
  SpatialEvidenceTracker t(config());auto v=observation();CHECK(accept(t,v));
  const auto x=t.evaluate(R+40*M,S+40*M);CHECK(x.valid && x.disposition==1 && x.source_ros_ns==R);
  CHECK(x.receipt_steady_ns==S+30*M && x.frame_sequence==1 && x.valid_until_ros_ns==R+300*M);
  auto w=observation(40*M,2);w.free_cells=4;w.unknown_cells=6;w.reason="unknown";
  CHECK(accept(t,w,40*M));CHECK(t.evaluate(R+80*M,S+80*M).disposition==2);
  auto occupied=observation(80*M,3);occupied.free_cells=4;occupied.unknown_cells=5;occupied.occupied_cells=1;
  occupied.sources[0].contribution=3;CHECK(!accept(t,occupied,80*M));
  CHECK(t.evaluate(R+120*M,S+120*M).reason=="spatial_occupied");
  CHECK(!accept(t,observation(120*M,4),120*M));  // No reset through newer clear.
  SpatialEvidenceTracker all_unknown(config());w=observation();w.free_cells=0;w.unknown_cells=10;w.sources[0].contribution=0;
  CHECK(accept(all_unknown,w));CHECK(all_unknown.evaluate(R+40*M,S+40*M).disposition==2);
  SpatialEvidenceTracker incomplete(config());w.evaluation_complete=false;CHECK(!accept(incomplete,w));
}
void duplicate_never_refreshes() {
  SpatialEvidenceTracker t(config());auto v=observation();CHECK(accept(t,v));
  CHECK(t.observe(v,"authenticated_gid",R+290*M,S+290*M));
  auto x=t.evaluate(R+300*M,S+300*M);CHECK(x.valid && x.receipt_steady_ns==S+30*M);
  CHECK(!t.observe(v,"authenticated_gid",R+300*M+1,S+300*M+1));
  CHECK(t.evaluate(R+301*M,S+301*M).reason=="spatial_frozen_evidence_expired");
  SpatialEvidenceTracker changed(config());CHECK(accept(changed,v));
  v.header_ros_ns+=M;v.evaluation_steady_ns+=M;CHECK(!changed.observe(v,"authenticated_gid",R+40*M,S+40*M));
  SpatialEvidenceTracker new_after_gap(config());CHECK(accept(new_after_gap,observation()));
  auto new_wait=observation(280*M,2);new_wait.free_cells=0;new_wait.unknown_cells=10;
  new_wait.sources[0].contribution=0;
  CHECK(!accept(new_after_gap,new_wait,280*M));
  CHECK(new_after_gap.evaluate(R+311*M,S+311*M).reason=="spatial_frozen_evidence_expired");
  SpatialEvidenceTracker new_at_deadline(config());CHECK(accept(new_at_deadline,observation()));
  CHECK(accept(new_at_deadline,observation(270*M,2),270*M));
  CHECK(new_at_deadline.evaluate(R+301*M,S+301*M).valid);
}
void immutable_anchor_and_source() {
  for (int change=0;change<9;++change) {
    SpatialEvidenceTracker t(config());auto v=observation();CHECK(accept(t,v));
    switch(change) {
      case 0:v.unknown_cells=1;v.free_cells=9;break;
      case 1:++v.input_generation;break;
      case 2:++v.evaluation_sequence;break;
      case 3:++v.sources[0].first_receipt_steady_ns;break;
      case 4:--v.sources[0].valid_until_steady_ns;break;
      case 5:v.sources[0].geometry_valid=false;break;
      case 6:v.sources[0].contribution=0;break;
      case 7:v.reason="changed";break;
      case 8:++v.anchor_source_ros_ns;break;
    }
    CHECK(!t.observe(v,"authenticated_gid",R+40*M,S+40*M));
  }
  SpatialEvidenceTracker t(config());auto v=observation();v.sources.push_back(source(1,-20*M));CHECK(accept(t,v));
  auto n=observation(40*M,2);n.sources.push_back(v.sources[1]);n.sources[1].contribution=0;
  CHECK(accept(t,n,40*M));  // Query-specific contribution may change, measurement may not.
  n=observation(80*M,3);n.sources.push_back(v.sources[1]);
  n.sources[1].first_receipt_ros_ns+=M;n.sources[1].first_receipt_steady_ns+=M;
  CHECK(!accept(t,n,80*M));CHECK(t.evaluate(R+120*M,S+120*M).reason=="spatial_source_same_sequence_mutated");
}
void identity_and_highwater() {
  for (int change=0;change<11;++change) {
    SpatialEvidenceTracker t(config());auto v=observation();CHECK(accept(t,v));auto n=observation(40*M,2);
    switch(change) {
      case 0:n.contract="other";break;case 1:n.query_digest="other";break;case 2:n.clock_boot_id="other";break;
      case 3:n.frame_id="other";break;case 4:n.producer_epoch="other";break;case 5:n.sources[0].producer_epoch="other";break;
      case 6:n.evaluation_sequence=1;break;case 7:n.input_generation=1;break;
      case 8:n.sources[0].frame_sequence=1;break;case 9:n.schema_version=2;break;case 10:n.sources[0].frame_id="other";break;
    }
    CHECK(!accept(t,n,40*M));
  }
  SpatialEvidenceTracker owner(config());CHECK(!owner.observe(observation(),"other",R+30*M,S+30*M));
  SpatialEvidenceTracker rollback(config());CHECK(accept(rollback,observation(40*M,2),40*M));CHECK(!accept(rollback,observation(),80*M));
  SpatialEvidenceTracker omitted(config());auto v=observation();v.sources.push_back(source(1,-20*M));CHECK(accept(omitted,v));
  CHECK(accept(omitted,observation(40*M,2),40*M));auto n=observation(80*M,3);n.sources.push_back(v.sources[1]);
  --n.sources[1].source_ros_ns;CHECK(!accept(omitted,n,80*M));
}
void sources_expiry_and_qualification() {
  SpatialEvidenceTracker optional(config());auto v=observation();auto old=source(1,-400*M);old.contribution=0;old.geometry_valid=false;
  v.sources.push_back(old);CHECK(accept(optional,v));CHECK(optional.evaluate(R+40*M,S+40*M).valid);
  SpatialEvidenceTracker arrived_expired(config());old.first_receipt_ros_ns=R;old.first_receipt_steady_ns=S;
  v.sources.back()=old;CHECK(accept(arrived_expired,v));
  for (int change=0;change<13;++change) {
    SpatialEvidenceTracker t(config());v=observation();
    switch(change) {
      case 0:v.sources[0].geometry_valid=false;break;
      case 1:v.sources[0].valid_until_ros_ns+=M;v.sources[0].valid_until_steady_ns+=M;break;
      case 2:v.sources[0].valid_until_steady_ns+=1;break;
      case 3:v.sources[0].first_receipt_ros_ns=v.sources[0].source_ros_ns-1;break;
      case 4:v.sources[0].source_ros_ns+=1;break;
      case 5:v.sources[0].first_receipt_steady_ns=v.evaluation_steady_ns+1;break;
      case 6:v.sources[0].contribution=4;break;
      case 7:v.sources.push_back(source(0));break;
      case 8:v.sources[0]=source(1);break;
      case 9:v.sources.push_back(source(2));v.sources.back().contribution=1;break;
      case 10:v.sources.push_back(source(1,-400*M));break;
      case 11:v.sources[0].contribution=0;break;
      case 12:v.sources[0].contribution=3;break;
    }
    CHECK(!accept(t,v));
  }
  SpatialEvidenceTracker limited(config());v=observation();auto c=source(1,-100*M);v.sources.push_back(c);CHECK(accept(limited,v));
  CHECK(limited.evaluate(R+200*M,S+200*M).valid);
  CHECK(!limited.evaluate(R+200*M+1,S+200*M+1).valid);  // Optional used Free is a real dependency.
  SpatialEvidenceTracker geometry(config());v=observation();c=source(1);c.geometry_valid=false;v.sources.push_back(c);CHECK(!accept(geometry,v));
}
void invalid_clocks_and_limits() {
  SpatialEvidenceTracker ros_back(config());CHECK(accept(ros_back,observation()));CHECK(!ros_back.evaluate(R+29*M,S+31*M).valid);
  SpatialEvidenceTracker steady_back(config());CHECK(accept(steady_back,observation()));CHECK(!steady_back.evaluate(R+31*M,S+29*M).valid);
  SpatialEvidenceTracker skew(config());CHECK(accept(skew,observation()));CHECK(!skew.evaluate(R+30*M,S+131*M).valid);
  for (int change=0;change<9;++change) {
    SpatialEvidenceTracker t(config());auto v=observation();
    switch(change) {
      case 0:v.total_cells=0;break;case 1:v.total_cells=4097;v.free_cells=4097;break;
      case 2:v.free_cells=std::numeric_limits<std::uint32_t>::max();break;
      case 3:v.anchor_frame_sequence=0;break;case 4:v.input_generation=0;break;
      case 5:v.evaluation_steady_ns=S+31*M;break;case 6:v.header_ros_ns=R+31*M;break;
      case 7:v.reason=std::string(513,'x');break;case 8:v.sources.clear();break;
    }
    CHECK(!accept(t,v));
  }
  SpatialEvidenceTracker hard(config());CHECK(accept(hard,observation()));hard.invalidate("upstream_stale");
  CHECK(hard.evaluate(R+40*M,S+40*M).reason=="upstream_stale");
  bool threw=false;auto c=config();c.source_identities.erase(c.source_identities.begin());
  try {SpatialEvidenceTracker invalid(c);} catch(const std::invalid_argument&) {threw=true;} CHECK(threw);
}
void typed_motion_unavailability_does_not_accept_or_reset() {
  SpatialEvidenceTracker t(config());CHECK(accept(t,observation()));
  CHECK(t.delivery_result().accepted());
  CHECK(!accept(t,observation(280*M,2),280*M));
  CHECK(t.delivery_result().state==SpatialDeliveryState::MotionUnavailable);
  CHECK(!accept(t,observation(330*M,3),330*M));
  CHECK(!t.evaluate(R+370*M,S+370*M).valid);
  SpatialEvidenceTracker expired(config());auto old=observation();
  old.evaluation_complete=false;old.reason="source_expired_before_egress";
  CHECK(!accept(expired,old,301*M));
  CHECK(expired.delivery_result().state==SpatialDeliveryState::MotionUnavailable);
  SpatialEvidenceTracker malformed(config());old.schema_version=2;
  CHECK(!accept(malformed,old,301*M));
  CHECK(malformed.delivery_result().state==SpatialDeliveryState::InternalFault);
  SpatialEvidenceTracker not_expired(config());old=observation();old.evaluation_complete=false;
  old.reason="source_expired_before_egress";CHECK(!accept(not_expired,old));
  CHECK(not_expired.delivery_result().state==SpatialDeliveryState::InternalFault);
  SpatialEvidenceTracker geometry(config());old=observation();old.sources[0].geometry_valid=false;
  CHECK(!accept(geometry,old));CHECK(geometry.delivery_result().state==SpatialDeliveryState::InternalFault);
  for(int mutation=0;mutation<5;++mutation){
    SpatialEvidenceTracker stopped(config());CHECK(accept(stopped,observation()));
    CHECK(!accept(stopped,observation(280*M,2),280*M));
    CHECK(stopped.delivery_result().state==SpatialDeliveryState::MotionUnavailable);
    auto next=observation(330*M,3);
    switch(mutation){
      case 0:next.producer_epoch="replaced";break;
      case 1:next.total_cells=0;break;
      case 2:next.anchor_frame_sequence=1;break;
      case 3:next.sources[0].first_receipt_ros_ns=next.sources[0].source_ros_ns-1;break;
      case 4:CHECK(!stopped.evaluate(R+309*M,S+311*M).valid);break;
    }
    CHECK(!accept(stopped,next,330*M));
    CHECK(stopped.delivery_result().state==SpatialDeliveryState::InternalFault);
  }
}
void coverage_recovers_only_deadlines_on_complete_fresh_new_anchors() {
  const auto policy=SpatialRecoveryPolicy::RecoveryOnNewAnchor;
  SpatialEvidenceTracker t(config(),policy);const auto old=observation();CHECK(accept(t,old));
  CHECK(!t.observe(old,"authenticated_gid",R+301*M,S+301*M));
  CHECK(t.delivery_result().state==SpatialDeliveryState::MotionUnavailable);
  CHECK(!t.observe(old,"authenticated_gid",R+302*M,S+302*M)); // No repeated-frame renewal.
  auto wait=observation(280*M,2);wait.free_cells=0;wait.unknown_cells=10;wait.sources[0].contribution=0;
  CHECK(accept(t,wait,280*M));
  auto result=t.evaluate(R+311*M,S+311*M);
  CHECK(result.valid && result.disposition==2 && result.frame_sequence==2);
  CHECK(result.receipt_steady_ns==S+310*M && result.valid_until_ros_ns==R+580*M);
  CHECK(accept(t,observation(330*M,3),330*M));
  CHECK(t.evaluate(R+361*M,S+361*M).disposition==1);

  // Recovery also works when the new callback detects the old missed interval,
  // with no intervening timer. Default sticky-policy tests above stay unchanged.
  SpatialEvidenceTracker gap(config(),policy);CHECK(accept(gap,old));
  CHECK(accept(gap,observation(280*M,2),280*M));
  CHECK(gap.evaluate(R+311*M,S+311*M).valid);
  gap.invalidate_deadline("spatial_snapshot_expired_after_tf");
  CHECK(!gap.evaluate(R+312*M,S+312*M).valid);
  CHECK(accept(gap,observation(330*M,3),330*M));

  SpatialEvidenceTracker expired(config(),policy);
  auto late=old;late.evaluation_complete=false;late.reason="source_expired_before_egress";
  CHECK(!accept(expired,late,301*M));
  CHECK(accept(expired,observation(330*M,2),330*M));

  // A new belly cannot refresh a reused contributing chest measurement.
  SpatialEvidenceTracker chest(config(),policy);auto first=old;first.sources.push_back(source(1));
  CHECK(accept(chest,first));auto next=observation(280*M,2);next.sources.push_back(first.sources[1]);
  CHECK(!accept(chest,next,280*M));
  CHECK(chest.delivery_result().state==SpatialDeliveryState::MotionUnavailable);
  next=observation(330*M,3);next.sources.push_back(source(1,330*M));
  next.sources.back().frame_sequence=2;
  CHECK(accept(chest,next,330*M));

  for(int hard=0;hard<4;++hard) {
    SpatialEvidenceTracker stopped(config(),policy);CHECK(accept(stopped,old));
    CHECK(!stopped.evaluate(R+301*M,S+301*M).valid);
    auto bad=observation(280*M,2);
    switch(hard) {
      case 0:bad.free_cells=9;bad.occupied_cells=1;bad.sources[0].contribution=3;break;
      case 1:bad.producer_epoch="replaced";break;
      case 2:bad.evaluation_complete=false;break;
      case 3:stopped.invalidate_motion("explicit_non_deadline_stop");break;
    }
    CHECK(!accept(stopped,bad,280*M));
    CHECK(!accept(stopped,observation(330*M,3),330*M));
    CHECK(!stopped.evaluate(R+361*M,S+361*M).valid);
    if(hard==0)CHECK(stopped.delivery_result().reason=="spatial_occupied");
  }
}
void coverage_recovers_occupancy_only_from_new_complete_clear() {
  const auto policy=SpatialRecoveryPolicy::CoverageObservations;
  auto occupied=observation();occupied.free_cells=9;occupied.occupied_cells=1;
  occupied.sources[0].contribution=3;
  SpatialEvidenceTracker coverage(config(),policy),supervisor(config());
  CHECK(!accept(coverage,occupied));CHECK(!accept(supervisor,occupied));
  CHECK(!accept(coverage,occupied,10*M)); // Identical repeats cannot recover.
  auto unknown=observation(40*M,2);unknown.free_cells=9;unknown.unknown_cells=1;
  CHECK(!accept(coverage,unknown,40*M));
  CHECK(coverage.evaluate(R+80*M,S+80*M).reason=="spatial_occupied");
  // Fresh belly plus an expired contributing chest cannot recover occupancy.
  auto stale=observation(320*M,3);stale.sources.push_back(source(1));
  CHECK(!accept(coverage,stale,320*M));
  auto clear=observation(360*M,4);
  CHECK(accept(coverage,clear,360*M));
  auto result=coverage.evaluate(R+400*M,S+400*M);
  CHECK(result.valid && result.disposition==1 && result.frame_sequence==4);
  CHECK(result.valid_until_ros_ns==clear.sources[0].valid_until_ros_ns);
  CHECK(!accept(supervisor,clear,360*M)); // Final motion still needs operator reset.
  CHECK(supervisor.evaluate(R+400*M,S+400*M).reason=="spatial_occupied");
  for(int fault=0;fault<4;++fault) {
    SpatialEvidenceTracker hard(config(),policy);CHECK(!accept(hard,occupied));
    auto next=observation(40*M,2);
    if(fault==0)hard.invalidate_motion("operator_stop");
    if(fault==1)next.producer_epoch="changed";
    if(fault==2)next.evaluation_complete=false;
    if(fault==3){next=occupied;next.free_cells=10;next.occupied_cells=0;next.sources[0].contribution=1;}
    CHECK(!accept(hard,next,40*M));
    CHECK(!accept(hard,observation(80*M,3),80*M));
    CHECK(!hard.evaluate(R+120*M,S+120*M).valid);
  }
  // A missed deadline followed by occupied data must require Clear, not Wait.
  SpatialEvidenceTracker gap(config(),policy);CHECK(accept(gap,observation()));
  CHECK(!gap.evaluate(R+301*M,S+301*M).valid);
  occupied=observation(320*M,2);occupied.free_cells=9;occupied.occupied_cells=1;
  occupied.sources[0].contribution=3;CHECK(!accept(gap,occupied,320*M));
  unknown=observation(360*M,3);unknown.free_cells=9;unknown.unknown_cells=1;
  CHECK(!accept(gap,unknown,360*M));
  CHECK(accept(gap,observation(400*M,4),400*M));
}

void low_speed_deadline_is_original_bounded_and_opt_in() {
  auto c=config();c.maximum_age_ns=500*M;
  bool airy_rejected=false;
  try {SpatialEvidenceTracker forbidden(c);}catch(const std::invalid_argument&){airy_rejected=true;}
  CHECK(airy_rejected); // A camera-only lease cannot relax Airy's original age.
  c.source_identities.pop_back();
  auto v=observation();v.header_ros_ns=R+350*M;v.evaluation_steady_ns=S+350*M;
  v.sources[0].valid_until_ros_ns=R+500*M;v.sources[0].valid_until_steady_ns=S+500*M;
  SpatialEvidenceTracker strict(config());CHECK(!accept(strict,v,330*M));
  SpatialEvidenceTracker slow(c);CHECK(accept(slow,v,330*M));
  CHECK(slow.evaluate(R+500*M,S+500*M).valid);
  CHECK(!slow.evaluate(R+500*M+1,S+500*M+1).valid);
  CHECK(!accept(slow,observation(510*M,2),510*M)); // Final latch stays sticky.
  SpatialEvidenceTracker repeat(c);CHECK(accept(repeat,v,330*M));
  CHECK(accept(repeat,v,450*M)); // Repeated result cannot renew original expiry.
  CHECK(!repeat.evaluate(R+501*M,S+501*M).valid);
  auto early=v;early.sources[0].valid_until_ros_ns=R+400*M;
  early.sources[0].valid_until_steady_ns=S+400*M;
  SpatialEvidenceTracker shorter(c);CHECK(accept(shorter,early,330*M));
  CHECK(!shorter.evaluate(R+401*M,S+401*M).valid);
  c.maximum_age_ns=1000*M;
  SpatialEvidenceTracker doubled(c);
  v.sources[0].valid_until_ros_ns=R+1000*M;v.sources[0].valid_until_steady_ns=S+1000*M;
  CHECK(accept(doubled,v,350*M));
  CHECK(doubled.evaluate(R+1000*M,S+1000*M).valid);
  CHECK(!doubled.evaluate(R+1000*M+1,S+1000*M+1).valid);
  for (auto invalid : {0LL,300000001LL,500000001LL,1000000001LL}) {
    c.maximum_age_ns=invalid;bool threw=false;
    try {SpatialEvidenceTracker bad(c);}catch(const std::invalid_argument&){threw=true;}
    CHECK(threw);
  }
}
}
int main() {
  low_speed_deadline_is_original_bounded_and_opt_in();
  coverage_recovers_occupancy_only_from_new_complete_clear();
  clear_wait_occupied_and_incomplete();duplicate_never_refreshes();immutable_anchor_and_source();
  identity_and_highwater();sources_expiry_and_qualification();invalid_clocks_and_limits();
  typed_motion_unavailability_does_not_accept_or_reset();
  coverage_recovers_only_deadlines_on_complete_fresh_new_anchors();
  std::cout << "spatial evidence tracker checks passed\n";
}
