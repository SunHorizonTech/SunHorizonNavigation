#include "sunhorizon_velocity_supervisor/spatial_coverage_decision_tracker.hpp"
#include <iostream>
#include <stdexcept>

using namespace sunhorizon_velocity_supervisor;
using namespace sunhorizon_obstacle_coverage;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(std::string(__func__)+":"+std::to_string(__LINE__)); } while(false)
namespace {
constexpr int64_t R=1791958258000000000LL, S=9000000000LL, M=1000000LL;
SpatialEvidenceTrackerConfig config() {
  return {"coverage_gid", "spatial-v1", "query", "boot", "base_footprint", "fusion_epoch",
      {{0,"camera_epoch","belly_optical"},{1,"camera_epoch","chest_optical"}}};
}
SpatialCoverageObservation spatial(int64_t offset=0, uint64_t seq=1, bool clear=true) {
  SpatialCoverageObservation v;
  v.schema_version=1;v.contract="spatial-v1";v.query_digest="query";v.clock_boot_id="boot";
  v.producer_epoch="fusion_epoch";v.frame_id="base_footprint";v.reason=clear?"clear":"unknown";
  v.header_ros_ns=R+offset+20*M;v.evaluation_steady_ns=S+offset+20*M;
  v.anchor_source_ros_ns=R+offset;v.anchor_frame_sequence=seq;v.evaluation_sequence=seq;v.input_generation=seq;
  v.evaluation_complete=true;v.total_cells=2;v.free_cells=clear?2:1;v.unknown_cells=clear?0:1;
  v.sources={{0,1,"camera_epoch","belly_optical",seq,R+offset,R+offset+10*M,S+offset+10*M,
      R+offset+300*M,S+offset+300*M,true}};
  return v;
}
CombinedCoverageEvidence decision(const SpatialCoverageObservation& v) {
  CombinedCoverageEvidence d;
  d.schema=2;d.disposition=v.unknown_cells?2:1;
  d.non_region_healthy=true;
  d.healthy=d.forward=d.near=d.disposition==1;d.header_ros_ns=v.header_ros_ns+5*M;
  d.frame_id=v.frame_id;d.profile="fusion_shadow";
  d.required_sources=d.healthy_sources={"airy","belly","chest"};return d;
}
bool observe(SpatialCoverageDecisionTracker& t, const SpatialCoverageObservation& v, int64_t offset=0) {
  return t.observe(decision(v),v,"coverage_gid",R+offset+30*M,S+offset+30*M);
}
void rgbd_mode_pins_upstream_without_weakening_spatial() {
  const auto rgbd=[](const auto& v){auto d=decision(v);d.profile="visual";
    d.required_sources=d.healthy_sources={"belly","chest"};return d;};
  SpatialCoverageDecisionTracker t(config(),"rgbd-only-v1");auto v=spatial();
  CHECK(t.observe(rgbd(v),v,"coverage_gid",R+30*M,S+30*M));
  CHECK(t.evaluate(R+40*M,S+40*M).valid && t.evaluate(R+40*M,S+40*M).disposition==1);
  v=spatial(40*M,2,false);CHECK(t.observe(rgbd(v),v,"coverage_gid",R+70*M,S+70*M));
  CHECK(t.evaluate(R+80*M,S+80*M).valid && t.evaluate(R+80*M,S+80*M).disposition==2);
  // Expired measurement remains a sticky fault, even with a fresh heartbeat.
  CHECK(!t.evaluate(R+341*M,S+341*M).valid);
  CHECK(!t.observe(rgbd(spatial(350*M,3)),spatial(350*M,3),"coverage_gid",R+380*M,S+380*M));
  for(int mutation=0;mutation<5;++mutation) {
    SpatialCoverageDecisionTracker bad(config(),"rgbd-only-v1");auto d=rgbd(spatial());
    if(mutation==0)d.profile="fusion_shadow";
    if(mutation==1)d.required_sources.push_back("airy");
    if(mutation==2)d.healthy_sources={"belly"};
    if(mutation==3)d.healthy_sources.push_back("chest");
    if(mutation==4){d.disposition=0;d.non_region_healthy=false;d.healthy=d.forward=d.near=false;}
    CHECK(!bad.observe(d,spatial(),"coverage_gid",R+30*M,S+30*M));
    CHECK(!bad.evaluate(R+40*M,S+40*M).valid);
  }
  SpatialCoverageDecisionTracker legacy(config());
  CHECK(!legacy.observe(rgbd(spatial()),spatial(),"coverage_gid",R+30*M,S+30*M));
  bool rejected=false;try{SpatialCoverageDecisionTracker bad(config(),"any");}catch(const std::invalid_argument&){rejected=true;}
  CHECK(rejected);
}
void normal_and_upstream() {
  SpatialCoverageDecisionTracker t(config());CHECK(observe(t,spatial()));
  CHECK(t.evaluate(R+40*M,S+40*M).disposition==1);
  auto wait=spatial(40*M,2,false);CHECK(observe(t,wait,40*M));
  CHECK(t.evaluate(R+80*M,S+80*M).valid);CHECK(t.evaluate(R+80*M,S+80*M).disposition==2);
  auto d=decision(wait);d.non_region_healthy=false;d.disposition=0;
  CHECK(!t.observe(d,wait,"coverage_gid",R+90*M,S+90*M));
  CHECK(t.evaluate(R+95*M,S+95*M).reason=="spatial_upstream_hard_fault");
  CHECK(!observe(t,spatial(100*M,3),100*M));
}
void no_latest_join() {
  for(int mutation=0;mutation<8;++mutation) {
    SpatialCoverageDecisionTracker t(config());auto v=spatial();auto d=decision(v);
    switch(mutation) {
      case 0:d.schema=1;break;case 1:++d.frame_sequence;break;case 2:++d.evaluation_sequence;break;
      case 3:++d.source_ros_ns;break;case 4:d.contract="other";break;
      case 5:d.disposition=2;d.healthy=d.forward=d.near=false;break;
      case 6:d.header_ros_ns=v.header_ros_ns-1;break;case 7:d.complete=true;break;
    }
    CHECK(!t.observe(d,v,"coverage_gid",R+30*M,S+30*M));
    CHECK(!t.evaluate(R+40*M,S+40*M).valid);
  }
}
void original_source_expiry() {
  SpatialCoverageDecisionTracker t(config());auto v=spatial();
  v.sources.push_back({1,1,"camera_epoch","chest_optical",1,R-200*M,R-190*M,S-190*M,
      R+100*M,S+100*M,true});
  CHECK(observe(t,v));auto d=decision(v);d.header_ros_ns=R+90*M;
  CHECK(t.observe(d,v,"coverage_gid",R+95*M,S+95*M));
  CHECK(!t.evaluate(R+101*M,S+101*M).valid); // Publisher is fresh; contributing chest is not.
  CHECK(!observe(t,spatial(110*M,2),110*M));
}
void unknown_is_not_clear_and_occupied_latches() {
  SpatialCoverageDecisionTracker t(config());auto v=spatial(0,1,false);CHECK(observe(t,v));
  CHECK(t.evaluate(R+40*M,S+40*M).disposition==2);
  v=spatial(40*M,2);v.free_cells=1;v.occupied_cells=1;v.sources[0].contribution=3;
  auto d=decision(v);d.disposition=0;d.healthy=d.forward=d.near=false;
  CHECK(!t.observe(d,v,"coverage_gid",R+70*M,S+70*M));
  CHECK(!observe(t,spatial(80*M,3),80*M));
}
void fresh_new_frame_cannot_hide_run64_expiry_gap() {
  SpatialCoverageDecisionTracker t(config());auto old=spatial(0,989);
  old.header_ros_ns=R+264440573;old.evaluation_steady_ns=S+264440573;
  old.sources[0].first_receipt_ros_ns=R+179124830;
  old.sources[0].first_receipt_steady_ns=S+179124830;
  CHECK(t.observe(decision(old),old,"coverage_gid",R+270000000,S+270000000));
  auto next=spatial(100123000,992);
  next.header_ros_ns=R+328683937;next.evaluation_steady_ns=S+328683937;
  next.sources[0].first_receipt_ros_ns=R+258557258;
  next.sources[0].first_receipt_steady_ns=S+258557258;
  CHECK(next.sources[0].valid_until_ros_ns>R+328683937);
  CHECK(old.sources[0].valid_until_ros_ns<R+328683937);
  auto d=decision(next);d.header_ros_ns=next.header_ros_ns;
  CHECK(!t.observe(d,next,"coverage_gid",R+328683937,S+328683937));
  CHECK(!t.evaluate(R+328683938,S+328683938).valid);
  CHECK(!observe(t,spatial(400*M,995),400*M));
}
void coverage_recovery_never_resets_the_final_tracker() {
  SpatialEvidenceTracker coverage(config(),SpatialRecoveryPolicy::RecoveryOnNewAnchor);
  SpatialCoverageDecisionTracker final(config());
  const auto first=spatial();
  CHECK(coverage.observe(first,"coverage_gid",R+30*M,S+30*M));
  CHECK(observe(final,first));
  const auto recovered=spatial(280*M,2);
  CHECK(coverage.observe(recovered,"coverage_gid",R+310*M,S+310*M));
  CHECK(coverage.evaluate(R+311*M,S+311*M).valid);
  CHECK(!observe(final,recovered,280*M)); // Callback detects the missed interval.
  CHECK(final.evaluate(R+311*M,S+311*M).reason=="spatial_frozen_evidence_expired");
  const auto latest=spatial(330*M,3);
  CHECK(coverage.observe(latest,"coverage_gid",R+360*M,S+360*M));
  CHECK(!observe(final,latest,330*M));
  CHECK(!final.evaluate(R+361*M,S+361*M).valid);
  // The node stages this separate tracker ONLY in its explicit idle reset;
  // merely receiving recovered coverage never replaces the live tracker.
  SpatialCoverageDecisionTracker proposed(config());
  CHECK(observe(proposed,latest,330*M));
  CHECK(proposed.evaluate(R+361*M,S+361*M).valid);
}
}
void low_speed_measurement_age_preserves_heartbeat_and_stop() {
  auto c=config();c.maximum_age_ns=500*M;auto v=spatial();
  v.header_ros_ns=R+350*M;v.evaluation_steady_ns=S+350*M;
  v.sources[0].valid_until_ros_ns=R+500*M;v.sources[0].valid_until_steady_ns=S+500*M;
  SpatialCoverageDecisionTracker slow(c);CHECK(observe(slow,v,330*M));
  CHECK(slow.evaluate(R+500*M,S+500*M).valid);
  CHECK(!slow.evaluate(R+501*M,S+501*M).valid);
  SpatialCoverageDecisionTracker heartbeat(c);v=spatial();
  v.sources[0].valid_until_ros_ns=R+500*M;v.sources[0].valid_until_steady_ns=S+500*M;
  CHECK(observe(heartbeat,v));
  CHECK(!heartbeat.evaluate(R+340*M,S+340*M).valid); // Transport still 300 ms.
}
int main() {
  low_speed_measurement_age_preserves_heartbeat_and_stop();
  rgbd_mode_pins_upstream_without_weakening_spatial();
  normal_and_upstream();no_latest_join();original_source_expiry();unknown_is_not_clear_and_occupied_latches();
  fresh_new_frame_cannot_hide_run64_expiry_gap();
  coverage_recovery_never_resets_the_final_tracker();
  std::cout << "spatial combined decision checks passed\n";
}
