#include "sunhorizon_obstacle_coverage/belly_health_policy.hpp"
#include "sunhorizon_obstacle_coverage/spatial_evidence_tracker.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace sunhorizon_obstacle_coverage;
#define CHECK(x) do {if (!(x)) throw std::runtime_error(std::to_string(__LINE__)+": " #x);} while(false)
constexpr int64_t R=10000000000LL,S=9000000000LL,M=1000000LL;
SpatialEvidenceTrackerConfig config() {
  return {"gid","spatial-v1","query","boot","base","producer",{{0,"camera","belly"}}};
}
SpatialCoverageObservation evidence() {
  SpatialCoverageObservation v;
  v.schema_version=1;v.contract="spatial-v1";v.query_digest="query";
  v.producer_epoch="producer";v.clock_boot_id="boot";v.frame_id="base";v.reason="clear";
  v.header_ros_ns=R+20*M;v.anchor_source_ros_ns=R;v.evaluation_steady_ns=S+20*M;
  v.anchor_frame_sequence=v.evaluation_sequence=v.input_generation=1;
  v.evaluation_complete=true;v.total_cells=v.free_cells=10;
  v.sources={{0,1,"camera","belly",1,R,R+10*M,S+10*M,R+300*M,S+300*M,true}};
  return v;
}
bool healthy(const SourceObservation& belly,double now=10.04) {
  CoverageFreshness limits;limits.belly_sec=.35;
  CoverageInputs input;input.now_sec=now;input.chest={true,true,now-.01,now-.005};input.belly=belly;
  return CoverageEvaluator(CoverageProfile::kVisual,limits).evaluate(input).healthy;
}
int main() {
  const SourceObservation old_processed{true,true,9.5,9.51};
  const BellyHealthPolicy direct_spatial(true,true);
  const auto source=direct_spatial.source_stamp(10.02,.02,.52);
  CHECK(source==10.0 && source!=10.02);
  const SourceObservation raw{true,true,source,10.02};
  CHECK(!direct_spatial.require_processed(true));CHECK(healthy(raw));
  CHECK(!healthy(require_processed_observation(raw,old_processed,10.04,.35,.30)));
  // No other mode loses the processed observer or original oldest-source rule.
  for(const auto flags:{std::pair<bool,bool>{false,false},{true,false},{false,true}}) {
    BellyHealthPolicy legacy(flags.first,flags.second);
    CHECK(legacy.require_processed(true));CHECK(!legacy.require_processed(false));
    CHECK(legacy.source_stamp(10.02,.02,.52)==9.5);
  }
  CHECK(!healthy({true,true,10.,9.5})); // Original raw status receipt cannot be replaced.
  CHECK(!healthy({true,true,9.5,10.02}));CHECK(!healthy({true,false,10.,10.02}));CHECK(!healthy({}));
  CHECK(direct_spatial.source_stamp(10.02,std::numeric_limits<double>::quiet_NaN(),.01)==0.);
  CHECK(direct_spatial.source_stamp(10.02,.02,std::numeric_limits<double>::quiet_NaN())==10.);
  SpatialEvidenceTracker tracker(config());auto v=evidence();
  CHECK(tracker.observe(v,"gid",R+30*M,S+30*M));
  CHECK(tracker.evaluate(R+40*M,S+40*M).disposition==1);
  CHECK(tracker.observe(v,"gid",R+290*M,S+290*M));
  // A still healthy raw stream cannot renew the source lease or duplicate receipt.
  CHECK(healthy({true,true,10.30,10.31},10.32));
  CHECK(!tracker.evaluate(R+301*M,S+301*M).valid);
  SpatialEvidenceTracker missing(config());CHECK(!missing.evaluate(R+40*M,S+40*M).valid);
  SpatialEvidenceTracker unknown(config());v=evidence();v.free_cells=0;v.unknown_cells=10;v.sources[0].contribution=0;
  CHECK(unknown.observe(v,"gid",R+30*M,S+30*M));
  CHECK(unknown.evaluate(R+40*M,S+40*M).disposition!=1);
  for(int mutation=0;mutation<4;++mutation) {
    SpatialEvidenceTracker wrong(config());v=evidence();CHECK(wrong.observe(v,"gid",R+30*M,S+30*M));
    if(mutation==0)v.sources[0].producer_epoch="other";
    if(mutation==1)++v.sources[0].frame_sequence;
    if(mutation==2)++v.sources[0].first_receipt_steady_ns;
    if(mutation==3){++v.sources[0].valid_until_ros_ns;++v.sources[0].valid_until_steady_ns;}
    CHECK(!wrong.observe(v,"gid",R+40*M,S+40*M));
  }
  std::cout<<"direct spatial belly health: duplicate age removed; original receipts, identities and spatial expiry retained\n";
}
