#include "sunhorizon_obstacle_coverage/region_support_tracker.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace sunhorizon_obstacle_coverage;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(std::string(__func__)+":"+std::to_string(__LINE__)+": " #x); } while(false)
namespace {
constexpr std::int64_t T = 1791533039000000000LL, S = 1000000000LL;
RegionSupportConfig config() { return {"depth/region_support","near-left","base_footprint","belly_optical","pinned contract bytes"}; }
void set(RegionSupportReport& r, const std::string& key, const std::string& v) {
  for (auto& kv : r.values) if (kv.first == key) {kv.second=v; return;}
  throw std::runtime_error("test_key_missing");
}
void stamp(RegionSupportReport& r,std::int64_t source) {
  for(const auto* k:{"source_stamp_ns","image_receipt_ros_ns","evaluation_ros_ns","paired_info_stamp_ns","tf_requested_stamp_ns"}) set(r,k,std::to_string(source));
  r.header_ros_ns=source;
}
RegionSupportReport report() {
  RegionSupportReport r; r.name="depth/region_support"; r.header_ros_ns=T;
  r.values={{"diagnostic_event","region_depth_support"},{"schema_version","1"},
    {"contract_encoding","length-prefixed-ascii-v1"},{"contract","pinned contract bytes"},
    {"region_id","near-left"},{"base_frame_id","base_footprint"},{"image_frame_id","belly_optical"},
    {"region_frame_sequence","1"},{"region_evaluation_sequence","1"},
    {"source_stamp_ns",std::to_string(T)},{"image_receipt_ros_ns",std::to_string(T)},
    {"image_receipt_steady_ns","100"},{"evaluation_ros_ns",std::to_string(T)},
    {"evaluation_steady_ns","200"},{"paired_info_stamp_ns",std::to_string(T)},
    {"tf_requested_stamp_ns",std::to_string(T)},{"state","ClearSupported"},{"reason","clear_supported"},
    {"tested_pixels","10"},{"intersecting_rays","8"},{"clear_rays","8"},{"obstacle_rays","0"},
    {"invalid_depth_rays","0"},{"front_occluded_rays","0"},{"margin_band_rays","0"},
    {"region_wall_ns","0"},{"region_thread_cpu_ns","-1"}};
  return r;
}
void unknown(RegionSupportReport& r,const std::string& reason) {
  r.level=1; set(r,"state","Unknown");set(r,"reason",reason);
  for(const auto* k:{"tested_pixels","intersecting_rays","clear_rays","obstacle_rays","invalid_depth_rays","front_occluded_rays","margin_band_rays"})set(r,k,"0");
}
void accepts_only_fresh_clear() {
  RegionSupportTracker t(config()); CHECK(!t.evaluate(T,S).healthy);
  CHECK(t.observe(report(),T,S)); CHECK(t.evaluate(T+300000000,S+300000000).healthy);
  CHECK(!t.evaluate(T+300000001,S+300000001).healthy);
  RegionSupportTracker u(config()); CHECK(u.observe(report(),T,S));
  CHECK(!u.evaluate(T,S+300000001).healthy); // ROS clock frozen cannot refresh local age.
  RegionSupportTracker o(config());auto r=report();r.level=1;set(r,"state","ObstacleSupported");set(r,"clear_rays","7");set(r,"obstacle_rays","1");
  CHECK(o.observe(r,T,S));CHECK(!o.evaluate(T,S).healthy);
}
void pending_to_terminal_preserves_first_receipt() {
  RegionSupportTracker t(config());auto p=report();unknown(p,"pending_exact_info");set(p,"paired_info_stamp_ns","0");set(p,"tf_requested_stamp_ns","0");
  CHECK(t.observe(p,T,S));CHECK(!t.evaluate(T,S).healthy);
  auto clear=report();set(clear,"region_evaluation_sequence","2");
  CHECK(t.observe(clear,T+100000000,S+100000000));auto d=t.evaluate(T+100000000,S+100000000);
  CHECK(d.healthy);CHECK(d.local_receipt_ros_ns==T && d.local_receipt_steady_ns==S);
  CHECK(!t.evaluate(T+100000001,S+300000001).healthy);
  set(clear,"region_evaluation_sequence","3");CHECK(!t.observe(clear,T+100000002,S+300000002));
  CHECK(!t.evaluate(T+100000002,S+300000002).healthy);
}
void terminal_unknown_revokes_and_never_rearms() {
  RegionSupportTracker t(config());CHECK(t.observe(report(),T,S));CHECK(t.evaluate(T,S).healthy);
  auto r=report();unknown(r,"conflicting_exact_info");set(r,"region_evaluation_sequence","2");
  CHECK(t.observe(r,T+1,S+1));CHECK(!t.evaluate(T+1,S+1).healthy);
  set(r,"reason","pending_exact_info");set(r,"region_evaluation_sequence","3");
  CHECK(t.observe(r,T+2,S+2));
  r=report();set(r,"region_evaluation_sequence","4");CHECK(!t.observe(r,T+3,S+3));
  CHECK(!t.evaluate(T+3,S+3).healthy);
}
void new_pending_invalidates_old_clear_and_fresh_new_frame_recovers() {
  RegionSupportTracker t(config());CHECK(t.observe(report(),T,S));auto r=report();stamp(r,T+1);
  set(r,"region_frame_sequence","2");set(r,"region_evaluation_sequence","2");unknown(r,"pending_exact_info");
  CHECK(t.observe(r,T+1,S+1));CHECK(!t.evaluate(T+1,S+1).healthy);
  r=report();stamp(r,T+2);set(r,"region_frame_sequence","3");set(r,"region_evaluation_sequence","3");
  CHECK(t.observe(r,T+2,S+2));CHECK(t.evaluate(T+2,S+2).healthy);
}
void malformed_matching_reports_are_sticky_but_unrelated_ignored() {
  RegionSupportTracker t(config());CHECK(t.observe(report(),T,S));auto r=report();r.name="depth";
  CHECK(!t.observe(r,T+1,S+1));CHECK(t.evaluate(T+1,S+1).healthy);
  r=report();r.values.push_back({"state","ClearSupported"});CHECK(!t.observe(r,T+2,S+2));
  r=report();stamp(r,T+3);set(r,"region_frame_sequence","2");set(r,"region_evaluation_sequence","2");
  CHECK(!t.observe(r,T+3,S+3));CHECK(!t.evaluate(T+3,S+3).healthy);
}
void identities_states_pairing_and_counts_are_checked() {
  for(const auto& kv:std::vector<std::pair<std::string,std::string>>{
      {"contract","different"},{"contract_encoding","sha256"},{"region_id","other"},
      {"base_frame_id","map"},{"image_frame_id","other_optical"},{"schema_version","2"},
      {"diagnostic_event","depth_callback"},{"state","clear"},{"paired_info_stamp_ns","0"},
      {"tf_requested_stamp_ns","0"},{"clear_rays","7"},{"intersecting_rays","0"},
      {"tested_pixels","7"},{"invalid_depth_rays","1"},{"obstacle_rays","1"},
      {"margin_band_rays","4194305"},{"region_thread_cpu_ns","-2"}}) {
    RegionSupportTracker t(config());auto r=report();set(r,kv.first,kv.second);CHECK(!t.observe(r,T,S));CHECK(!t.evaluate(T,S).healthy);
  }
  RegionSupportTracker t(config());auto r=report();r.level=1;CHECK(!t.observe(r,T,S));
}
void integers_are_exact_and_fail_overflow() {
  for(const auto* value:{"","-1","+1"," 1","1 ","1.0","01","NaN","18446744073709551616"}) {
    RegionSupportTracker t(config());auto r=report();set(r,"region_frame_sequence",value);CHECK(!t.observe(r,T,S));
  }
  for(const auto* value:{"0","9223372036854775808","18446744073709551615"}) {
    RegionSupportTracker t(config());auto r=report();set(r,"source_stamp_ns",value);CHECK(!t.observe(r,T,S));
  }
  RegionSupportTracker t(config());auto r=report();stamp(r,std::numeric_limits<std::int64_t>::max());
  set(r,"region_frame_sequence","18446744073709551615");set(r,"region_evaluation_sequence","18446744073709551615");
  CHECK(t.observe(r,std::numeric_limits<std::int64_t>::max(),S));CHECK(t.evaluate(std::numeric_limits<std::int64_t>::max(),S).healthy);
}
void sequence_and_source_regressions_fail() {
  for(int mode=0;mode<5;++mode) {
    RegionSupportTracker t(config());CHECK(t.observe(report(),T,S));auto r=report();
    if(mode==1){set(r,"region_evaluation_sequence","2");set(r,"region_frame_sequence","2");}
    if(mode==2){set(r,"region_evaluation_sequence","2");set(r,"image_receipt_steady_ns","101");}
    if(mode==3){set(r,"region_evaluation_sequence","2");set(r,"evaluation_steady_ns","199");}
    if(mode==4){stamp(r,T+1);set(r,"region_evaluation_sequence","2");set(r,"region_frame_sequence","2");set(r,"evaluation_steady_ns","199");}
    CHECK(!t.observe(r,T+1,S+1));CHECK(!t.evaluate(T+1,S+1).healthy);
  }
}
void clock_future_and_rollback_fail_closed() {
  RegionSupportTracker f(config());CHECK(!f.observe(report(),T-1,S));
  for(bool ros:{true,false}) {
    RegionSupportTracker t(config());CHECK(t.observe(report(),T,S));CHECK(t.evaluate(T+100,S+100).healthy);
    CHECK(!t.evaluate(T+(ros?99:100),S+(ros?100:99)).healthy);
    CHECK(!t.evaluate(T+101,S+101).healthy);
  }
}
void bounded_schema_and_explicit_invalidation() {
  for(int mode=0;mode<4;++mode){RegionSupportTracker t(config());auto r=report();
    if(mode==0)r.values.erase(r.values.begin());
    if(mode==1)set(r,"contract",std::string(16385,'x'));
    if(mode==2)for(int i=0;i<65;++i)r.values.push_back({"x"+std::to_string(i),"0"});
    if(mode==3)set(r,"reason",std::string(257,'x'));
    CHECK(!t.observe(r,T,S));}
  RegionSupportTracker t(config());CHECK(t.observe(report(),T,S));t.invalidate("duplicate_matching_status");CHECK(!t.evaluate(T,S).healthy);
}
std::string canonical_field(const std::string& value) { return std::to_string(value.size())+":"+value; }
RegionSupportConfig typed_config() {
  auto c=config(); c.typed_evaluation_required=true;
  c.contract=canonical_field("fixture-v1")+canonical_field("typed-region-evaluation-diagnostics-v1");
  return c;
}
RegionSupportReport typed_report() {
  auto r=report();set(r,"contract",typed_config().contract);
  r.values.insert(r.values.end(),{{"evaluation_complete","true"},{"evaluation_code","1"},{"disposition","1"}});
  return r;
}
RegionSupportReport margin_report() {
  auto r=typed_report();r.level=1;set(r,"state","Unknown");set(r,"reason","arbitrary_readable_detail");
  set(r,"evaluation_code","3");set(r,"disposition","2");
  set(r,"clear_rays","7");set(r,"margin_band_rays","1");return r;
}
void rejected_evidence(const RegionSupportDecision& d) {
  CHECK(!d.evaluation_complete && d.evaluation_code==RegionEvaluationCode::kRejected);
  CHECK(d.disposition==RegionDisposition::kHardFault);
}
void typed_contract_requires_real_final_canonical_field() {
  for(const auto& suffix:std::vector<std::string>{"pinned contract bytes",
      "typed-region-evaluation-diagnostics-v1",canonical_field("typed-region-evaluation-diagnostics-v1")+"x",
      canonical_field("xtyped-region-evaluation-diagnostics-v1"),
      "036:typed-region-evaluation-diagnostics-v1",
      canonical_field("typed-region-evaluation-diagnostics-v1")+canonical_field("after"),
      "999999999999999999999999999999:bad"}) {
    auto c=typed_config();c.contract=suffix;bool threw=false;
    try { RegionSupportTracker t(c); } catch(const std::invalid_argument&) { threw=true; }
    CHECK(threw);
  }
  RegionSupportTracker good(typed_config());rejected_evidence(good.evaluate(T,S));
}
void typed_clear_margin_and_obstacle_remain_original_healthy_gate() {
  RegionSupportTracker c(typed_config());CHECK(c.observe(typed_report(),T,S));auto d=c.evaluate(T,S);
  CHECK(d.healthy && d.evaluation_complete && d.evaluation_code==RegionEvaluationCode::kClear);
  CHECK(d.disposition==RegionDisposition::kClear);
  RegionSupportTracker m(typed_config());CHECK(m.observe(margin_report(),T,S));d=m.evaluate(T,S);
  CHECK(!d.healthy && d.evaluation_complete && d.evaluation_code==RegionEvaluationCode::kIncompleteRays);
  CHECK(d.disposition==RegionDisposition::kGeometryHold); // No reason-text inference.
  auto r=typed_report();r.level=1;set(r,"state","ObstacleSupported");set(r,"clear_rays","7");
  set(r,"obstacle_rays","1");set(r,"evaluation_code","2");set(r,"disposition","0");
  RegionSupportTracker o(typed_config());CHECK(o.observe(r,T,S));d=o.evaluate(T,S);
  CHECK(!d.healthy && d.evaluation_complete && d.evaluation_code==RegionEvaluationCode::kObstacle);
  CHECK(d.disposition==RegionDisposition::kHardFault);
}
void typed_mixed_rays_cannot_be_geometry_hold() {
  for(const char* key:{"obstacle_rays","invalid_depth_rays","front_occluded_rays"}) {
    auto r=margin_report();set(r,"clear_rays","6");set(r,key,"1");
    RegionSupportTracker bad(typed_config());CHECK(!bad.observe(r,T,S));rejected_evidence(bad.evaluate(T,S));
    set(r,"disposition","0");RegionSupportTracker good(typed_config());CHECK(good.observe(r,T,S));
    auto d=good.evaluate(T,S);CHECK(d.evaluation_complete && !d.healthy && d.disposition==RegionDisposition::kHardFault);
  }
  for(const char* key:{"invalid_depth_rays","front_occluded_rays"}) {
    auto r=margin_report();set(r,"margin_band_rays","0");set(r,key,"1");set(r,"disposition","0");
    RegionSupportTracker t(typed_config());CHECK(t.observe(r,T,S));CHECK(t.evaluate(T,S).disposition==RegionDisposition::kHardFault);
  }
}
void typed_missing_malformed_or_inconsistent_fields_reject() {
  for(const char* key:{"evaluation_complete","evaluation_code","disposition"}) {
    auto r=typed_report();for(auto it=r.values.begin();it!=r.values.end();++it)if(it->first==key){r.values.erase(it);break;}
    RegionSupportTracker t(typed_config());CHECK(!t.observe(r,T,S));rejected_evidence(t.evaluate(T,S));
  }
  for(const auto& kv:std::vector<std::pair<std::string,std::string>>{
      {"evaluation_complete","True"},{"evaluation_complete","1"},{"evaluation_complete","false"},
      {"evaluation_code","01"},{"evaluation_code","-1"},{"evaluation_code","4"},
      {"evaluation_code","0"},{"evaluation_code","1.0"},{"evaluation_code"," 1"},
      {"disposition","02"},{"disposition","3"},{"disposition","-1"},{"disposition","0"},
      {"paired_info_stamp_ns","0"},{"tf_requested_stamp_ns","0"},
      {"clear_rays","6"},{"intersecting_rays","0"},{"margin_band_rays","0"}}) {
    auto r=margin_report();set(r,kv.first,kv.second);RegionSupportTracker t(typed_config());
    CHECK(!t.observe(r,T,S));rejected_evidence(t.evaluate(T,S));
  }
  // Explicitly inconsistent types cannot be relabeled by plausible reason text.
  auto r=typed_report();set(r,"reason","incomplete_ray_support");set(r,"evaluation_code","3");set(r,"disposition","2");
  RegionSupportTracker t(typed_config());CHECK(!t.observe(r,T,S));
}
void typed_partial_rejection_never_authorizes_hold() {
  auto r=margin_report();set(r,"evaluation_complete","false");set(r,"evaluation_code","0");set(r,"disposition","0");
  set(r,"clear_rays","5");set(r,"paired_info_stamp_ns","0");set(r,"tf_requested_stamp_ns","0");
  RegionSupportTracker t(typed_config());CHECK(t.observe(r,T,S));auto d=t.evaluate(T,S);CHECK(!d.healthy);rejected_evidence(d);
  for(const auto& kv:std::vector<std::pair<std::string,std::string>>{{"evaluation_code","3"},{"disposition","2"},{"state","ClearSupported"}}) {
    auto bad=r;set(bad,kv.first,kv.second);RegionSupportTracker u(typed_config());CHECK(!u.observe(bad,T,S));
  }
}
void typed_stale_clock_and_invalidation_override_completed_evidence() {
  for(int mode=0;mode<5;++mode) {
    RegionSupportTracker t(typed_config());CHECK(t.observe(margin_report(),T,S));
    CHECK(t.evaluate(T,S).disposition==RegionDisposition::kGeometryHold);
    if(mode==0)rejected_evidence(t.evaluate(T+300000001,S+300000001));
    if(mode==1)rejected_evidence(t.evaluate(T,S+300000001));
    if(mode==2){(void)t.evaluate(T+1,S+1);rejected_evidence(t.evaluate(T,S+1));}
    if(mode==3){(void)t.evaluate(T+1,S+1);rejected_evidence(t.evaluate(T+1,S));}
    if(mode==4){t.invalidate("owner_changed");rejected_evidence(t.evaluate(T,S));}
  }
  RegionSupportTracker already_stale(typed_config());CHECK(already_stale.observe(margin_report(),T+300000001,S));
  rejected_evidence(already_stale.evaluate(T+300000001,S)); // observe is not a freshness grant.
  RegionSupportTracker future(typed_config());CHECK(!future.observe(margin_report(),T-1,S));rejected_evidence(future.evaluate(T,S));
}
void typed_complete_unknown_is_terminal_even_with_pending_reason() {
  for(bool misleading_reason:{false,true}) {
    RegionSupportTracker t(typed_config());auto r=margin_report();
    if(misleading_reason)set(r,"reason","pending_exact_info");
    CHECK(t.observe(r,T,S));CHECK(t.evaluate(T,S).disposition==RegionDisposition::kGeometryHold);
    set(r,"region_evaluation_sequence","2");CHECK(!t.observe(r,T+1,S+1));
    rejected_evidence(t.evaluate(T+1,S+1));
  }
  RegionSupportTracker t(typed_config());auto r=margin_report();CHECK(t.observe(r,T,S));
  unknown(r,"pending_exact_info");set(r,"evaluation_complete","false");set(r,"evaluation_code","0");set(r,"disposition","0");
  set(r,"region_evaluation_sequence","2");CHECK(t.observe(r,T+1,S+1));rejected_evidence(t.evaluate(T+1,S+1));
  r=margin_report();set(r,"region_evaluation_sequence","3");CHECK(!t.observe(r,T+2,S+2));
}
void typed_pending_completion_and_new_frame_recovery_preserve_receipt() {
  RegionSupportTracker t(typed_config());auto r=typed_report();unknown(r,"pending_exact_info");
  set(r,"evaluation_complete","false");set(r,"evaluation_code","0");set(r,"disposition","0");
  set(r,"paired_info_stamp_ns","0");set(r,"tf_requested_stamp_ns","0");CHECK(t.observe(r,T,S));
  r=margin_report();set(r,"region_evaluation_sequence","2");CHECK(t.observe(r,T+100000000,S+100000000));
  auto d=t.evaluate(T+100000000,S+100000000);CHECK(d.disposition==RegionDisposition::kGeometryHold);
  CHECK(d.local_receipt_ros_ns==T && d.local_receipt_steady_ns==S);
  rejected_evidence(t.evaluate(T+300000001,S+300000001));
  r=typed_report();stamp(r,T+300000002);set(r,"region_frame_sequence","2");set(r,"region_evaluation_sequence","3");
  CHECK(t.observe(r,T+300000002,S+300000002));d=t.evaluate(T+300000002,S+300000002);
  CHECK(d.healthy && d.disposition==RegionDisposition::kClear); // Evidence only; no supervisor operation.
}
void default_path_does_not_require_or_expose_typed_evidence() {
  RegionSupportTracker t(config());CHECK(t.observe(report(),T,S));auto d=t.evaluate(T,S);
  CHECK(d.healthy);rejected_evidence(d);
  auto r=report();r.values.insert(r.values.end(),{{"evaluation_complete","invalid"},{"evaluation_code","99"},{"disposition","2"}});
  RegionSupportTracker u(config());CHECK(u.observe(r,T,S));d=u.evaluate(T,S);CHECK(d.healthy);rejected_evidence(d);
  // Complete-looking unknown reports retain old behavior when explicit required=false.
  r=margin_report();set(r,"contract",config().contract);RegionSupportTracker v(config());CHECK(v.observe(r,T,S));
  set(r,"region_evaluation_sequence","2");CHECK(v.observe(r,T+1,S+1));rejected_evidence(v.evaluate(T+1,S+1));
}

void typed_field_capacity_and_sticky_sequence_failures() {
  auto r=margin_report();
  while(r.values.size()<64)r.values.push_back({"extra"+std::to_string(r.values.size()),"0"});
  RegionSupportTracker full(typed_config());CHECK(full.observe(r,T,S));
  CHECK(full.evaluate(T,S).disposition==RegionDisposition::kGeometryHold);
  r.values.push_back({"one_too_many","0"});RegionSupportTracker excess(typed_config());
  CHECK(!excess.observe(r,T,S));rejected_evidence(excess.evaluate(T,S));
  RegionSupportTracker seq(typed_config());CHECK(seq.observe(margin_report(),T,S));
  CHECK(!seq.observe(margin_report(),T+1,S+1));
  r=typed_report();stamp(r,T+2);set(r,"region_frame_sequence","2");set(r,"region_evaluation_sequence","2");
  CHECK(!seq.observe(r,T+2,S+2));rejected_evidence(seq.evaluate(T+2,S+2));
}
void typed_fresh_new_frame_can_replace_nonsticky_hardfault_evidence() {
  RegionSupportTracker t(typed_config());auto r=margin_report();
  set(r,"margin_band_rays","0");set(r,"invalid_depth_rays","1");set(r,"disposition","0");
  CHECK(t.observe(r,T,S));auto d=t.evaluate(T,S);
  CHECK(d.evaluation_complete && !d.healthy && d.disposition==RegionDisposition::kHardFault);
  r=typed_report();stamp(r,T+1);set(r,"region_frame_sequence","2");set(r,"region_evaluation_sequence","2");
  CHECK(t.observe(r,T+1,S+1));d=t.evaluate(T+1,S+1);
  CHECK(d.healthy && d.disposition==RegionDisposition::kClear);
}

}
int main(){
  accepts_only_fresh_clear();pending_to_terminal_preserves_first_receipt();
  terminal_unknown_revokes_and_never_rearms();new_pending_invalidates_old_clear_and_fresh_new_frame_recovers();
  malformed_matching_reports_are_sticky_but_unrelated_ignored();identities_states_pairing_and_counts_are_checked();
  integers_are_exact_and_fail_overflow();sequence_and_source_regressions_fail();clock_future_and_rollback_fail_closed();
  bounded_schema_and_explicit_invalidation();typed_contract_requires_real_final_canonical_field();
  typed_clear_margin_and_obstacle_remain_original_healthy_gate();typed_mixed_rays_cannot_be_geometry_hold();
  typed_missing_malformed_or_inconsistent_fields_reject();typed_partial_rejection_never_authorizes_hold();
  typed_stale_clock_and_invalidation_override_completed_evidence();typed_complete_unknown_is_terminal_even_with_pending_reason();
  typed_pending_completion_and_new_frame_recovery_preserve_receipt();default_path_does_not_require_or_expose_typed_evidence();
  typed_field_capacity_and_sticky_sequence_failures();typed_fresh_new_frame_can_replace_nonsticky_hardfault_evidence();
  std::cout<<"21 region tracker cases passed\n";
}
