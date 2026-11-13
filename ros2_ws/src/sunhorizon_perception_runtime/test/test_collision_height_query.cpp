#include "sunhorizon_perception_runtime/collision_height_query.hpp"
#include <iostream>
#include <limits>
#include <type_traits>

namespace p=sunhorizon_perception_runtime;
namespace d=p::detail;
namespace {
int checks=0;
void check(bool value) {++checks;if(!value)throw std::runtime_error("collision height check failed");}
template<class F> void rejects(F&& f) {bool rejected=false;try{f();}catch(const std::exception&){rejected=true;}check(rejected);}
void run() {
  static_assert(!std::is_constructible<p::CollisionHeightBinding,p::SpatialQueryPolicy>::value,
    "standalone callers cannot fabricate a Nav2 responsibility proof");
  p::SpatialQueryPolicy full,limited;limited.mode="planar-collision-height-v1";
  const std::vector<p::SpatialQueryBox> original{{{.2,.15,-.8},{.3,.25,-.7}}};
  const auto effective=d::effective_spatial_boxes(limited,"base_footprint",original);
  check(original[0].min[2]==-.8 && original[0].max[2]==-.7);
  check(effective[0].min[0]==.2 && effective[0].max[0]==.3 &&
    effective[0].min[1]==.15 && effective[0].max[1]==.25);
  check(effective[0].min[2]==-.75-1.e-5 && effective[0].max[2]==-.7);
  check(effective[0].min[2]<-.75 && effective[0].min[2]>-.8);
  check(d::effective_spatial_boxes(full,"base_footprint",original)[0].min==original[0].min);
  auto changed=original;changed[0].min[2]=-.75;
  rejects([&]{d::effective_spatial_boxes(limited,"base_footprint",changed);});
  rejects([&]{d::effective_spatial_boxes(limited,"odom",original);});
  changed=original;changed.push_back(original[0]);
  rejects([&]{d::effective_spatial_boxes(limited,"base_footprint",changed);});
  auto wrong=limited;wrong.mode="guess";rejects([&]{d::validate_spatial_query_policy(wrong);});
  wrong=limited;wrong.collision_min_height_m=-.74;rejects([&]{d::validate_spatial_query_policy(wrong);});
  wrong=limited;wrong.collision_max_height_m=std::numeric_limits<double>::quiet_NaN();
  rejects([&]{d::validate_spatial_query_policy(wrong);});
  std::map<std::string,std::set<std::string>> expected;
  std::vector<d::CollisionHeightSource> sources;
  for(const std::string consumer:{"local","global","collision"}) {
    expected[consumer]={"airy","belly","chest"};
    for(const std::string source:{"airy","belly","chest"})
      sources.push_back({consumer,source,consumer=="local"?"odom":consumer=="global"?"map":"base_footprint",true,-.75,source=="belly"?1.:2.});
    sources.push_back({consumer,"floor_clear","odom",false,-1.1,3.});
  }
  d::require_collision_height_sources(limited,sources,expected);++checks;
  auto bad=sources;bad[0].min_height=-.750001;
  rejects([&]{d::require_collision_height_sources(limited,bad,expected);});
  bad=sources;bad[0].frame="tilted";rejects([&]{d::require_collision_height_sources(limited,bad,expected);});
  bad=sources;bad[3].marking=true;rejects([&]{d::require_collision_height_sources(limited,bad,expected);});
  bad=sources;bad.erase(bad.begin());rejects([&]{d::require_collision_height_sources(limited,bad,expected);});
  bad=sources;bad.push_back(sources[0]);rejects([&]{d::require_collision_height_sources(limited,bad,expected);});
  bad=sources;for(auto& s:bad)if(s.consumer=="collision")s.max_height=1.;
  rejects([&]{d::require_collision_height_sources(limited,bad,expected);});
  const auto nan=std::numeric_limits<double>::quiet_NaN();
  d::require_exact_planar_pose(1.,2.,0.,0.,0.,std::sin(.2),std::cos(.2));++checks;
  rejects([&]{d::require_exact_planar_pose(0.,0.,1.e-100,0.,0.,0.,1.);});
  rejects([&]{d::require_exact_planar_pose(0.,0.,0.,1.e-100,0.,0.,1.);});
  rejects([&]{d::require_exact_planar_pose(0.,0.,0.,0.,1.e-100,0.,1.);});
  rejects([&]{d::require_exact_planar_pose(nan,0.,0.,0.,0.,0.,1.);});
  rejects([&]{d::require_exact_planar_pose(0.,0.,0.,0.,0.,0.,0.);});
  d::require_exact_base_identity(0.,0.,0.,0.,0.,0.,1.);++checks;
  rejects([&]{d::require_exact_base_identity(1.e-100,0.,0.,0.,0.,0.,1.);});
  rejects([&]{d::require_exact_base_identity(0.,0.,0.,0.,0.,1.,0.);});
}
}
int main(int argc,char**) {
  try {
    if(argc==2) {
      p::SpatialQueryPolicy policy;policy.mode="planar-collision-height-v1";
      std::cout<<d::spatial_query_canonical(policy,"base_footprint",{{{.2,.15,-.8},{.3,.25,-.7}}});return 0;
    }
    run();std::cout<<"PASS "<<checks<<" collision height checks\n";return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
