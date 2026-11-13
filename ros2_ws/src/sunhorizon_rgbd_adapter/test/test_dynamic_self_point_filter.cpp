#include "sunhorizon_rgbd_adapter/dynamic_self_point_filter.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace sunhorizon_rgbd_adapter;
namespace dsg=sunhorizon_obstacle_coverage::dynamic_self_geometry;
static int checks=0;
#define CHECK(x) do {++checks;if(!(x))throw std::runtime_error(#x);}while(false)
struct Fixture {
  std::vector<std::uint8_t> bytes;DynamicCloudView cloud;
  Fixture(const std::vector<std::array<double,3>>& points,bool big=false,bool doubles=false,bool padded=false) {
    const std::size_t n=doubles?8:4;cloud.width=padded?2:points.size();cloud.height=padded?points.size()/2:1;
    cloud.point_step=3*n+8;cloud.row_step=cloud.width*cloud.point_step+(padded?7:0);cloud.big_endian=big;
    cloud.xyz={DynamicPointField{0,n},{n,n},{2*n,n}};bytes.resize(cloud.row_step*cloud.height,0xa5);
    const std::uint16_t probe=1;const bool hostbig=*reinterpret_cast<const std::uint8_t*>(&probe)==0;
    for(std::size_t i=0;i<points.size();++i){auto* p=bytes.data()+(i/cloud.width)*cloud.row_step+(i%cloud.width)*cloud.point_step;
      for(std::size_t axis=0;axis<3;++axis){std::uint8_t b[8];if(doubles){double v=points[i][axis];std::memcpy(b,&v,8);}else{float v=points[i][axis];std::memcpy(b,&v,4);}if(big!=hostbig)std::reverse(b,b+n);std::memcpy(p+axis*n,b,n);}
      p[3*n]=static_cast<std::uint8_t>(i+17);
    }cloud.data=bytes.data();cloud.size=bytes.size();
  }
  std::vector<std::uint8_t> record(std::size_t i)const{const auto p=bytes.begin()+(i/cloud.width)*cloud.row_step+(i%cloud.width)*cloud.point_step;return {p,p+cloud.point_step};}
};
int main(){try{
  dsg::Mesh mesh;std::vector<dsg::Triangle> triangles{{{{{-2,-2,1},{2,-2,1},{2,2,1}}}},{{{{-2,-2,1},{2,2,1},{-2,2,1}}}}};
  CHECK(mesh.build(triangles,dsg::BuildLimits{}));
  std::vector<dsg::LinkObservation> links{{&mesh,"leg",{},true,true,true}};
  // Independent finite-surface per-point oracle, including its validation dummy and
  // counters. All nonself payload bytes must survive prepared batch filtering.
  std::vector<dsg::LinkObservation> twelve;
  for(std::size_t i=0;i<12;++i) {auto l=links[0];l.link_name="leg_"+std::to_string(i);
    l.optical_from_link.translation.x=.1*i;twelve.push_back(l);}
  Fixture large({{0,0,1},{0,0,.9},{0,0,1.1},{NAN,0,1},{5,0,1},{0,0,0}});
  auto batch=filter_dynamic_self_points(large.cloud,twelve,.02);CHECK(batch.complete);
  auto dummy=dsg::classify_body_surface({0,0,1},1,.02,twelve,dsg::QueryBudget{});
  std::size_t nodes=dummy.node_visits,triangles_tested=dummy.triangle_tests;
  std::array<std::size_t,5> classes{};std::vector<std::uint8_t> expected;
  const std::vector<std::array<double,3>> values{{0,0,1},{0,0,.9},{0,0,1.1},{NAN,0,1},{5,0,1},{0,0,0}};
  for(std::size_t i=0;i<values.size();++i) {
    auto label=dsg::Classification::Invalid;const auto p=values[i];
    if(std::isfinite(p[0]) && p[2]>0) {
      // Use the actual serialized float depth, as the production filter does.
      float x,y,z;auto bytes=large.record(i);std::memcpy(&x,bytes.data(),4);std::memcpy(&y,bytes.data()+4,4);std::memcpy(&z,bytes.data()+8,4);
      auto item=dsg::classify_body_surface({double(x)/z,double(y)/z,1},z,.02,twelve,dsg::QueryBudget{});CHECK(item.complete);
      label=item.classification;nodes+=item.node_visits;triangles_tested+=item.triangle_tests;
    }
    ++classes[static_cast<std::size_t>(label)];if(label!=dsg::Classification::SelfSurface) {
      auto bytes=large.record(i);expected.insert(expected.end(),bytes.begin(),bytes.end());}
  }
  CHECK(batch.data==expected && batch.classifications==classes);CHECK(batch.node_visits==nodes && batch.triangle_tests==triangles_tested);
  for(bool big:{false,true})for(bool doubles:{false,true}) {
    Fixture f({{0,0,1},{0,0,.9},{0,0,1.1},{0,0,0},{0,0,-1},{NAN,0,1},{5,0,1},{0,INFINITY,1}},big,doubles,true);
    const auto r=filter_dynamic_self_points(f.cloud,links,.02);CHECK(r.complete);CHECK(r.removed_count==1);CHECK(r.retained_count==7);
    CHECK(r.classifications[0]==1);CHECK(r.classifications[1]==2);CHECK(r.classifications[2]==4);CHECK(r.classifications[3]==1);
    CHECK(r.data.size()==7*f.cloud.point_step);
    for(std::size_t i=0;i<7;++i)CHECK(std::vector<std::uint8_t>(r.data.begin()+i*f.cloud.point_step,r.data.begin()+(i+1)*f.cloud.point_step)==f.record(i+1));
  }
  Fixture f({{0,0,1},{0,0,.9}});auto missing=links;missing[0].tf_valid=false;
  CHECK(!filter_dynamic_self_points(f.cloud,missing,.02).complete);
  Fixture invalid({{0,0,0},{NAN,0,1}});CHECK(!filter_dynamic_self_points(invalid.cloud,missing,.02).complete);
  auto shear=links;shear[0].optical_from_link.rotation[1]=.1;CHECK(!filter_dynamic_self_points(f.cloud,shear,.02).complete);
  auto bad=f.cloud;bad.xyz[1].offset=0;CHECK(!filter_dynamic_self_points(bad,links,.02).complete);
  bad=f.cloud;bad.size--;CHECK(!filter_dynamic_self_points(bad,links,.02).complete);
  bad=f.cloud;bad.row_step=1;CHECK(!filter_dynamic_self_points(bad,links,.02).complete);
  bad=f.cloud;bad.xyz[0].scalar_bytes=3;CHECK(!filter_dynamic_self_points(bad,links,.02).complete);
  bad=f.cloud;bad.width=std::numeric_limits<std::size_t>::max();CHECK(!filter_dynamic_self_points(bad,links,.02).complete);
  CHECK(!filter_dynamic_self_points(f.cloud,links,NAN).complete);CHECK(!filter_dynamic_self_points(f.cloud,links,.031).complete);
  CHECK(filter_dynamic_self_points(f.cloud,links,.03).complete);
  DynamicFilterBudget limit;limit.query.max_triangle_tests=1;auto r=filter_dynamic_self_points(f.cloud,links,.02,limit);CHECK(!r.complete);CHECK(r.data.empty());
  limit=DynamicFilterBudget{};limit.max_triangle_tests=2*dsg::classify_body_surface({0,0,1},1,.02,links,dsg::QueryBudget{}).triangle_tests;r=filter_dynamic_self_points(f.cloud,links,.02,limit);CHECK(!r.complete);CHECK(r.removed_count==1);CHECK(r.data.empty());
  Fixture empty_after({{0,0,1},{0,0,1}});r=filter_dynamic_self_points(empty_after.cloud,links,.02);CHECK(r.complete);CHECK(r.removed_count==2);CHECK(r.data.empty());
  std::cout<<"PASS "<<checks<<" checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<checks<<" "<<e.what()<<"\n";return 1;}}
