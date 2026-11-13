#include "sunhorizon_obstacle_coverage/dynamic_self_geometry.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>
#include <utility>

using namespace sunhorizon_obstacle_coverage::dynamic_self_geometry;
static std::size_t checks=0;
#define CHECK(value) do { ++checks; if (!(value)) { std::cerr << "CHECK failed at " << __LINE__ << ": " #value << '\n'; std::exit(1); } } while (false)

Triangle plane(double z,double x=0) {
  return {{{{x-2,-2,z},{x+2,-2,z},{x,2,z}}}};
}
LinkObservation link(const Mesh& mesh,const std::string& name="leg") {
  LinkObservation r;r.mesh=&mesh;r.link_name=name;
  r.identity_valid=r.tf_valid=r.covers_source_time=true;return r;
}
void expect(const Result& r,Classification classification) {
  CHECK(r.complete);CHECK(r.classification==classification);
}
void unobserved(const Result& r) {
  CHECK(!r.complete);CHECK(r.classification==Classification::Unobserved);
  CHECK(!r.has_first_surface);CHECK(std::isnan(r.first_axial_depth_m));
}
void same_result(const Result& a,const Result& b) {
  CHECK(a.classification==b.classification && a.complete==b.complete && a.has_first_surface==b.has_first_surface);
  CHECK((std::isnan(a.first_axial_depth_m) && std::isnan(b.first_axial_depth_m)) || a.first_axial_depth_m==b.first_axial_depth_m);
  CHECK(a.link_index==b.link_index && a.triangle_index==b.triangle_index);
  CHECK(a.node_visits==b.node_visits && a.triangle_tests==b.triangle_tests);
  CHECK((std::isnan(a.finite_surface_distance_m)&&std::isnan(b.finite_surface_distance_m))||a.finite_surface_distance_m==b.finite_surface_distance_m);
  CHECK(a.finite_surface_link_index==b.finite_surface_link_index && a.finite_surface_triangle_index==b.finite_surface_triangle_index);
  CHECK(std::strcmp(a.reason,b.reason)==0);
  CHECK(a.measured_reference_ambiguous==b.measured_reference_ambiguous);
  CHECK(a.ambiguous_measured_link_index==b.ambiguous_measured_link_index);
  CHECK(a.ambiguous_measured_triangle_index==b.ambiguous_measured_triangle_index);
  CHECK((std::isnan(a.ambiguous_min_axial_depth_m)&&std::isnan(b.ambiguous_min_axial_depth_m))||a.ambiguous_min_axial_depth_m==b.ambiguous_min_axial_depth_m);
  CHECK((std::isnan(a.ambiguous_max_axial_depth_m)&&std::isnan(b.ambiguous_max_axial_depth_m))||a.ambiguous_max_axial_depth_m==b.ambiguous_max_axial_depth_m);
  CHECK(a.has_measured_surface==b.has_measured_surface);
  CHECK((std::isnan(a.measured_axial_depth_m)&&std::isnan(b.measured_axial_depth_m))||a.measured_axial_depth_m==b.measured_axial_depth_m);
  CHECK(a.measured_link_index==b.measured_link_index&&a.measured_triangle_index==b.measured_triangle_index);
}
void prepared_equivalence() {
  std::vector<Mesh> meshes(12);std::vector<LinkObservation> links;
  for(std::size_t i=0;i<meshes.size();++i) {
    CHECK(meshes[i].build({plane(1),plane(1.5)},BuildLimits{}));
    auto item=link(meshes[i],"leg_"+std::to_string(i));item.optical_from_link.translation.x=.1*i;
    item.optical_from_link.translation.z=.01*i;links.push_back(item);
  }
  auto compare=[&](const std::vector<LinkObservation>& observations,const QueryBudget& budget) {
    const PreparedModel prepared(observations,budget);
    for(const Vec3 ray:std::vector<Vec3>{{0,0,1},{.5,.2,1},{5,5,1},{NAN,0,1},{0,0,2},{1.e308,0,1}})
      for(double depth:{0.,.5,1.,1.02,2.,double(INFINITY),double(NAN)}) for(double tolerance:{0.,.02,double(NAN)})
        same_result(classify(ray,depth,tolerance,observations,budget),classify(ray,depth,tolerance,prepared));
  };
  compare(links,QueryBudget{});
  auto budget=QueryBudget{};budget.max_node_visits=1;compare(links,budget);
  budget=QueryBudget{};budget.max_triangle_tests=1;compare(links,budget);
  budget=QueryBudget{};budget.max_links=1;compare(links,budget);
  budget=QueryBudget{};budget.max_node_visits=0;compare(links,budget);
  compare({},QueryBudget{});
  auto invalid=links;invalid[11].covers_source_time=false;compare(invalid,QueryBudget{});
  invalid=links;invalid[4].tf_valid=false;compare(invalid,QueryBudget{});
  invalid=links;invalid[2].identity_valid=false;compare(invalid,QueryBudget{});
  invalid=links;invalid[6].mesh=nullptr;compare(invalid,QueryBudget{});
  invalid=links;invalid[3].link_name=invalid[2].link_name;compare(invalid,QueryBudget{});
  invalid=links;invalid[7].optical_from_link.rotation[1]=.1;compare(invalid,QueryBudget{});
  invalid=links;invalid[8].optical_from_link.translation.x=INFINITY;compare(invalid,QueryBudget{});
  // Frame transform snapshot cannot be refreshed by later caller mutation.
  const PreparedModel original(links,QueryBudget{});
  const auto expected=classify({0,0,1},1,.02,original);
  links[0].optical_from_link.translation.z=10;
  same_result(expected,classify({0,0,1},1,.02,original));
  // Successful rebuild, failed rebuild, assignment and movement all invalidate
  // old borrowed snapshots even if the replacement is also ready/valid.
  CHECK(meshes[0].build({plane(2)},BuildLimits{}));unobserved(classify({0,0,1},1,.02,original));
  const PreparedModel rebuilt(links,QueryBudget{});
  CHECK(!meshes[0].build({},BuildLimits{}));unobserved(classify({0,0,1},1,.02,rebuilt));
  CHECK(meshes[0].build({plane(1)},BuildLimits{}));const PreparedModel assigned(links,QueryBudget{});
  meshes[0]=meshes[1];unobserved(classify({0,0,1},1,.02,assigned));
  const PreparedModel moved(links,QueryBudget{});Mesh moved_out=std::move(meshes[0]);
  CHECK(moved_out.ready());unobserved(classify({0,0,1},1,.02,moved));
  meshes[0]=std::move(moved_out);const PreparedModel move_assigned(links,QueryBudget{});
  meshes[0]=std::move(meshes[1]);unobserved(classify({0,0,1},1,.02,move_assigned));
}
std::vector<std::uint8_t> stl(const std::vector<Triangle>& triangles) {
  std::vector<std::uint8_t> bytes(84+triangles.size()*50,0);
  auto u32=[&](std::size_t pos,std::uint32_t value) {for(int a=0;a<4;++a)bytes[pos+a]=(value>>(8*a))&255;};
  u32(80,static_cast<std::uint32_t>(triangles.size()));
  for(std::size_t i=0;i<triangles.size();++i)for(std::size_t v=0;v<3;++v) {
    const auto& p=triangles[i].vertices[v];const double values[3]={p.x,p.y,p.z};
    for(std::size_t a=0;a<3;++a) {
      const float value=static_cast<float>(values[a]);std::uint32_t bits;
      std::memcpy(&bits,&value,4);u32(84+i*50+12+v*12+a*4,bits);
    }
  }
  return bytes;
}

void authoritative_measured_surface() {
  Mesh body;std::string error;const Triangle finite_patch={{{{-.1,-.1,1},{.1,-.1,1},{0,.1,1}}}};
  CHECK(body.build({plane(.7),finite_patch},BuildLimits{}));
  const std::vector<LinkObservation> links{link(body)};
  expect(classify({0,0,1},.7,.02,links,{}),Classification::SelfSurface);
  CHECK(body.configure_measured_surface(1,1,&error));const PreparedModel model(links,{});
  for(double depth:{.7,.95,.97}) {
    auto r=classify({0,0,1},depth,.02,model);expect(r,Classification::ExternalSurface);
    CHECK(std::strcmp(r.reason,"before_measured_surface")==0);
    CHECK(r.has_measured_surface&&r.measured_axial_depth_m==1.);
    CHECK(r.has_first_surface&&r.first_axial_depth_m==.7&&r.triangle_index==0&&r.measured_triangle_index==1);
    same_result(r,classify({0,0,1},depth,.02,links,{}));
  }
  for(double depth:{.99,1.,1.01})expect(classify({0,0,1},depth,.02,model),Classification::SelfSurface);
  auto r=classify({0,0,1},1.03,.02,model);expect(r,Classification::ModelConflict);
  CHECK(std::strcmp(r.reason,"behind_measured_surface")==0);
  // FLOAT32 round-trip boundary error is sub-micrometre, never a physical rim.
  r=classify({-.10000002,-.1,1},1,.02,model);expect(r,Classification::SelfSurface);CHECK(r.has_measured_surface);
  r=classify({-.100002,-.1,1},1,.02,model);expect(r,Classification::ModelConflict);CHECK(!r.has_measured_surface);
  r=classify({0,-.10000002,1},.97,.02,model);expect(r,Classification::ExternalSurface);CHECK(r.has_measured_surface);
  r=classify({0,-.10000002,1},.95,.02,model);expect(r,Classification::ExternalSurface);CHECK(r.has_measured_surface);
  Mesh far_coordinates;CHECK(far_coordinates.build({plane(10)},{}));CHECK(far_coordinates.configure_measured_surface(0,1));
  unobserved(classify({0,0,1},10,.02,PreparedModel({link(far_coordinates)},{})));
  CHECK(far_coordinates.build({plane(.7),{{{{0,-1,.5},{0,1,.5},{0,0,1.5}}}}},{}));
  CHECK(far_coordinates.configure_measured_surface(1,1));
  r=classify({0,0,1},.7,.02,PreparedModel({link(far_coordinates)},{}));
  expect(r,Classification::Unobserved);CHECK(r.measured_reference_ambiguous);
  CHECK(!r.has_measured_surface);CHECK(r.has_first_surface&&r.first_axial_depth_m==.7);
  // A finite near-parallel facet has multiple numerically admissible edge
  // ranges. This is a retained point, never nominal Self or owner failure.
  Mesh ambiguous;const Triangle grazing={{{{1.e-8,-1.e-8,.8},{1.e-8,1.e-8,1.},{1.e-8,2.e-8,1.2}}}};
  CHECK(ambiguous.build({plane(.7),grazing},{}));CHECK(ambiguous.configure_measured_surface(1,1));
  const std::vector<LinkObservation> ambiguous_links{link(ambiguous)};
  const auto grazing_result=classify({0,0,1},1.,.02,PreparedModel(ambiguous_links,{}));
  expect(grazing_result,Classification::Unobserved);
  CHECK(grazing_result.measured_reference_ambiguous&&!grazing_result.has_measured_surface);
  CHECK(std::strcmp(grazing_result.reason,"measured_roundoff_range_ambiguous")==0);
  CHECK(grazing_result.ambiguous_measured_link_index==0&&grazing_result.ambiguous_measured_triangle_index==1);
  CHECK(grazing_result.ambiguous_max_axial_depth_m-grazing_result.ambiguous_min_axial_depth_m>.01);
  for(double depth:{.93,.97,1.}) {
    const auto a=classify({0,0,1},depth,.02,PreparedModel(ambiguous_links,{}));
    expect(a,Classification::Unobserved);CHECK(!a.has_measured_surface);
    same_result(a,classify({0,0,1},depth,.02,ambiguous_links,{}));
  }
  // Complete remaining authorized links after ambiguity; later budget failure
  // must still be incomplete, rather than hidden by the retained point.
  const std::vector<LinkObservation> trailing_links{link(ambiguous),link(body,"trailing")};
  const auto trailing=classify({0,0,1},1.,.02,PreparedModel(trailing_links,{}));
  expect(trailing,Classification::Unobserved);CHECK(trailing.triangle_tests>grazing_result.triangle_tests);
  QueryBudget exhausted;exhausted.max_triangle_tests=trailing.triangle_tests-1;
  const auto hard_failure=classify({0,0,1},1.,.02,PreparedModel(trailing_links,exhausted));
  unobserved(hard_failure);CHECK(std::strcmp(hard_failure.reason,"triangle_budget")==0);
  // Beyond the unchanged roundoff bound, a miss uses the original nominal CAD.
  expect(classify({-2.e-6,0,1},.7,.02,PreparedModel(ambiguous_links,{})),Classification::SelfSurface);
  // A narrow nonunique reference entirely inside one class is decidable.
  // Its entire interval, rather than an arbitrarily selected edge, is tested.
  Mesh narrow;const Triangle narrow_grazing={{{{1.e-8,-1.e-8,.99999},{1.e-8,1.e-8,1.},{1.e-8,2.e-8,1.00001}}}};
  CHECK(narrow.build({plane(.7),narrow_grazing},{}));CHECK(narrow.configure_measured_surface(1,1));
  const std::vector<LinkObservation> narrow_links{link(narrow)};
  for(double depth:{.95,.97,1.,1.03,1.05}) {
    const auto a=classify({0,0,1},depth,.02,PreparedModel(narrow_links,{}));
    expect(a,depth<1.?Classification::ExternalSurface:(depth>1.?Classification::ModelConflict:Classification::SelfSurface));
    CHECK(a.measured_reference_ambiguous&&a.has_measured_surface);
    CHECK(a.ambiguous_max_axial_depth_m>a.ambiguous_min_axial_depth_m);
    same_result(a,classify({0,0,1},depth,.02,narrow_links,{}));
  }
  const auto interval=classify({0,0,1},1.,.02,PreparedModel(narrow_links,{}));
  const double midpoint=(interval.ambiguous_min_axial_depth_m+interval.ambiguous_max_axial_depth_m)/2;
  for(double depth:{midpoint-.02,midpoint+.02}) {
    const auto a=classify({0,0,1},depth,.02,PreparedModel(narrow_links,{}));
    expect(a,Classification::Unobserved);CHECK(a.measured_reference_ambiguous&&!a.has_measured_surface);
  }
  auto shifted=link(narrow,"shifted");shifted.optical_from_link.translation.z=.1;
  for(auto observations:std::vector<std::vector<LinkObservation>>{{link(narrow),shifted},{shifted,link(narrow)}}) {
    const auto a=classify({0,0,1},1.,.02,PreparedModel(observations,{}));
    expect(a,Classification::Unobserved);CHECK(a.ambiguous_min_axial_depth_m<1.&&a.ambiguous_max_axial_depth_m>1.09);
    same_result(a,classify({0,0,1},1.,.02,observations,{}));
  }
  // A unique hit in another link cannot be omitted from the envelope, even
  // after a nearer candidate was found; cross-class bounds stay Unobserved.
  Mesh exact_far;CHECK(exact_far.build({plane(1.1)},{}));CHECK(exact_far.configure_measured_surface(0,1));
  for(auto observations:std::vector<std::vector<LinkObservation>>{{link(narrow),link(exact_far,"far")},{link(exact_far,"far"),link(narrow)}}) {
    const auto a=classify({0,0,1},1.,.02,PreparedModel(observations,{}));
    expect(a,Classification::Unobserved);CHECK(a.measured_reference_ambiguous&&!a.has_measured_surface);
    CHECK(a.ambiguous_min_axial_depth_m<1.&&a.ambiguous_max_axial_depth_m==1.1);
    same_result(a,classify({0,0,1},1.,.02,observations,{}));
  }
  // Uncovered rays and a near miss of the open patch keep original nominal behavior.
  r=classify({.2,0,1},.7,.02,model);expect(r,Classification::SelfSurface);CHECK(!r.has_measured_surface);
  r=classify({.2,0,1},1,.02,model);expect(r,Classification::ModelConflict);CHECK(!r.has_measured_surface);
  r=classify({-.101,-.1,1},1,.02,model);expect(r,Classification::ModelConflict);CHECK(!r.has_measured_surface);
  // LiDAR range queries retain nearest nominal geometry even with measured authorization.
  r=classify_range({0,0,0},{0,0,1},.7,.02,model);expect(r,Classification::SelfSurface);CHECK(!r.has_measured_surface);
  r=classify_range({0,0,0},{0,0,1},1,.02,model);expect(r,Classification::ModelConflict);CHECK(!r.has_measured_surface);
  // A farther authorized face cannot be selected by observed depth; nearest wins across links.
  Mesh farther;CHECK(farther.build({plane(1.5)},{}));CHECK(farther.configure_measured_surface(0,1));
  std::vector<LinkObservation> pair{link(body),link(farther,"other")};
  r=classify({0,0,1},1.5,.02,PreparedModel(pair,{}));expect(r,Classification::ModelConflict);CHECK(r.measured_axial_depth_m==1.);
  std::swap(pair[0],pair[1]);r=classify({0,0,1},1.5,.02,PreparedModel(pair,{}));expect(r,Classification::ModelConflict);CHECK(r.measured_axial_depth_m==1.);
  // Fail-closed through measured traversal budgets, never an earlier partial Self.
  const auto complete=classify({0,0,1},1,.02,model);CHECK(complete.complete);
  QueryBudget tiny;tiny.max_triangle_tests=complete.triangle_tests-1;
  unobserved(classify({0,0,1},1,.02,PreparedModel(links,tiny)));
  tiny={};tiny.max_node_visits=complete.node_visits-1;unobserved(classify({0,0,1},1,.02,PreparedModel(links,tiny)));
  tiny={};tiny.max_triangle_tests=1;unobserved(classify({0,0,1},1,.02,PreparedModel(pair,tiny)));
  auto invalid=links;invalid[0].covers_source_time=false;unobserved(classify({0,0,1},1,.02,PreparedModel(invalid,{})));
  const auto diag=model.diagnostic_links();CHECK(diag[0].measured_triangle_begin==1&&diag[0].measured_triangle_count==1);
  Mesh copied=body;expect(classify({0,0,1},1,.02,PreparedModel({link(copied)},{})),Classification::SelfSurface);
  Mesh assigned;assigned=body;expect(classify({0,0,1},.97,.02,PreparedModel({link(assigned)},{})),Classification::ExternalSurface);
  Mesh moved=std::move(copied);expect(classify({0,0,1},.97,.02,PreparedModel({link(moved)},{})),Classification::ExternalSurface);
  Mesh move_assigned;move_assigned=std::move(assigned);expect(classify({0,0,1},.97,.02,PreparedModel({link(move_assigned)},{})),Classification::ExternalSurface);
  CHECK(body.configure_measured_surface(0,0));unobserved(classify({0,0,1},1,.02,model));
  expect(classify({0,0,1},.7,.02,PreparedModel(links,{})),Classification::SelfSurface);
  CHECK(!body.configure_measured_surface(1,0,&error));CHECK(!body.ready());
  CHECK(body.build({plane(.7),finite_patch},{}));CHECK(!body.configure_measured_surface(2,1));CHECK(!body.ready());
  CHECK(body.build({plane(.7),finite_patch},{}));CHECK(!body.configure_measured_surface(std::numeric_limits<std::size_t>::max(),2));CHECK(!body.ready());
  CHECK(body.build({plane(.7),finite_patch},{}));CHECK(body.configure_measured_surface(1,1));
  CHECK(body.build({plane(.7)},{}));CHECK(PreparedModel({link(body)},{}).diagnostic_links()[0].measured_triangle_count==0);
}

void authorized_finite_boundary() {
  Mesh patch;CHECK(patch.build({{{{{0,-.1,1},{0,.1,1},{.1,0,1}}}}},BuildLimits{}));
  const Vec3 ray{-.002,0,1};
  expect(classify(ray,1,.02,{link(patch)},QueryBudget{}),Classification::ExternalSurface);
  CHECK(patch.configure_boundary_patch(0,1,.003));
  PreparedModel model({link(patch)},QueryBudget{});
  const auto hit=classify(ray,1,.02,model);expect(hit,Classification::SelfSurface);
  CHECK(std::strcmp(hit.reason,"matches_authorized_finite_boundary")==0);
  CHECK(hit.has_first_surface && std::abs(hit.first_axial_depth_m-1.)<.00001);
  for(const auto offset:{.03,.05}) {
    const auto front=classify(ray,hit.first_axial_depth_m-offset,.02,model);
    expect(front,Classification::ExternalSurface);CHECK(!front.has_first_surface);
  }
  const auto distant=classify(ray,hit.first_axial_depth_m+.03,.02,model);
  expect(distant,Classification::ExternalSurface);CHECK(!distant.has_first_surface); // no virtual occlusion of measured ground
  const auto far=classify({-.004,0,1},1,.02,model);expect(far,Classification::ExternalSurface);
  CHECK(!far.has_first_surface && far.triangle_tests==0); // expanded AABB is only acceleration
  same_result(hit,classify(ray,1,.02,{link(patch)},QueryBudget{}));
  Vec3 unit=ray;const auto norm=std::hypot(unit.x,unit.z);unit.x/=norm;unit.z/=norm;
  expect(classify_range({0,0,0},unit,1,.02,model),Classification::ExternalSurface); // no LiDAR near-miss policy
  Mesh exact;CHECK(exact.build({plane(.98)},BuildLimits{}));
  PreparedModel both({link(patch,"patch"),link(exact,"original")},QueryBudget{});
  auto original=classify(ray,.95,.02,both);expect(original,Classification::ExternalSurface);
  CHECK(original.first_axial_depth_m==.98 && original.link_index==1);
  original=classify(ray,1.01,.02,both);expect(original,Classification::ModelConflict);
  CHECK(original.first_axial_depth_m==.98 && original.link_index==1); // would match the patch: first hit must win
  QueryBudget low;low.max_triangle_tests=2;unobserved(classify(ray,1,.02,PreparedModel({link(patch)},low)));
  low=QueryBudget{};low.max_node_visits=1;unobserved(classify(ray,1,.02,PreparedModel({link(patch)},low)));
  Mesh copied=patch;expect(classify(ray,1,.02,{link(copied)},QueryBudget{}),Classification::SelfSurface);
  Mesh moved=std::move(copied);expect(classify(ray,1,.02,{link(moved)},QueryBudget{}),Classification::SelfSurface);
  CHECK(patch.configure_boundary_patch(0,0,0.));unobserved(classify(ray,1,.02,model));
  expect(classify(ray,1,.02,{link(patch)},QueryBudget{}),Classification::ExternalSurface);
  CHECK(!patch.configure_boundary_patch(0,1,.0031));CHECK(!patch.configure_boundary_patch(1,1,.003));
  CHECK(!patch.configure_boundary_patch(1,0,0.));CHECK(!patch.configure_boundary_patch(0,0,.001));
  CHECK(!patch.configure_boundary_patch(0,1,0.));
}

void native_pixel_support() {
  const NativePixelSupport pixel{640,480,319,240,400,400,319,240};
  const Triangle edge{{{{.0003,-.001,1},{.0011,-.001,1},{.0003,.001,1}}}};
  Mesh body;CHECK(body.build({plane(.9),edge},{}));CHECK(body.configure_measured_surface(1,1));
  const std::vector<LinkObservation> links{link(body)};const PreparedModel model(links,{});
  auto query=[&](double z){return classify_body_surface({0,0,1},z,.03,model,pixel);};
  // The same 30mm metric bound applies with or without native raster support.
  for(double z:{.975,1.,1.025}) {
    const auto r=query(z);expect(r,Classification::SelfSurface);
    CHECK(r.finite_surface_distance_m<=.03);
    const auto plain=classify_body_surface({0,0,1},z,.03,model);
    expect(plain,Classification::SelfSurface);CHECK(plain.finite_surface_distance_m==r.finite_surface_distance_m);
  }
  for(double z:{.95,1.05}) {
    const auto r=query(z);CHECK(r.complete && r.classification!=Classification::SelfSurface);
  }
  // Two separated facets can span the pixel without supporting the return.
  Mesh layers;auto lower=edge,upper=edge;
  for(auto& v:lower.vertices)v.z=.8;
  for(auto& v:upper.vertices)v.z=1.2;
  CHECK(layers.build({lower,upper},{}));CHECK(layers.configure_measured_surface(0,2));
  const PreparedModel separated({link(layers)},{});
  auto r=classify_body_surface({0,0,1},1,.03,separated,pixel);
  expect(r,Classification::Unobserved);
  CHECK(r.ambiguous_min_axial_depth_m<=.8 && r.ambiguous_max_axial_depth_m>=1.2);
  auto invalid=pixel;invalid.width=0;
  same_result(classify_body_surface({0,0,1},1,.03,model),classify_body_surface({0,0,1},1,.03,model,invalid));
  invalid=pixel;invalid.fx=NAN;
  same_result(classify_body_surface({0,0,1},1,.03,model),classify_body_surface({0,0,1},1,.03,model,invalid));
  // Finite edge/vertex round caps are bounded; an infinite plane is not body.
  Mesh triangle;CHECK(triangle.build({{{{{0,0,1},{1,0,1},{0,1,1}}}}},{}));
  const PreparedModel finite({link(triangle)},{});
  for(double d:{.0299,.0301,.05}) {
    for(Vec3 p:std::vector<Vec3>{{.5,-d,1},{-d,0,1},{.2,.2,1+d}}) {
      r=classify_body_surface({p.x/p.z,p.y/p.z,1},p.z,.03,finite);
      CHECK(r.complete);CHECK((r.classification==Classification::SelfSurface)==(d<.03));
    }
  }
  r=classify_body_surface({2,2,1},1,.03,finite);expect(r,Classification::ExternalSurface);
  // Old axial matches must not bypass the shared metric bound on a slope.
  Mesh slope;CHECK(slope.build({{{{{0,-2,2},{2,-2,0},{1,2,1}}}}},{}));
  const PreparedModel inclined({link(slope)},{});
  r=classify_body_surface({1,0,1},1.02,.03,inclined);expect(r,Classification::SelfSurface);
  r=classify_body_surface({1,0,1},1.025,.03,inclined);CHECK(r.complete && r.classification!=Classification::SelfSurface);
  // Zero/tight tolerance, rigid source transforms, expired snapshots and all
  // geometry budgets retain their explicit contracts.
  CHECK(classify_body_surface({0,0,1},1.025,.01,model).classification!=Classification::SelfSurface);
  QueryBudget budget;budget.max_node_visits=1;
  CHECK(!classify_body_surface({0,0,1},1,.03,PreparedModel(links,budget),pixel).complete);
  budget={};budget.max_triangle_tests=1;
  CHECK(!classify_body_surface({0,0,1},1,.03,PreparedModel(links,budget),pixel).complete);
  CHECK(body.configure_measured_surface(0,0));
  CHECK(!classify_body_surface({0,0,1},1,.03,model,pixel).complete);
}

void finite_surface_bounds_budget() {
  Mesh mesh;CHECK(mesh.build({plane(1)},{}));
  std::vector<LinkObservation> links;
  for(unsigned i=0;i<12;++i)links.push_back(link(mesh,"leg_"+std::to_string(i)));
  // Twelve visibility roots plus twelve finite-distance roots, even when the
  // cached union rejects this point. The acceleration earns no budget credit.
  for(unsigned nodes:{23u,24u}) {
    QueryBudget budget;budget.max_node_visits=nodes;
    auto r=classify_body_surface({10,10,1},2,.03,PreparedModel(links,budget));
    CHECK(r.node_visits==nodes && r.triangle_tests==0);
    CHECK(r.complete==(nodes==24));
    CHECK(std::strcmp(r.reason,nodes==24?"no_self_intersection":"node_budget")==0);
    CHECK(r.classification==(nodes==24?Classification::ExternalSurface:Classification::Unobserved));
  }
  const PreparedModel old(links,{});
  CHECK(mesh.build({plane(2)},{}));
  const auto stale=classify_body_surface({10,10,1},2,.03,old);
  CHECK(!stale.complete && stale.node_visits==0);
  CHECK(std::strcmp(stale.reason,"model_identity_or_source_tf_invalid")==0);
}

void stereo_body_occlusion() {
  Mesh mesh;CHECK(mesh.build({Triangle{{Vec3{.055,-.02,.5},Vec3{.065,-.02,.5},Vec3{.06,.02,.5}}}},BuildLimits{}));
  QueryBudget budget;PreparedModel model({link(mesh)},budget);
  NativePixelSupport pixel{200,100,50,50,100,100,50,50,.12};
  const auto left=classify_body_surface({0,0,1},1.,.03,model,pixel);
  CHECK(left.complete&&left.classification==Classification::ExternalSurface&&!left.has_first_surface);
  const auto blocked=classify_native_body_filter({0,0,1},1.,.03,model,pixel);
  CHECK(measurement_body_occluded(blocked));CHECK(blocked.classification==Classification::ExternalSurface);
  CHECK(!blocked.has_first_surface&&!blocked.has_measured_surface);
  CHECK(blocked.node_visits>left.node_visits&&blocked.triangle_tests>left.triangle_tests);
  auto missing=pixel;missing.stereo_baseline_m=0.;
  const auto unqualified=classify_native_body_filter({0,0,1},1.,.03,model,missing);
  CHECK(!measurement_body_occluded(unqualified));
  for(double invalid:{-1.,double(NAN),double(INFINITY),2.}) {
    missing.stereo_baseline_m=invalid;
    CHECK(!measurement_body_occluded(classify_native_body_filter({0,0,1},1.,.03,model,missing)));
  }
  CHECK(!measurement_body_occluded(classify_native_body_filter({0,0,1},.3,.03,model,pixel))); // real foreground
  auto other=pixel;other.u=100;
  CHECK(!measurement_body_occluded(classify_native_body_filter({.5,0,1},1.,.03,model,other))); // visible box
  auto exhausted=unqualified;exhausted.node_visits=budget.max_node_visits;
  const auto failed=apply_stereo_body_occlusion_policy({0,0,1},1.,.03,model,pixel,exhausted);
  CHECK(!failed.complete&&!measurement_body_occluded(failed));CHECK(std::strcmp(failed.reason,"node_budget")==0);
  CHECK(!measurement_body_occluded(apply_stereo_body_occlusion_policy({0,0,2},1.,.03,model,pixel,left)));
  auto foreground=left;foreground.has_first_surface=true;
  same_result(foreground,apply_stereo_body_occlusion_policy({0,0,1},1.,.03,model,pixel,foreground));
}
int main(int argc,char** argv) {
  if(argc==3 && std::string(argv[1])=="--frame1258") {
    std::ifstream file(argv[2],std::ios::binary);
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)),{});
    Mesh shoe;
    CHECK(bytes.size()>84 && shoe.load_binary_stl(bytes.data(),bytes.size(),(bytes.size()-84)/50,{}));
    const PreparedModel model({link(shoe,"left_leg_ankle_roll_link")},{});
    // Both original belly endpoints, transformed to the left shoe at source_ns 1793761984707606000.
    CHECK(model.excludes_near_body_map_mark({.195711647531,.129702647709,-.045659687750}));
    CHECK(model.excludes_near_body_map_mark({.195730175324,.124728441543,-.045817199881}));
    return 0;
  }

  stereo_body_occlusion();
  finite_surface_bounds_budget();
  native_pixel_support();
  authoritative_measured_surface();
  authorized_finite_boundary();
  prepared_equivalence();
  Mesh mesh;std::string error;BuildLimits limits;QueryBudget budget;
  CHECK(mesh.build({plane(1)},limits,&error));CHECK(error.empty());
  auto owner=link(mesh);
  const PreparedModel range_model({owner},budget);
  expect(classify_range({0,0,2},{0,0,-1},1,0,range_model),Classification::SelfSurface);
  expect(classify_range({0,0,2},{0,0,-1},.8,.01,range_model),Classification::ExternalSurface);
  expect(classify_range({0,0,2},{0,0,-1},1.2,.01,range_model),Classification::ModelConflict);
  // Zero optical Z is valid for an omnidirectional ray after frame rotation.
  auto side_observation=owner;side_observation.optical_from_link.rotation={0,0,1,0,1,0,-1,0,0};
  const PreparedModel side_model({side_observation},budget);
  expect(classify_range({2,0,0},{-1,0,0},1,0,side_model),Classification::SelfSurface);
  unobserved(classify_range({0,0,0},{0,0,2},1,.01,range_model));
  unobserved(classify_range({NAN,0,0},{0,0,1},1,.01,range_model));
  // Matching remains relative to the provided lens origin, not frame origin.
  expect(classify_range({0,0,.25},{0,0,1},.75,0,range_model),Classification::SelfSurface);
  expect(classify_range({0,0,.25},{0,0,1},1.,.01,range_model),Classification::ModelConflict);
  auto r=classify({0,0,1},1,0,{owner},budget);
  expect(r,Classification::SelfSurface);CHECK(r.has_first_surface && r.first_axial_depth_m==1 && r.link_index==0 && r.triangle_index==0);
  expect(classify({0,0,1},.75,.125,{owner},budget),Classification::ExternalSurface);
  expect(classify({0,0,1},1.5,.125,{owner},budget),Classification::ModelConflict); // model overcoverage cannot hide a farther clear depth
  expect(classify({0,0,1},.875,.125,{owner},budget),Classification::SelfSurface);
  expect(classify({0,0,1},1.125,.125,{owner},budget),Classification::SelfSurface);
  expect(classify({0,0,1},1.1250001,.125,{owner},budget),Classification::ModelConflict);
  for(double depth:{0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
    expect(classify({0,0,1},depth,.01,{owner},budget),Classification::Invalid);

  // Unnormalized z=1 ray: axial depth1, while its Euclidean range is sqrt(2).
  expect(classify({1,0,1},1,1e-12,{owner},budget),Classification::SelfSurface);
  expect(classify({1,0,1},std::sqrt(2.),.01,{owner},budget),Classification::ModelConflict);
  unobserved(classify({0,0,.5},1,.01,{owner},budget));
  unobserved(classify({std::numeric_limits<double>::quiet_NaN(),0,1},1,.01,{owner},budget));
  for(double tolerance:{-1.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()})
    unobserved(classify({0,0,1},1,tolerance,{owner},budget));
  unobserved(classify({0,0,1},1,.01,{},budget));
  for(int fault=0;fault<5;++fault) {
    auto invalid=owner;
    if(fault==0)invalid.identity_valid=false;
    if(fault==1)invalid.tf_valid=false;
    if(fault==2)invalid.covers_source_time=false; // stale/extrapolated source-time TF
    if(fault==3)invalid.mesh=nullptr;
    if(fault==4)invalid.link_name.clear();
    unobserved(classify({0,0,1},1,.01,{owner,invalid},budget));
  }
  unobserved(classify({0,0,1},1,.01,{owner,owner},budget));
  for(int fault=0;fault<4;++fault) {
    auto invalid=owner;
    if(fault==0)invalid.optical_from_link.rotation[0]=2; // scale
    if(fault==1)invalid.optical_from_link.rotation[0]=-1; // reflection
    if(fault==2)invalid.optical_from_link.rotation[1]=.1; // shear
    if(fault==3)invalid.optical_from_link.translation.x=std::numeric_limits<double>::infinity();
    unobserved(classify({0,0,1},1,.01,{invalid},budget));
  }

  Mesh near,far;CHECK(near.build({plane(1)},limits));CHECK(far.build({plane(2)},limits));
  const auto n=link(near,"near"),f=link(far,"far");
  r=classify({0,0,1},1,.01,{f,n},budget);expect(r,Classification::SelfSurface);CHECK(r.link_index==1 && r.first_axial_depth_m==1);
  r=classify({0,0,1},2,.01,{f,n},budget);expect(r,Classification::ModelConflict);CHECK(r.link_index==1);
  r=classify({0,0,1},1,.01,{n,f},budget);expect(r,Classification::SelfSurface);CHECK(r.link_index==0);

  Mesh local;CHECK(local.build({plane(0)},limits));auto shifted=link(local);shifted.optical_from_link.translation.z=1;
  expect(classify({0,0,1},1,.001,{shifted},budget),Classification::SelfSurface);
  Mesh rotated;CHECK(rotated.build({{{{{1,-2,-2},{1,2,-2},{1,0,2}}}}},limits));auto turn=link(rotated);
  turn.optical_from_link.rotation={{0,0,-1,0,1,0,1,0,0}};
  expect(classify({0,0,1},1,.001,{turn},budget),Classification::SelfSurface);
  Triangle reversed=plane(1);std::swap(reversed.vertices[0],reversed.vertices[2]);
  CHECK(local.build({reversed},limits));expect(classify({0,0,1},1,.001,{link(local)},budget),Classification::SelfSurface);

  // A ray inside the AABB but outside the actual triangle is NOT self evidence.
  CHECK(local.build({{{{{-1,-1,1},{1,-1,1},{-1,1,1}}}}},limits));
  r=classify({.75,.75,1},1,.01,{link(local)},budget);expect(r,Classification::ExternalSurface);CHECK(!r.has_first_surface);
  CHECK(local.build({plane(-1)},limits));r=classify({0,0,1},1,.01,{link(local)},budget);
  expect(r,Classification::ExternalSurface);CHECK(!r.has_first_surface);
  CHECK(local.build({plane(1e-12)},limits));r=classify({0,0,1},1,.01,{link(local)},budget);
  expect(r,Classification::ExternalSurface);CHECK(!r.has_first_surface); // near-zero intersection excluded
  // Origin between two surfaces: first forward exit, not a whole model shadow.
  CHECK(local.build({plane(-1),plane(1)},limits));expect(classify({0,0,1},1,.001,{link(local)},budget),Classification::SelfSurface);

  BuildLimits split=limits;split.leaf_triangles=1;
  CHECK(local.build({plane(1),plane(2)},split));auto tiny=budget;tiny.max_node_visits=1;
  unobserved(classify({0,0,1},1,.01,{link(local)},tiny));
  CHECK(local.build({plane(1),plane(2)},limits));tiny=budget;tiny.max_triangle_tests=1;
  unobserved(classify({0,0,1},1,.01,{link(local)},tiny)); // a partial first hit is discarded
  tiny=budget;tiny.max_links=1;unobserved(classify({0,0,1},1,.01,{n,f},tiny));
  tiny=budget;tiny.max_triangle_tests=0;unobserved(classify({0,0,1},1,.01,{owner},tiny));
  tiny=budget;tiny.max_links=65;unobserved(classify({0,0,1},1,.01,{owner},tiny));

  Triangle degenerate=plane(1);degenerate.vertices[2]=degenerate.vertices[1];
  CHECK(!local.build({degenerate},limits,&error));CHECK(!local.ready());
  unobserved(classify({0,0,1},1,.01,{link(local)},budget));
  Triangle nan=plane(1);nan.vertices[0].x=std::numeric_limits<double>::quiet_NaN();CHECK(!local.build({nan},limits));
  BuildLimits cap=limits;cap.max_triangles=1;cap.leaf_triangles=1;CHECK(!local.build({plane(1),plane(2)},cap));
  cap=limits;cap.leaf_triangles=1;cap.max_nodes=1;CHECK(!local.build({plane(1),plane(2)},cap));
  cap=limits;cap.leaf_triangles=1;cap.max_depth=1;CHECK(!local.build({plane(1),plane(2)},cap));
  CHECK(!local.build({},limits));

  std::vector<Triangle> many;for(int i=0;i<4096;++i)many.push_back(plane(1,5.*i));
  CHECK(local.build(many,split));r=classify({0,0,1},1,.001,{link(local)},budget);
  expect(r,Classification::SelfSurface);CHECK(r.node_visits<64 && r.triangle_tests==1); // bounded acceleration, not per-ray full scan

  const auto bytes=stl({plane(1)});CHECK(local.load_binary_stl(bytes.data(),bytes.size(),1,limits,&error));
  expect(classify({0,0,1},1,0,{link(local)},budget),Classification::SelfSurface);
  CHECK(!local.load_binary_stl(bytes.data(),bytes.size()-1,1,limits));CHECK(!local.ready());
  auto extra=bytes;extra.push_back(0);CHECK(!local.load_binary_stl(extra.data(),extra.size(),1,limits));
  CHECK(!local.load_binary_stl(bytes.data(),bytes.size(),2,limits));
  CHECK(!local.load_binary_stl(nullptr,bytes.size(),1,limits));
  auto invalid_bytes=stl({nan});CHECK(!local.load_binary_stl(invalid_bytes.data(),invalid_bytes.size(),1,limits));
  invalid_bytes=stl({degenerate});CHECK(!local.load_binary_stl(invalid_bytes.data(),invalid_bytes.size(),1,limits));
  auto arbitrary_normal=bytes;for(int i=0;i<12;++i)arbitrary_normal[84+i]=255;
  CHECK(local.load_binary_stl(arbitrary_normal.data(),arbitrary_normal.size(),1,limits)); // ignore STL normal, use vertices
  cap=limits;cap.max_triangles=0;CHECK(!local.load_binary_stl(bytes.data(),bytes.size(),1,cap));

  // Optional local asset smoke: pairs of file path and independently pinned count.
  // Filesystem is test-only; the production component takes bytes and has no SHA/file dependency.
  CHECK((argc-1)%2==0);std::size_t asset_triangles=0;
  for(int i=1;i<argc;i+=2) {
    std::ifstream stream(argv[i],std::ios::binary);CHECK(stream.good());
    std::vector<std::uint8_t> data((std::istreambuf_iterator<char>(stream)),{});
    const auto count=static_cast<std::size_t>(std::stoull(argv[i+1]));
    CHECK(local.load_binary_stl(data.data(),data.size(),count,limits,&error));
    CHECK(local.ready() && local.triangle_count()==count);asset_triangles+=count;
  }
  std::cout << "dynamic_self_geometry checks=" << checks << " asset_triangles=" << asset_triangles << '\n';
  return 0;
}
