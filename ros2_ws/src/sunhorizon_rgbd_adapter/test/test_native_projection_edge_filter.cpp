#include "sunhorizon_rgbd_adapter/fused_depth_self_filter.hpp"
#include <iostream>
#include <stdexcept>
#include <cstring>
namespace dg=sunhorizon_obstacle_coverage::dynamic_self_geometry;
namespace rgbd=sunhorizon_rgbd_adapter;
#define CHECK(x) do {if(!(x))throw std::runtime_error(#x);}while(false)
int main() {try {
  // Finite vertical silhouette at u=101.5; source pixel center is u=100.
  // Its depth is .5m; the original native cell +/- .5 does not intersect it.
  dg::Mesh mesh;std::vector<dg::Triangle> triangles={
    {{{{.0075,-.1,.5},{.1,-.1,.5},{.1,.1,.5}}}},
    {{{{.0075,-.1,.5},{.1,.1,.5},{.0075,.1,.5}}}}};
  CHECK(mesh.build(triangles,{}));std::vector<dg::LinkObservation> links{{&mesh,"finite",{},true,true,true}};
  rgbd::DynamicFilterBudget budget;dg::PreparedModel model(links,budget.query);
  dg::NativePixelSupport pixel{201,1,100,0,100,100,100,0};
  const auto old=dg::classify_body_surface({0,0,1},.9,.03,model,pixel);
  CHECK(old.complete&&old.classification==dg::Classification::ExternalSurface);
  const auto edge=dg::classify_native_body_filter({0,0,1},.9,.03,model,pixel);
  CHECK(dg::projection_edge_suppressed(edge));
  CHECK(edge.classification==old.classification&&!edge.has_first_surface&&!edge.has_measured_surface);
  CHECK(!std::isfinite(edge.finite_surface_distance_m));
  // Ordinary nominal conflict and actual finite-surface Self stay geometric.
  auto inside=pixel;inside.u=103;
  CHECK(!dg::projection_edge_suppressed(dg::classify_native_body_filter({.03,0,1},.9,.03,model,inside)));
  CHECK(dg::classify_native_body_filter({.03,0,1},.5,.03,model,inside).classification==dg::Classification::SelfSurface);
  // Exact center distance, no additional half pixel or square-corner extension.
  auto outside=pixel;outside.u=99;
  CHECK(!dg::projection_edge_suppressed(dg::classify_native_body_filter({-.01,0,1},.9,.03,model,outside)));
  dg::Mesh corner;CHECK(corner.build({
    {{{{.0075,.0075,.5},{.1,.0075,.5},{.1,.1,.5}}}},
    {{{{.0075,.0075,.5},{.1,.1,.5},{.0075,.1,.5}}}}},{}));
  dg::PreparedModel corner_model({{&corner,"corner",{},true,true,true}},budget.query);
  CHECK(!dg::projection_edge_suppressed(dg::classify_native_body_filter({0,0,1},.9,.03,corner_model,pixel)));
  auto absent=pixel;absent.width=0;
  CHECK(!dg::projection_edge_suppressed(dg::classify_native_body_filter({0,0,1},.9,.03,model,absent)));
  auto limited=budget.query;limited.max_triangle_tests=old.triangle_tests;
  dg::PreparedModel bounded(links,limited);
  CHECK(!dg::classify_native_body_filter({0,0,1},.9,.03,bounded,pixel).complete);
  sensor_msgs::msg::Image image;image.header.frame_id="optical";image.header.stamp.sec=1;
  image.width=201;image.height=1;image.step=402;image.encoding="16UC1";image.data.resize(402);
  image.data[200]=0x84;image.data[201]=0x03; // native center index100, 900mm.
  sensor_msgs::msg::CameraInfo info;info.header=image.header;info.width=201;info.height=1;
  info.k={100,0,100,0,100,0,0,0,1};info.r={1,0,0,0,1,0,0,0,1};
  info.p={100,0,100,0,0,100,0,0,0,0,1,0};info.distortion_model="plumb_bob";info.d={0,0,0,0,0};
  CHECK(rgbd::qualified_native_depth_raster(image,info,.001));
  auto run=[&](rgbd::BodyGeometryWorker* worker){return rgbd::project_depth_filter_dynamic_self(image,info,.001,.1,2.,1,model,.03,budget,nullptr,INFINITY,worker);};
  const auto serial=run(nullptr);rgbd::BodyGeometryWorker worker([]{});const auto parallel=run(&worker);
  for(const auto* r:{&serial,&parallel}) {
    CHECK(r->filter.complete&&r->filter.input_count==1&&r->filter.retained_count==1&&r->filter.removed_count==0);
    CHECK(r->filter.ground_blocked==std::vector<uint8_t>{2});
    CHECK(r->self_support.empty()&&r->retained_sample_ids==std::vector<uint64_t>{100});
    CHECK(r->filter.classifications[std::size_t(dg::Classification::SelfSurface)]==0);
  }
  CHECK(serial.cloud.data==parallel.cloud.data);
  CHECK(serial.filter.node_visits==parallel.filter.node_visits&&serial.filter.triangle_tests==parallel.filter.triangle_tests);
  info.header.stamp.sec=2;const auto fallback=run(nullptr);
  CHECK(fallback.filter.complete&&fallback.filter.ground_blocked==std::vector<uint8_t>{0});
  std::cout<<"PASS native radius-2 edge policy, unchanged geometry, retained audit conservation and serial/helper parity\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
