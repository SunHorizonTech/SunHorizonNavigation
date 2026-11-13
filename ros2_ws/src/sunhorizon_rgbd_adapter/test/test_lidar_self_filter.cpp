#include "sunhorizon_rgbd_adapter/lidar_self_filter.hpp"
#include <iostream>
#include <set>
using namespace sunhorizon_rgbd_adapter;
namespace dg=sunhorizon_obstacle_coverage::dynamic_self_geometry;
static int checks=0;
#define CHECK(value) do{++checks;if(!(value))throw std::runtime_error(#value);}while(false)
int main(){try{
  dg::Mesh mesh;CHECK(mesh.build({{{{{-2,-2,1},{2,-2,1},{0,2,1}}}}},{}));
  CHECK(mesh.origin_radius_bound()>=3.);
  std::vector<dg::LinkObservation> links{{&mesh,"foot",{},true,true,true}};
  auto first=std::make_shared<const dg::PreparedModel>(links,dg::QueryBudget{});
  links[0].optical_from_link.translation.z=1.;
  auto second=std::make_shared<const dg::PreparedModel>(links,dg::QueryBudget{});
  auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header.frame_id="lidar";cloud->height=1;cloud->width=6;cloud->point_step=20;cloud->row_step=120;
  for(std::size_t i=0;i<4;++i){sensor_msgs::msg::PointField field;
    field.name=i<3?std::string(1,"xyz"[i]):"timestamp";field.offset=i*4;
    field.datatype=field.FLOAT32;field.count=1;cloud->fields.push_back(field);}
  cloud->data.resize(120);
  // Same endpoint at a different source time must use a different model.
  const std::array<float,6> depths{1.f,.5f,1.2f,NAN,1.f,10.f};
  const float later=.02f;
  for(std::size_t i=0;i<6;++i){std::memcpy(cloud->data.data()+i*20+8,&depths[i],4);
    if(i==4)std::memcpy(cloud->data.data()+i*20+12,&later,4);
    cloud->data[i*20+16]=uint8_t(i+1);}
  auto origins=std::make_shared<std::vector<std::array<float,3>>>(6,std::array<float,3>{0,0,0});
  (*origins)[0]={0,0,2}; // Valid negative-Z ray from the native lens origin.
  SelfExclusionDirectInput input;input.cloud=cloud;input.stamp.source_ns=1000000000LL;input.ray_origins=origins;
  const int64_t second_stamp=input.stamp.source_ns+std::llround(double(later)*1.e9);
  std::set<int64_t> queries;
  const LidarModelProvider provide=[&](const std::string& frame,int64_t time){
    CHECK(frame=="lidar");queries.insert(time);return time==second_stamp?second:first;};
  auto result=filter_lidar_self_points(input,provide,.01,false,3.);
  CHECK(result.cloud && result.cloud->data==cloud->data && result.removed_count==0);
  CHECK(result.self_surface_count==1 && result.conflict_count==2 && result.external_count==2 && result.outside_cad_envelope_count==1 && result.invalid_count==1);
  CHECK(result.ground_blocked==std::vector<uint8_t>({1,0,1,1,0,1}));
  CHECK(result.unique_prepared_models==2);
  CHECK(result.source_time_groups==2 && queries==std::set<int64_t>({1000000000LL,second_stamp}));
  // Shared immutable geometry never merges original source groups or skips
  // their provider eligibility checks; all original records stay in order.
  std::vector<int64_t> shared_queries;
  const auto shared_result=filter_lidar_self_points(input,[&](const auto&,int64_t time){
    shared_queries.push_back(time);return first;},.01,false,3.);
  CHECK(shared_queries==std::vector<int64_t>({input.stamp.source_ns,second_stamp}));
  CHECK(shared_result.source_time_groups==2 && shared_result.unique_prepared_models==1);
  CHECK(shared_result.cloud->data==cloud->data && shared_result.ray_origins==*origins);
  CHECK(shared_result.sample_ids==std::vector<uint64_t>({0,1,2,3,4,5}));
  CHECK(shared_result.ground_blocked==std::vector<uint8_t>({1,0,1,1,1,1}));
  result=filter_lidar_self_points(input,provide,.01,true,3.);
  CHECK(result.removed_count==1 && result.retained_count==5 && result.cloud->width==5);
  CHECK(result.cloud->data==std::vector<uint8_t>(cloud->data.begin()+20,cloud->data.end()));
  CHECK(result.ground_blocked==std::vector<uint8_t>({0,1,1,0,1}));
  const ExclusionBox box{{-.1,-.1,.49},{.1,.1,.51}};
  const auto compacted=exclude_self_framed_volumes(*result.cloud,{box},{ExclusionTransform{}},0.,&result.ground_blocked,&result.ray_origins,&result.sample_ids);
  CHECK(compacted.cloud && compacted.removed_count==1 && compacted.ground_blocked==std::vector<uint8_t>({1,1,0,1}));
  CHECK((result.ray_origins==std::vector<std::array<float,3>>(origins->begin()+1,origins->end())));
  CHECK((compacted.ray_origins==std::vector<std::array<float,3>>(origins->begin()+2,origins->end())));
  CHECK(result.sample_ids==std::vector<uint64_t>({1,2,3,4,5}));
  CHECK(compacted.sample_ids==std::vector<uint64_t>({2,3,4,5}));
  CHECK(result.cloud->header==cloud->header && result.cloud->fields==cloud->fields);
  // One unavailable source group prevents publication of any partial deletion.
  const LidarModelProvider missing=[&](const std::string&,int64_t time){return time==second_stamp?
    std::shared_ptr<const dg::PreparedModel>{}:first;};
  result=filter_lidar_self_points(input,missing,.01,true,3.);
  CHECK(result.unobserved_count==1 && result.removed_count==0 && result.cloud->data==cloud->data);
  // An incomplete query budget has the same conservative whole-frame outcome.
  auto budget=dg::QueryBudget{};budget.max_node_visits=0;
  auto invalid_model=std::make_shared<const dg::PreparedModel>(links,budget);
  result=filter_lidar_self_points(input,[&](const auto&,int64_t){return invalid_model;},.01,true,3.);
  CHECK(result.removed_count==0 && result.unobserved_count==5 && result.cloud->data==cloud->data);
  // Both endpoints are outside the CAD radius. Only a real ray missing the
  // body is External; a ray crossing its surface remains a conflict.
  auto far_cloud=std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud);
  far_cloud->width=2;far_cloud->row_step=40;far_cloud->data.assign(40,0);
  const float far_z=10.f,side_x=20.f;
  std::memcpy(far_cloud->data.data()+8,&far_z,4);
  std::memcpy(far_cloud->data.data()+20,&side_x,4);
  std::memcpy(far_cloud->data.data()+28,&far_z,4);
  input.cloud=far_cloud;input.ray_origins=std::make_shared<std::vector<std::array<float,3>>>(2,std::array<float,3>{0,0,0});
  result=filter_lidar_self_points(input,provide,.01,false,3.);
  CHECK(result.outside_cad_envelope_count==2 && result.conflict_count==1 && result.external_count==1);
  CHECK(result.ground_blocked==std::vector<uint8_t>({1,0}) && result.removed_count==0);
  // Audit uses the already evaluated point-time model and original SDK origin.
  // Its disabled/enabled forms must produce byte-identical normal outputs.
  auto negative=std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud);
  for(std::size_t i=0;i<6;++i){const float z=-depths[i];std::memcpy(negative->data.data()+i*20+8,&z,4);}
  auto actual_origins=std::make_shared<std::vector<std::array<float,3>>>(6,std::array<float,3>{.01f,.02f,.03f});
  input.cloud=negative;input.ray_origins=actual_origins;
  links[0].optical_from_link.translation.z=-2.;
  auto down_first=std::make_shared<const dg::PreparedModel>(links,dg::QueryBudget{});
  links[0].optical_from_link.translation.z=-3.;
  auto down_second=std::make_shared<const dg::PreparedModel>(links,dg::QueryBudget{});
  std::size_t model_calls=0;
  const LidarModelProvider down=[&](const auto&,int64_t ns){++model_calls;return ns==second_stamp?down_second:down_first;};
  const auto plain=filter_lidar_self_points(input,down,.01,true,3.);
  CHECK(!plain.audit);const auto plain_calls=model_calls;model_calls=0;
  const auto audited=filter_lidar_self_points(input,down,.01,true,3.,true);
  CHECK(audited.audit && model_calls==plain_calls);
  CHECK(audited.cloud->data==plain.cloud->data && audited.sample_ids==plain.sample_ids);
  CHECK(audited.ground_blocked==plain.ground_blocked && audited.ray_origins==plain.ray_origins);
  CHECK(audited.removed_count==plain.removed_count && audited.external_count==plain.external_count);
  bool saw_first=false,saw_later=false;
  for(const auto& point:audited.audit->points){
    CHECK(point.ray_origin==(*actual_origins)[point.sample_id]);CHECK(point.links.size()==1);
    CHECK(point.links[0].triangle_count==1);
    if(point.sample_id==0){saw_first=true;CHECK(point.source_ns==input.stamp.source_ns);
      CHECK(point.links[0].optical_from_link.translation.z==-2.);}
    if(point.sample_id==4){saw_later=true;CHECK(point.source_ns==second_stamp);
      CHECK(point.links[0].optical_from_link.translation.z==-3.);}
  }
  CHECK(saw_first && saw_later); // first row may be removed, but remains raw sample0 in diagnostic
  bool diagnostic_ran=false;
  CHECK(!try_lidar_diagnostic([&]{diagnostic_ran=true;throw std::bad_alloc();}));
  CHECK(diagnostic_ran);
  auto large=std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud);
  large->width=200;large->row_step=4000;large->data.assign(4000,0);
  for(std::size_t i=0;i<200;++i){const float x=float(i)*.001f,z=-.5f;
    std::memcpy(large->data.data()+i*20,&x,4);std::memcpy(large->data.data()+i*20+8,&z,4);}
  input.cloud=large;input.ray_origins=std::make_shared<const std::vector<std::array<float,3>>>(200,std::array<float,3>{0,0,0});
  const auto bounded=filter_lidar_self_points(input,down,.01,false,3.,true);
  CHECK(bounded.audit && bounded.audit->points.size()==32);
  CHECK(bounded.cloud->data==large->data && bounded.removed_count==0);
  for(const auto& point:bounded.audit->points)CHECK(point.sample_id<32);

  input.ray_origins.reset();bool rejected=false;
  try{filter_lidar_self_points(input,provide,.01,true);}catch(const std::invalid_argument&){rejected=true;}
  CHECK(rejected);
  std::cout<<"PASS "<<checks<<" checks\n";
}catch(const std::exception& error){std::cerr<<"FAIL "<<checks<<" "<<error.what()<<"\n";return 1;}}
