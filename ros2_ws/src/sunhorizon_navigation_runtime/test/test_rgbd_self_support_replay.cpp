// One externally supplied, SHA-bound writer frame. No ROS graph or live state.
#include <sunhorizon_rgbd_adapter/fused_depth_self_filter.hpp>
#include <sunhorizon_rgbd_adapter/depth_components.hpp>
#include <nav2_util/direct_cloud.hpp>
#include <nav2_costmap_2d/voxel_measurement_support.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nlohmann/json.hpp>
#include <openssl/sha.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <set>
using Json=nlohmann::json;
namespace dg=sunhorizon_obstacle_coverage::dynamic_self_geometry;
namespace rgbd=sunhorizon_rgbd_adapter;
static void require(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}
static std::vector<uint8_t> bytes(const std::filesystem::path& path){std::ifstream f(path,std::ios::binary);require(bool(f),"missing replay asset");return {std::istreambuf_iterator<char>(f),{}};}
static Json read(const std::filesystem::path& path){std::ifstream f(path);require(bool(f),"missing replay JSON");return Json::parse(f);}
static std::string sha(const std::vector<uint8_t>& b){unsigned char digest[SHA256_DIGEST_LENGTH];SHA256(b.data(),b.size(),digest);std::ostringstream s;s<<std::hex<<std::setfill('0');for(auto c:digest)s<<std::setw(2)<<unsigned(c);return s.str();}
int main(int argc,char**argv){try {
 require(argc==6,"usage: replay seal.json primary.bin camera-info.json mesh-manifest.json models-directory");
 const auto seal=read(argv[1]),info=read(argv[3]),manifest=read(argv[4]);
 const auto& c=seal.at("classification");const auto& writer=seal.at("actual_writer");
 require(writer.at("seq")==7881 && writer.at("voxel")==Json::array({521,234,0}),"expected captured global7881");
 sensor_msgs::msg::Image depth;const auto& raw=seal.at("primary_raw");
 depth.data=bytes(argv[2]);require(sha(depth.data)==raw.at("sha256"),"raw SHA mismatch");
 require(sha(bytes(argv[3]))==raw.at("camera_info_sha256"),"CameraInfo SHA mismatch");
 depth.width=raw.at("width");depth.height=raw.at("height");depth.step=raw.at("step");depth.encoding=raw.at("encoding");depth.is_bigendian=raw.at("is_bigendian").get<int>();
 const int64_t stamp=raw.at("header_ns");depth.header.stamp.sec=stamp/1000000000;depth.header.stamp.nanosec=stamp%1000000000;depth.header.frame_id=raw.at("frame_id");
 sensor_msgs::msg::CameraInfo camera;camera.header=depth.header;camera.width=info.at("width");camera.height=info.at("height");camera.distortion_model=info.at("distortion_model");camera.d=info.at("d").get<std::vector<double>>();camera.k=info.at("k").get<std::array<double,9>>();camera.r=info.at("r").get<std::array<double,9>>();camera.p=info.at("p").get<std::array<double,12>>();
 require(info.at("binning_x")==0&&info.at("binning_y")==0,"unexpected native binning");
 std::vector<dg::Mesh> meshes(12);std::vector<dg::LinkObservation> links;
 const auto& bodies=seal.at("body_model").at("links");require(manifest.size()==12&&bodies.size()==12,"mesh count mismatch");
 for(std::size_t i=0;i<12;++i){const auto path=std::filesystem::path(argv[5])/std::filesystem::path(manifest[i].at("path").get<std::string>()).filename();const auto data=bytes(path);require(sha(data)==manifest[i].at("sha256"),"mesh SHA mismatch");const auto& b=bodies[i];std::string error;
  require(meshes[i].load_binary_stl(data.data(),data.size(),b.at("triangle_count"),{},&error),error.c_str());
  require(meshes[i].configure_boundary_patch(b.at("boundary_triangle_begin"),b.at("boundary_triangle_count"),b.at("boundary_tolerance_m"),&error),error.c_str());
  require(meshes[i].configure_measured_surface(b.at("measured_triangle_begin"),b.at("measured_triangle_count"),&error),error.c_str());
  dg::RigidTransform tf;tf.rotation=b.at("optical_from_link").at("rotation").get<std::array<double,9>>();const auto t=b.at("optical_from_link").at("translation").get<std::array<double,3>>();tf.translation={t[0],t[1],t[2]};links.push_back({&meshes[i],manifest[i].at("link"),tf,true,true,true});
 }
 rgbd::DynamicFilterBudget budget;const auto& query=seal.at("body_model").at("query_budget");budget.query={query.at("max_links"),query.at("max_node_visits"),query.at("max_triangle_tests")};
 const dg::PreparedModel model(links,budget.query);
 // Source-bound6bff belly profile: component200, neighbour50mm, applied
 // only through1m; no new quality criterion is introduced by this replay.
 const auto mask=rgbd::depth_component_mask({depth.data.data(),depth.data.size(),depth.width,depth.height,depth.step,bool(depth.is_bigendian)},.001,.2,8.,.05,200,0,1.5);
 require(mask.valid,"original component mask incomplete");
 auto body=rgbd::project_depth_filter_dynamic_self(depth,camera,.001,.2,8.,c.at("source").at("raster_geometry").at("stride"),model,seal.at("body_model").at("self_tolerance_m"),budget,&mask.keep,1.0);
 require(body.filter.complete,"production BODY replay incomplete");
 const auto& recorded=seal.at("consumer_points");require(body.retained_sample_ids.size()==recorded.size(),"retained sample count changed");
 for(std::size_t i=0;i<recorded.size();++i)require(body.retained_sample_ids[i]==recorded[i][1],"retained original sample identity changed");
 const auto& id=c.at("source").at("identity");
 nav2_util::DirectCloud packet;packet.cloud=std::make_shared<sensor_msgs::msg::PointCloud2>(std::move(body.cloud));packet.kind=nav2_util::DirectCloudKind::ClassifiedObstacleFrame;packet.complete=true;packet.ground_qualified=true;packet.ground_basis=nav2_util::DirectGroundBasis::MeasuredSupport;
 packet.metadata={id.at("source_id"),id.at("epoch"),id.at("source_frame"),id.at("sequence"),id.at("source_ns"),id.at("first_receipt_ros_ns"),id.at("first_receipt_steady_ns")};
 auto classes=std::make_shared<std::vector<uint8_t>>();for(const auto& row:recorded)classes->push_back(row[8]);packet.classes=classes;
 auto support=std::make_shared<nav2_util::DirectEmpiricalGroundSupport>();support->metadata=packet.metadata;support->calibration_id=id.at("calibration_id");support->sample_ids=body.retained_sample_ids;support->self_support_complete=true;support->original_projected_samples=body.filter.input_count;
 std::set<uint64_t> rejected;for(const auto& v:c.at("source").at("measurement_quality").at("unreliable_sample_ids"))rejected.insert(v.get<uint64_t>());
 for(const auto& self:body.self_support){if(rejected.count(self.sample_id)){++support->self_support_quality_rejected_samples;continue;}support->self_samples.push_back({self.xyz,self.sample_id});}packet.empirical_ground_support=support;
 const auto& lease=c.at("reference").at("map_binding").at("alignment");
 // Structural packet validation only. Historical clocks are not renewed or
 // admitted to a running buffer; no replay current/liveness claim is made.
 packet.ground_validity=std::make_shared<nav2_util::DirectGroundValidity>(nav2_util::DirectGroundValidity{lease.at("geometry_valid_until_ros_ns"),lease.at("geometry_valid_until_steady_ns"),[]{return true;}});
 require(nav2_util::directCloudMatchesMetadata(packet),"new packet denominator identity/layout rejected");
 geometry_msgs::msg::TransformStamped tf;const auto& actual=seal.at("consumer").at("actual_header_transform");tf.header.frame_id=actual.at("parent");tf.child_frame_id=actual.at("child");tf.header.stamp=depth.header.stamp;const auto t=actual.at("translation").get<std::array<double,3>>();const auto q=actual.at("rotation_xyzw").get<std::array<double,4>>();tf.transform.translation.x=t[0];tf.transform.translation.y=t[1];tf.transform.translation.z=t[2];tf.transform.rotation.x=q[0];tf.transform.rotation.y=q[1];tf.transform.rotation.z=q[2];tf.transform.rotation.w=q[3];
 sensor_msgs::msg::PointCloud2 main_projected,self_source,self_projected;tf2::doTransform(*packet.cloud,main_projected,tf);
 self_source.header=depth.header;sensor_msgs::PointCloud2Modifier layout(self_source);layout.setPointCloud2Fields(3,"x",1,sensor_msgs::msg::PointField::FLOAT32,"y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32);layout.resize(support->self_samples.size());
 {sensor_msgs::PointCloud2Iterator<float>x(self_source,"x"),y(self_source,"y"),z(self_source,"z");for(const auto& self:support->self_samples){*x=self.position[0];*y=self.position[1];*z=self.position[2];++x;++y;++z;}}
 tf2::doTransform(self_source,self_projected,tf);
 struct Sample{uint64_t id;uint8_t kind;std::array<float,3> p;};std::vector<Sample> samples;
 {sensor_msgs::PointCloud2ConstIterator<float>x(main_projected,"x"),y(main_projected,"y"),z(main_projected,"z");for(std::size_t i=0;i<support->sample_ids.size();++i){require(std::abs(*x-recorded[i][5].get<float>())<1.e-6&&std::abs(*y-recorded[i][6].get<float>())<1.e-6&&std::abs(*z-recorded[i][7].get<float>())<1.e-6,"actual consumer projection mismatch");samples.push_back({support->sample_ids[i],classes->at(i),{*x,*y,*z}});++x;++y;++z;}}
 {sensor_msgs::PointCloud2ConstIterator<float>x(self_projected,"x"),y(self_projected,"y"),z(self_projected,"z");for(const auto& self:support->self_samples){samples.push_back({self.sample_id,4,{*x,*y,*z}});++x;++y;++z;}}
 std::sort(samples.begin(),samples.end(),[](const auto&a,const auto&b){return a.id<b.id;});
 nav2_costmap_2d::voxel_support::Frame frame;const auto origin=writer.at("origin").get<std::array<double,3>>();const auto size=writer.at("grid_size").get<std::array<unsigned,3>>();const double resolution=writer.at("resolution"),zr=writer.at("z_resolution");const double range=writer.at("obstacle_max_range"),minimum=writer.at("obstacle_min_range");
 for(const auto& sample:samples){const auto&p=sample.p;const double distance=(p[0]-t[0])*(p[0]-t[0])+(p[1]-t[1])*(p[1]-t[1])+(p[2]-t[2])*(p[2]-t[2]);if(distance>=range*range||distance<minimum*minimum||p[0]<origin[0]||p[1]<origin[1])continue;const unsigned x=(p[0]-origin[0])/resolution,y=(p[1]-origin[1])/resolution,z=(std::max(double(p[2]),origin[2])-origin[2])/zr;if(x>=size[0]||y>=size[1]||z>=size[2])continue;require(frame.add((std::size_t(y)*size[0]+x)*size[2]+z,sample.kind,sample.id),"duplicate or invalid frame sample");}
 const auto key=(std::size_t(234)*size[0]+521)*size[2];const auto& count=frame.cells.at(key);require(count.ground==0&&count.obstacle==3&&count.uncertain==0&&count.self==50&&count.valid()==53&&!count.supported(),"captured 53-sample denominator mismatch");
 Json candidate_voxels=Json::array();
 // Report every same-frame voxel with an Obstacle vote, plus every observed
 // voxel in the near-start window. This is evidence output, not a spatial rule.
 for(const auto& entry:frame.cells){const auto cell=entry.first;const auto& votes=entry.second;
  const auto z=cell%size[2],xy=cell/size[2],x=xy%size[0],y=xy/size[0];
  if(votes.obstacle==0 && !(x>=518&&x<=525&&y>=230&&y<=237))continue;
  candidate_voxels.push_back({{"voxel",Json::array({x,y,z})},{"ground",votes.ground},{"obstacle",votes.obstacle},{"uncertain",votes.uncertain},{"self",votes.self},{"denominator",votes.valid()},{"supported",votes.supported()}});
 }
 std::cout<<Json{{"candidate_voxels",candidate_voxels},{"source_sequence",7881},{"body_retained",body.retained_sample_ids.size()},{"body_self_samples",body.self_support.size()},{"ground",count.ground},{"obstacle",count.obstacle},{"uncertain",count.uncertain},{"self",count.self},{"denominator",count.valid()},{"supported",count.supported()},{"scope","production fused BODY and packet metadata; recorded classes; exact tf2 transform and production voxel Counts; live buffer integration separately tested"}}.dump(2)<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
