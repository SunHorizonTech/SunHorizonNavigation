// Independent opt-in candidate. Does not replace legacy R, CM clouds, or any
// original upstream health gate. Every complete result uses one belly anchor.
#include <algorithm>
#include <array>
#include <atomic>
#include <mutex>
#include <iostream>
#include <unistd.h>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include "sunhorizon_navigation_interfaces/msg/spatial_coverage_evidence.hpp"
#include "sunhorizon_navigation_interfaces/msg/obstacle_coverage_status.hpp"
#include "sunhorizon_rgbd_adapter/region_depth_producer.hpp"
#include "sunhorizon_rgbd_adapter/dynamic_self_model.hpp"
#include "sunhorizon_rgbd_adapter/spatial_source_transform.hpp"
#include "sunhorizon_rgbd_adapter/spatial_leg_pairing.hpp"
#include "sunhorizon_rgbd_adapter/spatial_frame_timing.hpp"
#include "sunhorizon_rgbd_adapter/spatial_visibility_sampling.hpp"
#include "sunhorizon_obstacle_coverage/spatial_visibility_fusion.hpp"

namespace sunhorizon_rgbd_adapter {
namespace sv = sunhorizon_obstacle_coverage::spatial_visibility;
namespace dm = dynamic_self_model;
namespace dg = sunhorizon_obstacle_coverage::dynamic_self_geometry;
using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;
using Evidence = sunhorizon_navigation_interfaces::msg::SpatialCoverageEvidence;
using Pairing = RegionExactPairing<Image, Info>;
namespace {
constexpr int64_t kAge = 300000000, kTfAllowance = 20000000;
int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000) return 0;
  return int64_t(stamp.sec)*1000000000 + stamp.nanosec;
}
template<class T> RegionFrameKey frame_key(const T& value) {
  return {stamp_ns(value.header.stamp), value.header.frame_id, value.width, value.height};
}
void identity(const std::string& s) {
  if (s.empty() || s.size()>128 || std::any_of(s.begin(),s.end(),
      [](unsigned char c){return c<33 || c>126;})) throw std::invalid_argument("invalid_spatial_identity");
}
std::string gid_hex(const uint8_t* bytes, std::size_t size) {
  std::ostringstream out;
  for (std::size_t i=0;i<size;++i) out << std::hex << std::setfill('0') << std::setw(2) << unsigned(bytes[i]);
  return out.str();
}
bool pinhole(const Info& info) {
  bool valid = (info.distortion_model=="plumb_bob" || info.distortion_model=="rational_polynomial") &&
      info.binning_x<=1 && info.binning_y<=1 && !info.roi.do_rectify &&
      info.roi.x_offset==0 && info.roi.y_offset==0 && info.roi.width==0 && info.roi.height==0;
  for (double d:info.d) valid = valid && d==0.;
  for (std::size_t i=0;i<9;++i) valid = valid && info.r[i]==(i%4==0?1.:0.);
  valid = valid && info.k[1]==0. && info.k[3]==0. && info.k[6]==0. && info.k[7]==0. && info.k[8]==1. &&
      std::isfinite(info.k[0]) && info.k[0]>0. && std::isfinite(info.k[4]) && info.k[4]>0.;
  for (std::size_t i=0;i<3;++i) {
    for (std::size_t j=0;j<3;++j) valid = valid && info.p[i*4+j]==info.k[i*3+j];
    valid = valid && info.p[i*4+3]==0.;
  }
  return valid;
}

}

class SpatialVisibilityNode : public rclcpp::Node {
 public:
  explicit SpatialVisibilityNode(const rclcpp::NodeOptions& options)
      : Node("sunhorizon_spatial_visibility", options) {
    if (!parameter<bool>("enabled",false)) return;
    if (std::string(get_fully_qualified_name())!="/sunhorizon_spatial_visibility" ||
        get_parameter("use_sim_time").as_bool()) throw std::invalid_argument("spatial_node_identity_or_clock");
    identity_mode_=parameter<std::string>("publisher_identity_mode","exact-gid-v1");
    if (identity_mode_!="exact-gid-v1" && identity_mode_!="startup-pin-once-v1") throw std::invalid_argument("spatial_identity_mode");
    contract_=parameter<std::string>("contract","");
    query_digest_=parameter<std::string>("query_digest","");
    producer_epoch_=parameter<std::string>("producer_epoch","");
    boot_=parameter<std::string>("clock_boot_id","");
    common_frame_=parameter<std::string>("common_frame","");
    if (!dm::sha_valid(contract_) || !dm::sha_valid(query_digest_)) throw std::invalid_argument("spatial_contract_requires_sha256");
    identity(producer_epoch_);identity(boot_);identity(common_frame_);
    std::ifstream boot_file("/proc/sys/kernel/random/boot_id");std::string actual_boot;std::getline(boot_file,actual_boot);
    if (actual_boot!=boot_) throw std::invalid_argument("spatial_boot_mismatch");
    tolerance_=parameter<double>("self_tolerance_m",0.);
    margin_=parameter<double>("clear_margin_m",0.);
    if (!std::isfinite(tolerance_) || tolerance_<=0. || tolerance_>dg::kSelfSurfaceToleranceM ||
        !std::isfinite(margin_) || margin_<0.) throw std::invalid_argument("spatial_tolerance_or_margin");
    const auto mins=parameter<std::vector<double>>("cell_min",{}),maxs=parameter<std::vector<double>>("cell_max",{});
    if (mins.empty() || mins.size()!=maxs.size() || mins.size()%3 || mins.size()/3>sv::kMaximumCells)
      throw std::invalid_argument("spatial_cell_count");
    // Digest is a streaming canonical digest, avoiding RegionCanonical's small
    // camera-contract size limit for a 4096-cell explicit query.
    std::string canonical="spatial-query-v1;";
    auto append=[&](const std::string& value){canonical+=std::to_string(value.size())+":"+value;};
    append(common_frame_);
    for (std::size_t i=0;i<mins.size()/3;++i) {
      sv::AxisAlignedCell cell;cell.id=uint32_t(i+1);
      for (std::size_t j=0;j<3;++j) {
        cell.min[j]=mins[i*3+j];cell.max[j]=maxs[i*3+j];
        if (!std::isfinite(cell.min[j])||!std::isfinite(cell.max[j])||cell.min[j]>=cell.max[j])
          throw std::invalid_argument("spatial_cell_bounds");
        RegionCanonical numbers;numbers.number(cell.min[j]).number(cell.max[j]);append(numbers.str());
      }
      query_.cells.push_back(cell);
    }
    if (dm::sha256(canonical)!=query_digest_) throw std::invalid_argument("spatial_query_digest_mismatch");
    query_.common_frame=common_frame_;query_.query_id=query_digest_;query_.contract_id=contract_;
    const auto paths=parameter<std::vector<std::string>>("model_mesh_paths",{});
    const auto hashes=parameter<std::vector<std::string>>("model_mesh_sha256",{});
    const auto counts=parameter<std::vector<int64_t>>("model_triangle_counts",{});
    const auto links=parameter<std::vector<std::string>>("model_link_names",{});
    const auto parents=parameter<std::vector<std::string>>("model_parent_frames",{});
    if (paths.size()!=12 || hashes.size()!=12 || counts.size()!=12 || links.size()!=12 || parents.size()!=12)
      throw std::invalid_argument("spatial_model_manifest_length");
    std::vector<dm::AssetSpec> specs;
    for (std::size_t i=0;i<12;++i) {
      identity(links[i]);identity(parents[i]);
      if (counts[i]<=0) throw std::invalid_argument("spatial_model_triangle_count");
      specs.push_back({paths[i],hashes[i],std::size_t(counts[i]),links[i],parents[i]});
      allowed_edges_.emplace(links[i],parents[i]);
    }
    model_=std::make_unique<dm::Model>(std::move(specs));
    private_fqn_=parameter<std::string>("private_tf_publisher", "");
    private_gid_=parameter<std::string>("private_tf_gid", "");
    private_static_fqn_=parameter<std::string>("private_static_publisher", "");
    private_static_gid_=parameter<std::string>("private_static_gid", "");
    identity(private_fqn_);
    if (private_fqn_.front()!='/' || !configured_gid(private_gid_) ||
        (private_static_fqn_.empty() && !private_static_gid_.empty())) throw std::invalid_argument("private_tf_publisher_pin");
    if (!private_static_fqn_.empty()) {identity(private_static_fqn_);
      if (private_static_fqn_.front()!='/' || !configured_gid(private_static_gid_)) throw std::invalid_argument("private_static_publisher_pin");}
    chest_enabled_=parameter<bool>("include_chest",false);
    configure_source(0,"belly","/sunhorizon/raw/rgbd/belly/depth/image","/sunhorizon/raw/rgbd/belly/depth/camera_info");
    if (chest_enabled_) configure_source(1,"chest","/sunhorizon/raw/rgbd/chest/aligned_depth/image","/sunhorizon/raw/rgbd/chest/aligned_depth/camera_info");
    global_=std::make_unique<tf2_ros::Buffer>(get_clock(),tf2::durationFromSec(2.));
    global_tf_node_=std::make_shared<rclcpp::Node>("spatial_global_tf_listener",rclcpp::NodeOptions().use_global_arguments(false).enable_rosout(false).start_parameter_services(false).start_parameter_event_publisher(false));
    global_listener_=std::make_unique<tf2_ros::TransformListener>(*global_,global_tf_node_,true);
    private_=std::make_unique<tf2_ros::Buffer>(get_clock(),tf2::durationFromSec(2.));
    private_->setUsingDedicatedThread(true);
    data_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    tf_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions tf_options;tf_options.callback_group=tf_group_;
    private_tf_=create_subscription<tf2_msgs::msg::TFMessage>("/sunhorizon/navigation/belly_self/tf",
        rclcpp::QoS(100).best_effort(),[this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg,const rclcpp::MessageInfo& info){private_tf(msg,info,false);},tf_options);
    if (!private_static_fqn_.empty()) private_static_=create_subscription<tf2_msgs::msg::TFMessage>("/sunhorizon/navigation/belly_self/tf_static",
        rclcpp::QoS(100).reliable().transient_local(),[this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg,const rclcpp::MessageInfo& info){private_tf(msg,info,true);},tf_options);
    if (identity_mode_=="startup-pin-once-v1") identity_output_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/sunhorizon/obstacle_coverage/spatial_identity_v1",rclcpp::QoS(1).reliable().transient_local());
    output_=create_publisher<Evidence>("/sunhorizon/obstacle_coverage/spatial_evidence_v1",rclcpp::QoS(1).reliable());
    rclcpp::SubscriptionOptions input_options;input_options.callback_group=data_group_;
    for (std::size_t index=0;index<(chest_enabled_?2U:1U);++index) {
      auto& source=sources_[index];
      source.images=create_subscription<Image>(source.image_topic,rclcpp::SensorDataQoS().keep_last(1),
          [this,index](Image::ConstSharedPtr msg,const rclcpp::MessageInfo& info){image(index,msg,info);},input_options);
      source.infos=create_subscription<Info>(source.info_topic,rclcpp::SensorDataQoS().keep_last(8),
          [this,index](Info::ConstSharedPtr msg,const rclcpp::MessageInfo& info){camera_info(index,msg,info);},input_options);
    }
    if (identity_mode_=="startup-pin-once-v1") identity_watch_=create_wall_timer(std::chrono::milliseconds(20),
        [this](){if (!terminal_.load() && !frozen_owners_valid()) fault("frozen_publisher_missing_or_changed");},data_group_);
    anchor_watch_=create_wall_timer(std::chrono::milliseconds(10),[this](){
      if (terminal_.load() || !pending_anchor_) return;
      try {if (evaluate(*pending_anchor_,true)) pending_anchor_.reset();}
      catch (const std::exception& error) {fault(error.what());}
    },data_group_);
    coverage_timing_hint_=create_subscription<sunhorizon_navigation_interfaces::msg::ObstacleCoverageStatus>(
        "/sunhorizon/obstacle_coverage/status",rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](sunhorizon_navigation_interfaces::msg::ObstacleCoverageStatus::ConstSharedPtr msg){
          const auto d=stamp_ns(msg->header.stamp);
          if (!terminal_.load() && spatial_expiry_timing_hint(msg->reason,d,first_valid_h_ns_)) {
            const auto ros=get_clock()->now().nanoseconds(),steady=dm::steady_ns();
            frame_timing_.freeze_hint(d,ros,steady);private_tf_timing_.freeze_hint(d,ros,steady);
          }
        },input_options); // Unverified observed hint only: no clock observe, wire, fault or reset.
  }
  ~SpatialVisibilityNode() override {dump_frame_timing(false);} // Flush an undumped hint on managed shutdown.
 private:
  template<class T> T parameter(const std::string& name,const T& value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only=true;
    return declare_parameter<T>(name,value,descriptor);
  }
  struct Sample {Pairing::Pending pending;Info::ConstSharedPtr info;};
  struct SourceState {
    std::string name,frame,epoch,calibration,image_topic,info_topic,image_fqn,info_fqn,image_gid,info_gid;
    double scale{0},minimum{0},maximum{0};
    Pairing pairs;uint64_t sequence{0},finished_sequence{0};
    std::deque<Sample> cache;
    rclcpp::Subscription<Image>::SharedPtr images;
    rclcpp::Subscription<Info>::SharedPtr infos;
  };
  struct Decoded {std::vector<float> depth;std::vector<sv::ReturnKind> kinds;sv::Source source;};
  void configure_source(std::size_t index,const std::string& name,const std::string& image_topic,const std::string& info_topic) {
    auto& source=sources_[index];source.name=name;source.image_topic=image_topic;source.info_topic=info_topic;
    source.frame=parameter<std::string>(name+"_frame","");source.epoch=parameter<std::string>(name+"_epoch","");
    source.calibration=parameter<std::string>(name+"_calibration","");
    source.image_fqn=parameter<std::string>(name+"_image_publisher","");source.info_fqn=parameter<std::string>(name+"_info_publisher","");
    source.image_gid=parameter<std::string>(name+"_image_gid","");source.info_gid=parameter<std::string>(name+"_info_gid","");
    source.scale=parameter<double>(name+"_depth_scale_m",0.);source.minimum=parameter<double>(name+"_minimum_depth_m",0.);
    source.maximum=parameter<double>(name+"_maximum_depth_m",0.);
    identity(source.frame);identity(source.epoch);identity(source.image_fqn);identity(source.info_fqn);
    if (source.image_fqn.front()!='/' || source.info_fqn.front()!='/' || source.calibration.empty() || source.calibration.size()>8192 ||
        !configured_gid(source.image_gid) || !configured_gid(source.info_gid) ||
        !std::isfinite(source.scale) || source.scale<=0. || !std::isfinite(source.minimum) || source.minimum<=0. ||
        !std::isfinite(source.maximum) || source.maximum<=source.minimum)
      throw std::invalid_argument("spatial_source_manifest");
    for (const auto& flag:{"rectified_pinhole_verified","metric_axial_depth_verified","depth_intrinsics_registration_verified",
        "optical_to_base_extrinsics_verified","source_time_and_transform_verified"})
      if (!parameter<bool>(name+"_"+flag,false)) throw std::invalid_argument("spatial_source_attestation_missing");
    query_.source_contracts.push_back({name,source.epoch,index==0});
  }
  static bool gid_valid(const std::string& gid) {
    return gid.size()==RMW_GID_STORAGE_SIZE*2 && gid!=std::string(gid.size(),'0') &&
        gid.find_first_not_of("0123456789abcdef")==std::string::npos;
  }
  bool configured_gid(const std::string& gid) const {
    return identity_mode_=="startup-pin-once-v1"?gid.empty():gid_valid(gid);
  }
  enum class IdentityCheck {Ready, Pending, Rejected};
  bool endpoint_unlocked(const std::string& topic,const std::string& expected,const std::string& fqn,const std::string& type) {
    if (expected.empty()) return false;
    const auto endpoints=get_publishers_info_by_topic(topic);
    if (endpoints.size()!=1) return false;
    const auto& endpoint=endpoints.front();
    std::string actual=endpoint.node_namespace();
    if (actual.empty() || actual.back()!='/') actual+='/';
    actual+=endpoint.node_name();const auto& gid=endpoint.endpoint_gid();
    return actual==fqn && gid_hex(gid.data(),gid.size())==expected && endpoint.topic_type()==type;
  }
  bool endpoint_identity(const std::string& topic,const std::string& expected,const std::string& fqn,const std::string& type) {
    std::lock_guard<std::mutex> lock(identity_mutex_);return endpoint_unlocked(topic,expected,fqn,type);
  }
  void identity_failure(const std::string& topic,const std::string& expected,const std::string& fqn,
      const std::string& actual) {
    const auto endpoints=get_publishers_info_by_topic(topic);
    std::string details;
    for (std::size_t i=0;i<std::min<std::size_t>(endpoints.size(),8);++i) {
      const auto& e=endpoints[i];const auto& gid=e.endpoint_gid();
      details+=" ["+e.node_namespace().substr(0,128)+"/"+e.node_name().substr(0,128)+
          " "+gid_hex(gid.data(),gid.size())+" "+e.topic_type().substr(0,128)+"]";
    }
    RCLCPP_ERROR(get_logger(),"spatial identity rejected topic=%s mode=%s expected_fqn=%s expected_gid=%s callback_gid=%s graph_count=%zu graph=%s",
        topic.c_str(),identity_mode_.c_str(),fqn.c_str(),expected.c_str(),actual.c_str(),endpoints.size(),details.c_str());
  }
  IdentityCheck publisher_identity(const std::string& topic,std::string& expected,
      const std::string& fqn,const std::string& type,const rclcpp::MessageInfo& info) {
    std::lock_guard<std::mutex> lock(identity_mutex_);
    const auto actual=gid_hex(info.get_rmw_message_info().publisher_gid.data,RMW_GID_STORAGE_SIZE);
    // DDS endpoint discovery can precede ROS node-name metadata. Only this
    // precise sentinel pair, with the correct type and the same nonzero entity
    // as the actual callback, is pending before first qualification.
    if (!qualified_topics_.count(topic)) {
      const auto endpoints=get_publishers_info_by_topic(topic);
      if (endpoints.size()==1) {
        const auto& endpoint=endpoints.front();const auto& graph_gid=endpoint.endpoint_gid();
        const auto& ns=endpoint.node_namespace();
        const bool unknown=endpoint.node_name()=="_NODE_NAME_UNKNOWN_" &&
            (ns=="_NODE_NAMESPACE_UNKNOWN_" || ns=="/_NODE_NAMESPACE_UNKNOWN_");
        if (unknown && endpoint.topic_type()==type && gid_valid(actual) &&
            gid_hex(graph_gid.data(),graph_gid.size())==actual &&
            ((expected.empty() && identity_mode_=="startup-pin-once-v1") || expected==actual))
          return IdentityCheck::Pending;
      }
    }
    if (expected.empty()) {
      if (identity_mode_!="startup-pin-once-v1") return IdentityCheck::Rejected;
      const auto endpoints=get_publishers_info_by_topic(topic);
      if (endpoints.empty()) return IdentityCheck::Pending;
      if (!gid_valid(actual) || !endpoint_unlocked(topic,actual,fqn,type)) {
        identity_failure(topic,expected,fqn,actual);return IdentityCheck::Rejected;
      }
      expected=actual;identity_report_dirty_=true; // Exactly once; never cleared/replaced.
    }
    if (actual==expected && endpoint_unlocked(topic,expected,fqn,type)) {
      if (qualified_topics_.insert(topic).second) identity_report_dirty_=true;
      return IdentityCheck::Ready;
    }
    identity_failure(topic,expected,fqn,actual);return IdentityCheck::Rejected;
  }
  bool frozen_owners_valid() {
    std::lock_guard<std::mutex> lock(identity_mutex_);
    // Before each endpoint freezes it contributes nothing and is not selected
    // from a missing/ambiguous graph. After freeze it can never rebind.
    auto valid=[&](const std::string& topic,const std::string& gid,const std::string& fqn,const std::string& type) {
      return gid.empty() || endpoint_unlocked(topic,gid,fqn,type);
    };
    for (std::size_t i=0;i<(chest_enabled_?2U:1U);++i) {
      const auto& source=sources_[i];
      if (!valid(source.image_topic,source.image_gid,source.image_fqn,"sensor_msgs/msg/Image") ||
          !valid(source.info_topic,source.info_gid,source.info_fqn,"sensor_msgs/msg/CameraInfo")) return false;
    }
    return valid("/sunhorizon/navigation/belly_self/tf",private_gid_,private_fqn_,"tf2_msgs/msg/TFMessage") &&
        (private_static_fqn_.empty() || valid("/sunhorizon/navigation/belly_self/tf_static",private_static_gid_,private_static_fqn_,"tf2_msgs/msg/TFMessage"));
  }
  IdentityCheck publisher(SourceState& source,bool is_image,const rclcpp::MessageInfo& info) {
    return publisher_identity(is_image?source.image_topic:source.info_topic,
        is_image?source.image_gid:source.info_gid,is_image?source.image_fqn:source.info_fqn,
        is_image?"sensor_msgs/msg/Image":"sensor_msgs/msg/CameraInfo",info);
  }
  bool identities_ready_unlocked() const {
    for (std::size_t index=0;index<(chest_enabled_?2U:1U);++index)
      if (sources_[index].image_gid.empty() || sources_[index].info_gid.empty() ||
          !qualified_topics_.count(sources_[index].image_topic) || !qualified_topics_.count(sources_[index].info_topic)) return false;
    return !private_gid_.empty() && qualified_topics_.count("/sunhorizon/navigation/belly_self/tf") &&
        (private_static_fqn_.empty() || (!private_static_gid_.empty() && qualified_topics_.count("/sunhorizon/navigation/belly_self/tf_static")));
  }
  bool identities_ready() {std::lock_guard<std::mutex> lock(identity_mutex_);return identities_ready_unlocked();}
  void identity_report() {
    if (!identity_output_) return;
    std::lock_guard<std::mutex> lock(identity_mutex_);
    if (!identity_report_dirty_) return;
    identity_report_dirty_=false;
    diagnostic_msgs::msg::DiagnosticArray report;report.header.stamp=get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus status;status.name="sunhorizon_spatial_visibility/publisher_identities";
    status.hardware_id=producer_epoch_;status.level=terminal_.load()?2:(identities_ready_unlocked()?0:1);status.message="frozen_endpoint_identities";
    auto add=[&](const std::string& key,const std::string& value){diagnostic_msgs::msg::KeyValue item;item.key=key;item.value=value;status.values.push_back(item);};
    add("schema_version","1");add("clock_boot_id",boot_);add("publisher_identity_mode",identity_mode_);
    add("binding_complete",identities_ready_unlocked() && !terminal_.load()?"true":"false");add("terminal",terminal_.load()?"true":"false");add("pid",std::to_string(::getpid()));
    auto endpoint=[&](const std::string& role,const std::string& topic,const std::string& fqn,const std::string& gid,const std::string& type){
      add(role+".topic",topic);add(role+".fqn",fqn);add(role+".gid",gid);add(role+".type",type);
    };
    for (std::size_t i=0;i<(chest_enabled_?2U:1U);++i) {
      const auto& source=sources_[i];
      endpoint(source.name+".image",source.image_topic,source.image_fqn,source.image_gid,"sensor_msgs/msg/Image");
      endpoint(source.name+".info",source.info_topic,source.info_fqn,source.info_gid,"sensor_msgs/msg/CameraInfo");
    }
    endpoint("private.tf","/sunhorizon/navigation/belly_self/tf",private_fqn_,private_gid_,"tf2_msgs/msg/TFMessage");
    if (!private_static_fqn_.empty()) endpoint("private.static","/sunhorizon/navigation/belly_self/tf_static",private_static_fqn_,private_static_gid_,"tf2_msgs/msg/TFMessage");
    const auto& own_gid=output_->get_gid();
    endpoint("producer.evidence","/sunhorizon/obstacle_coverage/spatial_evidence_v1",get_fully_qualified_name(),gid_hex(own_gid.data,RMW_GID_STORAGE_SIZE),"sunhorizon_navigation_interfaces/msg/SpatialCoverageEvidence");
    report.status.push_back(status);identity_output_->publish(report);
  }
  bool clocks(int64_t ros,int64_t steady) {
    previous_ros_=last_ros_;previous_steady_=last_steady_;last_ros_=ros;last_steady_=steady;
    return clock_guard_.observe(ros,steady);
  }
  bool fresh(const Sample& sample,int64_t ros,int64_t steady) const {
    return region_times_fresh(sample.pending.key.stamp_ns,sample.pending.receipt_ros_ns,
        sample.pending.receipt_steady_ns,ros,steady,.3) && ros-sample.pending.receipt_ros_ns<=kAge;
  }
  [[noreturn]] void anchor_failure(const char* phase,const char* predicate,
      const Sample& sample,int64_t ros,int64_t steady,bool clock_observed,
      int private_snapshot,int terminal_snapshot=-1) {
    const auto source=sample.pending.key.stamp_ns,receipt=sample.pending.receipt_ros_ns,
        receipt_steady=sample.pending.receipt_steady_ns;
    const auto age=[](int64_t now,int64_t then){return now>0 && then>0?now-then:int64_t(-1);};
    RCLCPP_ERROR(get_logger(),
        "spatial anchor rejection phase=%s predicate=%s frame_sequence=%llu source_ns=%lld image_receipt_ros_ns=%lld image_receipt_steady_ns=%lld now_ros_ns=%lld now_steady_ns=%lld previous_ros_ns=%lld previous_steady_ns=%lld source_age_ns=%lld receipt_ros_age_ns=%lld receipt_steady_age_ns=%lld required_tf_qualified=%d private_fault=%d terminal_snapshot=%d clock_observed=%d",
        phase,predicate,static_cast<unsigned long long>(sample.pending.sequence),static_cast<long long>(source),static_cast<long long>(receipt),static_cast<long long>(receipt_steady),
        static_cast<long long>(ros),static_cast<long long>(steady),static_cast<long long>(clock_observed?previous_ros_:last_ros_),static_cast<long long>(clock_observed?previous_steady_:last_steady_),
        static_cast<long long>(age(ros,source)),static_cast<long long>(age(ros,receipt)),static_cast<long long>(age(steady,receipt_steady)),
        int(required_tf_qualified_),private_snapshot,terminal_snapshot,int(clock_observed));
    throw std::runtime_error(std::string(phase)+":"+predicate);
  }
  const char* clock_predicate(int64_t ros,int64_t steady) const {
    if (ros<=0 || steady<=0) return "anchor_clock_nonpositive";
    if (ros<previous_ros_) return "anchor_ros_clock_rollback";
    if (steady<previous_steady_) return "anchor_steady_clock_rollback";
    return "anchor_clock_guard_invalid";
  }
  const char* freshness_predicate(const Sample& sample,int64_t ros,int64_t steady) const {
    const auto source=sample.pending.key.stamp_ns,receipt=sample.pending.receipt_ros_ns,
        receipt_steady=sample.pending.receipt_steady_ns;
    if (source<=0 || receipt<source || receipt_steady<=0 || ros<receipt || steady<receipt_steady)
      return "anchor_stale_time_order";
    if ((ros-source)/1.e9>.3) return "anchor_stale_source_age";
    if ((steady-receipt_steady)/1.e9>.3) return "anchor_stale_receipt_steady_age";
    return "anchor_stale_receipt_ros_age";
  }
  void guard_pending_anchor(int64_t ros,int64_t steady) {
    if (!pending_anchor_) return;
    if (private_fault_.load()) anchor_failure("leg_tf_wait","anchor_private_tf_fault",*pending_anchor_,ros,steady,true,1);
    if (!fresh(*pending_anchor_,ros,steady)) anchor_failure("leg_tf_wait",freshness_predicate(*pending_anchor_,ros,steady),*pending_anchor_,ros,steady,true,0);
    if (!frozen_owners_valid()) throw std::runtime_error("publisher_identity_changed_during_leg_tf_wait");
  }
  void image(std::size_t index,Image::ConstSharedPtr msg,const rclcpp::MessageInfo& info) {
    if (terminal_) return;
    try {
      auto& source=sources_[index];const auto ros=get_clock()->now().nanoseconds(),steady=dm::steady_ns();
      if (!clocks(ros,steady)) throw std::runtime_error("image_clock_rollback");
      guard_pending_anchor(ros,steady);
      const auto owner=publisher(source,true,info);identity_report();
      if (owner==IdentityCheck::Pending) return;
      if (owner!=IdentityCheck::Ready) throw std::runtime_error("image_publisher_identity");
      const auto key=frame_key(*msg);
      if (!region_key_valid(key) || key.frame!=source.frame || key.width>640 || key.height>480 || msg->encoding!="16UC1" ||
          msg->step<uint64_t(msg->width)*2 || uint64_t(msg->step)*msg->height!=msg->data.size() || msg->data.size()>2*1024*1024 ||
          msg->is_bigendian>1) throw std::runtime_error("image_schema_or_frame");
      ++source.sequence;
      if (index==0) {const auto& rmw=info.get_rmw_message_info();
        frame_timing_.ingress(source.sequence,key.stamp_ns,ros,steady,rmw.source_timestamp,rmw.received_timestamp);}
      if (!source.pairs.image({key,msg,source.sequence,ros,steady})) throw std::runtime_error("image_nonmonotonic");
      paired(index);
    } catch (const std::exception& error) {fault(error.what());}
  }
  void camera_info(std::size_t index,Info::ConstSharedPtr msg,const rclcpp::MessageInfo& info) {
    if (terminal_) return;
    try {
      auto& source=sources_[index];const auto ros=get_clock()->now().nanoseconds(),steady=dm::steady_ns();
      if (!clocks(ros,steady)) throw std::runtime_error("info_clock_rollback");
      guard_pending_anchor(ros,steady);
      const auto owner=publisher(source,false,info);identity_report();
      if (owner==IdentityCheck::Pending) return;
      if (owner!=IdentityCheck::Ready) throw std::runtime_error("info_publisher_identity");
      const auto key=frame_key(*msg);const auto canonical=region_calibration_contract(*msg);
      if (key.frame!=source.frame || canonical!=source.calibration || !pinhole(*msg) ||
          !source.pairs.info(key,canonical,msg)) throw std::runtime_error("info_contract_or_layout");
      // A contradictory late Info revokes already published evidence immediately.
      for (const auto& held:source.cache) if (held.pending.key==key && region_calibration_contract(*held.info)!=canonical)
        throw std::runtime_error("held_info_conflict");
      const auto* entry=source.pairs.match(key);
      if (entry && entry->conflict) throw std::runtime_error("exact_info_conflict");
      paired(index);
    } catch (const std::exception& error) {fault(error.what());}
  }
  void paired(std::size_t index) {
    if (!identities_ready()) return;
    auto& source=sources_[index];auto pending=source.pairs.pending();if (!pending) return;
    const auto* info=source.pairs.match(pending->key);if (!info) return;
    if (info->conflict || info->canonical!=source.calibration) throw std::runtime_error("paired_info_conflict");
    if (pending->sequence<=source.finished_sequence) return;
    source.finished_sequence=pending->sequence;
    Sample sample{*pending,info->info};source.pairs.clear();
    if (index==0) frame_timing_.paired(sample.pending.sequence,dm::steady_ns());
    if (index==0 && pending_anchor_) {
      if (dropped_anchors_==std::numeric_limits<uint64_t>::max()) throw std::runtime_error("pending_anchor_drop_overflow");
      ++dropped_anchors_;
      frame_timing_.dropped(sample.pending.sequence);
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,
          "spatial pending anchor full dropped=%llu held_sequence=%llu held_source_ns=%lld",
          static_cast<unsigned long long>(dropped_anchors_),static_cast<unsigned long long>(pending_anchor_->pending.sequence),
          static_cast<long long>(pending_anchor_->pending.key.stamp_ns));
      return;
    }
    if (source.cache.size()==8) source.cache.pop_front();source.cache.push_back(sample);
    if (index==0) {pending_anchor_=sample;if (evaluate(*pending_anchor_)) pending_anchor_.reset();}
  }
  void private_tf(tf2_msgs::msg::TFMessage::ConstSharedPtr msg,const rclcpp::MessageInfo& info,bool is_static) {
    if (terminal_.load()) return;
    // First live callback observation, not a converted wire timestamp or clock gate.
    SpatialPrivateTfTiming observed;observed.callback_ros_ns=get_clock()->now().nanoseconds();
    observed.callback_steady_ns=dm::steady_ns();observed.is_static=is_static;
    const auto& rmw=info.get_rmw_message_info();observed.rmw_source_ns=rmw.source_timestamp;
    observed.rmw_received_ns=rmw.received_timestamp;observed.message_count=msg->transforms.size();
    observed.scanned_count=std::min<std::size_t>(observed.message_count,256);
    for (std::size_t i=0;i<observed.scanned_count;++i) {
      const auto& tf=msg->transforms[i];
      for (std::size_t link=0;link<model_->specs().size();++link) if (tf.child_frame_id==model_->specs()[link].link_name) {
        // Retain raw stamp arithmetic even if original validation will reject it.
        observed.edge(link,int64_t(tf.header.stamp.sec)*1000000000+tf.header.stamp.nanosec);break;
      }
    }
    const auto timing_seq=private_tf_timing_.ingress(observed);
    SpatialPrivateTfTimingEnd end_timing(private_tf_timing_,timing_seq,dm::steady_ns);
    bool owner_ended=false,insertion_started=false;std::size_t inserted=0;
    std::string rejected_child,rejected_parent;int32_t rejected_sec=0;uint32_t rejected_nsec=0;int64_t old_stamp=-1;
    const auto remember=[&](const geometry_msgs::msg::TransformStamped& tf) {
      rejected_child=tf.child_frame_id.substr(0,128);rejected_parent=tf.header.frame_id.substr(0,128);
      rejected_sec=tf.header.stamp.sec;rejected_nsec=tf.header.stamp.nanosec;old_stamp=-1;
    };
    // Separate callback group; only whitelisted leg edges can enter this buffer.
    // Camera/base/odom TF in this namespace cannot overwrite the global chain.
    try {
      private_tf_timing_.stage(timing_seq,SpatialPrivateTfTimingRing::Stage::OwnerStart,dm::steady_ns());
      const auto owner=publisher_identity(is_static?"/sunhorizon/navigation/belly_self/tf_static":"/sunhorizon/navigation/belly_self/tf",
          is_static?private_static_gid_:private_gid_,is_static?private_static_fqn_:private_fqn_,"tf2_msgs/msg/TFMessage",info);
      private_tf_timing_.stage(timing_seq,SpatialPrivateTfTimingRing::Stage::OwnerEnd,dm::steady_ns());owner_ended=true;
      identity_report();if (owner==IdentityCheck::Pending) {private_tf_timing_.outcome(timing_seq,2);return;}
      if (owner!=IdentityCheck::Ready) throw std::runtime_error("private_tf_publisher_identity");
      if (msg->transforms.size()>256) throw std::runtime_error("private_tf_message_bound");
      std::vector<geometry_msgs::msg::TransformStamped> accepted;
      for (const auto& tf:msg->transforms) {
        const auto expected=allowed_edges_.find(tf.child_frame_id);
        if (expected==allowed_edges_.end()) continue;
        remember(tf);
        if (tf.header.frame_id!=expected->second || (!is_static && stamp_ns(tf.header.stamp)<=0))
          throw std::runtime_error("private_leg_tf_identity");
        (void)dm::rigid(tf.transform);accepted.push_back(tf);
      }
      // Same-edge identity/transform history is bounded by the twelve links.
      // A reordered or contradictory dynamic sample is never silently adopted.
      for (const auto& tf:accepted) {
        remember(tf);
        RegionCanonical value;
        value.add(tf.header.frame_id).add(tf.child_frame_id);
        for (double v:{tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z,
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w}) value.number(v);
        const auto now=stamp_ns(tf.header.stamp);const auto old=private_history_.find(tf.child_frame_id);
        if (old!=private_history_.end()) old_stamp=old->second.stamp;
        if (old!=private_history_.end() && (old->second.is_static!=is_static ||
            (!is_static && now<old->second.stamp) ||
            ((is_static || now==old->second.stamp) && value.str()!=old->second.canonical)))
          throw std::runtime_error("private_leg_tf_rollback_or_conflict");
        private_history_[tf.child_frame_id]={now,value.str(),is_static};
      }
      insertion_started=true;
      for (const auto& tf:accepted) {
        remember(tf);
        if (!private_->setTransform(tf,"spatial_pinned_leg_chain",is_static)) throw std::runtime_error("private_leg_tf_rejected");
        ++inserted;
      }
      private_tf_timing_.inserted(timing_seq,inserted,dm::steady_ns());
    } catch (const std::exception& error) {
      // No concurrent publication/state mutation: signal to the data group.
      private_fault_.store(true);
      fault(error.what()); // Preserve immediate revocation before diagnostic work.
      // The already claimed terminal snapshot may truthfully retain end=0.
      if (!owner_ended) private_tf_timing_.stage(timing_seq,SpatialPrivateTfTimingRing::Stage::OwnerEnd,dm::steady_ns());
      if (insertion_started) private_tf_timing_.inserted(timing_seq,inserted,dm::steady_ns());
      private_tf_timing_.outcome(timing_seq,3);
      // Always retain the original private error, even if the data group wins
      // the terminal publication lock first. Capture the actual checked edge,
      // including its previous history stamp for rollback/conflict failures.
      const auto bounded=[](const std::string& value){auto text=value.substr(0,128);
        for (auto& c:text) if (static_cast<unsigned char>(c)<=32 || c==';') c='?';return text;};
      RCLCPP_ERROR(get_logger(),"spatial private TF rejection reason=%s is_static=%d transforms=%zu checked_child=%s checked_parent=%s checked_sec=%d checked_nanosec=%u history_stamp_ns=%lld",
          error.what(),int(is_static),msg->transforms.size(),bounded(rejected_child).c_str(),bounded(rejected_parent).c_str(),
          rejected_sec,rejected_nsec,static_cast<long long>(old_stamp));
    }
  }
  Decoded decode(const Sample& sample,std::size_t index,int64_t anchor,int64_t deadline) {
    const auto& config=sources_[index];const auto& image=*sample.pending.image;const auto& info=*sample.info;
    const auto timing=[&](SpatialTimingStage stage){if (index==0) timing_stage(sample.pending.sequence,stage);};
    timing(SpatialTimingStage::ModelStart);
    const auto links=model_->observations(*global_,*private_,config.frame,sample.pending.key.stamp_ns,get_clock()->get_clock_type(),deadline);
    timing(SpatialTimingStage::ModelEnd);timing(SpatialTimingStage::CommonTfStart);
    const auto tf=spatial_source_transform(*global_,common_frame_,config.frame,
        sample.pending.key.stamp_ns,anchor,get_clock()->get_clock_type(),deadline);
    timing(SpatialTimingStage::CommonTfEnd);
    const auto rigid=dm::rigid(tf.transform);
    Decoded out;const auto count=std::size_t(image.width)*image.height;
    out.depth.resize(count);out.kinds.assign(count,sv::ReturnKind::kExternal);
    sv::DepthRaster depth;depth.width=image.width;depth.height=image.height;depth.fx=info.k[0];depth.fy=info.k[4];depth.cx=info.k[2];depth.cy=info.k[5];
    depth.optical_to_common.rotation=rigid.rotation;depth.optical_to_common.translation={rigid.translation.x,rigid.translation.y,rigid.translation.z};
    depth.optical_to_common.source_frame=config.frame;depth.optical_to_common.common_frame=common_frame_;
    depth.optical_to_common.source_stamp_ns=sample.pending.key.stamp_ns;depth.optical_to_common.query_stamp_ns=anchor;
    depth.minimum_depth_m=config.minimum;depth.maximum_depth_m=config.maximum;depth.clear_margin_m=margin_;depth.calibration={true,true,true,true,true};
    timing(SpatialTimingStage::SamplingStart);
    const auto needed=spatial_required_pixels(query_,depth);
    const dg::PreparedModel prepared(links,dg::QueryBudget{});
    std::size_t visits=0,triangles=0;
    for (uint32_t v=0;v<image.height;++v) for (uint32_t u=0;u<image.width;++u) {
      const auto byte=std::size_t(v)*image.step+std::size_t(u)*2,at=std::size_t(v)*image.width+u;
      const uint16_t raw=image.is_bigendian?(uint16_t(image.data[byte])<<8)|image.data[byte+1]:uint16_t(image.data[byte])|(uint16_t(image.data[byte+1])<<8);
      out.depth[at]=float(raw*config.scale);
      if (!std::isfinite(out.depth[at]) || out.depth[at]<config.minimum || out.depth[at]>config.maximum) {out.kinds[at]=sv::ReturnKind::kInvalid;continue;}
      if (!needed[at]) continue;
      const auto result=dg::classify_body_surface({(u-depth.cx)/depth.fx,(v-depth.cy)/depth.fy,1.},out.depth[at],tolerance_,prepared);
      if (!result.complete || result.node_visits>sv::kMaximumChecks-visits || result.triangle_tests>sv::kMaximumChecks-triangles)
        throw std::runtime_error("self_classification_incomplete_or_budget");
      visits+=result.node_visits;triangles+=result.triangle_tests;
      if (result.classification==dg::Classification::SelfSurface) out.kinds[at]=sv::ReturnKind::kSelfMatched;
      else if (result.classification==dg::Classification::ModelConflict) out.kinds[at]=sv::ReturnKind::kModelConflict;
      else if (result.classification!=dg::Classification::ExternalSurface) throw std::runtime_error("self_classification_unobserved");
    }
    timing(SpatialTimingStage::SamplingEnd);
    depth.depth_m=out.depth.data();depth.depth_elements=count;depth.row_stride_elements=image.width;depth.return_kinds=out.kinds.data();depth.kind_elements=count;
    out.source.source_id=config.name;out.source.epoch=config.epoch;out.source.frame_id=config.frame;out.source.frame_sequence=sample.pending.sequence;
    out.source.source_stamp_ros_ns=sample.pending.key.stamp_ns;out.source.local_receipt_ros_ns=sample.pending.receipt_ros_ns;
    out.source.local_receipt_steady_ns=sample.pending.receipt_steady_ns;out.source.expiry_budget_ns=kAge;out.source.required_for_evaluation=index==0;out.source.data=depth;
    return out;
  }
  bool evaluate(const Sample& anchor,bool timer=false) {
    const auto ros=get_clock()->now().nanoseconds(),steady=dm::steady_ns();
    frame_timing_.evaluate(anchor.pending.sequence,steady,timer);
    if (private_fault_.load()) anchor_failure("evaluate_entry","anchor_private_tf_fault",anchor,ros,steady,false,1);
    if (!clocks(ros,steady)) anchor_failure("evaluate_entry",clock_predicate(ros,steady),anchor,ros,steady,true,0);
    if (!fresh(anchor,ros,steady)) anchor_failure("evaluate_entry",freshness_predicate(anchor,ros,steady),anchor,ros,steady,true,0);
    if (required_tf_qualified_) {
      if (!frozen_owners_valid()) throw std::runtime_error("publisher_identity_changed_during_leg_tf_wait");
      timing_stage(anchor.pending.sequence,SpatialTimingStage::LegCheckStart);
      SpatialLegPendingTiming pending_timing;
      if (spatial_leg_availability(*private_,model_->specs(),anchor.pending.key.stamp_ns,get_clock()->get_clock_type(),&pending_timing)==
          SpatialLegAvailability::FuturePending) {
        timing_stage(anchor.pending.sequence,SpatialTimingStage::LegCheckEnd);
        frame_timing_.future_wait(anchor.pending.sequence,dm::steady_ns(),pending_timing.first_link_index,
            pending_timing.source_query_ns,pending_timing.latest_upper_ns);
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,
            "spatial pending future leg TF sequence=%llu source_ns=%lld",
            static_cast<unsigned long long>(anchor.pending.sequence),static_cast<long long>(anchor.pending.key.stamp_ns));
        return false; // Frozen source/receipt/sequence; no evidence or expiry renewal.
      }
      timing_stage(anchor.pending.sequence,SpatialTimingStage::LegCheckEnd);
    }
    query_.query_stamp_ns=anchor.pending.key.stamp_ns;
    const auto deadline=(required_tf_qualified_?dm::steady_ns():steady)+kTfAllowance;
    std::vector<Decoded> decoded;decoded.reserve(2);
    // BELLY can start before the global navigation TF chain. Publisher
    // qualification alone does not prove the complete source-time transforms.
    // Only explicit TF availability failures before that first qualification
    // may discard this anchor without publishing any spatial evidence.
    try {decoded.push_back(decode(anchor,0,query_.query_stamp_ns,deadline));}
    catch (const tf2::LookupException& error) {
      if (initial_tf_pending(anchor,error)) return true;
      throw;
    } catch (const tf2::ConnectivityException& error) {
      if (initial_tf_pending(anchor,error)) return true;
      throw;
    } catch (const tf2::ExtrapolationException& error) {
      if (initial_tf_pending(anchor,error)) return true;
      throw;
    } catch (const tf2::TimeoutException& error) {
      if (initial_tf_pending(anchor,error)) return true;
      throw;
    }
    // Qualification is independent of Clear: valid TF with invalid or self
    // depth still qualifies, and every later required TF failure is terminal.
    if (!required_tf_qualified_) {
      required_tf_qualified_=true;
      RCLCPP_INFO(get_logger(),"spatial required source-time TF qualified source_ns=%lld",
          static_cast<long long>(anchor.pending.key.stamp_ns));
    }
    if (chest_enabled_) {
      const auto& cache=sources_[1].cache;
      for (auto it=cache.rbegin();it!=cache.rend();++it) if (it->pending.key.stamp_ns<=query_.query_stamp_ns && fresh(*it,ros,steady)) {
        try {decoded.push_back(decode(*it,1,query_.query_stamp_ns,deadline));}
        catch (const std::exception& error) {RCLCPP_WARN(get_logger(),"optional spatial chest unavailable: %s",error.what());}
        break;
      }
    }
    std::vector<sv::Source> inputs;for (const auto& d:decoded) inputs.push_back(d.source);
    timing_stage(anchor.pending.sequence,SpatialTimingStage::FusionStart);
    const auto result=sv::evaluate(query_,inputs,get_clock()->now().nanoseconds(),dm::steady_ns());
    timing_stage(anchor.pending.sequence,SpatialTimingStage::FusionEnd);
    timing_stage(anchor.pending.sequence,SpatialTimingStage::OwnersStart);
    bool owners=frozen_owners_valid();
    for (const auto& input:inputs) {
      const auto& owner=sources_[input.source_id=="belly"?0:1];
      owners=owners && endpoint_identity(owner.image_topic,owner.image_gid,owner.image_fqn,"sensor_msgs/msg/Image") &&
          endpoint_identity(owner.info_topic,owner.info_gid,owner.info_fqn,"sensor_msgs/msg/CameraInfo");
    }
    owners=owners && endpoint_identity("/sunhorizon/navigation/belly_self/tf",private_gid_,private_fqn_,"tf2_msgs/msg/TFMessage");
    if (!private_static_fqn_.empty()) owners=owners && endpoint_identity("/sunhorizon/navigation/belly_self/tf_static",private_static_gid_,private_static_fqn_,"tf2_msgs/msg/TFMessage");
    timing_stage(anchor.pending.sequence,SpatialTimingStage::OwnersEnd);
    Evidence wire;const auto end_ros=get_clock()->now().nanoseconds(),end_steady=dm::steady_ns();
    wire.header.stamp=rclcpp::Time(end_ros,get_clock()->get_clock_type());wire.header.frame_id=common_frame_;wire.schema_version=1;
    wire.contract=contract_;wire.query_digest=query_digest_;wire.producer_epoch=producer_epoch_;wire.clock_boot_id=boot_;
    wire.anchor_source_ros_ns=anchor.pending.key.stamp_ns;wire.anchor_frame_sequence=anchor.pending.sequence;
    wire.input_generation=anchor.pending.sequence;wire.evaluation_steady_ns=end_steady;
    wire.total_cells=uint32_t(query_.cells.size());wire.evaluation_complete=result.evaluation_complete;wire.reason=result.reason;
    for (const auto& cell:result.cells) {
      if (cell.state==sv::CellState::kFree) ++wire.free_cells;
      else if (cell.state==sv::CellState::kOccupied) ++wire.occupied_cells;
      else ++wire.unknown_cells;
    }
    for (std::size_t index=0;index<inputs.size();++index) {
      const auto& input=inputs[index];sunhorizon_navigation_interfaces::msg::SpatialSourceEvidence source;
      source.source_id=input.source_id=="belly"?0:1;source.producer_epoch=input.epoch;source.frame_id=input.frame_id;
      source.frame_sequence=input.frame_sequence;source.source_ros_ns=input.source_stamp_ros_ns;
      source.first_receipt_ros_ns=input.local_receipt_ros_ns;source.first_receipt_steady_ns=input.local_receipt_steady_ns;
      source.valid_until_ros_ns=std::min(input.source_stamp_ros_ns+kAge,input.local_receipt_ros_ns+kAge);
      source.valid_until_steady_ns=input.local_receipt_steady_ns+(source.valid_until_ros_ns-input.local_receipt_ros_ns);
      const auto found=std::find_if(result.sources.begin(),result.sources.end(),[&](const auto& e){return e.source_id==input.source_id;});
      source.geometry_valid=found!=result.sources.end() && found->assessment==sv::SourceAssessment::kFresh;
      const auto contract=std::find_if(query_.source_contracts.begin(),query_.source_contracts.end(),[&](const auto& c){return c.source_id==input.source_id;});
      const uint8_t bit=uint8_t(1U<<std::distance(query_.source_contracts.begin(),contract));
      for (const auto& cell:result.cells) {if (cell.free_sources&bit) source.contribution|=1;if (cell.occupied_sources&bit) source.contribution|=2;}
      // Do not let an optional observation expiring during work leave any
      // accepted contribution; rejecting this evaluation avoids silent reuse.
      if ((input.required_for_evaluation || source.contribution) &&
          (end_ros>source.valid_until_ros_ns || end_steady>source.valid_until_steady_ns)) {
        wire.evaluation_complete=false;wire.reason="source_expired_during_evaluation";
      }
      wire.sources.push_back(source);
    }
    if (!owners) {wire.evaluation_complete=false;wire.reason="publisher_identity_changed_during_evaluation";}
    if (!clocks(end_ros,end_steady) || private_fault_.load()) {wire.evaluation_complete=false;wire.reason="evaluation_clock_or_private_tf_fault";}
    {
    std::lock_guard<std::mutex> lock(publication_mutex_);
    if (terminal_.load()) return true; // Private TF revoked during computation.
    if (private_fault_.load()) {wire.evaluation_complete=false;wire.reason="private_tf_fault_during_publication";}
    wire.evaluation_sequence=++evaluation_sequence_;
    last_=wire;has_last_=true;timing_stage(anchor.pending.sequence,SpatialTimingStage::PublishStart);output_->publish(wire);
    timing_stage(anchor.pending.sequence,SpatialTimingStage::PublishReturn);
    if (wire.evaluation_complete && first_valid_h_ns_==0) first_valid_h_ns_=end_ros;
    if (!wire.evaluation_complete) {terminal_.store(true);mark_identity_terminal();RCLCPP_ERROR(get_logger(),"spatial terminal: %s",wire.reason.c_str());}
    }
    if (!wire.evaluation_complete) dump_frame_timing();
    return true;
  }
  bool initial_tf_pending(const Sample& anchor,const tf2::TransformException& error) {
    if (required_tf_qualified_) return false;
    const auto ros=get_clock()->now().nanoseconds(),steady=dm::steady_ns();
    if (terminal_.load()) anchor_failure("initial_tf_pending","anchor_terminal_already_set",anchor,ros,steady,false,-1,1);
    if (private_fault_.load()) anchor_failure("initial_tf_pending","anchor_private_tf_fault",anchor,ros,steady,false,1,0);
    if (!clocks(ros,steady)) anchor_failure("initial_tf_pending",clock_predicate(ros,steady),anchor,ros,steady,true,0,0);
    if (!fresh(anchor,ros,steady)) anchor_failure("initial_tf_pending",freshness_predicate(anchor,ros,steady),anchor,ros,steady,true,0,0);
    if (!frozen_owners_valid()) throw std::runtime_error("publisher_identity_changed_during_initial_tf_wait");
    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,
        "spatial pending required TF before first qualification: %s",error.what());
    return true;
  }
  void mark_identity_terminal() {
    {std::lock_guard<std::mutex> lock(identity_mutex_);identity_report_dirty_=true;}
    identity_report();
  }
  void fault(const std::string& reason) {
    {
    std::lock_guard<std::mutex> lock(publication_mutex_);
    if (terminal_.exchange(true)) return;
    mark_identity_terminal();
    RCLCPP_ERROR(get_logger(),"spatial terminal: %s",reason.c_str());
    if (has_last_) {
      auto revoked=last_;revoked.evaluation_complete=false;revoked.reason="terminal_revocation:"+reason;
      revoked.header.stamp=get_clock()->now();revoked.evaluation_steady_ns=dm::steady_ns();revoked.evaluation_sequence=++evaluation_sequence_;
      output_->publish(revoked);  // Original anchor/measurement/receipt/expiry unchanged.
    }
    }
    dump_frame_timing(); // Only after the original terminal/revocation action.
  }
  void timing_stage(uint64_t seq,SpatialTimingStage stage) {frame_timing_.stage(seq,stage,dm::steady_ns());}
  void dump_frame_timing(bool terminal_tail=true) noexcept {
    try {
      // Claim all bounded snapshots before any formatting/I/O. No ring lock
      // survives the copy, and in-flight callbacks may have end=0 in the copy.
      const auto hint=frame_timing_.hint_snapshot_once();
      const auto private_hint=private_tf_timing_.hint_snapshot_once();
      const auto tail=terminal_tail?frame_timing_.snapshot_once():SpatialFrameTimingRing::Snapshot{};
      const auto private_tail=terminal_tail?private_tf_timing_.snapshot_once():SpatialPrivateTfTimingRing::Snapshot{};
      log_frame_timing(hint,"first_observed_expiry_hint");
      log_private_tf_timing(private_hint,"first_observed_expiry_hint");
      if (terminal_tail) log_frame_timing(tail,"producer_terminal");
      if (terminal_tail) log_private_tf_timing(private_tail,"producer_terminal");
    } catch (...) {} // Diagnostic failure cannot alter terminal or wire behavior.
  }
  void log_frame_timing(const SpatialFrameTimingRing::Snapshot& snapshot,const char* kind) {
      if (!snapshot.claimed) return;
      const auto& oldest=snapshot.records[0];const auto& newest=snapshot.records[snapshot.count?snapshot.count-1:0];
      RCLCPP_ERROR(get_logger(),"spatial timing begin schema=1 kind=%s records=%zu capacity=256 overwritten=%llu oldest_seq=%llu newest_seq=%llu oldest_source_ns=%lld newest_source_ns=%lld trigger_d_ns=%lld trigger_ros_ns=%lld trigger_steady_ns=%lld scope=belly_required rmw_zero=unknown stages=steady_ns hint_source=unverified first_actual_expiry=unknown",
          kind,snapshot.count,static_cast<unsigned long long>(snapshot.overwritten),static_cast<unsigned long long>(oldest.sequence),static_cast<unsigned long long>(newest.sequence),
          static_cast<long long>(oldest.source_ns),static_cast<long long>(newest.source_ns),static_cast<long long>(snapshot.trigger_d_ns),
          static_cast<long long>(snapshot.trigger_ros_ns),static_cast<long long>(snapshot.trigger_steady_ns));
      const std::array<const char*,14> names{{"leg_check_start","leg_check_end","model_start","model_end","common_tf_start","common_tf_end",
          "sampling_start","sampling_end","fusion_start","fusion_end","owners_start","owners_end","publish_start","publish_return"}};
      for (std::size_t i=0;i<snapshot.count;++i) {
        const auto& v=snapshot.records[i];std::ostringstream row;
        row<<"spatial timing frame seq="<<v.sequence<<" source_ns="<<v.source_ns<<" callback_ros_ns="<<v.callback_ros_ns
           <<" callback_steady_ns="<<v.callback_steady_ns<<" rmw_source_ns="<<v.rmw_source_ns<<" rmw_received_ns="<<v.rmw_received_ns
           <<" paired_ns="<<v.paired_ns<<" first_evaluate_ns="<<v.first_evaluate_ns<<" last_evaluate_ns="<<v.last_evaluate_ns
           <<" evaluate_timer="<<int(v.last_evaluate_timer)<<" attempts="<<v.attempts<<" dropped="<<int(v.dropped)
           <<" first_future_wait_ns="<<v.first_future_wait_ns<<" last_future_wait_ns="<<v.last_future_wait_ns
           <<" last_future_link_index="<<v.last_future_link_index<<" last_future_source_query_ns="<<v.last_future_source_query_ns
           <<" last_future_latest_upper_ns="<<v.last_future_latest_upper_ns;
        for (std::size_t j=0;j<names.size();++j) row<<' '<<names[j]<<"_ns="<<v.stages[j];
        RCLCPP_ERROR(get_logger(),"%s",row.str().c_str());
      }
      RCLCPP_ERROR(get_logger(),"spatial timing end kind=%s records=%zu",kind,snapshot.count);
  }
  void log_private_tf_timing(const SpatialPrivateTfTimingRing::Snapshot& snapshot,const char* kind) {
    if (!snapshot.claimed) return;
    const auto oldest=snapshot.count?snapshot.records[0].sequence:0;
    const auto newest=snapshot.count?snapshot.records[snapshot.count-1].sequence:0;
    std::ostringstream header;
    header<<"spatial private_tf timing begin schema=1 kind="<<kind<<" records="<<snapshot.count
       <<" capacity=256 overwritten="<<snapshot.overwritten<<" oldest_seq="<<oldest<<" newest_seq="<<newest
       <<" trigger_d_ns="<<snapshot.trigger_d_ns<<" trigger_ros_ns="<<snapshot.trigger_ros_ns
       <<" trigger_steady_ns="<<snapshot.trigger_steady_ns<<" rmw_zero=unknown partial_end_zero=unknown hint_source=unverified";
    for (std::size_t i=0;i<model_->specs().size();++i) header<<" link"<<i<<'='<<model_->specs()[i].link_name;
    RCLCPP_ERROR(get_logger(),"%s",header.str().c_str());
    for (std::size_t i=0;i<snapshot.count;++i) {
      const auto& v=snapshot.records[i];std::ostringstream row;
      row<<"spatial private_tf timing frame seq="<<v.sequence<<" is_static="<<int(v.is_static)
         <<" message_count="<<v.message_count<<" scanned_count="<<v.scanned_count<<" inserted_count="<<v.inserted_count
         <<" rmw_source_ns="<<v.rmw_source_ns<<" rmw_received_ns="<<v.rmw_received_ns
         <<" callback_ros_ns="<<v.callback_ros_ns<<" callback_steady_ns="<<v.callback_steady_ns
         <<" owner_start_ns="<<v.owner_start_ns<<" owner_end_ns="<<v.owner_end_ns
         <<" inserted_end_ns="<<v.inserted_end_ns<<" callback_end_ns="<<v.callback_end_ns<<" outcome="<<v.outcome;
      for (std::size_t link=0;link<12;++link) row<<" link"<<link<<"_count="<<v.link_count[link]
         <<" link"<<link<<"_min_source_ns="<<v.link_min_source_ns[link]<<" link"<<link<<"_max_source_ns="<<v.link_max_source_ns[link];
      RCLCPP_ERROR(get_logger(),"%s",row.str().c_str());
    }
    RCLCPP_ERROR(get_logger(),"spatial private_tf timing end kind=%s records=%zu",kind,snapshot.count);
  }
  std::string contract_,query_digest_,producer_epoch_,boot_,common_frame_;
  std::string identity_mode_,private_fqn_,private_gid_,private_static_fqn_,private_static_gid_;
  std::mutex identity_mutex_;bool identity_report_dirty_{true};
  std::set<std::string> qualified_topics_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr identity_output_;
  rclcpp::TimerBase::SharedPtr identity_watch_,anchor_watch_;
  std::optional<Sample> pending_anchor_;uint64_t dropped_anchors_{0};
  SpatialFrameTimingRing frame_timing_;
  SpatialPrivateTfTimingRing private_tf_timing_;
  int64_t first_valid_h_ns_{0};
  rclcpp::Subscription<sunhorizon_navigation_interfaces::msg::ObstacleCoverageStatus>::SharedPtr coverage_timing_hint_;
  double tolerance_{0},margin_{0};bool chest_enabled_{false},has_last_{false},required_tf_qualified_{false};
  std::atomic<bool> terminal_{false};std::mutex publication_mutex_;
  uint64_t evaluation_sequence_{0};RegionClockGuard clock_guard_;
  int64_t last_ros_{0},last_steady_{0},previous_ros_{0},previous_steady_{0};
  sv::Query query_;std::array<SourceState,2> sources_;
  struct PrivateHistory {int64_t stamp;std::string canonical;bool is_static;};
  std::map<std::string,PrivateHistory> private_history_;
  std::map<std::string,std::string> allowed_edges_;std::atomic<bool> private_fault_{false};
  std::unique_ptr<dm::Model> model_;std::unique_ptr<tf2_ros::Buffer> global_,private_;
  rclcpp::Node::SharedPtr global_tf_node_;
  std::unique_ptr<tf2_ros::TransformListener> global_listener_;
  rclcpp::CallbackGroup::SharedPtr data_group_,tf_group_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr private_tf_,private_static_;
  rclcpp::Publisher<Evidence>::SharedPtr output_;Evidence last_;
};
}  // namespace sunhorizon_rgbd_adapter
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    rclcpp::NodeOptions options;options.enable_rosout(false).start_parameter_services(false).start_parameter_event_publisher(false);
    auto node=std::make_shared<sunhorizon_rgbd_adapter::SpatialVisibilityNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{},3);
    executor.add_node(node);executor.spin();rclcpp::shutdown();return 0;
  } catch (const std::exception& error) {std::cerr<<"spatial_visibility_node: "<<error.what()<<'\n';rclcpp::shutdown();return 2;}
}
