#include <algorithm>
#include <atomic>
#include <mutex>
#include <thread>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <cmath>
#include <functional>
#include <memory>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include "sunhorizon_rgbd_adapter/dynamic_self_model.hpp"
#include "sunhorizon_rgbd_adapter/dynamic_self_point_filter.hpp"
#include "sunhorizon_rgbd_adapter/spatial_leg_pairing.hpp"

#include "sunhorizon_rgbd_adapter/pointcloud_self_exclusion.hpp"
#include "sunhorizon_rgbd_adapter/lidar_self_filter.hpp"
#include "sunhorizon_rgbd_adapter/pointcloud_self_exclusion_direct.hpp"
#include "sunhorizon_navigation_core/latest_observer.hpp"
#include "sunhorizon_rgbd_adapter/rgbd_freshness.hpp"

namespace sunhorizon_rgbd_adapter {
namespace {
using Cloud = sensor_msgs::msg::PointCloud2;
using Status = diagnostic_msgs::msg::DiagnosticStatus;
double thread_cpu_seconds() {
  timespec value{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0) return -1.;
  return double(value.tv_sec) + double(value.tv_nsec)*1.e-9;
}

class PointcloudSelfExclusion final : public rclcpp::Node, public PointcloudSelfExclusionDirect {
 public:
  explicit PointcloudSelfExclusion(
    SelfExclusionDirectOptions direct = {}, SelfExclusionDirectSink sink = {})
    : Node("sunhorizon_pointcloud_self_exclusion", direct.node_options),
      direct_options_(std::move(direct)), direct_sink_(std::move(sink)) {
    if (direct_sink_ && (!direct_options_.transforms || direct_options_.source_identity.empty() ||
      direct_options_.source_epoch.empty() || direct_options_.input_frame.empty()))
      throw std::invalid_argument("direct_self_filter_requires_actual_source_and_shared_transforms");
    enabled_ = parameter("enabled", false);
    const auto mode = parameter<std::string>("self_exclusion_mode", "aabb-v1");
    if (mode != "aabb-v1" && mode != "dynamic-mesh-v1")
      throw std::invalid_argument("unknown_self_exclusion_mode");
    dynamic_mesh_ = mode == "dynamic-mesh-v1";
    base_frame_ = parameter<std::string>("base_frame", "base_footprint");
    const auto minimum = parameter<std::vector<double>>(
        "exclusion_min_xyz", {0., 0., 0.});
    const auto maximum = parameter<std::vector<double>>(
        "exclusion_max_xyz", {0., 0., 0.});
    max_age_ = parameter("max_cloud_age_sec", .30);
    tf_timeout_ = parameter("tf_timeout_sec", .02);
    max_removed_fraction_ = parameter("max_removed_fraction", .02);
    shadow_sensor_range_ = parameter("shadow_sensor_range_m", 0.);
    const bool sensor_origin_accepted = parameter("cloud_origin_is_sensor", false);
    if (minimum.size() != 3 || maximum.size() != 3) {
      throw std::invalid_argument("exclusion bounds must each contain three coordinates");
    }
    std::copy(minimum.begin(), minimum.end(), box_.minimum.begin());
    std::copy(maximum.begin(), maximum.end(), box_.maximum.begin());
    if ((enabled_ && !dynamic_mesh_ && !valid_exclusion_box(box_)) || base_frame_.empty() ||
        !std::isfinite(max_age_) || max_age_ <= 0. || max_age_ > 1. ||
        !std::isfinite(tf_timeout_) || tf_timeout_ < 0. || tf_timeout_ > .05 ||
        !std::isfinite(max_removed_fraction_) || max_removed_fraction_ <= 0. ||
        max_removed_fraction_ > .10 || !std::isfinite(shadow_sensor_range_) ||
        shadow_sensor_range_ < 0. || shadow_sensor_range_ > 2. ||
        (shadow_sensor_range_ > 0. && !sensor_origin_accepted)) {
      throw std::invalid_argument("invalid self-exclusion bounds or freshness/TF/fraction limits");
    }
    boxes_.push_back(box_);
    const auto extra_min = parameter<std::vector<double>>("additional_exclusion_min_xyz", {});
    const auto extra_max = parameter<std::vector<double>>("additional_exclusion_max_xyz", {});
    if (extra_min.size() != extra_max.size() || extra_min.size() % 3 != 0 ||
        extra_min.size() > 3 * (kMaxSelfExclusionBoxes - 1)) {
      throw std::invalid_argument("additional xyz box arrays must match, contain triples, and total at most " +
          std::to_string(kMaxSelfExclusionBoxes - 1) + " boxes");
    }
    for (std::size_t i = 0; i < extra_min.size(); i += 3) {
      ExclusionBox extra{};
      std::copy_n(extra_min.begin() + i, 3, extra.minimum.begin());
      std::copy_n(extra_max.begin() + i, 3, extra.maximum.begin());
      if (!valid_exclusion_box(extra)) throw std::invalid_argument("invalid additional self volume");
      boxes_.push_back(extra);
    }
    frames_ = parameter<std::vector<std::string>>("exclusion_frames", {});
    if (frames_.empty()) frames_.assign(boxes_.size(), base_frame_);
    if (frames_.size() != boxes_.size() ||
        std::any_of(frames_.begin(), frames_.end(), [](const auto& frame) {
          return frame.empty() || frame.front() == '/';
        })) {
      throw std::invalid_argument("exclusion_frames must name one nonempty TF frame per box");
    }
    const auto input = parameter<std::string>("input_topic", "/sunhorizon/airy/points");
    const auto output = parameter<std::string>(
        "output_topic", "/sunhorizon/navigation/airy/points");
    if (dynamic_mesh_) {
      if (!enabled_ || input != "/sunhorizon/navigation/belly_self/components" ||
          output != "/sunhorizon/navigation/belly/points" || base_frame_ != "base_link" ||
          shadow_sensor_range_ != 0. || !extra_min.empty() || (frames_ != std::vector<std::string>{base_frame_}) ||
          std::any_of(minimum.begin(), minimum.end(), [](double v){return v != 0.;}) ||
          std::any_of(maximum.begin(), maximum.end(), [](double v){return v != 0.;}) || tf_timeout_ <= 0. || tf_timeout_ > .02)
        throw std::invalid_argument("dynamic_mesh_requires_exclusive_belly_optical_mode");
      load_dynamic_model();
    }
    if (input.empty() || output.empty() || input == output) {
      throw std::invalid_argument("input and output topics must be distinct and nonempty");
    }
    if (direct_sink_ && (dynamic_mesh_ || !enabled_))
      throw std::invalid_argument("direct_airy_filter_requires_original_enabled_aabb_mode");
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    if (!direct_sink_ || direct_options_.publish_ros_compatibility) {
      cloud_pub_ = create_publisher<Cloud>(output, qos);
      if (direct_sink_) direct_observer_ = std::make_unique<CloudObserver>(
        [this](auto cloud) {cloud_pub_->publish(*cloud);},
        [this](std::exception_ptr) {direct_failed_.store(true);});
    }
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        parameter<std::string>("diagnostics_topic",
        "/sunhorizon/navigation/airy/self_exclusion_diagnostics"), 1);
    tf_buffer_ = direct_sink_ ? direct_options_.transforms : std::make_shared<tf2_ros::Buffer>(get_clock());
    if (dynamic_mesh_) {
      // This listener must use the authoritative global camera chain even when
      // the owning belly launch remaps its legacy /tf to the private model.
      rclcpp::NodeOptions options; options.use_global_arguments(false);
      options.start_parameter_services(false); options.enable_rosout(false);
      global_tf_node_ = std::make_shared<rclcpp::Node>("sunhorizon_dynamic_self_global_tf", options);
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, global_tf_node_, true);
      private_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      private_tf_node_ = std::make_shared<rclcpp::Node>("sunhorizon_dynamic_self_private_tf", options);
      private_tf_sub_ = private_tf_node_->create_subscription<tf2_msgs::msg::TFMessage>(
        "/sunhorizon/navigation/belly_self/tf", rclcpp::QoS(100).reliable().durability_volatile(),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg,const rclcpp::MessageInfo& info){on_private_tf(*msg,info);});
      private_tf_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
      private_tf_executor_->add_node(private_tf_node_);

    } else if (!direct_sink_) tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    if (!direct_sink_) cloud_sub_ = create_subscription<Cloud>(input, qos,
        std::bind(&PointcloudSelfExclusion::on_cloud, this, std::placeholders::_1));
    if (dynamic_mesh_ && (std::string(cloud_sub_->get_topic_name()) != "/sunhorizon/navigation/belly_self/components" ||
        std::string(cloud_pub_->get_topic_name()) != "/sunhorizon/navigation/belly/points"))
      throw std::invalid_argument("dynamic_belly_topics_cannot_be_remapped");
    reason_ = enabled_ ? "waiting_for_cloud" : "disabled_no_output";
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      if (enabled_ && last_stamp_sec_ > 0. &&
          !measurement_fresh(measurement_age_seconds(now().seconds(), last_stamp_sec_), max_age_)) {
        reason_ = "cloud_measurement_stale";
      }
      publish_status();
    });
    if (dynamic_mesh_) {
      pending_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {
        if (pending_dynamic_ && on_dynamic_cloud(*pending_dynamic_)) pending_dynamic_.reset();
      }); // Default mutually-exclusive group; private TF has its own executor.
      private_tf_thread_ = std::thread([this](){
        try {private_tf_executor_->spin();} catch(const std::exception&){private_tf_invalid_=true;}
      });
      private_tf_buffer_->setUsingDedicatedThread(true);
    }
  }

  ~PointcloudSelfExclusion() override {
    stop();
    if(private_tf_executor_)private_tf_executor_->cancel();
    if(private_tf_thread_.joinable())private_tf_thread_.join();
  }

  rclcpp::Node::SharedPtr node() override {return shared_from_this();}
  bool failed() const override {return direct_failed_.load();}
  void stop() override {
    direct_closed_.store(true);
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (direct_observer_) {direct_observer_->close(); direct_observer_.reset();}
  }
  bool receive(std::shared_ptr<const SelfExclusionDirectInput> input) override {
    const auto result=receive_result(std::move(input));
    return result.accepted() || result.state==SelfExclusionDirectState::Duplicate;
  }
  SelfExclusionDirectResult receive_result(std::shared_ptr<const SelfExclusionDirectInput> input) override {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    using State=SelfExclusionDirectState;
    if (!direct_sink_ || direct_closed_.load() || direct_failed_.load())
      return record_direct(State::InternalFault,"direct_filter_not_active");
    if (!input || !input->cloud) return record_direct(State::InternalFault,"direct_filter_input_missing");
    const auto & stamp = input->stamp;
    const auto source = int64_t(input->cloud->header.stamp.sec)*1000000000LL + input->cloud->header.stamp.nanosec;
    if (stamp.source_identity != direct_options_.source_identity || stamp.epoch != direct_options_.source_epoch ||
      input->cloud->header.frame_id != direct_options_.input_frame || stamp.source_ns != source ||
      input->cloud->header.stamp.nanosec>=1000000000U || stamp.raw_source_ns <= 0 || !stamp.sequence || source <= 0 ||
      stamp.first_receipt_ros_ns < source || stamp.first_receipt_steady_ns <= 0) {
      return record_direct(State::InternalFault,"direct_filter_identity_or_clock_invalid");
    }
    const auto ros=now().nanoseconds(),steady=dynamic_self_model::steady_ns();
    if(ros<=0 || steady<=0 || ros<direct_last_ros_ || steady<direct_last_steady_ ||
       ros<stamp.first_receipt_ros_ns || steady<stamp.first_receipt_steady_ns)
      return record_direct(State::InternalFault,"direct_filter_clock_regressed_or_future",ros,steady);
    direct_last_ros_=ros;direct_last_steady_=steady;
    if (direct_last_ && stamp.sequence == direct_last_->stamp.sequence && source == direct_last_->stamp.source_ns) {
      const auto & old = direct_last_->stamp;
      if (stamp.raw_source_ns != old.raw_source_ns || stamp.first_receipt_ros_ns != old.first_receipt_ros_ns ||
        stamp.first_receipt_steady_ns != old.first_receipt_steady_ns ||
        stamp.normalization_start_steady_ns!=old.normalization_start_steady_ns ||
        stamp.normalization_end_steady_ns!=old.normalization_end_steady_ns ||
        stamp.normalization_thread_cpu_ns!=old.normalization_thread_cpu_ns)
        return record_direct(State::InternalFault,"direct_filter_duplicate_metadata_changed",ros,steady);
      return record_direct(State::Duplicate,"direct_filter_duplicate",ros,steady); // No output or renewal.
    }
    if (direct_last_ && (stamp.sequence <= direct_last_->stamp.sequence || source <= direct_last_->stamp.source_ns)) {
      return record_direct(State::InternalFault,"direct_filter_source_reordered",ros,steady);
    }
    direct_last_ = input; direct_current_ = std::move(input); direct_emitted_ = false;
    direct_result_={State::InternalFault,"direct_filter_no_result",ros,steady};
    try{on_cloud(direct_current_->cloud);}
    catch(...){
      record_direct(State::InternalFault,"direct_filter_processing_exception");
      direct_current_.reset();throw;
    }
    direct_current_.reset();
    if(!direct_emitted_ && direct_result_.reason=="direct_filter_no_result")
      return record_direct(State::InternalFault,"direct_filter_unclassified_no_output");
    return direct_result_;
  }
 private:
  SelfExclusionDirectResult record_direct(SelfExclusionDirectState state,const std::string& reason) {
    if(!direct_sink_)return {};
    return record_direct(state,reason,now().nanoseconds(),dynamic_self_model::steady_ns());
  }
  SelfExclusionDirectResult record_direct(SelfExclusionDirectState state,const std::string& reason,
      int64_t ros,int64_t steady) {
    if(!direct_sink_)return {};
    direct_result_={state,reason,ros,steady};
    if(state==SelfExclusionDirectState::InternalFault)direct_failed_.store(true);
    return direct_result_;
  }
  void emit_cloud(Cloud cloud) {
    if (!direct_sink_) {cloud_pub_->publish(std::move(cloud)); return;}
    if (!direct_current_ || direct_closed_.load() || direct_failed_.load() || !fresh(cloud)) return;
    auto output = std::make_shared<SelfExclusionDirectOutput>();
    output->input = direct_current_;
    output->cloud = std::make_shared<const Cloud>(std::move(cloud));
    output->ground_blocked=ground_blocked_;
    output->ray_origins=retained_ray_origins_;output->sample_ids=retained_sample_ids_;
    output->lidar_audit=retained_lidar_audit_;
    output->input_count = input_count_; output->removed_count = removed_count_; output->retained_count = retained_count_;
    output->tf_wall_sec = tf_wall_sec_; output->filter_wall_sec = filter_wall_sec_;
    output->filter_thread_cpu_sec = filter_thread_cpu_sec_;
    direct_sink_(output);
    if (direct_observer_) direct_observer_->enqueue(output->cloud);
    direct_emitted_ = true;
    record_direct(SelfExclusionDirectState::Accepted,"filtered");
  }
  template<class T> T parameter(const std::string& name, const T& value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }

  void load_dynamic_model() {
    publisher_identity_mode_ = parameter<std::string>("publisher_identity_mode", "exact-gid-v1");
    if(publisher_identity_mode_ != "exact-gid-v1" && publisher_identity_mode_ != "startup-pin-once-v1")
      throw std::invalid_argument("publisher_identity_mode_invalid");
    private_tf_owner_node_ = parameter<std::string>("private_tf_publisher", "");
    private_tf_owner_gid_ = parameter<std::string>("private_tf_gid", "");
    if(private_tf_owner_node_.empty() || private_tf_owner_node_.size()>128 ||
       private_tf_owner_node_.front()!='/' ||
       private_tf_owner_node_.find_first_not_of("/ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_")!=std::string::npos ||
       (publisher_identity_mode_ == "startup-pin-once-v1" ? !private_tf_owner_gid_.empty() :
        (private_tf_owner_gid_.size()!=2*RMW_GID_STORAGE_SIZE ||
         private_tf_owner_gid_.find_first_not_of("0123456789abcdef")!=std::string::npos ||
         private_tf_owner_gid_.find_first_not_of('0')==std::string::npos)))
      throw std::invalid_argument("private_tf_owner_identity_required");
    self_tolerance_ = parameter("self_tolerance_m", sunhorizon_obstacle_coverage::dynamic_self_geometry::kSelfSurfaceToleranceM);
    if (!std::isfinite(self_tolerance_) || self_tolerance_ <= 0. || self_tolerance_ > sunhorizon_obstacle_coverage::dynamic_self_geometry::kSelfSurfaceToleranceM)
      throw std::invalid_argument("invalid_self_tolerance_m");
    const auto paths = parameter<std::vector<std::string>>("model_mesh_paths", {});
    const auto hashes = parameter<std::vector<std::string>>("model_mesh_sha256", {});
    const auto counts = parameter<std::vector<int64_t>>("model_triangle_counts", {});
    const auto links = parameter<std::vector<std::string>>("model_link_names", {});
    const auto parents = parameter<std::vector<std::string>>("model_parent_frames", {});
    if (paths.size() != 12 || hashes.size() != 12 || counts.size() != 12 ||
        links.size() != 12 || parents.size() != 12)
      throw std::invalid_argument("dynamic_model_array_layout");
    std::vector<dynamic_self_model::AssetSpec> specs;
    for (std::size_t i=0;i<12;++i) {
      if (counts[i] <= 0 || counts[i] > 1000000) throw std::invalid_argument("model_triangle_count");
      specs.push_back({paths[i], hashes[i], static_cast<std::size_t>(counts[i]), links[i], parents[i]});
    }
    model_ = std::make_unique<dynamic_self_model::Model>(std::move(specs));
  }

  static std::string gid_hex(const uint8_t* bytes,std::size_t size) {
    std::ostringstream out;
    for(std::size_t i=0;i<size;++i)out<<std::hex<<std::setfill('0')<<std::setw(2)<<unsigned(bytes[i]);
    return out.str();
  }
  bool private_tf_graph(std::string& actual_gid) {
    const auto endpoints=private_tf_node_->get_publishers_info_by_topic("/sunhorizon/navigation/belly_self/tf");
    if(endpoints.empty()) {if(private_tf_owner_seen_)private_tf_invalid_=true;return false;}
    if(endpoints.size()!=1) {private_tf_invalid_=true;return false;}
    const auto& e=endpoints.front();const auto& gid=e.endpoint_gid();
    actual_gid=gid_hex(gid.data(),gid.size());
    if(e.topic_type()!="tf2_msgs/msg/TFMessage" || actual_gid.size()!=2*RMW_GID_STORAGE_SIZE ||
       actual_gid.find_first_not_of('0')==std::string::npos) {private_tf_invalid_=true;return false;}
    const auto ns=e.node_namespace(),name=e.node_name();
    const bool unknown_ns=ns=="_NODE_NAMESPACE_UNKNOWN_" || ns=="/_NODE_NAMESPACE_UNKNOWN_";
    const bool unknown_name=name=="_NODE_NAME_UNKNOWN_";
    if(unknown_ns || unknown_name) {
      const auto split=private_tf_owner_node_.find_last_of('/');
      const auto wanted_ns=split==0 ? "/" : private_tf_owner_node_.substr(0,split);
      const auto wanted_name=private_tf_owner_node_.substr(split+1);
      // Only the RMW discovery placeholders may be Pending before qualification.
      // Any known contradicting field remains a hard identity failure.
      if(private_tf_owner_seen_ || (!unknown_ns && ns!=wanted_ns) ||
         (!unknown_name && name!=wanted_name))private_tf_invalid_=true;
      return false;
    }
    auto fqn=ns;if(fqn.empty()||fqn.back()!='/')fqn+='/';fqn+=name;
    if(fqn!=private_tf_owner_node_) {private_tf_invalid_=true;return false;}
    return true;
  }
  bool private_tf_owner_current() {
    std::string actual;if(!private_tf_graph(actual))return false;
    std::lock_guard<std::mutex> lock(private_identity_mutex_);
    return private_tf_owner_seen_ && !private_tf_owner_gid_.empty() && actual==private_tf_owner_gid_;
  }
  void on_private_tf(const tf2_msgs::msg::TFMessage& msg,const rclcpp::MessageInfo& info) {
    const auto& gid=info.get_rmw_message_info().publisher_gid;
    const auto message_gid=gid_hex(gid.data,RMW_GID_STORAGE_SIZE);
    std::string graph_gid;
    if(!private_tf_graph(graph_gid)) {
      std::lock_guard<std::mutex> lock(private_identity_mutex_);
      if(private_tf_owner_seen_ || (!graph_gid.empty() && message_gid!=graph_gid) ||
         (!private_tf_owner_gid_.empty() && message_gid!=private_tf_owner_gid_))private_tf_invalid_=true;
      return;
    }
    {
      std::lock_guard<std::mutex> lock(private_identity_mutex_);
      if(private_tf_owner_gid_.empty() && publisher_identity_mode_=="startup-pin-once-v1" &&
         !private_tf_invalid_ && message_gid==graph_gid) {
        private_tf_owner_gid_=graph_gid;private_tf_owner_seen_=true;
        RCLCPP_INFO(get_logger(),"{\"event\":\"dynamic_self_private_tf_identity_frozen\",\"topic\":\"/sunhorizon/navigation/belly_self/tf\",\"type\":\"tf2_msgs/msg/TFMessage\",\"publisher\":\"%s\",\"gid\":\"%s\",\"identity_mode\":\"startup-pin-once-v1\"}",
          private_tf_owner_node_.c_str(),private_tf_owner_gid_.c_str());
      }
      if(private_tf_owner_gid_.empty() || message_gid!=private_tf_owner_gid_ || graph_gid!=private_tf_owner_gid_) {
        private_tf_invalid_=true;return;
      }
      private_tf_owner_seen_=true;
    }
    for (const auto& tf : msg.transforms) {
      const bool approved = std::any_of(model_->specs().begin(), model_->specs().end(),
        [&](const auto& spec){return tf.header.frame_id == spec.parent_frame && tf.child_frame_id == spec.link_name;});
      // In particular, never admit the old private camera transform.
      if (!approved) {
        if(std::any_of(model_->specs().begin(),model_->specs().end(),
             [&](const auto& spec){return tf.child_frame_id==spec.link_name;}))private_tf_invalid_=true;
        continue;
      }
      try {
        dynamic_self_model::rigid(tf.transform);
        const int64_t source = static_cast<int64_t>(tf.header.stamp.sec)*1000000000LL + tf.header.stamp.nanosec;
        if (source <= 0 || !private_tf_buffer_->setTransform(tf,"approved_private_leg_model",false))
          private_tf_invalid_ = true;
      } catch (const std::exception&) {private_tf_invalid_=true;}
    }
  }

  struct PendingCloud {
    Cloud::ConstSharedPtr cloud;
    int64_t source_ns,receipt_ros_ns,receipt_steady_ns;
    int64_t last_ros_ns{0},last_steady_ns{0};
  };
  bool pending_fresh(PendingCloud& pending) {
    const auto ros=get_clock()->now().nanoseconds(),steady=dynamic_self_model::steady_ns();
    const auto budget=static_cast<int64_t>(max_age_*1.e9);
    const bool valid=ros>=pending.last_ros_ns && steady>=pending.last_steady_ns &&
      pending.source_ns>0 && ros>=pending.source_ns && ros>=pending.receipt_ros_ns &&
      steady>=pending.receipt_steady_ns && ros-pending.source_ns<=budget &&
      ros-pending.receipt_ros_ns<=budget && steady-pending.receipt_steady_ns<=budget;
    pending.last_ros_ns=ros;pending.last_steady_ns=steady;return valid;
  }
  // false holds this same source/first receipt; true consumes or rejects it.
  bool on_dynamic_cloud(PendingCloud& pending) {
    const auto& cloud=*pending.cloud;
    if (!pending_fresh(pending)) {reason_="dynamic_pending_source_or_receipt_stale";publish_status();return true;}
    const bool owner_current=private_tf_owner_current();
    if(!owner_current && private_tf_owner_seen_)private_tf_invalid_=true;
    if (cloud.header.frame_id != "camera_belly_depth_optical_frame" || private_tf_invalid_ || !owner_current) {
      reason_="dynamic_optical_frame_or_private_tf_invalid";publish_status();return true;
    }
    if (required_tf_qualified_) {
      try {
        if (spatial_leg_availability(*private_tf_buffer_,model_->specs(),pending.source_ns,
            get_clock()->get_clock_type())==SpatialLegAvailability::FuturePending) {
          const bool first_wait=reason_!="dynamic_future_leg_tf_pending";
          reason_="dynamic_future_leg_tf_pending";if(first_wait)publish_status();return false;
        }
      } catch (const std::exception&) {
        private_tf_invalid_=true;
        reason_="dynamic_private_source_time_tf_invalid";publish_status();return true;
      }
    }
    const auto start=std::chrono::steady_clock::now();
    std::vector<sunhorizon_obstacle_coverage::dynamic_self_geometry::LinkObservation> observations;
    try {
      observations=model_->observations(*tf_buffer_,*private_tf_buffer_,cloud.header.frame_id,pending.source_ns,
        get_clock()->get_clock_type(),dynamic_self_model::steady_ns()+static_cast<int64_t>(tf_timeout_*1.e9));
    } catch (const std::exception&) {
      tf_wall_sec_=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      reason_="dynamic_model_source_time_tf_unavailable";publish_status();return true;
    }
    required_tf_qualified_=true; // Complete exact geometry, independent of self matches/output count.
    tf_wall_sec_=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    DynamicCloudView view{cloud.data.data(),cloud.data.size(),cloud.width,cloud.height,
      cloud.point_step,cloud.row_step,cloud.is_bigendian,{}};
    std::array<bool,3> found{};const std::array<std::string,3> names{"x","y","z"};
    for(const auto& field:cloud.fields)for(std::size_t i=0;i<3;++i)if(field.name==names[i]) {
      const std::size_t bytes=field.datatype==sensor_msgs::msg::PointField::FLOAT32 ? 4 :
        field.datatype==sensor_msgs::msg::PointField::FLOAT64 ? 8 : 0;
      if(found[i]||field.count!=1||bytes==0){reason_="invalid_xyz_fields";publish_status();return true;}
      found[i]=true;view.xyz[i]={field.offset,bytes};
    }
    if(!std::all_of(found.begin(),found.end(),[](bool v){return v;})) {
      reason_="missing_xyz_fields";publish_status();return true;
    }
    const auto filter_start=std::chrono::steady_clock::now();
    const auto cpu_start=thread_cpu_seconds();
    auto result=filter_dynamic_self_points(view,observations,self_tolerance_);
    const auto cpu_end=thread_cpu_seconds();
    filter_thread_cpu_sec_ = cpu_start>=0. && cpu_end>=0. ? cpu_end-cpu_start : -1.;
    filter_wall_sec_=std::chrono::duration<double>(std::chrono::steady_clock::now()-filter_start).count();
    input_count_=result.input_count;removed_count_=result.removed_count;retained_count_=result.retained_count;
    dynamic_classes_=result.classifications;reason_=result.reason;
    if(!result.complete){publish_status();return true;}
    if(!exclusion_fraction_allowed(removed_count_,input_count_,max_removed_fraction_)) {
      reason_="removed_fraction_exceeded_no_output";publish_status();return true;
    }
    if(private_tf_invalid_ || !private_tf_owner_current()){private_tf_invalid_=true;reason_="private_tf_identity_changed_during_filter";publish_status();return true;}
    if(!fresh(cloud) || !pending_fresh(pending)){reason_="cloud_became_stale_during_filter";publish_status();return true;}
    if(retained_count_==0){reason_="no_retained_points_no_free_space_output";publish_status();return true;}
    Cloud output;output.header=cloud.header;output.fields=cloud.fields;
    output.height=1;output.width=static_cast<uint32_t>(retained_count_);
    output.is_bigendian=cloud.is_bigendian;output.point_step=cloud.point_step;
    output.row_step=output.width*output.point_step;output.is_dense=cloud.is_dense;
    output.data=std::move(result.data);emit_cloud(std::move(output));
    ++published_count_;publish_status();return true;
  }

  bool fresh(const Cloud& cloud) {
    if (direct_current_) {
      const auto & stamp = direct_current_->stamp;
      const auto ros = now().nanoseconds(), steady = dynamic_self_model::steady_ns();
      const int64_t limit = int64_t(max_age_*1.e9);
      const bool ordered = ros >= direct_last_ros_ && steady >= direct_last_steady_;
      direct_last_ros_ = ros; direct_last_steady_ = steady;
      if (!ordered || ros < stamp.first_receipt_ros_ns || ros < stamp.source_ns ||
        steady < stamp.first_receipt_steady_ns) {
        record_direct(SelfExclusionDirectState::InternalFault,"direct_filter_clock_regressed_or_future",ros,steady);
        return false;
      }
      if(ros-stamp.source_ns>limit || ros-stamp.first_receipt_ros_ns>limit ||
         steady-stamp.first_receipt_steady_ns>limit){
        record_direct(SelfExclusionDirectState::MotionUnavailable,"direct_filter_source_or_receipt_expired",ros,steady);
        return false;
      }
      return true;
    }
    const double stamp = static_cast<double>(cloud.header.stamp.sec) +
        cloud.header.stamp.nanosec * 1.e-9;
    return measurement_fresh(measurement_age_seconds(get_clock()->now().seconds(), stamp), max_age_);
  }

  void on_cloud(const Cloud::ConstSharedPtr cloud) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    const auto first_ros=dynamic_mesh_?get_clock()->now().nanoseconds():0;
    const auto first_steady=dynamic_mesh_?dynamic_self_model::steady_ns():0;
    if (dynamic_mesh_ && pending_dynamic_) {
      // Verify the old deadline first; even a fresh incoming cloud cannot renew
      // or replace it. An input frame/time or owner contradiction suppresses it.
      const bool old_fresh=pending_fresh(*pending_dynamic_);
      const bool owner_current=private_tf_owner_current();
      if(!owner_current && private_tf_owner_seen_)private_tf_invalid_=true;
      if(dropped_pending_clouds_<std::numeric_limits<uint64_t>::max())++dropped_pending_clouds_;
      if (!old_fresh || !fresh(*cloud) || cloud->header.frame_id!="camera_belly_depth_optical_frame" ||
          private_tf_invalid_ || !owner_current) {
        pending_dynamic_.reset();reason_=old_fresh?"dynamic_pending_input_or_owner_invalid":"dynamic_pending_source_or_receipt_stale";
        publish_status();return;
      }
      if (on_dynamic_cloud(*pending_dynamic_)) pending_dynamic_.reset();
      return; // The incoming cloud is discarded even if the old item just completed.
    }
    tf_wall_sec_ = filter_wall_sec_ = 0.;
    filter_thread_cpu_sec_ = -1.;
    ground_blocked_.reset();retained_ray_origins_.reset();retained_sample_ids_.reset();retained_lidar_audit_.reset();
    input_count_ = static_cast<std::size_t>(cloud->width) * cloud->height;
    removed_count_ = retained_count_ = 0;
    if(dynamic_mesh_)dynamic_classes_.fill(0);
    last_stamp_sec_ = static_cast<double>(cloud->header.stamp.sec) +
        cloud->header.stamp.nanosec * 1.e-9;
    input_age_sec_ = measurement_age_seconds(now().seconds(), last_stamp_sec_);
    if (!enabled_) {
      reason_ = "disabled_no_output";
      record_direct(SelfExclusionDirectState::InternalFault,reason_);
      return;
    }
    if (!fresh(*cloud) || cloud->header.frame_id.empty()) {
      reason_ = "cloud_measurement_stale_or_invalid";
      publish_status();
      return;
    }
    if (dynamic_mesh_) {
      pending_dynamic_=PendingCloud{cloud,int64_t(cloud->header.stamp.sec)*1000000000+cloud->header.stamp.nanosec,
        first_ros,first_steady};
      if (on_dynamic_cloud(*pending_dynamic_)) pending_dynamic_.reset();return;
    }
    std::vector<std::optional<ExclusionTransform>> transforms;
    const auto started = std::chrono::steady_clock::now();
    try {
      // Never request TimePointZero/latest. A missing exact-time transform drops
      // this frame so downstream source freshness must expire independently.
      // All lookups share one bounded wait budget, regardless of volume count.
      std::map<std::string, ExclusionTransform> cache;
      for (const auto& frame : frames_) {
        if (cache.count(frame) == 0) {
          const double elapsed = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - started).count();
          const auto stamped = tf_buffer_->lookupTransform(
              frame, cloud->header.frame_id,
              rclcpp::Time(cloud->header.stamp, get_clock()->get_clock_type()),
              rclcpp::Duration::from_seconds(std::max(0., tf_timeout_ - elapsed)));
          const auto& rotation = stamped.transform.rotation;
          tf2::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w);
          if (!std::isfinite(q.length2()) || std::abs(q.length2() - 1.) > .001) {
            reason_ = "invalid_transform_quaternion";
            record_direct(SelfExclusionDirectState::InternalFault,reason_);
            publish_status();
            return;
          }
          ExclusionTransform transform;
          const tf2::Matrix3x3 matrix(q);
          for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t col = 0; col < 3; ++col) {
              transform.rotation[row * 3 + col] = matrix[row][col];
            }
          }
          const auto& t = stamped.transform.translation;
          transform.translation = {t.x, t.y, t.z};
          cache.emplace(frame, transform);
        }
        transforms.push_back(cache.at(frame));
      }
    } catch (const tf2::TransformException& error) {
      tf_wall_sec_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
      reason_ = "cloud_stamp_transform_unavailable";
      const bool unavailable=dynamic_cast<const tf2::TimeoutException*>(&error) ||
        dynamic_cast<const tf2::LookupException*>(&error) ||
        dynamic_cast<const tf2::ConnectivityException*>(&error) ||
        dynamic_cast<const tf2::ExtrapolationException*>(&error);
      record_direct(unavailable ? SelfExclusionDirectState::MotionUnavailable : SelfExclusionDirectState::InternalFault,
        unavailable ? reason_ : "direct_filter_transform_invalid");
      publish_status();
      return;
    }
    tf_wall_sec_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    const auto filter_started = std::chrono::steady_clock::now();
    const auto cpu_started = thread_cpu_seconds();
    std::optional<LidarSelfFilterResult> lidar;
    if(direct_options_.lidar_model || direct_options_.lidar_model_for_scan){
      if(shadow_sensor_range_!=0.)throw std::invalid_argument("lidar_cad_cannot_remove_shadow_returns");
      const auto provider=direct_options_.lidar_model_for_scan ?
        direct_options_.lidar_model_for_scan(std::size_t(cloud->width)*cloud->height) : direct_options_.lidar_model;
      const bool audit=direct_options_.lidar_audit_requested &&
        direct_options_.lidar_audit_requested->load() && lidar_audit_attempts_<8;
      if(audit && ++lidar_audit_attempts_==8)direct_options_.lidar_audit_requested->store(false);
      lidar=filter_lidar_self_points(*direct_current_,provider,
        direct_options_.lidar_surface_tolerance_m,direct_options_.lidar_remove_self_surfaces,
        direct_options_.lidar_surface_radius_bound_m,audit);
      if(audit && !lidar->audit)direct_options_.lidar_audit_requested->store(false);
      retained_lidar_audit_=lidar->audit;
      lidar_diagnostics_.emplace();
      lidar_diagnostics_->self_surface_count=lidar->self_surface_count;
      lidar_diagnostics_->external_count=lidar->external_count;
      lidar_diagnostics_->conflict_count=lidar->conflict_count;
      lidar_diagnostics_->unobserved_count=lidar->unobserved_count;
      lidar_diagnostics_->invalid_count=lidar->invalid_count;
      lidar_diagnostics_->source_time_groups=lidar->source_time_groups;
      lidar_diagnostics_->unique_prepared_models=lidar->unique_prepared_models;
      lidar_diagnostics_->outside_cad_envelope_count=lidar->outside_cad_envelope_count;
    }
    auto result = exclude_self_framed_volumes(lidar?*lidar->cloud:*cloud, boxes_, transforms, shadow_sensor_range_,
      lidar?&lidar->ground_blocked:nullptr,
      lidar?&lidar->ray_origins:(direct_current_?direct_current_->ray_origins.get():nullptr),
      lidar?&lidar->sample_ids:nullptr);
    if(lidar || (direct_current_ && direct_current_->ray_origins))
      retained_ray_origins_=std::make_shared<const std::vector<std::array<float,3>>>(std::move(result.ray_origins));
    if(lidar)retained_sample_ids_=std::make_shared<const std::vector<uint64_t>>(std::move(result.sample_ids));
    if(lidar)ground_blocked_=std::make_shared<const std::vector<uint8_t>>(std::move(result.ground_blocked));
    if(lidar){result.input_count=lidar->input_count;result.removed_count+=lidar->removed_count;}
    const auto cpu_ended = thread_cpu_seconds();
    filter_thread_cpu_sec_ = cpu_started>=0. && cpu_ended>=0. ? cpu_ended-cpu_started : -1.;
    filter_wall_sec_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - filter_started).count();
    input_count_ = result.input_count;
    removed_count_ = result.removed_count;
    retained_count_ = result.retained_count;
    reason_ = result.reason;
    if (!result.cloud) {
      record_direct(reason_=="empty_input_cloud" ? SelfExclusionDirectState::MotionUnavailable :
        SelfExclusionDirectState::InternalFault,reason_);
      publish_status();
      return;
    }
    if (!exclusion_fraction_allowed(removed_count_, input_count_, max_removed_fraction_)) {
      reason_ = "removed_fraction_exceeded_no_output";
      record_direct(SelfExclusionDirectState::MotionUnavailable,reason_);
      publish_status();
      return;
    }
    if (!fresh(*cloud)) {
      reason_ = "cloud_became_stale_during_filter";
      publish_status();
      return;
    }
    emit_cloud(std::move(*result.cloud));
    ++published_count_;
    publish_status();
  }

  void publish_status() {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    diagnostic_msgs::msg::DiagnosticArray report;
    report.header.stamp = now();
    Status status;
    status.name = dynamic_mesh_ ? "navigation_belly_dynamic_self_exclusion" : "navigation_airy_self_exclusion";
    status.hardware_id = base_frame_;
    status.level = reason_ == "filtered" ? Status::OK : Status::WARN;
    status.message = reason_;
    auto add = [&status](const std::string& key, const std::string& value) {
      diagnostic_msgs::msg::KeyValue entry;
      entry.key = key;
      entry.value = value;
      status.values.push_back(std::move(entry));
    };
    if(lidar_diagnostics_){
      add("cad_self_surface_candidates",std::to_string(lidar_diagnostics_->self_surface_count));
      add("cad_external",std::to_string(lidar_diagnostics_->external_count));
      add("cad_outside_envelope_retained",std::to_string(lidar_diagnostics_->outside_cad_envelope_count));
      add("cad_model_conflict",std::to_string(lidar_diagnostics_->conflict_count));
      add("cad_invalid",std::to_string(lidar_diagnostics_->invalid_count));
      add("cad_surface_radius_bound_m",std::to_string(direct_options_.lidar_surface_radius_bound_m));
      add("cad_unobserved",std::to_string(lidar_diagnostics_->unobserved_count));
      add("cad_source_time_groups",std::to_string(lidar_diagnostics_->source_time_groups));
      add("cad_unique_prepared_models",std::to_string(lidar_diagnostics_->unique_prepared_models));
      add("cad_removal_enabled",direct_options_.lidar_remove_self_surfaces?"true":"false");
      add("cad_surface_tolerance_m",std::to_string(direct_options_.lidar_surface_tolerance_m));
      add("cad_tolerance_is_physical_bound","false");
    }
    add("enabled", enabled_ ? "true" : "false");
    if (direct_last_) {
      const auto & stamp = direct_last_->stamp;
      add("source_identity", stamp.source_identity); add("source_epoch", stamp.epoch);
      add("source_sequence", std::to_string(stamp.sequence));
      add("raw_source_ns", std::to_string(stamp.raw_source_ns));
      add("normalized_source_ns", std::to_string(stamp.source_ns));
      add("normalization_start_steady_ns", std::to_string(stamp.normalization_start_steady_ns));
      add("normalization_end_steady_ns", std::to_string(stamp.normalization_end_steady_ns));
      add("normalization_thread_cpu_ns", std::to_string(stamp.normalization_thread_cpu_ns));
      add("first_receipt_ros_ns", std::to_string(stamp.first_receipt_ros_ns));
      add("first_receipt_steady_ns", std::to_string(stamp.first_receipt_steady_ns));
    }
    if (dynamic_mesh_) {
      add("pending_cloud",pending_dynamic_?"true":"false");
      add("dropped_pending_clouds",std::to_string(dropped_pending_clouds_));
      if(pending_dynamic_) {
        add("pending_source_ns",std::to_string(pending_dynamic_->source_ns));
        add("pending_first_receipt_ros_ns",std::to_string(pending_dynamic_->receipt_ros_ns));
        add("pending_first_receipt_steady_ns",std::to_string(pending_dynamic_->receipt_steady_ns));
      }
      add("self_exclusion_mode", "dynamic-mesh-v1");
      add("publisher_identity_mode", publisher_identity_mode_);
      add("private_tf_publisher", private_tf_owner_node_);
      {
        std::lock_guard<std::mutex> lock(private_identity_mutex_);
        add("private_tf_frozen_gid",private_tf_owner_gid_);
      }
      add("private_tf_identity_frozen",private_tf_owner_seen_ ? "true" : "false");
      add("private_tf_identity_invalid",private_tf_invalid_ ? "true" : "false");
      add("self_tolerance_m", std::to_string(self_tolerance_));
      add("model_link_count", std::to_string(model_->specs().size()));
      const std::array<std::string,5> names{"self_surface","external_surface","invalid","model_conflict","unobserved"};
      for (std::size_t i=0;i<5;++i)add(names[i]+"_count",std::to_string(dynamic_classes_[i]));
      add("free_space_inference", "none");
    }
    add("exclusion_box_count", std::to_string(dynamic_mesh_ ? 0 : boxes_.size()));
    add("shadow_sensor_range_m", std::to_string(shadow_sensor_range_));
    add("input_age_sec", std::to_string(input_age_sec_));
    add("tf_lookup_wall_sec", std::to_string(tf_wall_sec_));
    add("filter_wall_sec", std::to_string(filter_wall_sec_));
    add("filter_thread_cpu_sec", std::to_string(filter_thread_cpu_sec_));
    add("input_count", std::to_string(input_count_));
    add("removed_count", std::to_string(removed_count_));
    add("retained_count", std::to_string(retained_count_));
    add("published_cloud_count", std::to_string(published_count_));
    add("cloud_stamp_sec", std::to_string(last_stamp_sec_));
    add("cloud_age_sec", std::to_string(
        measurement_age_seconds(now().seconds(), last_stamp_sec_)));
    report.status.push_back(std::move(status));
    status_pub_->publish(report);
  }

  bool enabled_{false}, dynamic_mesh_{false};
  bool required_tf_qualified_{false};std::optional<PendingCloud> pending_dynamic_;
  uint64_t dropped_pending_clouds_{0};rclcpp::TimerBase::SharedPtr pending_timer_;
  std::atomic<bool> private_tf_invalid_{false},private_tf_owner_seen_{false};
  std::string private_tf_owner_node_,private_tf_owner_gid_,publisher_identity_mode_;
  mutable std::mutex private_identity_mutex_;
  double self_tolerance_{sunhorizon_obstacle_coverage::dynamic_self_geometry::kSelfSurfaceToleranceM};std::array<std::size_t,5> dynamic_classes_{};
  std::unique_ptr<dynamic_self_model::Model> model_;
  rclcpp::Node::SharedPtr global_tf_node_,private_tf_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> private_tf_executor_;
  std::thread private_tf_thread_;
  std::unique_ptr<tf2_ros::Buffer> private_tf_buffer_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr private_tf_sub_;
  ExclusionBox box_{};
  std::vector<ExclusionBox> boxes_;
  std::vector<std::string> frames_;
  std::string base_frame_, reason_;
  double max_age_{.30}, tf_timeout_{.02}, max_removed_fraction_{.02}, last_stamp_sec_{0.};
  double shadow_sensor_range_{0.};
  double input_age_sec_{0.}, tf_wall_sec_{0.}, filter_wall_sec_{0.}, filter_thread_cpu_sec_{-1.};
  std::size_t input_count_{0}, removed_count_{0}, retained_count_{0}, published_count_{0};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  using CloudObserver = sunhorizon_navigation_core::LatestObserver<Cloud>;
  std::unique_ptr<CloudObserver> direct_observer_;
  std::shared_ptr<const std::vector<uint8_t>> ground_blocked_;
  std::shared_ptr<const std::vector<std::array<float,3>>> retained_ray_origins_;
  std::shared_ptr<const std::vector<uint64_t>> retained_sample_ids_;
  std::optional<LidarSelfFilterResult> lidar_diagnostics_;
  std::shared_ptr<const LidarSelfAudit> retained_lidar_audit_;
  std::size_t lidar_audit_attempts_{0};
  SelfExclusionDirectOptions direct_options_;
  SelfExclusionDirectSink direct_sink_;
  std::shared_ptr<const SelfExclusionDirectInput> direct_current_, direct_last_;
  std::recursive_mutex state_mutex_;
  std::atomic<bool> direct_closed_{false}, direct_failed_{false};
  bool direct_emitted_{false};
  SelfExclusionDirectResult direct_result_;
  int64_t direct_last_ros_{0}, direct_last_steady_{0};
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace
std::shared_ptr<PointcloudSelfExclusionDirect> make_pointcloud_self_exclusion_direct(
  SelfExclusionDirectOptions options, SelfExclusionDirectSink sink) {
  if (!sink) throw std::invalid_argument("direct_self_filter_requires_nonempty_sink");
  return std::make_shared<PointcloudSelfExclusion>(std::move(options),std::move(sink));
}
rclcpp::Node::SharedPtr make_pointcloud_self_exclusion_ros_node() {
  return std::make_shared<PointcloudSelfExclusion>();
}
}  // namespace sunhorizon_rgbd_adapter
