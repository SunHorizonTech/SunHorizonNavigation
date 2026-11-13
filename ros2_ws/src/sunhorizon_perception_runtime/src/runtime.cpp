#include "sunhorizon_perception_runtime/runtime.hpp"
#include "sunhorizon_perception_runtime/runtime_config.hpp"
#include "sunhorizon_perception_runtime/body_geometry_backend.hpp"
#include "body_pair_witness.hpp"
#include "sunhorizon_perception_runtime/body_role_dispatch.hpp"
#include "chest_candidate_history.hpp"
#include "sunhorizon_perception_runtime/rgbd_health.hpp"
#include "sunhorizon_perception_runtime/joined_future.hpp"
#include "sunhorizon_perception_runtime/role_completion.hpp"
#include "sunhorizon_perception_runtime/tf_failure_detail.hpp"
#include "sunhorizon_perception_runtime/attended_geometry.hpp"
#include "sunhorizon_perception_runtime/ground_classification_snapshot.hpp"
#include "sunhorizon_perception_runtime/spatial_evidence_delivery.hpp"
#include "sunhorizon_perception_runtime/source_time_transform.hpp"
#include "sunhorizon_camera_runtime/latest_frame_worker.hpp"
#include "sunhorizon_rgbd_adapter/spatial_source_transform.hpp"
#include "sunhorizon_rgbd_adapter/region_depth_pairing.hpp"
#include "sunhorizon_rgbd_adapter/pointcloud_self_exclusion.hpp"
#include <tf2_ros/transform_listener.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <deque>
#include <cstdlib>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <map>
#include <cstring>
#include <tuple>
#include <utility>

namespace sunhorizon_perception_runtime {
namespace cam=sunhorizon_camera_runtime;
namespace dm=sunhorizon_rgbd_adapter::dynamic_self_model;
using Frame=cam::CameraFrame::ConstSharedPtr;
using Output=std::shared_ptr<const PerceptionOutput>;
struct PreparedFrame {
  Frame input;
  std::shared_ptr<const PreparedQualityMask> prepared_quality;
  int64_t preparation_start_steady_ns{0}, preparation_end_steady_ns{0};
  int64_t compute_enqueue_steady_ns{0}; // Belly only: sampled immediately before immutable fanout/replace.
  detail::ThreadSchedulingDelta preparation_scheduling;
  detail::ComputeThreadState preparation_thread_state;
};
using Work=std::shared_ptr<const PreparedFrame>;
using Evidence=sunhorizon_navigation_interfaces::msg::SpatialCoverageEvidence;
template<class T>using Worker=cam::LatestFrameWorker<T>;
namespace {
int64_t stamp(const builtin_interfaces::msg::Time& s){return int64_t(s.sec)*1000000000LL+s.nanosec;}
}
struct PerceptionRuntime::Impl {
  PerceptionRuntime& node;
  RuntimeConfig config;
  bool fresh(const cam::CameraFrame& f,int64_t ros,int64_t steady) const {
    return sunhorizon_rgbd_adapter::region_times_fresh(stamp(f.source_stamp),
      f.first_sdk_callback_ros_ns,f.first_sdk_callback_steady_ns,ros,steady,
      double(config.camera_maximum_age_ns)/1e9);
  }
  std::shared_ptr<JointKinematics> joints;
  dm::Model model;
  RgbdHealth health;
  const detail::ChestCandidateHistoryPolicy chest_history{config.camera_maximum_age_ns,
    health.config().target_rate_hz,health.config().max_pair_delta_sec};
  std::array<std::unique_ptr<PerceptionEngine>,2> engines,raw_engines,preparation_engines;
  using RegionOutput=std::shared_ptr<const RegionDiagnosticOutput>;
  std::array<std::unique_ptr<sunhorizon_rgbd_adapter::RegionDepthProducer>,2> regions;
  std::array<std::unique_ptr<Worker<RegionOutput>>,2> region_workers;
  std::array<rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr,2> region_pub;
  std::array<Frame,2> region_frame;
  std::array<RegionOutput,2> last_region;
  RegionOutputSink region_sink;
  SpatialOutputSink spatial_sink;HealthOutputSink health_sink;
  std::mutex health_delivery_mutex; // Snapshot + direct delivery only; never DDS.
  std::shared_ptr<tf2_ros::Buffer> global;
  std::unique_ptr<const detail::AttendedGeometry> attended_geometry;
  ground_relative::GroundReferenceOwner::ConstSharedPtr ground_owner;
  RegistrationGroundGeometryProvider ground_geometry;
  bool ground_reference_deferred{false};
  rclcpp::Node::SharedPtr tf_node;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::array<std::unique_ptr<Worker<Frame>>,2> admissions;
  std::unique_ptr<Worker<Work>> compute;
  using ChestTask=std::packaged_task<Output()>;
  std::unique_ptr<Worker<ChestTask>> chest_compute;
  std::unique_ptr<Worker<Work>> body_only_chest_compute;
  std::array<std::unique_ptr<Worker<Output>>,2> clouds;
  std::array<std::unique_ptr<Worker<Work>>,2> raw_clouds;
  std::unique_ptr<Worker<Evidence>> evidence_worker;
  std::unique_ptr<Worker<diagnostic_msgs::msg::DiagnosticArray>> diagnostics_worker;
  const std::string ground_snapshot_directory=detail::ground_snapshot_directory_from_environment();
  const bool body_replay_enabled=[] {
    const auto* flag=std::getenv("SUNHORIZON_BODY_REPLAY_SNAPSHOT");
    return flag && std::strcmp(flag,"1")==0;
  }();
  std::array<std::atomic<bool>,2> body_replay_selected{{false,false}};
  std::array<Output,2> pending_body_replay;
  std::array<std::atomic<bool>,2> ground_snapshot_selected{{false,false}};
  std::array<ground_relative::ClassifiedObstacleFrame::ConstSharedPtr,2> pending_ground_snapshot;
  std::unique_ptr<Worker<sunhorizon_navigation_interfaces::msg::RgbdStatus>> health_worker;
  std::array<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr,2> cloud_pub,raw_cloud_pub;
  rclcpp::Publisher<Evidence>::SharedPtr evidence_pub;
  rclcpp::Publisher<sunhorizon_navigation_interfaces::msg::RgbdStatus>::SharedPtr health_pub;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub;
  rclcpp::TimerBase::SharedPtr timer;
  std::atomic<bool> terminal{false},stopping{false},direct_tf_configured{false};
  std::atomic<bool> joint_ingress_failure_logged{false};
  std::mutex state_mutex,wait_mutex,stop_mutex;
  std::condition_variable wake;
  std::string fault_reason;
  std::deque<Work> chest;
  detail::BodyPairEvictions chest_evictions; // Protected by existing state_mutex; diagnostic only.
  detail::SpatialEvidenceDelivery<Evidence> spatial_delivery;
  uint64_t last_chest_sequence{0};
  bool handoff_failure_logged{false}; // Sole compute owner, failure path only.
  bool publish_body_clouds{true},publish_raw_projection_clouds{true};
  std::atomic<bool> qualified{false}; // Belly qualification; each role freezes one original read.
  int engine_thread_nice{0};
  bool body_geometry_parallel_enabled{false};
  std::string body_geometry_compute_backend{"cpu"};
  std::array<std::string,2> body_geometry_role_backends;
  std::vector<int64_t> engine_thread_cpu_set;
  // Each slot is touched only by that role's corresponding sole owner.
  std::array<std::optional<detail::ComputeThreadState>,2> body_threads,preparation_threads,geometry_helper_threads;
  SpatialStartupGate spatial_startup;
  std::atomic<bool> collision_height_bound{false};
  sunhorizon_rgbd_adapter::RegionClockGuard clock_guard; // Sole timer owner.
  PerceptionOutputSink output_sink;
  Impl(PerceptionRuntime& owner,bool defer_ground,bool external_tf_listener):node(owner),config(load_runtime_config(node)),
    joints(std::make_shared<JointKinematics>(config.joint_kinematics)),model(config.model_specs),health(node,"rgbd.") {
    try {
    if(!config.enabled || node.get_parameter("use_sim_time").as_bool())
      throw std::invalid_argument("perception_requires_enabled_realtime_contract");
    rcl_interfaces::msg::ParameterDescriptor thread_parameter;thread_parameter.read_only=true;
    const auto ground_file=node.declare_parameter<std::string>("ground_reference_file","",thread_parameter);
    const auto ground_sha=node.declare_parameter<std::string>("ground_reference_sha256","",thread_parameter);
    ground_reference_deferred=defer_ground;
    if(!defer_ground)ground_owner=ground_relative::GroundReferenceOwner::load(ground_file,ground_sha,config.clock_boot_id);
    const auto nice=node.declare_parameter<int64_t>("engine_thread_nice",0,thread_parameter);
    detail::validate_compute_thread_nice(nice);engine_thread_nice=static_cast<int>(nice);
    engine_thread_cpu_set=node.declare_parameter<std::vector<int64_t>>("engine_thread_cpu_set",{},thread_parameter);
    detail::validate_compute_thread_cpu_set(engine_thread_cpu_set);
    body_geometry_compute_backend=node.declare_parameter<std::string>("body_geometry_compute_backend","cpu",thread_parameter);
    for(size_t i=0;i<2;++i) {
      const std::string role=i==0?"belly":"chest";
      body_geometry_role_backends[i]=detail::resolve_body_geometry_backend(body_geometry_compute_backend,
        node.declare_parameter<std::string>(role+"_body_geometry_compute_backend","inherit",thread_parameter));
    }
    body_geometry_parallel_enabled=node.declare_parameter("body_geometry_parallel_enabled",false,thread_parameter);
    if(body_geometry_parallel_enabled&&engine_thread_cpu_set!=std::vector<int64_t>{8,9,10,11})
      throw std::invalid_argument("body_geometry_parallel_requires_explicit_8_9_10_11_pool");
    global=std::make_shared<tf2_ros::Buffer>(node.get_clock(),tf2::durationFromSec(2.));
    if(external_tf_listener) {
      tf_node=std::make_shared<rclcpp::Node>("fusion_global_tf",rclcpp::NodeOptions().use_global_arguments(false)
        .enable_rosout(false).start_parameter_services(false).start_parameter_event_publisher(false));
      tf_listener=std::make_unique<tf2_ros::TransformListener>(*global,tf_node,true);
    }
    for(size_t i=0;i<2;++i){
      engines[i]=std::make_unique<PerceptionEngine>(config.sources[i].engine);
      if(detail::body_geometry_cpu_parallel(body_geometry_role_backends[i],body_geometry_parallel_enabled))engines[i]->enable_parallel_body_geometry([this,i]{check_geometry_helper_thread(i);});
      if(body_geometry_role_backends[i]=="cuda_arm11")engines[i]->enable_cuda_body_geometry();
      raw_engines[i]=std::make_unique<PerceptionEngine>(config.sources[i].engine);
      preparation_engines[i]=std::make_unique<PerceptionEngine>(config.sources[i].engine);
    }
    publish_body_clouds=node.declare_parameter("publish_body_clouds",true);
    publish_raw_projection_clouds=node.declare_parameter("publish_raw_projection_clouds",true);
    if(publish_raw_projection_clouds){
      raw_cloud_pub[0]=node.create_publisher<sensor_msgs::msg::PointCloud2>("/sunhorizon/rgbd/belly/points",rclcpp::SensorDataQoS().keep_last(1));
      raw_cloud_pub[1]=node.create_publisher<sensor_msgs::msg::PointCloud2>("/sunhorizon/rgbd/points",rclcpp::SensorDataQoS().keep_last(1));
    }
    if(publish_body_clouds){
    cloud_pub[0]=node.create_publisher<sensor_msgs::msg::PointCloud2>("/sunhorizon/navigation/belly/points",rclcpp::SensorDataQoS().keep_last(1));
    cloud_pub[1]=node.create_publisher<sensor_msgs::msg::PointCloud2>("/sunhorizon/navigation/chest/points",rclcpp::SensorDataQoS().keep_last(1));
    }
    evidence_pub=node.create_publisher<Evidence>("/sunhorizon/obstacle_coverage/spatial_evidence_v1",rclcpp::QoS(1).reliable());
    health_pub=node.create_publisher<sunhorizon_navigation_interfaces::msg::RgbdStatus>("/sunhorizon/rgbd/status",rclcpp::QoS(1).reliable());
    diagnostics_pub=node.create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/sunhorizon/perception/timing",rclcpp::QoS(32).best_effort());
    auto allowed=[this]{return !terminal.load()&&!stopping.load();};
    auto failure=[this](std::exception_ptr e){try{if(e)std::rethrow_exception(e);}catch(const std::exception& x){fault(x.what());}catch(...){fault("unknown_worker_failure");}};
    for(size_t i=0;i<2;++i)if(config.sources[i].region){
      const auto& options=*config.sources[i].region;
      region_pub[i]=node.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        options.diagnostics_topic,rclcpp::QoS(1).reliable().durability_volatile());
      region_workers[i]=std::make_unique<Worker<RegionOutput>>([this,i](auto& output){
        // Publish the immutable original report. Wire source/receipt deadlines
        // are unchanged; the original RegionSupportTracker expires queued data.
        region_pub[i]->publish(*output->diagnostics);
      },allowed,failure);
      regions[i]=std::make_unique<sunhorizon_rgbd_adapter::RegionDepthProducer>(options,
        node.get_clock(),config.sources[i].region_producer_node_name,[this,i](const auto& report){
          auto output=std::make_shared<RegionDiagnosticOutput>();
          output->input=region_frame[i];
          output->diagnostics=std::make_shared<const diagnostic_msgs::msg::DiagnosticArray>(report);
          last_region[i]=output;
          if(region_sink)region_sink(output); // Bounded immutable ownership transfer only.
          region_workers[i]->replace(std::make_unique<RegionOutput>(std::move(output)));
        },[this](const std::string& target,const std::string& source,int64_t time,double wait){
          return global->lookupTransform(target,source,rclcpp::Time(time,node.get_clock()->get_clock_type()),
            rclcpp::Duration::from_seconds(wait));
        });
    }
    health_worker=std::make_unique<Worker<sunhorizon_navigation_interfaces::msg::RgbdStatus>>(
      [this](auto& status){
        // Queueing/publication cannot renew measurement ages. Re-snapshot at
        // egress so a delayed healthy status cannot hide a stalled computation.
        {std::lock_guard<std::mutex> lock(health_delivery_mutex);
          status=health.snapshot(node.now().nanoseconds(),dm::steady_ns());
          if(health_sink)health_sink(std::make_shared<const sunhorizon_navigation_interfaces::msg::RgbdStatus>(status));
        }
        health_pub->publish(status);
      },[this]{return !stopping.load();},failure);
    diagnostics_worker=std::make_unique<Worker<diagnostic_msgs::msg::DiagnosticArray>>(
      [this](auto& d){
        diagnostics_pub->publish(d);
        // This existing observer owns all serialization and disk I/O. Capture
        // failure is diagnostic only and never changes sensor/ground health.
        for(std::size_t role=0;role<pending_body_replay.size();++role) {
          const auto output=std::atomic_exchange(&pending_body_replay[role],Output{});
          if(output)try {
            const auto path=detail::write_body_replay_snapshot(ground_snapshot_directory,*output,config);
            RCLCPP_INFO(node.get_logger(),"BODY replay snapshot: %s",path.c_str());
          }catch(const std::exception& error) {
            RCLCPP_WARN(node.get_logger(),"BODY replay snapshot failed: %s",error.what());
          }catch(...) {RCLCPP_WARN(node.get_logger(),"BODY replay snapshot failed");}
        }
        for(std::size_t role=0;role<pending_ground_snapshot.size();++role) {
          const auto snapshot=std::atomic_exchange(&pending_ground_snapshot[role],
            ground_relative::ClassifiedObstacleFrame::ConstSharedPtr{});
          if(snapshot)try {
            const auto path=detail::write_ground_classification_snapshot(
              ground_snapshot_directory,role==0?"belly":"chest",*snapshot,ground_owner.get());
            RCLCPP_INFO(node.get_logger(),"ground classification snapshot: %s",path.c_str());
          }catch(const std::exception& error) {
            RCLCPP_WARN(node.get_logger(),"ground classification snapshot failed: %s",error.what());
          }catch(...) {
            RCLCPP_WARN(node.get_logger(),"ground classification snapshot failed");
          }
        }
      },[this]{return !stopping.load();},failure);
    evidence_worker=std::make_unique<Worker<Evidence>>([this](auto& e){publish_evidence(e);},[this]{return !stopping.load();},failure);
    for(size_t i=0;i<2;++i)clouds[i]=std::make_unique<Worker<Output>>([this,i](auto& o){
      const auto ros=node.now().nanoseconds(),steady=dm::steady_ns();
      if(!fresh(*o->input,ros,steady)||terminal.load())return;
      if(!publish_body_clouds)return;
      cloud_pub[i]->publish(o->frame.body_cloud);
      if(!output_sink && !terminal.load())detail::complete_health_and_notify(
        [this,&o]{return health.complete(*o->input,stamp(o->frame.body_cloud.header.stamp),node.now().nanoseconds(),dm::steady_ns());},
        [this]{notify_health_completion();});
    },allowed,failure);
    for(size_t i=0;i<2;++i)raw_clouds[i]=std::make_unique<Worker<Work>>([this,i](Work& work){
      if(terminal.load()||stopping.load())return;
      auto input=source_input(work->input,i);input.prepared_quality=work->prepared_quality;
      auto raw=std::make_shared<PerceptionOutput>();raw->input=work->input;
      raw->preparation_start_steady_ns=work->preparation_start_steady_ns;
      raw->preparation_end_steady_ns=work->preparation_end_steady_ns;
      raw->preparation_scheduling=work->preparation_scheduling;
      raw->preparation_thread_state=work->preparation_thread_state;
      raw->compute_start_steady_ns=dm::steady_ns();
      auto& scheduling=detail::current_thread_scheduling();const auto before=scheduling.sample();
      raw->frame=raw_engines[i]->project_raw(input);
      raw->engine_scheduling=detail::scheduling_delta(before,scheduling.sample());
      raw->compute_end_steady_ns=dm::steady_ns();
      // This terminal owner never waits for FK/body. Its original pair/depth
      // sources and SDK receipt independently retain the original raw deadline.
      if(raw_fresh(raw,node.now().nanoseconds(),dm::steady_ns()) && !terminal.load() && !stopping.load()){
        if(output_sink)output_sink(raw);
        if(publish_raw_projection_clouds && !terminal.load() && !stopping.load() &&
           raw_fresh(raw,node.now().nanoseconds(),dm::steady_ns()))
          raw_cloud_pub[i]->publish(*raw->frame.raw_projection_cloud);
        timing(raw,{});
      }
    },allowed,failure);
    if(config.include_chest)
      chest_compute=std::make_unique<Worker<ChestTask>>([](ChestTask& task){task();},allowed,failure);
    else body_only_chest_compute=std::make_unique<Worker<Work>>([this](Work& work){
      const auto& frame=work->input;
      if(!fresh(*frame,node.now().nanoseconds(),dm::steady_ns()))return;
      auto query=config.query;query.query_stamp_ns=stamp(frame->source_stamp);
      if(query.source_contracts.size()==1)query.source_contracts.push_back({"chest",config.sources[1].epoch,false});
      const auto output=decode_and_deliver(work,1,query,qualified.load());
      if(output)timing(output,{}); // Its own source clock and role health; no spatial vote.
    },allowed,failure);
    compute=std::make_unique<Worker<Work>>([this](Work& f){process(f);},allowed,failure);
    for(size_t i=0;i<2;++i)admissions[i]=std::make_unique<Worker<Frame>>([this,i](Frame& f){
      if(!f || !f->source_validated || !f->depth || !f->depth_info ||
         f->depth->header.frame_id!=config.sources[i].frame ||
         f->depth_scale_m!=config.sources[i].engine.depth_scale_m ||
         !runtime_camera_info_matches(config.sources[i],*f->depth_info))
        throw std::runtime_error("sdk_frame_contract_mismatch");
      const auto admission=health.admit(*f,node.now().nanoseconds(),dm::steady_ns());
      if(!admission.accepted||!admission.computation_allowed)return;
      const auto thread_state=check_compute_thread(i,true);
      // Mandatory preparation only: no XYZ, raw sink or DDS on this owner.
      auto prepared=std::make_shared<PreparedFrame>();prepared->input=f;
      prepared->preparation_thread_state=thread_state;
      prepared->preparation_start_steady_ns=dm::steady_ns();
      auto& scheduling=detail::current_thread_scheduling();const auto before=scheduling.sample();
      prepared->prepared_quality=preparation_engines[i]->prepare(source_input(f,i));
      prepared->preparation_scheduling=detail::scheduling_delta(before,scheduling.sample());
      prepared->preparation_end_steady_ns=dm::steady_ns();
      if(i==0)prepared->compute_enqueue_steady_ns=dm::steady_ns();
      Work work=std::move(prepared); // Immutable fanout; each engine owns its scratch.
      detail::dispatch_prepared_body(i,config.include_chest,work,
        [this](const Work& value){compute->replace(std::make_unique<Work>(value));},
        [this](const Work& value){body_only_chest_compute->replace(std::make_unique<Work>(value));},
        [this](const Work& value){
          detail::ChestCandidateRetirements<Work> retired;
          const auto history_now_ros_ns=node.now().nanoseconds();
          {std::lock_guard<std::mutex> lock(state_mutex);
            detail::retain_chest_candidate(chest,value,chest_history,history_now_ros_ns,retired,
              [](const Work& candidate){return detail::BodyPairIdentity{stamp(candidate->input->source_stamp),candidate->input->sequence};},
              [this](detail::BodyPairIdentity identity){chest_evictions.note(identity);});}
        });
      if(config.sources[i].engine.raw_projection_enabled)
        raw_clouds[i]->replace(std::make_unique<Work>(std::move(work)));
    },allowed,failure);
    timer=node.create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
    } catch (...) {stop();throw;}
  }
  ~Impl(){stop();}
  bool fault(const std::string& reason){
    // SDK/ingress threads never call a ROS publisher or wait for compute here.
    {std::lock_guard<std::mutex> lock(state_mutex);if(terminal.load())return false;
      fault_reason=reason.substr(0,512);terminal.store(true);}
    health.fault(reason);
    wake.notify_all();
    return true;
  }
  void stop(){
    std::lock_guard<std::mutex> lifecycle(stop_mutex);
    if(stopping.exchange(true))return;
    if(timer){try{timer->cancel();}catch(...){}}
    wake.notify_all();
    spatial_delivery.close(); // No direct delivery may start after this drain.
    // Cancel only output admission. GPU buffers remain owned until each engine
    // is destroyed after the existing compute-owner joins below.
    for(auto& engine:engines)if(engine)engine->request_stop_body_geometry();
    for(auto& w:admissions)if(w)w->request_stop();
    if(compute)compute->request_stop();
    if(chest_compute)chest_compute->request_stop();
    if(body_only_chest_compute)body_only_chest_compute->request_stop();
    for(auto& w:clouds)if(w)w->request_stop();
    for(auto& w:raw_clouds)if(w)w->request_stop();
    if(evidence_worker)evidence_worker->request_stop();
    if(diagnostics_worker)diagnostics_worker->request_stop();
    if(health_worker)health_worker->request_stop();
    for(auto& w:region_workers)if(w)w->request_stop();
    for(auto& w:admissions)if(w)w->join();
    if(compute)compute->join();
    if(chest_compute)chest_compute->join();
    if(body_only_chest_compute)body_only_chest_compute->join();
    for(auto& w:clouds)if(w)w->join();
    for(auto& w:raw_clouds)if(w)w->join();
    if(evidence_worker)evidence_worker->join();
    if(diagnostics_worker)diagnostics_worker->join();
    if(health_worker)health_worker->join();
    for(auto& w:region_workers)if(w)w->join();
  }
  JointKinematicsLookup wait_fk(const Frame& f,bool was_qualified){
    for(;;){
      auto result=joints->lookup(stamp(f->source_stamp),[this]{
        return JointKinematics::ClockSample{node.now().nanoseconds(),dm::steady_ns()};
      });
      const auto fail=[&](const std::string& reason){
        const auto message=reason+result.failure_detail(f->role,f->sequence,
          f->first_sdk_callback_ros_ns,f->first_sdk_callback_steady_ns);
        // Latch the original failure before any optional logging/rosout can
        // block. Only the actual first-latch winner may log its snapshot; a
        // woken peer reporting fusion_stopped_during_fk must not replace it.
        const bool first_fault=fault(message);
        if(first_fault && !joint_ingress_failure_logged.exchange(true))
          RCLCPP_ERROR(node.get_logger(),"JOINT_INGRESS_FIRST_FAILURE %s",
            result.ingress_failure_detail(f->role,f->sequence).c_str());
        throw std::runtime_error(message);
      };
      const auto disposition=joint_frame_disposition(result.status,
        fresh(*f,node.now().nanoseconds(),dm::steady_ns()),was_qualified);
      if(disposition==JointFrameDisposition::InternalFault)
        fail(result.status==JointKinematicsStatus::Fault ? result.reason :
          "qualified_joint_history_lost:"+result.reason);
      if(terminal.load()||stopping.load())fail("fusion_stopped_during_fk");
      if(disposition==JointFrameDisposition::Ready)return result;
      if(disposition==JointFrameDisposition::Unavailable){
        if(result.status==JointKinematicsStatus::Ready || result.status==JointKinematicsStatus::FuturePending ||
           result.status==JointKinematicsStatus::Missing){
          result.status=JointKinematicsStatus::QueryExpired;
          result.reason="source_expired_waiting_for_joint_history";
        }
        result.transforms.clear();
        return result;
      }
      // Frozen anchor waits only for its own bracket. New pending SDK frames
      // replace each other; neither their timestamps nor receipts renew it.
      std::unique_lock<std::mutex> lock(wait_mutex);wake.wait_for(lock,std::chrono::milliseconds(1));
    }
  }
  FrameInput source_input(const Frame& f,size_t index)const{
    FrameInput input;input.image=f->depth;input.info=f->depth_info;input.measurement_quality=f->measurement_quality;
    input.sdk_depth_device_us=f->depth_time.device_us;input.sdk_depth_global_us=f->depth_time.global_us;
    input.sdk_depth_frame_index=f->depth_frame_index;input.sdk_depth_scale_m=f->sdk_depth_scale_m;
    input.identity={config.sources[index].name,config.sources[index].epoch,config.sources[index].calibration,
      f->sequence,stamp(f->source_stamp),f->first_sdk_callback_ros_ns,f->first_sdk_callback_steady_ns,
      config.camera_maximum_age_ns,index==0};
    if(index==1 && f->color)input.raw_projection_header=f->color->header;
    input.calibration=config.sources[index].attestations;
    return input;
  }
  bool raw_fresh(const Output& output,int64_t ros,int64_t steady)const{
    const auto& r=output->frame;const auto& f=*output->input;
    if(!r.raw_projection_complete || !r.raw_projection_cloud || r.raw_projection_expiry_budget_ns<=0)return false;
    const auto age=double(r.raw_projection_expiry_budget_ns)/1e9;
    return sunhorizon_rgbd_adapter::region_times_fresh(r.raw_projection_source_ns,
      f.first_sdk_callback_ros_ns,f.first_sdk_callback_steady_ns,ros,steady,age) &&
      sunhorizon_rgbd_adapter::region_times_fresh(r.raw_projection_raw_source_ns,
      f.first_sdk_callback_ros_ns,f.first_sdk_callback_steady_ns,ros,steady,age);
  }
  detail::ComputeThreadState check_compute_thread(size_t index,bool preparation){
    if(index>=body_threads.size())throw std::runtime_error("compute_thread_control:invalid_role_index");
    auto& owner=(preparation ? preparation_threads : body_threads)[index];
    detail::ComputeThreadSyscalls calls;
    try{
      const auto* name=preparation ? (index==0 ? "shn_prep_belly" : "shn_prep_chest") :
                                    (index==0 ? "shn_body_belly" : "shn_body_chest");
      const auto owner_cpus=detail::compute_owner_cpu_set(name,engine_thread_cpu_set);
      if(!owner)owner=detail::configure_self_compute_thread(calls,engine_thread_nice,name,engine_thread_cpu_set);
      // Read on the executing owner every frame; each stage/role retains its
      // own TID and may not authorize work using another owner's state.
      const auto state=detail::read_self_compute_thread(calls,!owner_cpus.empty(),false);
      detail::require_compute_thread_identity(state,owner->tid);
      return state;
    }catch(const std::exception& error){
      throw std::runtime_error(std::string("compute_thread_control:")+
        (preparation ? "prepare:" : "body:")+config.sources[index].name+":"+error.what());
    }
  }
  void check_geometry_helper_thread(size_t index) {
    if(index>=geometry_helper_threads.size())throw std::runtime_error("compute_thread_control:invalid_geometry_role");
    auto& owner=geometry_helper_threads[index];detail::ComputeThreadSyscalls calls;
    const auto* name=index==0?"shn_geom_belly":"shn_geom_chest";
    try {
      if(!owner)owner=detail::configure_self_compute_thread(calls,engine_thread_nice,name,engine_thread_cpu_set);
      const auto actual=detail::read_self_compute_thread(calls,true,false);
      detail::require_compute_thread_identity(actual,owner->tid);
      // Later scheduling drift is observed, never silently repaired. Preserve
      // the original owner TID while recording this frame's actual state.
      owner=actual;
    }catch(const std::exception& error){throw std::runtime_error(std::string("compute_thread_control:geometry:")+config.sources[index].name+":"+error.what());}
  }
  Output decode(const Work& work,size_t index,const sv::Query& query,bool was_qualified){
    const auto thread_state=check_compute_thread(index,false);
    const auto& f=work->input;
    const auto started=dm::steady_ns();
    const auto fk=wait_fk(f,was_qualified);const auto fk_end=dm::steady_ns();
    if(fk.status!=JointKinematicsStatus::Ready){
      RCLCPP_WARN_THROTTLE(node.get_logger(),*node.get_clock(),1000,"JOINT_FRAME_UNAVAILABLE %s%s",
        fk.reason.c_str(),fk.failure_detail(f->role,f->sequence,
          f->first_sdk_callback_ros_ns,f->first_sdk_callback_steady_ns).c_str());
      return {};
    }
    const auto deadline=fk_end+20000000LL;
    detail::TfFailureDetail tf_detail;
    tf_detail.role=f->role.c_str();tf_detail.sequence=f->sequence;
    tf_detail.source_ns=stamp(f->source_stamp);tf_detail.query_ns=query.query_stamp_ns;
    tf_detail.fk_end_ns=fk_end;tf_detail.deadline_ns=deadline;
    tf_detail.begin_ns=dm::steady_ns();tf_detail.cpu_begin_ns=detail::diagnostic_thread_cpu_ns();
    auto& tf_scheduling=detail::current_thread_scheduling();
    const auto tf_before=tf_scheduling.sample();
    // Sampling consumes the same original FK-end-based 20 ms budget. Only an
    // exception samples the end or formats a diagnostic; no deadline is reset.
    const auto geometry=[&]{
      try {
        const auto* fixed=attended_geometry ? attended_geometry->select(
          config.joint_kinematics.tf_mode,index,stamp(f->source_stamp),query.query_stamp_ns) : nullptr;
        const auto camera=[&] {
          if (fixed) {
            (void)dm::remaining(deadline);
            return fixed->optical_from_base;
          }
          return dm::rigid(global->lookupTransform(config.sources[index].frame,"base_link",
            rclcpp::Time(stamp(f->source_stamp),node.get_clock()->get_clock_type()),dm::remaining(deadline)).transform);
        }();
        tf_detail.camera_end_ns=dm::steady_ns();
        std::vector<dg::RigidTransform> base_from_links;
        for(const auto& transform:fk.transforms)base_from_links.push_back(dm::rigid(transform.transform));
        auto links=model.observations(camera,base_from_links,stamp(f->source_stamp));
        tf_detail.observations_end_ns=dm::steady_ns();
        tf_detail.spatial_begin_ns=dm::steady_ns();
        const auto rigid=[&] {
          if (fixed) {
            (void)dm::remaining(deadline);
            return fixed->common_from_optical;
          }
          const auto tf=sunhorizon_rgbd_adapter::spatial_source_transform(*global,config.common_frame,
            config.sources[index].frame,stamp(f->source_stamp),query.query_stamp_ns,node.get_clock()->get_clock_type(),deadline);
          return dm::rigid(tf.transform);
        }();
        return std::make_pair(std::move(links),rigid);
      } catch(const tf2::TransformException& error) {
        tf_detail.failure_ns=dm::steady_ns();tf_detail.cpu_end_ns=detail::diagnostic_thread_cpu_ns();
        tf_detail.current_ros_ns=node.now().nanoseconds();
        tf_detail.scheduling=detail::scheduling_delta(tf_before,tf_scheduling.sample());
        detail::rethrow_tf_failure(error,tf_detail);
      }
    }();
    const auto& links=geometry.first;const auto& rigid=geometry.second;
    auto input=source_input(f,index);input.prepared_quality=work->prepared_quality;
    input.capture_body_model_diagnostic=!ground_snapshot_directory.empty();
    input.capture_body_replay_diagnostic=body_replay_enabled && !ground_snapshot_directory.empty() &&
      !body_replay_selected[index].load();
    input.optical_to_common.rotation=rigid.rotation;
    input.optical_to_common.translation={rigid.translation.x,rigid.translation.y,rigid.translation.z};
    input.optical_to_common.source_frame=config.sources[index].frame;
    input.optical_to_common.common_frame=config.common_frame;
    input.optical_to_common.source_stamp_ns=stamp(f->source_stamp);
    input.optical_to_common.query_stamp_ns=query.query_stamp_ns;input.calibration=config.sources[index].attestations;
    if(ground_owner){
      input.ground_owner=ground_owner;
      ground_relative::SourceTimeTransform transform;
      transform.source_frame=config.sources[index].frame;transform.common_frame=ground_owner->reference_frame();
      transform.source_ns=transform.common_ns=stamp(f->source_stamp);
      if(ground_owner->registration_map_asset()) {
        input.ground_transform_diagnostic.reason="registration_geometry_provider_missing";
        if(ground_geometry) {
          ground_relative::SourceIdentity identity{input.identity.source_id,input.identity.epoch,
            input.identity.calibration_id,config.sources[index].frame,ground_owner->reference_frame(),
            input.identity.sequence,input.identity.source_stamp_ros_ns,input.identity.receipt_ros_ns,
            input.identity.receipt_steady_ns,input.identity.expiry_budget_ns};
          input.acquire_ground_geometry=[this,identity,f]() {
            return detail::wait_for_registration_ground_geometry(identity,
              [this](const auto& original){return ground_geometry(original,{original.source_ns},false);},
              [this,&f]{return fresh(*f,node.now().nanoseconds(),dm::steady_ns());},
              [this]{return terminal.load()||stopping.load();},
              [this]{std::unique_lock<std::mutex> lock(wait_mutex);wake.wait_for(lock,std::chrono::milliseconds(1));});
          };
        }
      }else try{
        // Uses the original FK deadline; this independent source-time lookup
        // neither renews it nor reuses coverage's potentially moving frame.
        const auto value=dm::rigid(global->lookupTransform(transform.common_frame,transform.source_frame,
          rclcpp::Time(transform.source_ns,node.get_clock()->get_clock_type()),dm::remaining(deadline)).transform);
        transform.rotation=value.rotation;transform.translation={value.translation.x,value.translation.y,value.translation.z};
        transform.qualified=true;
      }catch(const tf2::TransformException&){
        // Missing reference geometry cannot contribute a single clearing point.
      }
      input.ground_source_transform=std::move(transform);
    }
    auto out=std::make_shared<PerceptionOutput>();out->input=f;
    out->engine_thread_state=thread_state;
    out->compute_start_steady_ns=started;out->fk_wait_ns=fk_end-started;out->joints=fk.evidence;
    auto& scheduling=detail::current_thread_scheduling();
    const auto scheduling_before=scheduling.sample();
    out->frame=engines[index]->process(input,links,query);
    if(out->frame.ground_query_cancelled)return {};
    if(body_geometry_parallel_enabled&&geometry_helper_threads[index])
      out->geometry_helper_thread_state=*geometry_helper_threads[index];
    if(ground_owner && !ground_owner->supported_segmentation() && ground_owner->clearance_grid() &&
        out->frame.classified_obstacles) {
      const auto begin=dm::steady_ns(),cpu=detail::diagnostic_thread_cpu_ns();
      out->frame.near_ground_clearance=near_ground_clearance::evaluate(out->frame.decoded.source(),
        out->frame.classified_obstacles,ground_owner->config(),*ground_owner->clearance_grid(),
        node.now().nanoseconds(),begin);
      out->frame.timings.near_ground_clearance={double(detail::diagnostic_thread_cpu_ns()-cpu)/1e6,
        double(dm::steady_ns()-begin)/1e6};
    }
    out->engine_scheduling=detail::scheduling_delta(scheduling_before,scheduling.sample());
    out->preparation_start_steady_ns=work->preparation_start_steady_ns;
    out->preparation_end_steady_ns=work->preparation_end_steady_ns;
    out->preparation_scheduling=work->preparation_scheduling;
    out->preparation_thread_state=work->preparation_thread_state;
    // Body results never carry a terminal raw cloud or completion credit. The
    // independent raw owner is the sole source of each raw projection delivery.
    if(regions[index] && out->frame.obstacle_input_health.available){
      region_frame[index]=f;last_region[index].reset();
      const auto& raster=out->frame.decoded.depth();
      regions[index]->prepared(f->depth,f->depth_info,raster.data(),raster.size(),f->sequence,
        f->first_sdk_callback_ros_ns,f->first_sdk_callback_steady_ns);
      out->region=last_region[index];region_frame[index].reset();
    }
    out->compute_end_steady_ns=dm::steady_ns();
    auto& body=out->frame.body_filter;
    if(body.complete && (!sunhorizon_rgbd_adapter::exclusion_fraction_allowed(body.removed_count,body.input_count,
        config.sources[index].max_removed_fraction) || body.retained_count==0)) {
      body.complete=false;body.reason="removed_fraction_or_empty_cloud_gate";out->frame.body_cloud.data.clear();
    }
    if(!body.complete)health.reject_computation(*f,body.reason);
    if(!out->frame.obstacle_input_health.available) {
      // Do not turn an expired registration owner into fresh all-Uncertain
      // obstacle marks. Keep the diagnostic frame and all existing map state.
      health.reject_computation(*f,out->frame.obstacle_input_health.reason,true);
      std::lock_guard<std::mutex> lock(health_delivery_mutex);
      const auto status=health.snapshot(node.now().nanoseconds(),dm::steady_ns());
      if(health_sink)health_sink(std::make_shared<const sunhorizon_navigation_interfaces::msg::RgbdStatus>(status));
    }
    return out;
  }
  Output decode_and_deliver(const Work& work,size_t index,const sv::Query& query,bool was_qualified){
    Output out;
    try{out=decode(work,index,query,was_qualified);}
    catch(const tf2::TransformException& error){
      if(detail::transient_tf_unavailable(error)){
        RCLCPP_WARN_THROTTLE(node.get_logger(),*node.get_clock(),1000,"FRAME_TF_UNAVAILABLE %s",error.what());
        return {};
      }
      fault(error.what());
      throw;
    }catch(const std::exception& error){fault(error.what());throw;}
    catch(...){fault("unknown_worker_failure");throw;}
    // A dropped query cannot renew completion or invalidate an earlier accepted
    // result before its original expiry. The independent raw lane is unchanged.
    if(!out)return {};
    // decode has finished every mutation, including fraction/empty rejection.
    // Coverage keeps this same immutable result until the original anchor join.
    detail::complete_role(out->frame.body_filter.complete && out->frame.obstacle_input_health.available &&
      fresh(*out->input,node.now().nanoseconds(),dm::steady_ns()),
      [this]{return !terminal.load()&&!stopping.load();},
      [this,&out]{if(output_sink)output_sink(out);},
      [this,&out]{if(output_sink)detail::complete_health_and_notify(
        [this,&out]{return health.complete(*out->input,
          stamp(out->frame.body_cloud.header.stamp),node.now().nanoseconds(),dm::steady_ns());},
        [this]{notify_health_completion();});},
      [this,index,&out]{if(publish_body_clouds)clouds[index]->replace(std::make_unique<Output>(out));},
      [this](const std::string& reason){fault(reason);});
    return out;
  }
  void process(const Work& work){
    const auto process_entry_steady_ns=dm::steady_ns();
    const auto& belly=work->input;
    if(!fresh(*belly,node.now().nanoseconds(),dm::steady_ns()))return;
    auto query=config.query;query.query_stamp_ns=stamp(belly->source_stamp);
    // The engine needs the chest source contract even when coverage excludes it.
    if(query.source_contracts.size()==1)query.source_contracts.push_back({"chest",config.sources[1].epoch,false});
    // Only a contributing chest belongs to this frozen spatial anchor.
    // Body-only chest is dispatched by its own admission owner above.
    Work chosen;
    detail::BodyPairWitness pair_witness;
    if(config.include_chest){
      const auto selection_limit=detail::chest_selection_limit(
        config.include_chest,query.query_stamp_ns,node.now().nanoseconds());
      const auto lock_attempt_steady_ns=dm::steady_ns();
      {std::lock_guard<std::mutex> lock(state_mutex);
        const auto lock_acquired_steady_ns=dm::steady_ns();
        for(auto it=chest.rbegin();it!=chest.rend();++it)
          if(stamp((*it)->input->source_stamp)<=selection_limit && (*it)->input->sequence>last_chest_sequence){chosen=*it;break;}
        pair_witness=detail::freeze_body_pair_witness(chest,
          {stamp(belly->source_stamp),belly->sequence},
          {work->compute_enqueue_steady_ns,process_entry_steady_ns,lock_attempt_steady_ns,lock_acquired_steady_ns},
          selection_limit,last_chest_sequence,chosen,chest_evictions,
          [](const Work& value){return detail::BodyPairIdentity{stamp(value->input->source_stamp),value->input->sequence};});
        pair_witness.selection_steady_ns=dm::steady_ns();}
    }
    const bool was_qualified=qualified.load();
    std::future<Output> chest_future;
    const bool chosen_fresh=chosen && fresh(*chosen->input,node.now().nanoseconds(),dm::steady_ns());
    pair_witness.chosen_fresh=chosen_fresh;
    if(config.include_chest && chosen_fresh){
      auto chest_query=query;
      auto task=std::make_unique<ChestTask>([this,chosen,chest_query,was_qualified]{
        return decode_and_deliver(chosen,1,chest_query,was_qualified);
      });
      chest_future=task->get_future();
      chest_compute->replace(std::move(task));
    }
    JoinedFuture<Output> completion(std::move(chest_future));
    std::vector<Output> decoded;
    bool belly_unavailable=false;
    int64_t belly_return_steady_ns=-1;
    auto belly_output=decode_and_deliver(work,0,query,was_qualified);
    belly_return_steady_ns=dm::steady_ns();
    if(belly_output)decoded.push_back(std::move(belly_output));else belly_unavailable=true;
    if(!belly_unavailable && decoded.front()->frame.obstacle_input_health.available)qualified.store(true);
    if(completion.valid()){
      try{
        auto output=completion.get();
        if(output)decoded.push_back(std::move(output));
        last_chest_sequence=chosen->input->sequence;
      }
      catch(const std::future_error&){if(stopping.load()||terminal.load())return;throw;}
    }
    // Even an unavailable belly query must retrieve the chest outcome:
    // packaged_task stores exceptions, so wait alone would hide a chest fault.
    if(belly_unavailable)return;
    if(terminal.load()||stopping.load())return;
    if(std::any_of(decoded.begin(),decoded.end(),[](const auto& output){return !output->frame.obstacle_input_health.available;})) {
      for(const auto& output:decoded)timing(output,{},config.include_chest && output->input==belly?&pair_witness:nullptr);
      return; // No healthy spatial contribution/completion from upstream failure.
    }
    std::vector<const DecodedSource*> views{&decoded[0]->frame.decoded};
    if(config.include_chest && decoded.size()>1)views.push_back(&decoded[1]->frame.decoded);
    auto coverage_query=config.query;coverage_query.query_stamp_ns=query.query_stamp_ns;
    auto evaluation=PerceptionEngine::evaluate(coverage_query,views,node.now().nanoseconds(),dm::steady_ns());
    for(const auto& output:decoded)timing(output,evaluation.timing,config.include_chest && output->input==belly?&pair_witness:nullptr);
    if(terminal.load()||stopping.load())return;
    // Qualification observes the ORIGINAL current health after body completion.
    // Waiting suppresses only spatial wire evidence, never body/raw/region work
    // or health. A missing proof stays Unknown in the independent coverage owner.
    const auto startup=spatial_startup.snapshot();
    if(startup.deferred && startup.owner_ready && !startup.opened){
      // Observe owner readiness BEFORE sampling health, so a racing release
      // cannot qualify with a snapshot taken before actual STARTUP success.
      const auto status=health.snapshot(node.now().nanoseconds(),dm::steady_ns());
      // The acquisition barrier follows the health read as well; a frame
      // arriving while that snapshot was being obtained is not a new proof.
      const auto ros=node.now().nanoseconds(),steady=dm::steady_ns();
      if(spatial_startup.qualify(status.state==status.STATE_HEALTHY,ros,steady))
        RCLCPP_INFO(node.get_logger(),"SPATIAL_FIRST_QUALIFIED ros_ns=%lld steady_ns=%lld accepted_pair_rate_hz=%.3f",
          static_cast<long long>(ros),static_cast<long long>(steady),status.accepted_pair_rate_hz);
    }
    for(const auto* view:views){const auto& id=view->identity();
      if(!spatial_startup.allows(id.source_stamp_ros_ns,id.receipt_ros_ns,id.receipt_steady_ns))return;
    }
    auto wire=evidence(coverage_query,views,evaluation.visibility);
    SpatialHandoffTrace handoff;
    try {
      deliver_evidence(std::move(wire),&handoff);
    } catch(const std::exception& error) {
      fault(error.what());
      log_handoff_failure(decoded,evaluation.timing,handoff,belly_return_steady_ns);
      throw;
    } catch(...) {
      fault("unknown_worker_failure");
      log_handoff_failure(decoded,evaluation.timing,handoff,belly_return_steady_ns);
      throw;
    }
    if(handoff.rejected)log_handoff_failure(decoded,evaluation.timing,handoff,belly_return_steady_ns);
  }
  Evidence evidence(const sv::Query& query,const std::vector<const DecodedSource*>& views,const sv::Result& result){
    Evidence e;e.header.stamp=node.now();e.header.frame_id=config.common_frame;e.schema_version=1;
    e.contract=config.contract;e.query_digest=config.query_digest;e.producer_epoch=config.producer_epoch;e.clock_boot_id=config.clock_boot_id;
    e.anchor_source_ros_ns=views[0]->identity().source_stamp_ros_ns;e.anchor_frame_sequence=views[0]->identity().sequence;
    e.input_generation=e.anchor_frame_sequence;e.evaluation_steady_ns=dm::steady_ns();
    e.total_cells=query.cells.size();e.evaluation_complete=result.evaluation_complete;e.reason=result.reason;
    if(!result.evaluation_complete && result.reason=="required_source_unavailable_or_invalid") {
      bool expired=false,invalid=false;
      for(const auto& source:result.sources)if(source.required_for_evaluation){
        expired|=source.assessment==sv::SourceAssessment::kUnavailableExpired;
        invalid|=source.assessment!=sv::SourceAssessment::kFresh &&
          source.assessment!=sv::SourceAssessment::kUnavailableExpired;
      }
      if(expired&&!invalid)e.reason="source_expired_before_egress";
    }
    for(const auto& c:result.cells){if(c.state==sv::CellState::kFree)++e.free_cells;else if(c.state==sv::CellState::kOccupied)++e.occupied_cells;else ++e.unknown_cells;}
    for(const auto* view:views){const auto s=view->source();sunhorizon_navigation_interfaces::msg::SpatialSourceEvidence w;
      w.source_id=s.source_id=="belly"?0:1;w.producer_epoch=s.epoch;w.frame_id=s.frame_id;w.frame_sequence=s.frame_sequence;
      w.source_ros_ns=s.source_stamp_ros_ns;w.first_receipt_ros_ns=s.local_receipt_ros_ns;w.first_receipt_steady_ns=s.local_receipt_steady_ns;
      w.valid_until_ros_ns=std::min(s.source_stamp_ros_ns+s.expiry_budget_ns,s.local_receipt_ros_ns+s.expiry_budget_ns);
      w.valid_until_steady_ns=s.local_receipt_steady_ns+w.valid_until_ros_ns-s.local_receipt_ros_ns;
      for(const auto& r:result.sources)if(r.source_id==s.source_id)w.geometry_valid=r.assessment==sv::SourceAssessment::kFresh;
      const auto bit=uint8_t(1U<<w.source_id);
      for(const auto& c:result.cells){if(c.free_sources&bit)w.contribution|=1;if(c.occupied_sources&bit)w.contribution|=2;}
      e.sources.push_back(w);
    }
    return e;
  }
  struct SpatialHandoffTrace {
    bool rejected{false};
    int64_t enter_ros_ns{-1},enter_steady_ns{-1},return_ros_ns{-1},return_steady_ns{-1};
  };
  void log_handoff_failure(const std::vector<Output>& decoded,const EngineDuration& fusion,
      const SpatialHandoffTrace& handoff,int64_t belly_return_steady_ns){
    if(handoff_failure_logged || !handoff.rejected || decoded.empty())return;
    handoff_failure_logged=true;
    const auto& belly=*decoded.front();
    const auto chest_end=decoded.size()>1 ? decoded[1]->compute_end_steady_ns : -1;
    // Boundary observations around receive_spatial(), not the tracker's own
    // atomic gate sample. Keep this failed frame even if latest timing moves on.
    RCLCPP_ERROR(node.get_logger(),
      "SPATIAL_HANDOFF_REJECTED anchor_seq=%llu source_ns=%lld body_begin_steady_ns=%lld body_end_steady_ns=%lld "
      "belly_return_steady_ns=%lld engine_wall_ms=%.3f engine_cpu_ms=%.3f engine_runqueue_ns=%lld "
      "engine_schedstat_available=%d engine_schedstat_status=%d chest_end_steady_ns=%lld "
      "fusion_wall_ms=%.3f fusion_cpu_ms=%.3f direct_enter_ros_ns=%lld direct_enter_steady_ns=%lld "
      "direct_return_ros_ns=%lld direct_return_steady_ns=%lld",
      static_cast<unsigned long long>(belly.input->sequence),
      static_cast<long long>(stamp(belly.input->source_stamp)),
      static_cast<long long>(belly.compute_start_steady_ns),static_cast<long long>(belly.compute_end_steady_ns),
      static_cast<long long>(belly_return_steady_ns),belly.frame.timings.total.wall_ms,belly.frame.timings.total.cpu_ms,
      static_cast<long long>(belly.engine_scheduling.runqueue_wait_ns),
      static_cast<int>(belly.engine_scheduling.schedstat_available),static_cast<int>(belly.engine_scheduling.status),
      static_cast<long long>(chest_end),
      fusion.wall_ms,fusion.cpu_ms,static_cast<long long>(handoff.enter_ros_ns),
      static_cast<long long>(handoff.enter_steady_ns),static_cast<long long>(handoff.return_ros_ns),
      static_cast<long long>(handoff.return_steady_ns));
    // The same failed belly output already owns its accepted joint snapshot.
    // Preserve provenance even when coverage, rather than FK, faults first.
    RCLCPP_ERROR(node.get_logger(),
      "SPATIAL_HANDOFF_JOINT_INGRESS role=%s camera_sequence=%llu camera_source_ns=%lld "
      "sdk_receipt_ros_ns=%lld sdk_receipt_steady_ns=%lld fk_wait_ns=%lld %s",
      belly.input->role.c_str(),static_cast<unsigned long long>(belly.input->sequence),
      static_cast<long long>(stamp(belly.input->source_stamp)),
      static_cast<long long>(belly.input->first_sdk_callback_ros_ns),
      static_cast<long long>(belly.input->first_sdk_callback_steady_ns),
      static_cast<long long>(belly.fk_wait_ns),belly.joints.ingress_detail().c_str());
  }
  void guard_evidence(Evidence& e,int64_t ros,int64_t steady){
    detail::expire_spatial_evidence(e,ros,steady);
    std::lock_guard<std::mutex> lock(state_mutex);
    if(terminal.load()){e.evaluation_complete=false;e.reason="terminal_revocation:"+fault_reason;}
  }
  void prepare_evidence(Evidence& e){
    const auto ros=node.now().nanoseconds(),steady=dm::steady_ns();
    guard_evidence(e,ros,steady);
    // These are delivery preparation clocks. Original acquisition, receipt and
    // validity fields are immutable; the DDS observer cannot renew them.
    e.header.stamp=rclcpp::Time(ros,node.get_clock()->get_clock_type());
    e.evaluation_steady_ns=steady;
  }
  void direct_evidence(Evidence& e,SpatialHandoffTrace* handoff=nullptr){
    // The original FIRST-publication deadline remains closed after expiry.
    // Suppress egress, preserving sensors/localization when control is held;
    // main alone decides whether this startup timeout terminates the session.
    if(stopping.load() || !spatial_startup.publication_allowed(dm::steady_ns()))return;
    const auto immutable=std::make_shared<const Evidence>(e);
    if(handoff){handoff->enter_ros_ns=node.now().nanoseconds();handoff->enter_steady_ns=dm::steady_ns();}
    using State=sunhorizon_obstacle_coverage::SpatialDeliveryState;
    sunhorizon_obstacle_coverage::SpatialDeliveryResult result;
    try {
      result=spatial_sink ? spatial_sink(immutable) :
        sunhorizon_obstacle_coverage::SpatialDeliveryResult{e.evaluation_complete ? State::Accepted :
          e.reason=="source_expired_before_egress" ? State::MotionUnavailable : State::InternalFault,e.reason};
    }
    catch(...){
      if(handoff){handoff->rejected=true;handoff->return_ros_ns=node.now().nanoseconds();
        handoff->return_steady_ns=dm::steady_ns();}
      throw;
    }
    if(handoff && (!e.evaluation_complete || !result.accepted())){
      handoff->rejected=true;handoff->return_ros_ns=node.now().nanoseconds();
      handoff->return_steady_ns=dm::steady_ns();
    }
    detail::apply_spatial_delivery_result(e,result,[this](const std::string& reason){fault(reason);});
  }
  void observe_evidence(Evidence e){
    evidence_worker->replace(std::make_unique<Evidence>(std::move(e)));
  }
  void deliver_evidence(Evidence e,SpatialHandoffTrace* handoff){
    for(const auto& source:e.sources)
      if(!spatial_startup.allows(source.source_ros_ns,source.first_receipt_ros_ns,source.first_receipt_steady_ns))return;
    spatial_delivery.submit(std::move(e),[this](Evidence& value){prepare_evidence(value);},
      [this,handoff](Evidence& value){direct_evidence(value,handoff);},
      [this](Evidence value){observe_evidence(std::move(value));});
  }
  void publish_evidence(Evidence e){
    if(!spatial_startup.publication_allowed(dm::steady_ns()))return;
    // DDS observation owns no direct sequence, last snapshot or sink call.
    // In-flight old publications may finish, but cannot revive direct health.
    for(const auto& source:e.sources)
      if(!spatial_startup.allows(source.source_ros_ns,source.first_receipt_ros_ns,source.first_receipt_steady_ns))return;
    guard_evidence(e,node.now().nanoseconds(),dm::steady_ns());
    evidence_pub->publish(e);
    if(e.evaluation_complete && !terminal.load() && !stopping.load()){
      const auto& source=e.sources.front();
      const auto published_steady=dm::steady_ns();
      if(spatial_startup.published(source.source_ros_ns,source.first_receipt_ros_ns,source.first_receipt_steady_ns,
          source.frame_sequence,published_steady))
        RCLCPP_INFO(node.get_logger(),"SPATIAL_FIRST_PUBLISHED sequence=%llu source_ns=%lld",
          static_cast<unsigned long long>(source.frame_sequence),static_cast<long long>(source.source_ros_ns));
    }
    // Direct delivery has already classified producer/consumer errors. An
    // observer delayed beyond the original expiry only publishes unavailable;
    // it cannot stop latest computation or the independent localization lanes.
  }
  void notify_health_completion(){
    // A new BODY result must not wait for the next 50 ms timer to become visible.
    // This is only a bounded wakeup of the same egress owner: it re-snapshots
    // original source/receipt ages under health_delivery_mutex before delivery.
    health_worker->replace(std::make_unique<sunhorizon_navigation_interfaces::msg::RgbdStatus>());
  }
  void tick(){
    const auto ros=node.now().nanoseconds(),steady=dm::steady_ns();
    if(!clock_guard.observe(ros,steady))fault("runtime_clock_rollback");
    if(terminal.load()){
      std::string reason;
      {std::lock_guard<std::mutex> lock(state_mutex);reason=fault_reason;}
      health.fault(reason);
      spatial_delivery.revoke([this](Evidence& value){prepare_evidence(value);},
        [this](Evidence& value){direct_evidence(value);},
        [this](Evidence value){observe_evidence(std::move(value));});
    }
    health_worker->replace(std::make_unique<sunhorizon_navigation_interfaces::msg::RgbdStatus>(health.snapshot(ros,steady)));
  }
  void timing(const Output& out,const EngineDuration& fusion,const detail::BodyPairWitness* pairing=nullptr){
    if(body_replay_enabled && !ground_snapshot_directory.empty() &&
       out->frame.body_filter.complete && out->frame.body_replay_diagnostic &&
       (out->input->role=="belly" || out->input->role=="chest")) {
      const auto role=out->input->role=="belly"?0:1;
      bool expected=false;
      if(body_replay_selected[role].compare_exchange_strong(expected,true))
        std::atomic_store(&pending_body_replay[role],out);
    }

    if(!ground_snapshot_directory.empty() &&
        (out->input->role=="belly"||out->input->role=="chest") && out->frame.classified_obstacles) {
      const std::size_t role=out->input->role=="belly"?0:1;
      const auto& frame=out->frame.classified_obstacles;
      const auto wanted=role==0?ground_relative::Qualification::Qualified:
        ground_relative::Qualification::InsufficientSupportGeometry;
      if(frame->ground().transform_diagnostic.provider_state==0 &&
          frame->ground().result==wanted && (role!=0||frame->ground().safety_qualified)) {
        bool expected=false;
        if(ground_snapshot_selected[role].compare_exchange_strong(expected,true))
          std::atomic_store(&pending_ground_snapshot[role],frame);
      }
    }
    diagnostic_msgs::msg::DiagnosticArray d;d.header.stamp=node.now();
    diagnostic_msgs::msg::DiagnosticStatus status;status.name="perception_same_source_v1";status.hardware_id=out->input->role;
    status.message="wall includes scheduling; source age includes upstream clock and delivery";
    auto add=[&](const std::string& name,auto value){diagnostic_msgs::msg::KeyValue k;k.key=name;k.value=std::to_string(value);status.values.push_back(std::move(k));};
    add("sequence",out->input->sequence);add("source_ns",stamp(out->input->source_stamp));
    add("body_only_independent_roles",!config.include_chest);
    if(!config.include_chest && out->input->role=="chest")
      add("body_role_compute_replacements",body_only_chest_compute->stats().replaced);
    // Only this belly BODY report carries its own immutable selection witness.
    // No mutable queue state is reread during diagnostic serialization.
    if(pairing){const auto& pair=*pairing;
      add("body_pair_witness_version",1);
      add("body_pair_belly_sequence",pair.belly.sequence);add("body_pair_belly_source_ns",pair.belly.source_ns);
      add("body_pair_anchor_source_ns",pair.belly.source_ns);
      add("body_pair_selection_limit_ros_ns",pair.anchor_source_ns);
      add("body_pair_enqueue_steady_ns",pair.wait.enqueue_steady_ns);
      add("body_pair_process_entry_steady_ns",pair.wait.process_entry_steady_ns);
      add("body_pair_lock_attempt_steady_ns",pair.wait.lock_attempt_steady_ns);
      add("body_pair_lock_acquired_steady_ns",pair.wait.lock_acquired_steady_ns);
      add("body_pair_selection_steady_ns",pair.selection_steady_ns);
      add("body_pair_deque_count",pair.deque_count);
      add("body_pair_oldest_source_ns",pair.oldest.source_ns);add("body_pair_oldest_sequence",pair.oldest.sequence);
      add("body_pair_newest_source_ns",pair.newest.source_ns);add("body_pair_newest_sequence",pair.newest.sequence);
      add("body_pair_last_processed_chest_sequence",pair.last_processed_sequence);
      add("body_pair_too_new_count",pair.too_new_count);add("body_pair_causal_already_processed_count",pair.causal_already_processed_count);
      add("body_pair_chosen",pair.chosen);add("body_pair_chosen_source_ns",pair.chosen_identity.source_ns);
      add("body_pair_chosen_sequence",pair.chosen_identity.sequence);add("body_pair_chosen_fresh",pair.chosen_fresh);
      diagnostic_msgs::msg::KeyValue reason;reason.key="body_pair_selection_reason";reason.value=pair.selection_reason;
      status.values.push_back(std::move(reason));
      add("body_pair_evictions",pair.evictions.count);
      add("body_pair_last_evicted_source_ns",pair.evictions.last.source_ns);
      add("body_pair_last_evicted_sequence",pair.evictions.last.sequence);
    }
    add("obstacle_input_available",out->frame.obstacle_input_health.available);
    {diagnostic_msgs::msg::KeyValue value;value.key="obstacle_input_reason";value.value=out->frame.obstacle_input_health.reason;status.values.push_back(std::move(value));}
    add("body_stereo_occlusion_enabled",config.sources[out->input->role=="belly"?0:1].engine.body_stereo_occlusion_enabled);
    add("stereo_body_baseline_m",out->frame.stats.stereo_body_baseline_m);
    add("stereo_body_geometry_qualified",out->frame.stats.stereo_body_baseline_m>0.);
    add("stereo_ir_pair_exact",bool(out->input->measurement_quality&&out->input->measurement_quality->exact_rectified_pair));
    {diagnostic_msgs::msg::KeyValue item;item.key="stereo_body_geometry_reason";item.value=out->frame.stats.stereo_body_geometry_reason;status.values.push_back(std::move(item));}
    add("measurement_body_occluded_samples",std::count(out->frame.body_filter.ground_blocked.begin(),out->frame.body_filter.ground_blocked.end(),uint8_t(3)));
    add("body_complete",out->frame.body_filter.complete);add("raw_projection_complete",out->frame.raw_projection_complete);
    add("sdk_receipt_ros_ns",out->input->first_sdk_callback_ros_ns);add("sdk_receipt_steady_ns",out->input->first_sdk_callback_steady_ns);
    add("compute_start_steady_ns",out->compute_start_steady_ns);add("compute_end_steady_ns",out->compute_end_steady_ns);
    add("fk_wait_ns",out->fk_wait_ns);add("joint_source_ns",out->joints.source_stamp_ns);
    add("joint_receipt_ros_ns",out->joints.receipt_stamp_ns);add("joint_receipt_steady_ns",out->joints.receipt_steady_ns);
    add("joint_accepted_source_interval_ns",out->joints.accepted_source_interval_ns);
    add("joint_accepted_receipt_interval_ns",out->joints.accepted_receipt_interval_ns);
    add("joint_fk_wall_ns",out->joints.fk_wall_ns);add("joint_fk_thread_cpu_ns",out->joints.fk_thread_cpu_ns);
    add("sdk_queue_ns",out->input->timing.queue_wait_wall_ns());add("sdk_convert_wall_ns",out->input->timing.worker_convert.wall_ns());
    add("sdk_convert_cpu_ns",out->input->timing.worker_convert.thread_cpu_ns());
    add("prepare_start_steady_ns",out->preparation_start_steady_ns);
    add("prepare_end_steady_ns",out->preparation_end_steady_ns);
    add("prepare_cpu_ms",out->frame.timings.source_prepare.cpu_ms);
    add("prepare_wall_ms",out->frame.timings.source_prepare.wall_ms);
    const auto& prep=out->preparation_scheduling;
    add("prepare_thread_tid",prep.tid);
    add("prepare_thread_state_available",out->preparation_thread_state.available);
    add("prepare_thread_actual_tid",out->preparation_thread_state.tid);
    add("prepare_thread_actual_policy",out->preparation_thread_state.policy);
    add("prepare_thread_actual_nice",out->preparation_thread_state.nice);
    const auto affinity=[&](const std::string& prefix,const detail::ComputeThreadState& state){
      add(prefix+"_affinity_available",state.affinity_available);
      diagnostic_msgs::msg::KeyValue value;value.key=prefix+"_actual_cpu_set";
      for(const auto cpu:state.cpu_set){if(!value.value.empty())value.value+=",";value.value+=std::to_string(cpu);}
      status.values.push_back(std::move(value));
    };
    affinity("prepare_thread",out->preparation_thread_state);
    add("prepare_schedstat_available",prep.schedstat_available);
    add("prepare_schedstat_status",static_cast<int>(prep.status));
    add("prepare_schedstat_running_ns",prep.running_ns);add("prepare_schedstat_runqueue_wait_ns",prep.runqueue_wait_ns);
    add("prepare_schedstat_sampling_wall_ns",prep.sampling_wall_ns);
    add("engine_cpu_ms",out->frame.timings.total.cpu_ms);add("engine_wall_ms",out->frame.timings.total.wall_ms);
    const auto& scheduling=out->engine_scheduling;
    add("engine_thread_tid",scheduling.tid);
    add("engine_thread_state_available",out->engine_thread_state.available);
    add("engine_thread_actual_tid",out->engine_thread_state.tid);
    add("engine_thread_actual_policy",out->engine_thread_state.policy);
    add("engine_thread_actual_nice",out->engine_thread_state.nice);
    affinity("engine_thread",out->engine_thread_state);
    {diagnostic_msgs::msg::KeyValue backend;backend.key="body_geometry_compute_backend";backend.value=body_geometry_role_backends[out->input->role=="belly"?0:1];status.values.push_back(std::move(backend));}
    const bool role_parallel=detail::body_geometry_cpu_parallel(body_geometry_role_backends[out->input->role=="belly"?0:1],body_geometry_parallel_enabled);
    add("body_geometry_parallel_enabled",role_parallel);
    add("body_geometry_chunk_points",role_parallel?128:0);
    add("body_geometry_helper_cpu_ms",out->frame.timings.body_geometry_helper_cpu_ms);
    add("engine_main_plus_geometry_cpu_ms",out->frame.timings.total.cpu_ms+out->frame.timings.body_geometry_helper_cpu_ms);
    add("body_main_plus_geometry_cpu_ms",out->frame.timings.body_filter.cpu_ms+out->frame.timings.body_geometry_helper_cpu_ms);
    add("geometry_helper_thread_actual_tid",out->geometry_helper_thread_state.tid);
    add("geometry_helper_thread_actual_nice",out->geometry_helper_thread_state.nice);
    add("geometry_helper_thread_actual_policy",out->geometry_helper_thread_state.policy);
    affinity("geometry_helper_thread",out->geometry_helper_thread_state);

    add("engine_schedstat_available",scheduling.schedstat_available);
    add("engine_schedstat_status",static_cast<int>(scheduling.status));
    add("engine_schedstat_running_ns",scheduling.running_ns);
    add("engine_schedstat_runqueue_wait_ns",scheduling.runqueue_wait_ns);
    add("engine_schedstat_timeslices",scheduling.timeslices);
    add("engine_schedstat_window_begin_steady_ns",scheduling.window_begin_steady_ns);
    add("engine_schedstat_window_end_steady_ns",scheduling.window_end_steady_ns);
    add("engine_schedstat_sampling_wall_ns",scheduling.sampling_wall_ns);
    add("engine_rusage_available",scheduling.rusage_available);
    add("engine_voluntary_context_switches",scheduling.voluntary_switches);
    add("engine_involuntary_context_switches",scheduling.involuntary_switches);
    add("model_cpu_ms",out->frame.timings.model_prepare.cpu_ms);add("quality_cpu_ms",out->frame.timings.quality_decode.cpu_ms);
    add("body_cpu_ms",out->frame.timings.body_filter.cpu_ms);add("coverage_cpu_ms",out->frame.timings.coverage_classify.cpu_ms);
    add("obstacle_classify_cpu_ms",out->frame.timings.obstacle_classify.cpu_ms);
    add("obstacle_classify_wall_ms",out->frame.timings.obstacle_classify.wall_ms);
    add("near_ground_clearance_cpu_ms",out->frame.timings.near_ground_clearance.cpu_ms);
    add("near_ground_clearance_wall_ms",out->frame.timings.near_ground_clearance.wall_ms);
    if(out->frame.near_ground_clearance) {
      const auto& proof=*out->frame.near_ground_clearance;
      std::size_t free=0,occupied=0,unknown=0;
      for(const auto& tile:proof.tiles) {
        if(tile.state==sv::CellState::kFree)++free;
        else if(tile.state==sv::CellState::kOccupied)++occupied;
        else ++unknown;
      }
      add("near_ground_complete",proof.evaluation_complete);add("near_ground_qualified",proof.ground_qualified);
      add("near_ground_free_tiles",free);add("near_ground_occupied_tiles",occupied);
      add("near_ground_unknown_tiles",unknown);add("near_ground_visibility_checks",proof.visibility_checks);
    }
    if (out->frame.classified_obstacles) {
      const auto& frame=*out->frame.classified_obstacles;
      add("ground_qualified",frame.ground().safety_qualified);
      add("ground_qualification",static_cast<int>(frame.ground().result));
      add("ground_observation_only",frame.ground().observation_only);
      add("ground_reference_geometry_valid",frame.ground().reference_geometry_valid);
      add("ground_support_valid",frame.ground().support_valid);
      add("ground_transform_provider_state",frame.ground().transform_diagnostic.provider_state);
      diagnostic_msgs::msg::KeyValue transform_reason;
      transform_reason.key="ground_transform_reason";
      transform_reason.value=frame.ground().transform_diagnostic.reason;
      status.values.push_back(std::move(transform_reason));
      add("ground_candidate_ground",frame.ground().candidate_ground_points);
      add("ground_candidate_obstacle",frame.ground().candidate_obstacle_points);
      add("ground_candidate_uncertain",frame.ground().candidate_uncertain_points);
      add("classified_mark_points",frame.marking_indices().size());
      add("classified_clear_points",frame.clearing_indices().size());
    }
    add("raw_projection_cpu_ms",out->frame.timings.raw_projection.cpu_ms);
    add("raw_projection_wall_ms",out->frame.timings.raw_projection.wall_ms);
    add("raw_projection_source_ns",out->frame.raw_projection_source_ns);
    add("raw_projection_raw_source_ns",out->frame.raw_projection_raw_source_ns);
    add("raw_projection_expiry_budget_ns",out->frame.raw_projection_expiry_budget_ns);
    add("quality_mask_reuses",out->frame.stats.quality_mask_reuses);
    const auto startup=spatial_startup.snapshot();
    add("spatial_startup_deferred",startup.deferred);add("spatial_startup_owner_ready",startup.owner_ready);
    add("spatial_startup_open_ros_ns",startup.opened_ros_ns);add("spatial_startup_open_steady_ns",startup.opened_steady_ns);
    add("spatial_startup_deadline_steady_ns",startup.deadline_steady_ns);
    add("spatial_startup_first_published_steady_ns",startup.first_published_steady_ns);
    add("spatial_startup_first_published_sequence",startup.first_published_sequence);
    add("fusion_cpu_ms",fusion.cpu_ms);add("fusion_wall_ms",fusion.wall_ms);
    add("compute_replacements",compute->stats().replaced);add("source_age_at_timing_ms",(node.now().nanoseconds()-stamp(out->input->source_stamp))/1.e6);
    d.status.push_back(std::move(status));diagnostics_worker->replace(std::make_unique<diagnostic_msgs::msg::DiagnosticArray>(std::move(d)));
  }
};
PerceptionRuntime::PerceptionRuntime(const rclcpp::NodeOptions& options,bool defer_ground_reference)
  :PerceptionRuntime(options,defer_ground_reference,true){}
PerceptionRuntime::PerceptionRuntime(const rclcpp::NodeOptions& options,bool defer_ground_reference,
    bool external_tf_listener):Node("sunhorizon_perception_runtime",options),
  impl_(std::make_unique<Impl>(*this,defer_ground_reference,external_tf_listener)){}
PerceptionRuntime::~PerceptionRuntime(){stop();}
ground_relative::GroundReferenceOwner::ConstSharedPtr PerceptionRuntime::ground_reference_owner()const{return impl_->ground_owner;}
void PerceptionRuntime::configure_registration_ground(const std::string& actual_map_sha256){
  if(!impl_->ground_reference_deferred || impl_->joints->evidence().history_samples)
    throw std::logic_error("registration_ground_requires_unstarted_runtime");
  const auto file=get_parameter("ground_reference_file").as_string();
  const auto sha=get_parameter("ground_reference_sha256").as_string();
  std::optional<ground_relative::RegistrationMapIdentity> actual;
  if(!file.empty()) {
    if(!dm::sha_valid(actual_map_sha256))throw std::invalid_argument("registration_ground_map_digest_missing");
    actual=ground_relative::RegistrationMapIdentity{actual_map_sha256,"ndt-registration-pcd-v1"};
  }
  impl_->ground_owner=ground_relative::GroundReferenceOwner::load(file,sha,impl_->config.clock_boot_id,actual);
  impl_->ground_reference_deferred=false;
}
void PerceptionRuntime::set_registration_ground_geometry_provider(RegistrationGroundGeometryProvider provider){
  if(impl_->ground_geometry || !provider || impl_->joints->evidence().history_samples)
    throw std::logic_error("registration_geometry_requires_unstarted_runtime");
  impl_->ground_geometry=std::move(provider);
}
RegistrationGroundGeometryProvider PerceptionRuntime::registration_ground_geometry_provider()const{return impl_->ground_geometry;}
void PerceptionRuntime::configure_direct_transforms(std::shared_ptr<tf2_ros::Buffer> buffer, JointKinematics::TransformSink compatibility){
  if(!buffer || impl_->direct_tf_configured.load() || impl_->joints->evidence().history_samples)
    throw std::logic_error("direct_tf_requires_unstarted_runtime");
  std::unique_ptr<const detail::AttendedGeometry> geometry;
  if (impl_->config.joint_kinematics.tf_mode == "attended_direct") {
    // Compile only the validated fixed asset before acquisition. Never derive
    // these immutable constants from the shared, dynamically updated buffer.
    geometry=std::make_unique<const detail::AttendedGeometry>(impl_->config.joint_kinematics,
      impl_->config.common_frame,std::array<std::string,2>{
        impl_->config.sources[0].frame,impl_->config.sources[1].frame});
  }
  impl_->joints->mirror_into(buffer,std::move(compatibility));
  impl_->tf_listener.reset();impl_->tf_node.reset();
  buffer->setUsingDedicatedThread(true); // Joint and localization owners update independently.
  impl_->attended_geometry=std::move(geometry);
  impl_->global=std::move(buffer);impl_->direct_tf_configured.store(true);
}
std::shared_ptr<JointKinematics> PerceptionRuntime::kinematics()const{return impl_->joints;}
double PerceptionRuntime::self_surface_diagnostic_tolerance_m()const{
  return impl_->config.sources[0].engine.self_tolerance_m;
}
double PerceptionRuntime::lidar_surface_radius_bound(const std::string& source_frame)const{
  const auto& specs=impl_->model.specs();double radius=0.;
  const auto inf=std::numeric_limits<double>::infinity();
  for(std::size_t i=0;i<specs.size();++i)radius=std::max(radius,std::nextafter(
    impl_->joints->origin_radius_bound_from_base(specs[i].link_name)+impl_->model.mesh_origin_radius_bound(i),inf));
  return std::nextafter(radius+impl_->joints->origin_radius_bound_from_base(source_frame),inf);
}
std::shared_ptr<const dg::PreparedModel> PerceptionRuntime::lidar_model(
    const std::string& source_frame,int64_t source_ns)const{
  // Compatibility caller owns one query, hence no cross-scan storage.
  return lidar_model_provider_for_scan(1)(source_frame,source_ns);
}
sunhorizon_rgbd_adapter::LidarModelProvider PerceptionRuntime::lidar_model_provider_for_scan(
    std::size_t maximum_entries)const{
  using Key=std::tuple<std::shared_ptr<const JointNumericGeometry>,std::string,std::array<uint64_t,12>>;
  // Model's meshes/specs and query budget are private, immutable owner state.
  // This map belongs to ONE scan closure, never to the runtime or joint owner.
  return [this,maximum_entries,cache=std::map<Key,std::shared_ptr<const dg::PreparedModel>>{}]
      (const std::string& source_frame,int64_t source_ns)mutable -> std::shared_ptr<const dg::PreparedModel>{
    if(source_frame.empty() || source_ns<=0)throw std::invalid_argument("lidar_model_source_invalid");
    if(impl_->terminal.load() || impl_->stopping.load())return {};
    const auto fk=impl_->joints->lookup_numeric(source_ns,[this]{return JointKinematics::ClockSample{
      now().nanoseconds(),dm::steady_ns()};});
    if(fk.status==JointKinematicsStatus::Fault)throw std::runtime_error("lidar_model_joint_fault:"+fk.reason);
    if(fk.status!=JointKinematicsStatus::Ready)return {};
    try{
      const auto* fixed=impl_->attended_geometry?impl_->attended_geometry->select_source(
        impl_->config.joint_kinematics.tf_mode,source_frame,source_ns):nullptr;
      const auto sensor=fixed?*fixed:dm::rigid(detail::source_time_transform(
        *impl_->global,source_frame,"base_link",source_ns).transform);
      if(!fk.numeric_geometry || fk.numeric_geometry->transforms.size()!=12)
        throw std::logic_error("lidar_model_incomplete_numeric_geometry");
      // Only exact canonical samples share preparation. Moving/interpolated
      // poses avoid keys/map allocations entirely. Owning the numerical pose in
      // the key prevents address recycling after joint-history rollover.
      std::optional<Key> key;
      if(fk.numeric_geometry_reused){
        key.emplace(fk.numeric_geometry,source_frame,std::array<uint64_t,12>{});
        std::size_t index=0;auto& bits=std::get<2>(*key);
        auto append=[&](double value){std::memcpy(&bits[index++],&value,sizeof(value));};
        for(const auto value:sensor.rotation)append(value);
        append(sensor.translation.x);append(sensor.translation.y);append(sensor.translation.z);
        // Original gates and actual source-time sensor qualification precede
        // EVERY hit. Signed zero/ULP differences remain distinct sensor poses.
        const auto found=cache.find(*key);if(found!=cache.end())return found->second;
      }
      std::vector<dg::RigidTransform> base_from_links;base_from_links.reserve(12);
      for(const auto& tf:fk.numeric_geometry->transforms)base_from_links.push_back(dm::rigid(tf));
      const auto links=impl_->model.observations(sensor,base_from_links,source_ns);
      auto prepared=std::make_shared<const dg::PreparedModel>(links,impl_->config.sources[0].engine.body_budget.query);
      // Full cache is only a missed optimization, never missing geometry. At
      // most one entry per queried original group, bounded by scan records.
      if(key && cache.size()<maximum_entries)cache.emplace(std::move(*key),prepared);
      return prepared;
    }catch(const tf2::TransformException&){return {};}
  };
}
void PerceptionRuntime::admit(Frame f){if(!f||impl_->terminal.load()||impl_->stopping.load())return;
  if(impl_->config.query_policy.height_limited() && !impl_->collision_height_bound.load()){
    fault("collision_height_requires_actual_unified_nav2_binding");return;
  }
  impl_->spatial_startup.note_input();
  if(impl_->config.joint_kinematics.tf_mode!="attended_global_tf" && !impl_->direct_tf_configured.load()){
    fault("explicit_direct_tf_mode_was_not_configured");return;
  }
  const size_t i=f->role=="belly"?0:f->role=="chest"?1:2;
  if(i>1){fault("unsupported_camera_role");return;}impl_->admissions[i]->replace(std::make_unique<Frame>(std::move(f)));}
void PerceptionRuntime::fault(const std::string& s){impl_->fault(s);}
bool PerceptionRuntime::failed()const{return impl_->terminal.load();}
std::string PerceptionRuntime::fault_reason()const{
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  return impl_->fault_reason;
}
void PerceptionRuntime::defer_spatial_startup(){impl_->spatial_startup.defer();}
void PerceptionRuntime::release_spatial_startup(int64_t deadline){
  if(impl_->config.query_policy.height_limited() && !impl_->collision_height_bound.load())
    throw std::logic_error("collision_height_requires_actual_unified_nav2_binding");
  impl_->spatial_startup.owner_ready(deadline);
}
SpatialStartupStatus PerceptionRuntime::spatial_startup_status()const{return impl_->spatial_startup.snapshot();}
SpatialQueryPolicy PerceptionRuntime::spatial_query_policy()const{return impl_->config.query_policy;}
void PerceptionRuntime::bind_collision_height_responsibility(const CollisionHeightBinding& binding){
  if(!impl_->direct_tf_configured.load() || impl_->joints->evidence().history_samples ||
      !impl_->spatial_startup.snapshot().deferred || !(binding.policy()==impl_->config.query_policy) ||
      impl_->collision_height_bound.exchange(true))
    throw std::logic_error("collision_height_binding_requires_unstarted_matching_unified_owner");
}
void PerceptionRuntime::stop(){if(impl_)impl_->stop();}
void PerceptionRuntime::set_spatial_output_sink(SpatialOutputSink sink){impl_->spatial_sink=std::move(sink);}
void PerceptionRuntime::set_health_output_sink(HealthOutputSink sink){impl_->health_sink=std::move(sink);}
rclcpp::Publisher<sunhorizon_navigation_interfaces::msg::SpatialCoverageEvidence>::SharedPtr PerceptionRuntime::spatial_publisher()const{return impl_->evidence_pub;}
void PerceptionRuntime::set_region_output_sink(RegionOutputSink sink){impl_->region_sink=std::move(sink);}
void PerceptionRuntime::set_output_sink(PerceptionOutputSink sink){impl_->output_sink=std::move(sink);}
}
