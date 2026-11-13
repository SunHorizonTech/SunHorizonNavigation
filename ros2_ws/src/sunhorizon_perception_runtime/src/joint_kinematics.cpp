#include "sunhorizon_perception_runtime/joint_kinematics.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <cmath>
#include <deque>
#include <cstring>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <shared_mutex>
#include <stdexcept>
#include <utility>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2/buffer_core.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <urdf/model.h>

namespace sunhorizon_perception_runtime {
namespace {
constexpr int64_t kMaximumStamp =
    (static_cast<int64_t>(std::numeric_limits<int32_t>::max()) + 1) * 1000000000LL - 1;

int64_t steady_time_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
int64_t thread_cpu_ns() {
  timespec value{};
  if(clock_gettime(CLOCK_THREAD_CPUTIME_ID,&value)!=0)
    throw std::runtime_error("joint_thread_cpu_clock_unavailable");
  return int64_t(value.tv_sec)*1000000000LL+value.tv_nsec;
}
bool valid_stamp(int64_t stamp) { return stamp > 0 && stamp <= kMaximumStamp; }

geometry_msgs::msg::TransformStamped make_transform(
    const std::string& parent, const std::string& child, const KDL::Frame& frame,
    int64_t stamp) {
  geometry_msgs::msg::TransformStamped result;
  result.header.frame_id = parent;
  result.child_frame_id = child;
  result.header.stamp.sec = static_cast<int32_t>(stamp / 1000000000LL);
  result.header.stamp.nanosec = static_cast<uint32_t>(stamp % 1000000000LL);
  result.transform.translation.x = frame.p.x();
  result.transform.translation.y = frame.p.y();
  result.transform.translation.z = frame.p.z();
  frame.M.GetQuaternion(result.transform.rotation.x, result.transform.rotation.y,
                        result.transform.rotation.z, result.transform.rotation.w);
  const double values[]{result.transform.translation.x, result.transform.translation.y,
                        result.transform.translation.z, result.transform.rotation.x,
                        result.transform.rotation.y, result.transform.rotation.z,
                        result.transform.rotation.w};
  for (double value : values) {
    if (!std::isfinite(value)) throw std::invalid_argument("joint_kinematics_nonfinite_pose");
  }
  return result;
}

struct EdgePose {
  tf2::Quaternion rotation{0.,0.,0.,1.};
  tf2::Vector3 translation{0.,0.,0.};
};
EdgePose edge_transform(const geometry_msgs::msg::Transform& value) {
  const auto& q=value.rotation;const auto& p=value.translation;
  return {tf2::Quaternion(q.x,q.y,q.z,q.w),tf2::Vector3(p.x,p.y,p.z)};
}

bool exact_transform(const geometry_msgs::msg::Transform& a,const geometry_msgs::msg::Transform& b) {
  const double one[]{a.translation.x,a.translation.y,a.translation.z,a.rotation.x,a.rotation.y,a.rotation.z,a.rotation.w};
  const double two[]{b.translation.x,b.translation.y,b.translation.z,b.rotation.x,b.rotation.y,b.rotation.z,b.rotation.w};
  return std::memcmp(one,two,sizeof(one))==0;
}
geometry_msgs::msg::Transform numeric_transform(const EdgePose& pose) {
  geometry_msgs::msg::Transform value;const auto& t=pose.translation;const auto& q=pose.rotation;
  value.translation.x=t.x();value.translation.y=t.y();value.translation.z=t.z();
  value.rotation.x=q.x();value.rotation.y=q.y();value.rotation.z=q.z();value.rotation.w=q.w();return value;
}
void validate_joint(const urdf::Joint& joint) {
  if (joint.mimic) throw std::invalid_argument("joint_kinematics_mimic_unsupported");
  if (joint.type != urdf::Joint::FIXED && joint.type != urdf::Joint::REVOLUTE &&
      joint.type != urdf::Joint::CONTINUOUS && joint.type != urdf::Joint::PRISMATIC) {
    throw std::invalid_argument("joint_kinematics_joint_type_unsupported");
  }
  const auto& origin = joint.parent_to_joint_origin_transform;
  const double values[]{origin.position.x, origin.position.y, origin.position.z,
                        origin.rotation.x, origin.rotation.y, origin.rotation.z,
                        origin.rotation.w, joint.axis.x, joint.axis.y, joint.axis.z};
  for (double value : values) {
    if (!std::isfinite(value)) throw std::invalid_argument("joint_kinematics_nonfinite_model");
  }
  const double quaternion_norm = origin.rotation.x * origin.rotation.x +
      origin.rotation.y * origin.rotation.y + origin.rotation.z * origin.rotation.z +
      origin.rotation.w * origin.rotation.w;
  if (std::abs(quaternion_norm - 1.0) > 1e-6)
    throw std::invalid_argument("joint_kinematics_invalid_model_rotation");
  if (joint.type != urdf::Joint::FIXED) {
    const double axis_norm = joint.axis.x * joint.axis.x + joint.axis.y * joint.axis.y +
                            joint.axis.z * joint.axis.z;
    if (!std::isfinite(axis_norm) || axis_norm < 1e-12)
      throw std::invalid_argument("joint_kinematics_invalid_model_axis");
  }
}
}  // namespace

struct JointKinematics::Impl {
  struct Edge {
    std::string parent;
    std::string child;
    KDL::Segment segment;
    std::size_t position_index;
  };
  struct TraversalEdge {
    std::size_t parent, dynamic_index;
    EdgePose fixed;
  };
  struct Sample {
    int64_t source_stamp_ns;
    std::vector<geometry_msgs::msg::TransformStamped> edges;
    std::shared_ptr<const JointNumericGeometry> geometry;
  };

  explicit Impl(JointKinematicsOptions value) : options(std::move(value)) {
    if (options.expected_joint_names.size() != 13 || options.target_frames.size() != 12 ||
        options.base_frame.empty() || options.root_frame.empty() ||
        options.cache_time_ns <= 0 || options.maximum_age_ns <= 0 ||
        options.future_tolerance_ns < 0 || options.cache_time_ns > kMaximumStamp ||
        options.maximum_age_ns > kMaximumStamp || options.future_tolerance_ns > kMaximumStamp ||
        options.max_history_samples < 2 || options.max_history_samples > 16384) {
      throw std::invalid_argument("joint_kinematics_options_invalid");
    }
    std::map<std::string, std::size_t> indices;
    for (std::size_t index = 0; index < options.expected_joint_names.size(); ++index) {
      if (options.expected_joint_names[index].empty() ||
          !indices.emplace(options.expected_joint_names[index], index).second)
        throw std::invalid_argument("joint_kinematics_joint_names_invalid");
    }
    const std::set<std::string> targets(options.target_frames.begin(), options.target_frames.end());
    if (targets.size() != options.target_frames.size() || targets.count("") ||
        targets.count(options.base_frame))
      throw std::invalid_argument("joint_kinematics_target_frames_invalid");

    urdf::Model model;
    if (!model.initString(options.urdf_xml) || !model.getRoot() ||
        model.getRoot()->name != options.root_frame || !model.getLink(options.base_frame))
      throw std::invalid_argument("joint_kinematics_model_invalid");

    const auto inf=std::numeric_limits<double>::infinity();
    const auto bound_links=[&](const auto& self,const urdf::LinkConstSharedPtr& parent,double radius)->void {
      origin_radii[parent->name]=radius;
      for(const auto& child:parent->child_links){
        const auto& joint=*child->parent_joint;const auto& p=joint.parent_to_joint_origin_transform.position;
        const double length=std::hypot(std::hypot(p.x,p.y),p.z);
        self(self,child,joint.type==urdf::Joint::PRISMATIC ? inf : std::nextafter(radius+length,inf));
      }
    };
    bound_links(bound_links,model.getRoot(),0.);
    if(options.tf_mode=="attended_direct" || options.tf_mode=="attended_measured_direct")for(std::size_t pass=0;pass<options.attended_static_edges.size();++pass)
      for(const auto& edge:options.attended_static_edges){
        const auto parent=origin_radii.find(edge.header.frame_id);
        if(parent==origin_radii.end() || edge.child_frame_id==options.root_frame)continue;
        const auto& p=edge.transform.translation;
        origin_radii[edge.child_frame_id]=std::nextafter(parent->second+std::hypot(std::hypot(p.x,p.y),p.z),inf);
      }

    std::set<std::string> retained_joints;
    const auto retain_path = [&](const std::string& child) {
      auto link = model.getLink(child);
      if (!link) throw std::invalid_argument("joint_kinematics_model_link_missing");
      while (link->name != options.root_frame) {
        if (!link->parent_joint) throw std::invalid_argument("joint_kinematics_disconnected_model");
        retained_joints.insert(link->parent_joint->name);
        link = link->getParent();
        if (!link) throw std::invalid_argument("joint_kinematics_disconnected_model");
      }
    };
    retain_path(options.base_frame);
    for (const auto& target : options.target_frames) {
      // A target must descend from base, not merely share the URDF root.
      auto link = model.getLink(target);
      while (link && link->name != options.base_frame) link = link->getParent();
      if (!link) throw std::invalid_argument("joint_kinematics_target_outside_base");
      retain_path(target);
    }
    for (const auto& name : options.expected_joint_names) {
      const auto joint = model.getJoint(name);
      if (!joint || joint->type == urdf::Joint::FIXED)
        throw std::invalid_argument("joint_kinematics_expected_joint_missing");
      retain_path(joint->child_link_name);
    }
    // Keep fixed sensor descendants of observed paths as well. Never retain a
    // subtree requiring an unobserved movable joint (e.g. the arms).
    for (const auto& entry : model.links_) {
      auto link = entry.second;
      bool observable = true;
      while (link && link->name != options.root_frame) {
        const auto joint = link->parent_joint;
        if (!joint || (joint->type != urdf::Joint::FIXED && !indices.count(joint->name))) {
          observable = false; break;
        }
        link = link->getParent();
      }
      if (observable && link) retain_path(entry.first);
    }
    for (const auto& name : retained_joints) {
      const auto joint = model.getJoint(name);
      validate_joint(*joint);
      if (joint->type != urdf::Joint::FIXED && !indices.count(name))
        throw std::invalid_argument("joint_kinematics_model_joint_unobserved");
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree))
      throw std::invalid_argument("joint_kinematics_kdl_model_invalid");
    for (const auto& name : retained_joints) {
      const auto joint = model.getJoint(name);
      const auto segment = tree.getSegment(joint->child_link_name);
      if (segment == tree.getSegments().end())
        throw std::invalid_argument("joint_kinematics_kdl_segment_missing");
      if (joint->type == urdf::Joint::FIXED) {
        static_edges.push_back(make_transform(joint->parent_link_name, joint->child_link_name,
                                         segment->second.segment.pose(0.0), 0));
      } else {
        dynamic_edges.push_back(Edge{joint->parent_link_name, joint->child_link_name,
                                      segment->second.segment, indices.at(name)});
      }
    }
    if (dynamic_edges.size() != options.expected_joint_names.size())
      throw std::invalid_argument("joint_kinematics_model_joint_incomplete");
    // Compile paths once from this same parsed model. Parent indices precede
    // children; shared ancestors are evaluated only once per exact source time.
    std::map<std::string,std::size_t> nodes{{options.base_frame,0}},dynamic_indices;
    std::map<std::string,EdgePose> fixed_by_child;
    for(std::size_t i=0;i<dynamic_edges.size();++i)dynamic_indices.emplace(dynamic_edges[i].child,i);
    for(const auto& edge:static_edges)fixed_by_child.emplace(edge.child_frame_id,edge_transform(edge.transform));
    traversal.push_back({0,std::numeric_limits<std::size_t>::max(),EdgePose{}});
    const auto retain_node=[&](const auto& self,const std::string& child)->std::size_t {
      const auto found=nodes.find(child);if(found!=nodes.end())return found->second;
      const auto link=model.getLink(child);
      if(!link || !link->parent_joint)throw std::invalid_argument("joint_kinematics_traversal_path_invalid");
      const auto parent=self(self,link->parent_joint->parent_link_name);
      const auto moving=dynamic_indices.find(child);
      const auto index=traversal.size();
      traversal.push_back({parent,moving==dynamic_indices.end()?std::numeric_limits<std::size_t>::max():moving->second,
        moving==dynamic_indices.end()?fixed_by_child.at(child):EdgePose{}});
      nodes.emplace(child,index);return index;
    };
    for(const auto& target:options.target_frames)target_nodes.push_back(retain_node(retain_node,target));
    buffer = make_buffer({});
  }

  std::shared_ptr<tf2::BufferCore> make_buffer(const std::deque<Sample>& samples) const {
    auto result = std::make_shared<tf2::BufferCore>(tf2::Duration(options.cache_time_ns));
    for (const auto& edge : static_edges) {
      if (!result->setTransform(edge, "in_process_joint_kinematics", true))
        throw std::invalid_argument("joint_kinematics_static_edge_rejected");
    }
    for (const auto& sample : samples) {
      for (const auto& edge : sample.edges) {
        if (!result->setTransform(edge, "in_process_joint_kinematics", false))
          throw std::invalid_argument("joint_kinematics_dynamic_edge_rejected");
      }
    }
    return result;
  }

  std::shared_ptr<const JointNumericGeometry> compose(const std::vector<EdgePose>& edges) const {
    std::vector<EdgePose> nodes(traversal.size());
    for(std::size_t i=1;i<traversal.size();++i) {
      const auto& node=traversal[i];const auto& edge=node.dynamic_index==std::numeric_limits<std::size_t>::max()?node.fixed:edges[node.dynamic_index];
      const auto& parent=nodes[node.parent];nodes[i]={parent.rotation*edge.rotation,
        tf2::quatRotate(parent.rotation,edge.translation)+parent.translation};
    }
    auto result=std::make_shared<JointNumericGeometry>();result->transforms.reserve(target_nodes.size());
    for(const auto target:target_nodes)result->transforms.push_back(numeric_transform(nodes[target]));
    return result;
  }

  JointKinematicsOptions options;
  std::map<std::string,double> origin_radii;
  TransformSink transform_sink;
  std::vector<Edge> dynamic_edges;
  std::vector<TraversalEdge> traversal;
  std::vector<std::size_t> target_nodes;
  std::vector<geometry_msgs::msg::TransformStamped> static_edges;
  // The only producer can rebuild its bounded cache without holding the state
  // lock. This also safely serializes accidental concurrent ingest callers.
  std::mutex input_mutex;
  mutable std::shared_mutex state_mutex;
  std::shared_ptr<tf2::BufferCore> buffer;
  std::deque<Sample> history;
  JointKinematicsEvidence latest;
  bool input_available{false};
  bool fault_latched{false};
  std::string reason;
};

JointKinematics::JointKinematics(JointKinematicsOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}
JointKinematics::~JointKinematics() = default;

JointKinematicsIngestResult JointKinematics::ingest(
    const sunhorizon_platform_adapter_casbot02::LiveJointStateDecision& decision,
    int64_t receipt_stamp_ns, int64_t receipt_steady_ns, JointIngressTrace ingress) {
  const auto wall_start=steady_time_ns(),cpu_start=thread_cpu_ns();
  auto& state = *impl_;
  std::lock_guard<std::mutex> producer_lock(state.input_mutex);
  const auto reject = [&](const std::string& reason, bool latch) {
    std::unique_lock<std::shared_mutex> lock(state.state_mutex);
    state.input_available = false;
    state.fault_latched = state.fault_latched || latch;
    if (state.reason.empty() || !state.fault_latched || latch) state.reason = reason;
    return JointKinematicsIngestResult{false, state.fault_latched, state.reason, state.latest};
  };
  {
    std::shared_lock<std::shared_mutex> lock(state.state_mutex);
    if (state.fault_latched)
      return {false, true, state.reason, state.latest};
  }
  if (!decision.accepted || decision.fault_latched)
    return reject("joint_kinematics_gate_rejected:" + decision.reason, decision.fault_latched);
  if (decision.names != state.options.expected_joint_names ||
      decision.positions.size() != state.options.expected_joint_names.size())
    return reject("joint_kinematics_joint_structure_invalid", true);
  for (double position : decision.positions) {
    if (!std::isfinite(position)) return reject("joint_kinematics_position_nonfinite", true);
  }
  if ((state.options.tf_mode=="attended_direct" || state.options.tf_mode=="attended_measured_direct")) {
    const auto waist=std::find(decision.names.begin(),decision.names.end(),"waist_yaw");
    if(waist==decision.names.end() || std::abs(decision.positions[std::distance(decision.names.begin(),waist)])>.10)
      return reject("attended_waist_forward_pose_contract_violated",true);
  }
  if (!valid_stamp(decision.source_stamp_ns) || !valid_stamp(receipt_stamp_ns) ||
      receipt_steady_ns <= 0)
    return reject("joint_kinematics_input_clock_invalid", true);
  if (decision.source_stamp_ns <= state.latest.source_stamp_ns)
    return reject("joint_kinematics_source_not_increasing", false);
  if (receipt_steady_ns < state.latest.receipt_steady_ns ||
      receipt_stamp_ns < state.latest.receipt_stamp_ns)
    return reject("joint_kinematics_receipt_clock_regressed", true);
  const int64_t age = receipt_stamp_ns - decision.source_stamp_ns;
  if (age > state.options.maximum_age_ns || age < -state.options.future_tolerance_ns)
    return reject("joint_kinematics_source_timing_invalid", false);

  Impl::Sample sample{decision.source_stamp_ns, {}, {}};
  try {
    for (const auto& edge : state.dynamic_edges) {
      sample.edges.push_back(make_transform(edge.parent, edge.child,
          edge.segment.pose(decision.positions[edge.position_index]), decision.source_stamp_ns));
    }
  } catch (const std::exception&) {
    return reject("joint_kinematics_pose_invalid", true);
  }
  const auto same_previous=!state.history.empty()&&sample.edges.size()==state.history.back().edges.size()&&
    std::equal(sample.edges.begin(),sample.edges.end(),state.history.back().edges.begin(),
      [](const auto& a,const auto& b){return exact_transform(a.transform,b.transform);});
  if(same_previous)sample.geometry=state.history.back().geometry;
  else {std::vector<EdgePose> edges;edges.reserve(sample.edges.size());
    for(const auto& edge:sample.edges)edges.push_back(edge_transform(edge.transform));
    sample.geometry=state.compose(edges);}

  // Count bounds are independent of source rate. At rollover retain the newest
  // half plus this sample; only pointer replacement occurs under the read lock.
  std::shared_ptr<tf2::BufferCore> replacement;
  std::deque<Impl::Sample> retained;
  if (state.history.size() >= state.options.max_history_samples) {
    const auto first = state.history.begin() + state.history.size() / 2;
    retained.assign(first, state.history.end());
    retained.push_back(sample);
    try {
      replacement = state.make_buffer(retained);
    } catch (const std::exception&) {
      return reject("joint_kinematics_cache_rebuild_failed", true);
    }
  }
  // Destroy the retired allocations after releasing the state lock as well.
  std::shared_ptr<tf2::BufferCore> retired_buffer;
  std::deque<Impl::Sample> retired_history;
  std::unique_lock<std::shared_mutex> lock(state.state_mutex);
  if (replacement) {
    retired_buffer = std::move(state.buffer);
    retired_history = std::move(state.history);
    state.buffer = std::move(replacement);
    state.history = std::move(retained);
  } else {
    for (const auto& edge : sample.edges) {
      if (!state.buffer->setTransform(edge, "in_process_joint_kinematics", false)) {
        state.input_available = false;
        state.fault_latched = true;
        state.reason = "joint_kinematics_dynamic_edge_rejected";
        return {false, true, state.reason, state.latest};
      }
    }
    state.history.push_back(std::move(sample));
  }
  while (!state.history.empty() && decision.source_stamp_ns -
         state.history.front().source_stamp_ns > state.options.cache_time_ns)
    state.history.pop_front();
  // Mirror original per-joint sample edges, never latest-time composed poses.
  // On rollover sample moved into retained; obtain the same last sample.
  if (state.transform_sink) {
    try { state.transform_sink(state.history.back().edges, false); }
    catch (...) {
      state.input_available=false; state.fault_latched=true;
      state.reason="joint_kinematics_direct_transform_sink_failed";
      return {false,true,state.reason,state.latest};
    }
  }
  const auto previous=state.latest;
  const auto wall_end=steady_time_ns(),cpu_end=thread_cpu_ns();
  if (ingress.available) {
    ingress.ingest_begin = {wall_start, cpu_start};
    // This is the accepted commit boundary while state_mutex is held, not the
    // eventual function return or an unlocked reread which could name a new sample.
    ingress.ingest_commit = {wall_end, cpu_end};
  }
  state.latest = {decision.source_stamp_ns, receipt_stamp_ns, receipt_steady_ns,
                  state.history.size(),
                  previous.source_stamp_ns ? decision.source_stamp_ns-previous.source_stamp_ns : 0,
                  previous.receipt_steady_ns ? receipt_steady_ns-previous.receipt_steady_ns : 0,
                  wall_end-wall_start,cpu_end-cpu_start,ingress};
  state.input_available = true;
  state.reason.clear();
  return {true, false, {}, state.latest};
}

std::string JointKinematicsLookup::failure_detail(const std::string& role, uint64_t sequence,
    int64_t sdk_receipt_ros_ns, int64_t sdk_receipt_steady_ns) const {
  const auto& d=age_diagnostics;
  std::ostringstream out;
  // Compact keys keep this snapshot and camera identity inside the existing
  // 512-byte first-fault latch. a=[joint source, joint receipt, query] ages.
  out << " fk{role=" << role << ",seq=" << sequence << ",q=" << query_source_stamp_ns
      << ",r=" << d.now_ros_ns << ",s=" << d.now_steady_ns
      << ",j=" << evidence.source_stamp_ns << ",jr=" << evidence.receipt_steady_ns
      << ",max=" << d.maximum_age_ns << ",bad=" << static_cast<unsigned>(d.stale_mask)
      << ",e=" << d.evaluated << ",a=[" << d.joint_source_age_ns << ','
      << d.joint_receipt_age_ns << ',' << d.query_source_age_ns << ']'
      << ",sdk_r=" << sdk_receipt_ros_ns << ",sdk_s=" << sdk_receipt_steady_ns << '}';
  return out.str();
}

std::string JointKinematicsEvidence::ingress_detail() const {
  const auto& trace = ingress;
  const auto delta = [](int64_t begin, int64_t end) {
    return begin < 0 || end < begin ? int64_t(-1) : end - begin;
  };
  std::ostringstream out;
  out << "latest_joint_source_ns=" << source_stamp_ns
      << " latest_callback_ros_ns=" << receipt_stamp_ns
      << " latest_callback_steady_ns=" << receipt_steady_ns
      << " trace_available=" << trace.available
      << " rmw_source_timestamp_ns=" << trace.rmw_source_timestamp_ns
      << " rmw_source_available=" << (trace.rmw_source_timestamp_ns > 0)
      << " rmw_received_timestamp_ns=" << trace.rmw_received_timestamp_ns
      << " rmw_received_available=" << (trace.rmw_received_timestamp_ns > 0);
  const auto point = [&](const char* name, const JointIngressClockPoint& value) {
    out << ' ' << name << "_steady_ns=" << value.steady_ns
        << ' ' << name << "_thread_cpu_ns=" << value.thread_cpu_ns;
  };
  point("callback_entry", trace.callback_entry);
  point("owner_end", trace.owner_end);
  point("gate_end", trace.gate_end);
  point("ingest_begin", trace.ingest_begin);
  point("ingest_commit", trace.ingest_commit);
  // These owner-side intervals may include scheduling. RMW values above stay
  // raw; no clock-domain subtraction is used to claim DDS-internal latency.
  const auto phase = [&](const char* name, const JointIngressClockPoint& begin,
      const JointIngressClockPoint& end) {
    out << ' ' << name << "_wall_ns=" << delta(begin.steady_ns, end.steady_ns)
        << ' ' << name << "_thread_cpu_ns=" << delta(begin.thread_cpu_ns, end.thread_cpu_ns);
  };
  phase("callback_to_owner", trace.callback_entry, trace.owner_end);
  phase("owner_to_gate", trace.owner_end, trace.gate_end);
  phase("gate_to_ingest", trace.gate_end, trace.ingest_begin);
  out << " fk_wall_ns=" << fk_wall_ns
      << " fk_thread_cpu_ns=" << fk_thread_cpu_ns;
  return out.str();
}

std::string JointKinematicsLookup::ingress_failure_detail(
    const std::string& role, uint64_t sequence) const {
  std::ostringstream out;
  out << "role=" << role << " camera_sequence=" << sequence
      << " query_source_ns=" << query_source_stamp_ns
      << " status=" << static_cast<int>(status) << " reason=" << reason
      << " now_ros_ns=" << age_diagnostics.now_ros_ns
      << " now_steady_ns=" << age_diagnostics.now_steady_ns
      << " maximum_age_ns=" << age_diagnostics.maximum_age_ns
      << " stale_mask=" << static_cast<unsigned>(age_diagnostics.stale_mask)
      << ' ' << evidence.ingress_detail();
  return out.str();
}

JointKinematicsLookup JointKinematics::lookup(
    int64_t source_stamp_ns,const ClockSupplier& clock) const {
  return lookup_impl(source_stamp_ns,clock,false);
}
JointKinematicsLookup JointKinematics::lookup_shared_edges(
    int64_t source_stamp_ns,const ClockSupplier& clock) const {
  return lookup_impl(source_stamp_ns,clock,true);
}
JointKinematicsLookup JointKinematics::lookup_numeric(int64_t source_stamp_ns,const ClockSupplier& clock) const {
  return lookup_impl(source_stamp_ns,clock,true,true);
}
JointKinematicsLookup JointKinematics::lookup_measured_waist(int64_t source_ns,const ClockSupplier& clock) const {
  return lookup_impl(source_ns,clock,false,false,true);
}
geometry_msgs::msg::Transform JointKinematics::neutral_base_from_waist() const {
  for(const auto& edge:impl_->dynamic_edges)if(edge.child=="waist_yaw_link"){
    if(edge.parent!=impl_->options.base_frame)throw std::logic_error("measured_waist_must_be_base_relative");
    // KDL edge from the one parsed and receipted model, evaluated at q=0.
    const auto frame=edge.segment.pose(0.);
    geometry_msgs::msg::Transform out;out.translation.x=frame.p.x();out.translation.y=frame.p.y();out.translation.z=frame.p.z();
    frame.M.GetQuaternion(out.rotation.x,out.rotation.y,out.rotation.z,out.rotation.w);return out;
  }
  throw std::logic_error("measured_waist_edge_missing");
}
JointKinematicsLookup JointKinematics::lookup_impl(
    int64_t source_stamp_ns, const ClockSupplier& clock,bool shared_edges,bool numeric_only,bool waist_only) const {
  if (!clock) throw std::invalid_argument("joint_kinematics_clock_supplier_required");
  const auto& state = *impl_;
  std::shared_lock<std::shared_mutex> lock(state.state_mutex);
  const auto now = clock();
  const auto now_ros_ns = now.ros_ns, now_steady_ns = now.steady_ns;
  JointKinematicsLookup result;
  result.query_source_stamp_ns = source_stamp_ns;
  result.evidence = state.latest;
  auto& ages=result.age_diagnostics;
  ages.now_ros_ns=now_ros_ns; ages.now_steady_ns=now_steady_ns;
  ages.maximum_age_ns=state.options.maximum_age_ns;
  const auto failure = [&](JointKinematicsStatus status, const std::string& reason) {
    result.status = status;
    result.reason = reason;
    result.transforms.clear();
    result.numeric_geometry.reset();result.numeric_geometry_reused=false;
    return result;
  };
  if (state.fault_latched) return failure(JointKinematicsStatus::Fault, state.reason);
  if (!valid_stamp(source_stamp_ns) || !valid_stamp(now_ros_ns) || now_steady_ns <= 0)
    return failure(JointKinematicsStatus::Fault, "joint_kinematics_query_clock_invalid");
  if (!state.input_available || state.history.empty())
    return failure(JointKinematicsStatus::Missing,
                   state.reason.empty() ? "joint_kinematics_no_input" : state.reason);
  ages.evaluated=true;
  ages.joint_source_age_ns=now_ros_ns-state.latest.source_stamp_ns;
  ages.joint_receipt_age_ns=now_steady_ns-state.latest.receipt_steady_ns;
  ages.query_source_age_ns=now_ros_ns-source_stamp_ns;
  if (ages.joint_source_age_ns>state.options.maximum_age_ns)
    ages.stale_mask|=JointKinematicsAgeDiagnostics::JointSource;
  if (ages.joint_receipt_age_ns>state.options.maximum_age_ns)
    ages.stale_mask|=JointKinematicsAgeDiagnostics::JointReceipt;
  if (now_steady_ns < state.latest.receipt_steady_ns ||
      now_ros_ns - state.latest.source_stamp_ns < -state.options.future_tolerance_ns ||
      now_ros_ns - source_stamp_ns < -state.options.future_tolerance_ns)
    return failure(JointKinematicsStatus::Fault, "joint_kinematics_query_clock_regressed");
  if (now_ros_ns - state.latest.source_stamp_ns > state.options.maximum_age_ns ||
      now_steady_ns - state.latest.receipt_steady_ns > state.options.maximum_age_ns)
    return failure(JointKinematicsStatus::Stale, "joint_kinematics_source_or_receipt_stale");
  // Query time selects retained exact-time history; it is not a live joint
  // freshness input. Camera owners enforce their original source/completion
  // deadline, while the independent joint source/receipt watchdog stays above.
  if (source_stamp_ns > state.latest.source_stamp_ns)
    return failure(JointKinematicsStatus::FuturePending, "joint_kinematics_upper_bound_pending");
  if (source_stamp_ns < state.history.front().source_stamp_ns)
    return failure(JointKinematicsStatus::QueryExpired, "joint_kinematics_history_expired");
  try {
    if(waist_only) {
      result.transforms.push_back(state.buffer->lookupTransform(state.options.base_frame,"waist_yaw_link",
        tf2::TimePoint(tf2::Duration(source_stamp_ns))));
    } else if(!shared_edges) {
      for (const auto& target : state.options.target_frames) {
        result.transforms.push_back(state.buffer->lookupTransform(state.options.base_frame, target,
                                     tf2::TimePoint(tf2::Duration(source_stamp_ns))));
      }
    }else{
      const auto upper=std::lower_bound(state.history.begin(),state.history.end(),source_stamp_ns,
        [](const auto& sample,int64_t stamp){return sample.source_stamp_ns<stamp;});
      if(upper==state.history.end())return failure(JointKinematicsStatus::Fault,"joint_kinematics_exact_transform_failed");
      const auto lower=upper->source_stamp_ns==source_stamp_ns?upper:std::prev(upper);
      const double ratio=lower==upper?0.:double(source_stamp_ns-lower->source_stamp_ns)/
        double(upper->source_stamp_ns-lower->source_stamp_ns);
      std::vector<EdgePose> edges;edges.reserve(state.dynamic_edges.size());
      bool canonical=lower->geometry==upper->geometry;
      for(std::size_t i=0;i<state.dynamic_edges.size();++i) {
        const auto& one=lower->edges[i].transform;
        tf2::Vector3 translation(one.translation.x,one.translation.y,one.translation.z);
        tf2::Quaternion rotation(one.rotation.x,one.rotation.y,one.rotation.z,one.rotation.w);
        if(lower!=upper) {
          const auto& two=upper->edges[i].transform;
          translation.setInterpolate3(translation,tf2::Vector3(two.translation.x,two.translation.y,two.translation.z),ratio);
          rotation=tf2::slerp(rotation,tf2::Quaternion(two.rotation.x,two.rotation.y,two.rotation.z,two.rotation.w),ratio);
        }
        edges.push_back({rotation,translation});
        // Preserve original tf2 arithmetic even for equal endpoints: an ULP
        // difference is a distinct geometry, never an epsilon cache hit.
        canonical=canonical&&exact_transform(numeric_transform(edges.back()),one);
      }
      const auto geometry=numeric_only&&canonical?lower->geometry:state.compose(edges);
      if(numeric_only){result.numeric_geometry=geometry;result.numeric_geometry_reused=canonical;}
      else {
        result.transforms.reserve(state.options.target_frames.size());
        for(std::size_t i=0;i<state.options.target_frames.size();++i) {
          geometry_msgs::msg::TransformStamped value;
          value.header.frame_id=state.options.base_frame;value.child_frame_id=state.options.target_frames[i];
          value.header.stamp.sec=static_cast<int32_t>(source_stamp_ns/1000000000LL);
          value.header.stamp.nanosec=static_cast<uint32_t>(source_stamp_ns%1000000000LL);
          value.transform=geometry->transforms[i];result.transforms.push_back(std::move(value));
        }
      }
    }
  } catch (const tf2::TransformException&) {
    // At this point every target must have the same valid dynamic bounds.
    return failure(JointKinematicsStatus::Fault, "joint_kinematics_exact_transform_failed");
  }
  result.status = JointKinematicsStatus::Ready;
  return result;
}

void JointKinematics::mirror_into(std::shared_ptr<tf2::BufferCore> buffer, TransformSink compatibility) {
  const auto& contract=impl_->options;
  const bool fixed_attended=contract.tf_mode=="attended_direct";
  const bool measured=contract.tf_mode=="attended_measured_direct";
  const bool attended=fixed_attended || measured;
  if(attended){
    if(contract.attended_usage!="attended_heading_correction" || contract.root_frame!="base_link" ||
       contract.attended_tf_asset_sha256.empty() || contract.attended_tf_receipt_sha256.empty() ||
       contract.attended_static_edges.size()!=8)
      throw std::logic_error("attended_direct_requires_original_accepted_asset_and_usage");
  }else if(contract.tf_mode!="production_direct" || contract.production_tf_receipt_sha256.empty())
    throw std::logic_error("production_direct_tf_requires_complete_accepted_receipt");
  if(!buffer)throw std::invalid_argument("direct_tf_buffer_missing");
  if(attended){
    // A fixed descendant retained from the URDF must not silently replace an
    // accepted sensor edge. Validate the complete writer set before inserting
    // anything or calling the compatibility sink. These edge sets never change.
    std::set<std::string> fixed_children;
    for(const auto& edge:contract.attended_static_edges)
      if(!fixed_children.insert(edge.child_frame_id).second)
        throw std::logic_error("attended_direct_duplicate_fixed_child");
    for(const auto& edge:impl_->static_edges)
      if(fixed_children.count(edge.child_frame_id))
        throw std::logic_error("attended_direct_urdf_static_child_conflict");
    for(const auto& edge:impl_->dynamic_edges)
      if(edge.child!="waist_yaw_link" && fixed_children.count(edge.child))
        throw std::logic_error("attended_direct_urdf_dynamic_child_conflict");
  }
  std::vector<geometry_msgs::msg::TransformStamped> accepted_fixed;
  if(attended)for(const auto& edge:contract.attended_static_edges){
    // New explicit candidate: canonical owns body residual, this owner owns
    // measured waist. Never insert a competing fixed writer for either child.
    if(measured && (edge.child_frame_id=="base_link" || edge.child_frame_id=="waist_yaw_link"))continue;
    if(!buffer->setTransform(edge,"original_attended_heading_correction_geometry",true))
      throw std::runtime_error("attended_direct_static_edge_rejected");
    accepted_fixed.push_back(edge);
  }
  if(attended && compatibility)compatibility(accepted_fixed,true);
  const std::set<std::string> private_targets(contract.target_frames.begin(),contract.target_frames.end());
  set_transform_sink([buffer,attended,fixed_attended,compatibility,private_targets](const auto& edges,bool fixed){
    for(const auto& edge:edges){
      // Body filtering and spatial classification use the private exact-source
      // FK snapshot. No public sensor/navigation path traverses these targets.
      if(!fixed && private_targets.count(edge.child_frame_id))continue;
      // The original attended owner uses a FIXED q=0 waist. The live source
      // limit is still checked at ingest; it cannot redefine that fixed edge.
      if(fixed_attended && edge.child_frame_id=="waist_yaw_link")continue;
      if(!buffer->setTransform(edge,"validated_in_process_joint_fk",fixed))
        throw std::runtime_error("direct_tf_edge_rejected");
    }
    if(compatibility && !edges.empty()){
      if(fixed && !attended)compatibility(edges,true);
      else if(!fixed_attended){
        // Independent coverage needs the sensor chain, never the leg tree.
        // Production waist remains dynamic; attended waist was seeded fixed.
        std::vector<geometry_msgs::msg::TransformStamped> sensors;
        for(const auto& edge:edges)if(edge.child_frame_id=="waist_yaw_link")sensors.push_back(edge);
        if(!sensors.empty())compatibility(sensors,false);
      }
    }
  });
}

void JointKinematics::set_transform_sink(TransformSink sink) {
  std::lock_guard<std::mutex> producer(impl_->input_mutex);
  std::unique_lock<std::shared_mutex> lock(impl_->state_mutex);
  if (!sink || impl_->input_available || !impl_->history.empty() || impl_->transform_sink)
    throw std::logic_error("joint_transform_sink_requires_unstarted_owner");
  sink(impl_->static_edges, true);
  impl_->transform_sink=std::move(sink);
}

JointKinematicsEvidence JointKinematics::evidence() const {
  std::shared_lock<std::shared_mutex> lock(impl_->state_mutex);
  return impl_->latest;
}

double JointKinematics::origin_radius_bound_from_base(const std::string& frame)const{
  const auto base=impl_->origin_radii.find(impl_->options.base_frame),target=impl_->origin_radii.find(frame);
  const auto inf=std::numeric_limits<double>::infinity();
  if(base==impl_->origin_radii.end() || target==impl_->origin_radii.end())return inf;
  return std::nextafter(base->second+target->second,inf);
}
const JointKinematicsOptions& JointKinematics::options() const noexcept {
  return impl_->options;
}
}  // namespace sunhorizon_perception_runtime
