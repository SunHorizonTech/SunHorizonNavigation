#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#include <atomic>
#include <chrono>
#include <future>
#include <thread>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2/buffer_core.h>
#include <urdf/model.h>
#include "sunhorizon_perception_runtime/joint_kinematics.hpp"
#include "sunhorizon_perception_runtime/measured_waist_geometry.hpp"
#include "sunhorizon_camera_runtime/latest_frame_worker.hpp"

namespace {
using namespace sunhorizon_perception_runtime;
using sunhorizon_platform_adapter_casbot02::LiveJointStateDecision;
constexpr int64_t kStart = 1000000000LL;
constexpr int64_t kStep = 20000000LL;

// Geometry is extracted verbatim from the twelve leg joints and waist in the
// audited casbot02_7dof_shell.urdf (source audit 2026-10-07). No meshes are needed.
// Production generate_robot_a_production_tf_drafts.py calls the waist joint
// "waist_yaw"; the manufacturer source calls it "waist_yaw_joint". The fixed
// nonidentity model_root is an additional test fixture for static path handling.
const char* kRobotAUrdf = R"urdf(
<robot name="robot_a_measured_joint_fixture">
  <link name="model_root" />
  <link name="base_link" />
  <joint name="root_fixed" type="fixed">
    <parent link="model_root" />
    <child link="base_link" />
    <origin xyz="0.3 -0.2 0.1" rpy="0.1 0.2 0.3" />
  </joint>
  <link name="left_leg_pelvic_pitch_link" />
  <joint name="left_leg_pelvic_pitch_joint" type="revolute">
    <origin xyz="0 0.061 0.0575" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="left_leg_pelvic_pitch_link" />
    <axis xyz="0 1 0" />
    <limit lower="-1.9199" upper="1.5708" effort="150" velocity="14" />
  </joint>
  <link name="left_leg_pelvic_roll_link" />
  <joint name="left_leg_pelvic_roll_joint" type="revolute">
    <origin xyz="-0.0028 0.0815 -0.055" rpy="0 0 0" />
    <parent link="left_leg_pelvic_pitch_link" />
    <child link="left_leg_pelvic_roll_link" />
    <axis xyz="1 0 0" />
    <limit lower="-0.17453" upper="1.5708" effort="150" velocity="14" />
  </joint>
  <link name="left_leg_pelvic_yaw_link" />
  <joint name="left_leg_pelvic_yaw_joint" type="revolute">
    <origin xyz="0 0 -0.0975" rpy="0 0 0" />
    <parent link="left_leg_pelvic_roll_link" />
    <child link="left_leg_pelvic_yaw_link" />
    <axis xyz="0 0 1" />
    <limit lower="-1.5708" upper="1.5708" effort="60" velocity="14" />
  </joint>
  <link name="left_leg_knee_pitch_link" />
  <joint name="left_leg_knee_pitch_joint" type="revolute">
    <origin xyz="-0.02 0 -0.2735" rpy="0 0 0" />
    <parent link="left_leg_pelvic_yaw_link" />
    <child link="left_leg_knee_pitch_link" />
    <axis xyz="0 1 0" />
    <limit lower="0" upper="2.5307" effort="150" velocity="14" />
  </joint>
  <link name="left_leg_ankle_pitch_link" />
  <joint name="left_leg_ankle_pitch_joint" type="revolute">
    <origin xyz="0 0 -0.453" rpy="0 0 0" />
    <parent link="left_leg_knee_pitch_link" />
    <child link="left_leg_ankle_pitch_link" />
    <axis xyz="0 1 0" />
    <limit lower="-0.87266" upper="0.50614" effort="60" velocity="14" />
  </joint>
  <link name="left_leg_ankle_roll_link" />
  <joint name="left_leg_ankle_roll_joint" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="left_leg_ankle_pitch_link" />
    <child link="left_leg_ankle_roll_link" />
    <axis xyz="1 0 0" />
    <limit lower="-0.50614" upper="0.50614" effort="60" velocity="14" />
  </joint>
  <link name="right_leg_pelvic_pitch_link" />
  <joint name="right_leg_pelvic_pitch_joint" type="revolute">
    <origin xyz="0 -0.061 0.0575" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="right_leg_pelvic_pitch_link" />
    <axis xyz="0 1 0" />
    <limit lower="-1.9199" upper="1.5708" effort="150" velocity="14" />
  </joint>
  <link name="right_leg_pelvic_roll_link" />
  <joint name="right_leg_pelvic_roll_joint" type="revolute">
    <origin xyz="-0.0028 -0.0815 -0.055" rpy="0 0 0" />
    <parent link="right_leg_pelvic_pitch_link" />
    <child link="right_leg_pelvic_roll_link" />
    <axis xyz="1 0 0" />
    <limit lower="-1.5708" upper="0.17453" effort="150" velocity="14" />
  </joint>
  <link name="right_leg_pelvic_yaw_link" />
  <joint name="right_leg_pelvic_yaw_joint" type="revolute">
    <origin xyz="0 0 -0.0975" rpy="0 0 0" />
    <parent link="right_leg_pelvic_roll_link" />
    <child link="right_leg_pelvic_yaw_link" />
    <axis xyz="0 0 1" />
    <limit lower="-1.5708" upper="1.5708" effort="60" velocity="14" />
  </joint>
  <link name="right_leg_knee_pitch_link" />
  <joint name="right_leg_knee_pitch_joint" type="revolute">
    <origin xyz="-0.02 0 -0.2735" rpy="0 0 0" />
    <parent link="right_leg_pelvic_yaw_link" />
    <child link="right_leg_knee_pitch_link" />
    <axis xyz="0 1 0" />
    <limit lower="0" upper="2.5307" effort="150" velocity="14" />
  </joint>
  <link name="right_leg_ankle_pitch_link" />
  <joint name="right_leg_ankle_pitch_joint" type="revolute">
    <origin xyz="0 0 -0.453" rpy="0 0 0" />
    <parent link="right_leg_knee_pitch_link" />
    <child link="right_leg_ankle_pitch_link" />
    <axis xyz="0 1 0" />
    <limit lower="-0.87266" upper="0.50614" effort="60" velocity="14" />
  </joint>
  <link name="right_leg_ankle_roll_link" />
  <joint name="right_leg_ankle_roll_joint" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="right_leg_ankle_pitch_link" />
    <child link="right_leg_ankle_roll_link" />
    <axis xyz="1 0 0" />
    <limit lower="-0.50614" upper="0.50614" effort="60" velocity="14" />
  </joint>
  <link name="waist_yaw_link" />
  <joint name="waist_yaw" type="revolute">
    <origin xyz="0 0 0.15424" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="waist_yaw_link" />
    <axis xyz="0 0 1" />
    <limit lower="-1.5708" upper="1.5708" effort="60" velocity="14" />
  </joint>
</robot>
)urdf";

JointKinematicsOptions options() {
  JointKinematicsOptions result;
  result.urdf_xml = kRobotAUrdf;
  result.root_frame = "model_root";
  result.maximum_age_ns = 5000000000LL;
  for (const auto& side : {"left", "right"}) {
    for (const auto& joint : {"pelvic_pitch", "pelvic_roll", "pelvic_yaw", "knee_pitch",
                              "ankle_pitch", "ankle_roll"}) {
      const std::string stem = std::string(side) + "_leg_" + joint;
      result.expected_joint_names.push_back(stem + "_joint");
      result.target_frames.push_back(stem + "_link");
    }
  }
  result.expected_joint_names.push_back("waist_yaw");
  return result;
}

LiveJointStateDecision sample(const JointKinematicsOptions& config, int64_t stamp,
                              double phase = 0.0) {
  LiveJointStateDecision value;
  value.accepted = true;
  value.source_stamp_ns = stamp;
  value.names = config.expected_joint_names;
  for (std::size_t i = 0; i < value.names.size(); ++i)
    value.positions.push_back(0.3 * std::sin(phase + 0.4 * static_cast<double>(i)));
  return value;
}

JointKinematicsLookup lookup_at(const JointKinematics& model, int64_t source,
    int64_t ros, int64_t steady) {
  return model.lookup(source,[ros,steady]{return JointKinematics::ClockSample{ros,steady};});
}

// Frozen version of the old transport path: RSP takes each KDL segment.pose(q)
// and publishes that individual joint edge. A separate BufferCore receives the
// edges. This deliberately does not call the new implementation's converters.
void old_rsp_publish(tf2::BufferCore& buffer, const JointKinematicsOptions& config,
                     const LiveJointStateDecision& value) {
  urdf::Model model;
  ASSERT_TRUE(model.initString(config.urdf_xml));
  KDL::Tree tree;
  ASSERT_TRUE(kdl_parser::treeFromUrdfModel(model, tree));
  std::map<std::string, double> positions;
  for (std::size_t i = 0; i < value.names.size(); ++i)
    positions[value.names[i]] = value.positions[i];
  for (const auto& item : tree.getSegments()) {
    if (item.first == config.root_frame) continue;
    const auto& edge = item.second;
    const auto& joint = edge.segment.getJoint();
    const bool fixed = joint.getType() == KDL::Joint::None;
    const auto pose = edge.segment.pose(fixed ? 0.0 : positions.at(joint.getName()));
    geometry_msgs::msg::TransformStamped message;
    message.header.frame_id = edge.parent->first;
    message.child_frame_id = item.first;
    message.header.stamp.sec = static_cast<int32_t>(value.source_stamp_ns / 1000000000LL);
    message.header.stamp.nanosec = static_cast<uint32_t>(value.source_stamp_ns % 1000000000LL);
    message.transform.translation.x = pose.p.x();
    message.transform.translation.y = pose.p.y();
    message.transform.translation.z = pose.p.z();
    pose.M.GetQuaternion(message.transform.rotation.x, message.transform.rotation.y,
                         message.transform.rotation.z, message.transform.rotation.w);
    ASSERT_TRUE(buffer.setTransform(message, "old_rsp_tf_transport", fixed));
  }
}

void equal_transform(const geometry_msgs::msg::TransformStamped& actual,
                     const geometry_msgs::msg::TransformStamped& expected) {
  EXPECT_EQ(actual.header.frame_id, expected.header.frame_id);
  EXPECT_EQ(actual.child_frame_id, expected.child_frame_id);
  EXPECT_EQ(actual.header.stamp.sec, expected.header.stamp.sec);
  EXPECT_EQ(actual.header.stamp.nanosec, expected.header.stamp.nanosec);
  EXPECT_NEAR(actual.transform.translation.x, expected.transform.translation.x, 1e-12);
  EXPECT_NEAR(actual.transform.translation.y, expected.transform.translation.y, 1e-12);
  EXPECT_NEAR(actual.transform.translation.z, expected.transform.translation.z, 1e-12);
  EXPECT_NEAR(actual.transform.rotation.x, expected.transform.rotation.x, 1e-12);
  EXPECT_NEAR(actual.transform.rotation.y, expected.transform.rotation.y, 1e-12);
  EXPECT_NEAR(actual.transform.rotation.z, expected.transform.rotation.z, 1e-12);
  EXPECT_NEAR(actual.transform.rotation.w, expected.transform.rotation.w, 1e-12);
}

TEST(JointKinematics, RealRobotAEdgesMatchOldRspTfAtExactAndInterpolatedSourceTimes) {
  const auto config = options();
  JointKinematics model(config);
  tf2::BufferCore oracle(tf2::Duration(config.cache_time_ns));
  EXPECT_TRUE(std::isinf(model.origin_radius_bound_from_base("unmodelled_sensor")));
  for (int i = 0; i < 20; ++i) {
    const auto value = sample(config, kStart + i * kStep, 0.17 * i);
    ASSERT_TRUE(model.ingest(value, value.source_stamp_ns + 1000, 100 + i * kStep).accepted);
    old_rsp_publish(oracle, config, value);
  }
  for (int i = 0; i < 39; ++i) {
    const int64_t query = kStart + i * kStep / 2;
    const auto actual = lookup_at(model,query, kStart + 19 * kStep + 1000, 100 + 19 * kStep);
    ASSERT_EQ(actual.status, JointKinematicsStatus::Ready) << actual.reason;
    ASSERT_EQ(actual.transforms.size(), 12u);
    EXPECT_EQ(actual.query_source_stamp_ns, query);
    for (std::size_t j = 0; j < config.target_frames.size(); ++j) {
      equal_transform(actual.transforms[j], oracle.lookupTransform(config.base_frame,
          config.target_frames[j], tf2::TimePoint(tf2::Duration(query))));
      const auto& p=actual.transforms[j].transform.translation;
      EXPECT_LE(std::hypot(std::hypot(p.x,p.y),p.z),model.origin_radius_bound_from_base(config.target_frames[j]));
    }
  }
}

TEST(JointKinematics, SharedEdgesMatchTf2AtDistinctNanosecondTimesAndQuaternionWrap) {
  auto config=options();std::reverse(config.target_frames.begin(),config.target_frames.end());
  JointKinematics model(config);tf2::BufferCore oracle(tf2::Duration(config.cache_time_ns));
  constexpr int64_t start=1792126250000000000LL,interval=17000001LL;
  for(int i=0;i<31;++i){
    auto value=sample(config,start+i*interval,.17*i);
    value.positions[0]=i%2?3.12:-3.12; // shortest quaternion arc, not angle lerp
    ASSERT_TRUE(model.ingest(value,value.source_stamp_ns+1000,100+i*interval).accepted);
    old_rsp_publish(oracle,config,value);
  }
  const auto now=start+30*interval+1000;const int64_t steady=100+30*interval;
  const auto clock=[&]{return JointKinematics::ClockSample{now,steady};};
  // Every timestamp is evaluated independently, including exact endpoints and
  // nearby ns values which must not collapse into a floating-point time bin.
  for(int i=0;i<=200;++i){
    const auto query=start+(30*interval*i)/200;
    const auto shared=model.lookup_shared_edges(query,clock);
    ASSERT_EQ(shared.status,JointKinematicsStatus::Ready)<<shared.reason;
    ASSERT_EQ(shared.transforms.size(),12U);EXPECT_EQ(shared.query_source_stamp_ns,query);
    for(std::size_t j=0;j<12;++j)equal_transform(shared.transforms[j],oracle.lookupTransform(
      config.base_frame,config.target_frames[j],tf2::TimePoint(tf2::Duration(query))));
  }
  for(const auto query:{start-1,start+30*interval+1}){
    const auto legacy=model.lookup(query,clock),shared=model.lookup_shared_edges(query,clock);
    EXPECT_EQ(shared.status,legacy.status);EXPECT_EQ(shared.reason,legacy.reason);EXPECT_TRUE(shared.transforms.empty());
  }
  const auto stale_clock=[&]{return JointKinematics::ClockSample{now+config.maximum_age_ns+1,steady};};
  const auto legacy=model.lookup(start,stale_clock),shared=model.lookup_shared_edges(start,stale_clock);
  EXPECT_EQ(shared.status,legacy.status);EXPECT_EQ(shared.reason,legacy.reason);EXPECT_TRUE(shared.transforms.empty());
}

TEST(JointKinematics, DirectCacheReceivesExactEdgesWithoutRosRoundTrip) {
  auto config=options();
  const auto end=config.urdf_xml.find("</robot>");
  config.urdf_xml.insert(end,R"(<link name="optical"/><joint name="camera_fixed" type="fixed"><parent link="waist_yaw_link"/><child link="optical"/><origin xyz="0.1 0.2 0.3" rpy="0.2 0.1 0.4"/></joint>)");
  JointKinematics model(config);
  tf2::BufferCore direct(tf2::Duration(config.cache_time_ns)),oracle(tf2::Duration(config.cache_time_ns));
  model.set_transform_sink([&](const auto& edges,bool fixed){
    for(const auto& edge:edges) ASSERT_TRUE(direct.setTransform(edge,"direct",fixed));
  });
  for(int i=0;i<3;++i){
    auto input=sample(config,kStart+i*kStep,.1*i);
    ASSERT_TRUE(model.ingest(input,input.source_stamp_ns+1000,100+i*kStep).accepted);
    old_rsp_publish(oracle,config,input);
  }
  const auto time=tf2::TimePoint(tf2::Duration(kStart+kStep/2));
  equal_transform(direct.lookupTransform(config.base_frame,"optical",time),
                  oracle.lookupTransform(config.base_frame,"optical",time));
  EXPECT_THROW(model.set_transform_sink([](const auto&,bool){}),std::logic_error);
}

TEST(JointKinematics, InterpolatesEveryEdgeNotAnglesOrComposedFootPose) {
  const auto config = options();
  JointKinematics model(config);
  auto first = sample(config, kStart);
  auto second = sample(config, kStart + kStep);
  first.positions[0] = 3.1;
  second.positions[0] = -3.1;
  first.positions[1] = -0.5;
  second.positions[1] = 0.7;
  ASSERT_TRUE(model.ingest(first, first.source_stamp_ns, 100).accepted);
  ASSERT_TRUE(model.ingest(second, second.source_stamp_ns, 100 + kStep).accepted);
  const auto mid = lookup_at(model,kStart + kStep / 2, second.source_stamp_ns, 100 + kStep);
  ASSERT_EQ(mid.status, JointKinematicsStatus::Ready);
  // Angle interpolation would yield hip pitch zero; tf2 shortest-arc rotation
  // interpolation yields pi, which has a quaternion y component near one.
  EXPECT_GT(std::abs(mid.transforms[0].transform.rotation.y), 0.999);
  tf2::BufferCore composed(tf2::Duration(config.cache_time_ns));
  for (const auto* value : {&first, &second}) {
    const auto exact = lookup_at(model,value->source_stamp_ns, second.source_stamp_ns, 100 + kStep);
    ASSERT_EQ(exact.status, JointKinematicsStatus::Ready);
    ASSERT_TRUE(composed.setTransform(exact.transforms[5], "composed_foot_shortcut", false));
  }
  const auto shortcut = composed.lookupTransform(config.base_frame, config.target_frames[5],
      tf2::TimePoint(tf2::Duration(kStart + kStep / 2)));
  const auto& good = mid.transforms[5].transform.translation;
  const auto& bad = shortcut.transform.translation;
  EXPECT_GT(std::abs(good.x-bad.x) + std::abs(good.y-bad.y) + std::abs(good.z-bad.z), 1e-3);
}

TEST(JointKinematics, HardHistoryBoundRolloverRetainsExactTfInterpolation) {
  auto config = options();
  config.max_history_samples = 4;
  JointKinematics model(config);
  tf2::BufferCore oracle(tf2::Duration(config.cache_time_ns));
  for (int i = 0; i < 24; ++i) {
    const auto value = sample(config, kStart + i * kStep, 0.17 * i);
    ASSERT_TRUE(model.ingest(value, value.source_stamp_ns, 100 + i * kStep).accepted);
    old_rsp_publish(oracle, config, value);
    EXPECT_LE(model.evidence().history_samples, 4u);
    if (i == 0) continue;
    const auto actual = lookup_at(model,value.source_stamp_ns - kStep / 2,
                                    value.source_stamp_ns, 100 + i * kStep);
    ASSERT_EQ(actual.status, JointKinematicsStatus::Ready) << actual.reason;
    for (std::size_t j = 0; j < config.target_frames.size(); ++j)
      equal_transform(actual.transforms[j], oracle.lookupTransform(config.base_frame,
          config.target_frames[j], tf2::TimePoint(tf2::Duration(value.source_stamp_ns-kStep/2))));
  }
  EXPECT_EQ(lookup_at(model,kStart, kStart + 23*kStep, 100 + 23*kStep).status,
            JointKinematicsStatus::QueryExpired);
}

TEST(JointKinematics, StaticEdgesInsideTargetPathMatchOldRspAndSurviveRollover) {
  auto config = options();
  config.max_history_samples = 2;
  config.urdf_xml.insert(config.urdf_xml.find("</robot>"),
      "<link name=\"left_foot_collision\"/>"
      "<joint name=\"left_foot_fixed\" type=\"fixed\">"
      "<parent link=\"left_leg_ankle_roll_link\"/><child link=\"left_foot_collision\"/>"
      "<origin xyz=\"0.04 -0.02 -0.08\" rpy=\"0.2 -0.1 0.3\"/></joint>");
  config.target_frames[5] = "left_foot_collision";
  JointKinematics model(config);
  tf2::BufferCore oracle(tf2::Duration(config.cache_time_ns));
  for (int i = 0; i < 8; ++i) {
    const auto value = sample(config, kStart + i*kStep, 0.2*i);
    ASSERT_TRUE(model.ingest(value, value.source_stamp_ns, 100+i*kStep).accepted);
    old_rsp_publish(oracle, config, value);
    const int64_t query = value.source_stamp_ns - (i == 0 ? 0 : kStep/2);
    const auto actual = lookup_at(model,query, value.source_stamp_ns, 100+i*kStep);
    ASSERT_EQ(actual.status, JointKinematicsStatus::Ready);
    equal_transform(actual.transforms[5], oracle.lookupTransform(config.base_frame,
        config.target_frames[5], tf2::TimePoint(tf2::Duration(query))));
  }
}

TEST(JointKinematics, CacheTimeExpiryIsStaleAndFutureRequiresFreshSourceAndReceipt) {
  auto config = options();
  config.cache_time_ns = 2*kStep;
  config.maximum_age_ns = 3*kStep;
  JointKinematics model(config);
  EXPECT_EQ(lookup_at(model,kStart, kStart, 100).status, JointKinematicsStatus::Missing);
  for (int i = 0; i < 6; ++i) {
    const auto value = sample(config, kStart + i*kStep);
    ASSERT_TRUE(model.ingest(value, value.source_stamp_ns, 100 + i*kStep).accepted);
  }
  EXPECT_EQ(lookup_at(model,kStart, kStart+5*kStep, 100+5*kStep).status, JointKinematicsStatus::QueryExpired);
  const auto waiting = lookup_at(model,kStart+5*kStep+1, kStart+5*kStep+1, 100+5*kStep+1);
  EXPECT_EQ(waiting.status, JointKinematicsStatus::FuturePending);
  EXPECT_TRUE(waiting.transforms.empty());
  EXPECT_EQ(waiting.query_source_stamp_ns, kStart+5*kStep+1);
  EXPECT_EQ(waiting.evidence.source_stamp_ns, kStart+5*kStep);
  EXPECT_EQ(lookup_at(model,kStart+5*kStep+1, kStart+9*kStep, 100+9*kStep).status,
            JointKinematicsStatus::Stale);
  EXPECT_EQ(lookup_at(model,kStart+5*kStep, kStart+5*kStep, 100+9*kStep).status,
            JointKinematicsStatus::Stale);
}

TEST(JointKinematics, LateRejectedAndOutOfOrderInputsNeverRefreshEvidence) {
  const auto config = options();
  JointKinematics model(config);
  const auto first = sample(config, kStart);
  ASSERT_TRUE(model.ingest(first, kStart+10, 100).accepted);
  const auto old = model.ingest(first, kStart+kStep, 100+kStep);
  EXPECT_FALSE(old.accepted);
  EXPECT_EQ(old.evidence.source_stamp_ns, kStart);
  EXPECT_EQ(old.evidence.receipt_stamp_ns, kStart+10);
  EXPECT_EQ(old.evidence.receipt_steady_ns, 100);
  EXPECT_EQ(lookup_at(model,kStart, kStart+kStep, 100+kStep).status, JointKinematicsStatus::Missing);
  auto fresh = sample(config, kStart+2*kStep);
  ASSERT_TRUE(model.ingest(fresh, fresh.source_stamp_ns, 100+2*kStep).accepted);
  fresh.accepted = false;
  fresh.reason = "source_owner_not_unique";
  EXPECT_FALSE(model.ingest(fresh, kStart+3*kStep, 100+3*kStep).accepted);
  EXPECT_EQ(model.evidence().receipt_steady_ns, 100+2*kStep);
  EXPECT_EQ(lookup_at(model,kStart+2*kStep, kStart+3*kStep, 100+3*kStep).status,
            JointKinematicsStatus::Missing);
}

TEST(JointKinematics, StructuralAndNonfiniteFailuresLatchUntilReconstruction) {
  const auto config = options();
  for (int mode = 0; mode < 4; ++mode) {
    JointKinematics model(config);
    auto value = sample(config, kStart);
    if (mode == 0) value.positions[1] = std::numeric_limits<double>::quiet_NaN();
    if (mode == 1) value.names[1] = "other_joint";
    if (mode == 2) value.positions.pop_back();
    if (mode == 3) {value.accepted = false; value.fault_latched = true;}
    EXPECT_TRUE(model.ingest(value, kStart, 100).fault_latched);
    EXPECT_FALSE(model.ingest(sample(config, kStart+kStep), kStart+kStep, 100+kStep).accepted);
    const auto result = lookup_at(model,kStart, kStart+kStep, 100+kStep);
    EXPECT_EQ(result.status, JointKinematicsStatus::Fault);
    EXPECT_TRUE(result.transforms.empty());
  }
}

TEST(JointKinematics, InvalidAndRegressedClocksCannotProduceOrRefreshEvidence) {
  const auto config = options();
  JointKinematics model(config);
  ASSERT_TRUE(model.ingest(sample(config, kStart), kStart+10, 100).accepted);
  EXPECT_EQ(lookup_at(model,0, kStart+10, 100).status, JointKinematicsStatus::Fault);
  EXPECT_EQ(lookup_at(model,kStart, kStart+10, 99).status, JointKinematicsStatus::Fault);
  EXPECT_EQ(model.evidence().receipt_steady_ns, 100);
  const auto regression = model.ingest(sample(config, kStart+kStep), kStart+kStep, 99);
  EXPECT_TRUE(regression.fault_latched);
  EXPECT_EQ(regression.evidence.source_stamp_ns, kStart);
  EXPECT_EQ(regression.evidence.receipt_stamp_ns, kStart+10);
  EXPECT_EQ(regression.evidence.receipt_steady_ns, 100);
  EXPECT_EQ(lookup_at(model,kStart, kStart+kStep, 100+kStep).status, JointKinematicsStatus::Fault);
}

TEST(JointKinematics, QuerySamplesClocksOnlyAfterBlockedIngestCommitsItsSnapshot) {
  const auto config=options();JointKinematics model(config);
  std::promise<void> writer_locked,release_writer,query_started,clock_sampled;
  auto writer_released=release_writer.get_future().share();
  auto writer_entered=writer_locked.get_future(),sampled=clock_sampled.get_future();
  std::atomic<bool> pause_writer{false};
  std::atomic<int64_t> ros{kStart+10},steady{100};
  model.set_transform_sink([&](const auto&,bool fixed){
    if(!fixed && pause_writer.exchange(false)){
      // This existing hook runs inside ingest's unique state lock, before the
      // newer latest receipt is committed. No test-only production lock hook.
      writer_locked.set_value();writer_released.wait();
      ros.store(kStart+kStep+10);steady.store(100+kStep);
    }
  });
  ASSERT_TRUE(model.ingest(sample(config,kStart),kStart+10,100).accepted);
  pause_writer.store(true);
  auto writer=std::async(std::launch::async,[&]{return model.ingest(
    sample(config,kStart+kStep),kStart+kStep+10,100+kStep);});
  EXPECT_EQ(writer_entered.wait_for(std::chrono::seconds(2)),std::future_status::ready);
  auto query=std::async(std::launch::async,[&]{
    query_started.set_value();
    return model.lookup(kStart+kStep,[&]{clock_sampled.set_value();
      return JointKinematics::ClockSample{ros.load(),steady.load()};});
  });
  query_started.get_future().wait();
  // Sampling before acquiring state_mutex would signal here and read the OLD
  // clock, then falsely reject the newer receipt after this writer commits.
  EXPECT_EQ(sampled.wait_for(std::chrono::milliseconds(30)),std::future_status::timeout);
  release_writer.set_value();
  const auto input=writer.get();const auto result=query.get();
  ASSERT_TRUE(input.accepted);ASSERT_EQ(result.status,JointKinematicsStatus::Ready)<<result.reason;
  EXPECT_EQ(result.evidence.source_stamp_ns,kStart+kStep);
  EXPECT_EQ(result.evidence.receipt_steady_ns,100+kStep);
  EXPECT_EQ(result.age_diagnostics.now_ros_ns,kStart+kStep+10);
  EXPECT_EQ(result.age_diagnostics.now_steady_ns,100+kStep);
  EXPECT_EQ(result.age_diagnostics.joint_source_age_ns,10);
  EXPECT_EQ(result.age_diagnostics.joint_receipt_age_ns,0);
  EXPECT_EQ(sampled.wait_for(std::chrono::seconds(0)),std::future_status::ready);
  // Fixed historical clocks remain invalid against that same new snapshot:
  // the fix changes sampling order, never clamps a regressed clock to receipt.
  const auto regressed=lookup_at(model,kStart+kStep,kStart+10,100);
  EXPECT_EQ(regressed.status,JointKinematicsStatus::Fault);
  EXPECT_EQ(regressed.reason,"joint_kinematics_query_clock_regressed");
}

TEST(JointKinematics, IngestCannotChangeSnapshotDuringClockSamplingAndLookup) {
  const auto config=options();JointKinematics model(config);
  ASSERT_TRUE(model.ingest(sample(config,kStart),kStart+10,100).accepted);
  std::promise<void> sampling,release_query,writer_started;
  auto released=release_query.get_future().share();
  auto query=std::async(std::launch::async,[&]{return model.lookup(kStart,[&]{
    sampling.set_value();released.wait();return JointKinematics::ClockSample{kStart+10,100};
  });});
  sampling.get_future().wait();
  auto writer=std::async(std::launch::async,[&]{writer_started.set_value();
    return model.ingest(sample(config,kStart+kStep),kStart+kStep+10,100+kStep);});
  writer_started.get_future().wait();
  EXPECT_EQ(writer.wait_for(std::chrono::milliseconds(30)),std::future_status::timeout);
  release_query.set_value();
  const auto result=query.get();const auto input=writer.get();
  ASSERT_EQ(result.status,JointKinematicsStatus::Ready)<<result.reason;
  EXPECT_EQ(result.evidence.source_stamp_ns,kStart);
  EXPECT_EQ(result.evidence.receipt_steady_ns,100);
  ASSERT_TRUE(input.accepted);
  EXPECT_EQ(model.evidence().source_stamp_ns,kStart+kStep);
}

TEST(JointKinematics, InjectedClocksKeepRealRollbackFutureAndAgeBoundaries) {
  const auto config=options();JointKinematics model(config);
  ASSERT_TRUE(model.ingest(sample(config,kStart),kStart+10,100).accepted);
  EXPECT_EQ(lookup_at(model,kStart,kStart+10,99).status,JointKinematicsStatus::Fault);
  EXPECT_EQ(lookup_at(model,kStart,kStart-config.future_tolerance_ns-1,100).status,JointKinematicsStatus::Fault);
  EXPECT_EQ(lookup_at(model,kStart+config.future_tolerance_ns+11,kStart+10,100).status,JointKinematicsStatus::Fault);
  EXPECT_EQ(lookup_at(model,kStart,kStart+config.maximum_age_ns+1,100).status,JointKinematicsStatus::Stale);
  EXPECT_EQ(lookup_at(model,kStart,kStart+10,100+config.maximum_age_ns+1).status,JointKinematicsStatus::Stale);
  EXPECT_EQ(lookup_at(model,kStart,kStart+config.maximum_age_ns,100+config.maximum_age_ns).status,JointKinematicsStatus::Ready);
  EXPECT_EQ(lookup_at(model,kStart,kStart+10,100).status,JointKinematicsStatus::Ready);
  EXPECT_EQ(model.evidence().receipt_stamp_ns,kStart+10);
  EXPECT_EQ(model.evidence().receipt_steady_ns,100);
  EXPECT_THROW(model.lookup(kStart,JointKinematics::ClockSupplier{}),std::invalid_argument);
}

TEST(JointKinematics, StaleDiagnosticsDistinguishLiveJointAgePredicates) {
  auto config=options();config.maximum_age_ns=250000000LL;
  JointKinematics model(config);
  ASSERT_TRUE(model.ingest(sample(config,kStart),kStart,100).accepted);
  const auto budget=config.maximum_age_ns;
  struct Query {int64_t source,ros,steady;uint8_t mask;};
  const Query cases[]{
    {kStart+budget+1,kStart+budget+1,101,JointKinematicsAgeDiagnostics::JointSource},
    {kStart,kStart+10,100+budget+1,JointKinematicsAgeDiagnostics::JointReceipt},
  };
  for(const auto& query:cases) {
    SCOPED_TRACE(static_cast<unsigned>(query.mask));
    const auto result=lookup_at(model,query.source,query.ros,query.steady);
    EXPECT_EQ(result.status,JointKinematicsStatus::Stale);
    EXPECT_EQ(result.reason,"joint_kinematics_source_or_receipt_stale");
    EXPECT_TRUE(result.transforms.empty());EXPECT_EQ(result.query_source_stamp_ns,query.source);
    const auto& d=result.age_diagnostics;
    EXPECT_TRUE(d.evaluated);EXPECT_EQ(d.stale_mask,query.mask);
    EXPECT_EQ(d.now_ros_ns,query.ros);EXPECT_EQ(d.now_steady_ns,query.steady);
    EXPECT_EQ(d.maximum_age_ns,budget);
    EXPECT_EQ(d.joint_source_age_ns,query.ros-kStart);
    EXPECT_EQ(d.joint_receipt_age_ns,query.steady-100);
    EXPECT_EQ(d.query_source_age_ns,query.ros-query.source);
    EXPECT_EQ(result.evidence.source_stamp_ns,kStart);EXPECT_EQ(result.evidence.receipt_steady_ns,100);
    EXPECT_EQ(model.evidence().receipt_steady_ns,100); // A diagnostic lookup never renews input.
  }
  const auto boundary=lookup_at(model,kStart,kStart+budget,100+budget);
  EXPECT_EQ(boundary.status,JointKinematicsStatus::Ready);EXPECT_TRUE(boundary.reason.empty());
  EXPECT_EQ(boundary.age_diagnostics.stale_mask,0);EXPECT_TRUE(boundary.age_diagnostics.evaluated);
  const auto all=lookup_at(model,kStart-budget-1,kStart+budget+1,100+budget+1);
  EXPECT_EQ(all.status,JointKinematicsStatus::Stale);EXPECT_EQ(all.age_diagnostics.stale_mask,3);
}

TEST(JointKinematics, Run72CameraAgeUsesRetainedHistoryAndOwnerDeadline) {
  auto config=options();config.maximum_age_ns=250000000LL;
  JointKinematics model(config);
  // Relative timing is the run72 frozen failure snapshot: query age 278.414ms,
  // joint source 14.390ms and joint receipt 14.112ms. Receipt ROS is a fixture.
  constexpr int64_t query=1792460250459588000LL,now=1792460250738001530LL;
  constexpr int64_t steady=85369383201635LL,joint=1792460250723611407LL;
  constexpr int64_t joint_steady=85369369089213LL;
  ASSERT_TRUE(model.ingest(sample(config,query),query+1,joint_steady-(joint-query)).accepted);
  ASSERT_TRUE(model.ingest(sample(config,joint),joint+1,joint_steady).accepted);
  const auto original=model.evidence();
  const auto historical=lookup_at(model,query,now,steady);
  ASSERT_EQ(historical.status,JointKinematicsStatus::Ready);
  EXPECT_EQ(historical.age_diagnostics.stale_mask,0);
  EXPECT_EQ(historical.age_diagnostics.joint_source_age_ns,14390123LL);
  EXPECT_EQ(historical.age_diagnostics.joint_receipt_age_ns,14112422LL);
  EXPECT_EQ(historical.age_diagnostics.query_source_age_ns,278413530LL);
  EXPECT_EQ(historical.transforms.size(),config.target_frames.size());
  const auto missing=lookup_at(model,query-1,now,steady);
  EXPECT_EQ(missing.status,JointKinematicsStatus::QueryExpired);
  EXPECT_EQ(missing.reason,"joint_kinematics_history_expired");
  EXPECT_TRUE(missing.transforms.empty());

  using Task=std::packaged_task<JointFrameDisposition()>;
  std::atomic_bool terminal{false};std::atomic<int> completed{0};
  sunhorizon_camera_runtime::LatestFrameWorker<Task> worker(
    [](Task& task){task();},[&]{return !terminal.load();},[&](auto){terminal.store(true);});
  const auto run=[&](int64_t source,int64_t ros,int64_t monotonic,bool source_fresh=true){
    auto task=std::make_unique<Task>([&,source,ros,monotonic,source_fresh]{
      const auto result=lookup_at(model,source,ros,monotonic);
      const auto disposition=joint_frame_disposition(result.status,source_fresh,true);
      if(disposition==JointFrameDisposition::InternalFault){terminal.store(true);throw std::runtime_error(result.reason);}
      if(disposition==JointFrameDisposition::Ready)++completed;
      return disposition;
    });
    auto result=task->get_future();worker.replace(std::move(task));return result;
  };
  EXPECT_EQ(run(query-1,now,steady).get(),JointFrameDisposition::Unavailable);
  EXPECT_FALSE(terminal.load());EXPECT_EQ(completed.load(),0);
  EXPECT_EQ(model.evidence().source_stamp_ns,original.source_stamp_ns);
  EXPECT_EQ(model.evidence().receipt_steady_ns,original.receipt_steady_ns);
  EXPECT_EQ(run(query,now,steady).get(),JointFrameDisposition::Ready);
  EXPECT_EQ(completed.load(),1); // 278ms camera query has exact-time history.
  // The owner still denies an expired camera even with fresh live joints and
  // retained history; no lookup result can renew its original frame deadline.
  EXPECT_EQ(run(query,now+30000000LL,steady+30000000LL,false).get(),JointFrameDisposition::Unavailable);
  EXPECT_EQ(completed.load(),1);

  const auto expired_ros=joint+config.maximum_age_ns+1;
  const auto expired_steady=joint_steady+config.maximum_age_ns+1;
  EXPECT_EQ(run(expired_ros,expired_ros,expired_steady).get(),JointFrameDisposition::Unavailable);
  EXPECT_FALSE(terminal.load());EXPECT_EQ(completed.load(),1);
  EXPECT_EQ(model.evidence().receipt_steady_ns,original.receipt_steady_ns);
  const auto next=expired_ros+kStep,next_steady=expired_steady+kStep;
  ASSERT_TRUE(model.ingest(sample(config,next),next,next_steady).accepted);
  EXPECT_EQ(run(next,next,next_steady).get(),JointFrameDisposition::Ready);
  EXPECT_EQ(completed.load(),2); // Actual new input, never renewed old evidence.
  EXPECT_THROW(run(next,next,next_steady-1).get(),std::runtime_error);
  EXPECT_TRUE(terminal.load());EXPECT_EQ(completed.load(),2);
  worker.request_stop();worker.join();
}

TEST(JointKinematics, FrameDispositionPreservesFaultsAndBoundsPendingQueries) {
  for(const bool qualified:{false,true}) {
    EXPECT_EQ(joint_frame_disposition(JointKinematicsStatus::Fault,false,qualified),JointFrameDisposition::InternalFault);
    EXPECT_EQ(joint_frame_disposition(JointKinematicsStatus::Stale,true,qualified),JointFrameDisposition::Unavailable);
    EXPECT_EQ(joint_frame_disposition(JointKinematicsStatus::QueryExpired,true,qualified),JointFrameDisposition::Unavailable);
    EXPECT_EQ(joint_frame_disposition(JointKinematicsStatus::FuturePending,true,qualified),JointFrameDisposition::Wait);
    EXPECT_EQ(joint_frame_disposition(JointKinematicsStatus::FuturePending,false,qualified),JointFrameDisposition::Unavailable);
    EXPECT_EQ(joint_frame_disposition(JointKinematicsStatus::Ready,false,qualified),JointFrameDisposition::Unavailable);
  }
  EXPECT_EQ(joint_frame_disposition(JointKinematicsStatus::Missing,true,false),JointFrameDisposition::Wait);
  EXPECT_EQ(joint_frame_disposition(JointKinematicsStatus::Missing,false,true),JointFrameDisposition::InternalFault);
}

TEST(JointKinematics, UnevaluatedDiagnosticsNeverPretendToMeasureInputAge) {
  const auto config=options();JointKinematics model(config);
  const auto missing=lookup_at(model,kStart,kStart+10,100);
  EXPECT_EQ(missing.status,JointKinematicsStatus::Missing);
  EXPECT_FALSE(missing.age_diagnostics.evaluated);EXPECT_EQ(missing.age_diagnostics.joint_source_age_ns,-1);
  EXPECT_EQ(missing.age_diagnostics.now_ros_ns,kStart+10);EXPECT_EQ(missing.age_diagnostics.now_steady_ns,100);
  ASSERT_TRUE(model.ingest(sample(config,kStart),kStart,100).accepted);
  const auto invalid=lookup_at(model,0,kStart+10,100);
  EXPECT_EQ(invalid.status,JointKinematicsStatus::Fault);EXPECT_EQ(invalid.reason,"joint_kinematics_query_clock_invalid");
  EXPECT_FALSE(invalid.age_diagnostics.evaluated);EXPECT_EQ(invalid.age_diagnostics.query_source_age_ns,-1);
}

TEST(JointKinematics, FailureDetailPreservesCameraAndLockedSnapshotWithinFirstFaultLimit) {
  auto config=options();config.maximum_age_ns=250000000LL;JointKinematics model(config);
  ASSERT_TRUE(model.ingest(sample(config,kStart),kStart,100).accepted);
  const auto query=kStart-config.maximum_age_ns-1;
  const auto result=lookup_at(model,query,kStart,101);
  const auto detail=result.failure_detail("chest",760,123,456);
  EXPECT_NE(detail.find("role=chest,seq=760,q="+std::to_string(query)),std::string::npos);
  EXPECT_NE(detail.find(",r="+std::to_string(kStart)+",s=101,j="+std::to_string(kStart)+",jr=100"),std::string::npos);
  EXPECT_NE(detail.find(",max=250000000,bad=0,e=1,a=[0,1,250000001]"),std::string::npos);
  EXPECT_NE(detail.find(",sdk_r=123,sdk_s=456}"),std::string::npos);
  // Formatting must not read newer mutable evidence after the failed lookup.
  ASSERT_TRUE(model.ingest(sample(config,kStart+kStep),kStart+kStep,100+kStep).accepted);
  EXPECT_EQ(result.failure_detail("chest",760,123,456),detail);
  auto maximum=result;
  maximum.query_source_stamp_ns=std::numeric_limits<int64_t>::max();
  maximum.evidence.source_stamp_ns=maximum.evidence.receipt_steady_ns=std::numeric_limits<int64_t>::max();
  auto& d=maximum.age_diagnostics;
  d.now_ros_ns=d.now_steady_ns=d.maximum_age_ns=std::numeric_limits<int64_t>::max();
  d.joint_source_age_ns=d.joint_receipt_age_ns=d.query_source_age_ns=std::numeric_limits<int64_t>::max();
  const auto fault="qualified_joint_history_lost:"+result.reason+
    maximum.failure_detail("chest",std::numeric_limits<uint64_t>::max(),
      std::numeric_limits<int64_t>::max(),std::numeric_limits<int64_t>::max());
  EXPECT_LE(fault.size(),512U);EXPECT_EQ(fault.substr(0,512),fault);
}

TEST(JointKinematics, AcceptedIngressTraceIsPairedWithItsSourceAndRejectedTraceCannotReplaceIt) {
  const auto config = options();
  JointKinematics model(config);
  JointIngressTrace trace;
  trace.available = true;
  trace.rmw_source_timestamp_ns = kStart + 2;
  trace.rmw_received_timestamp_ns = kStart + 3;
  trace.callback_entry = {100, 1000};
  trace.owner_end = {110, 1010};
  trace.gate_end = {120, 1015};
  const auto accepted = model.ingest(sample(config, kStart), kStart + 20, 100, trace);
  ASSERT_TRUE(accepted.accepted);
  const auto& stored = accepted.evidence.ingress;
  EXPECT_TRUE(stored.available);
  EXPECT_EQ(accepted.evidence.source_stamp_ns, kStart);
  EXPECT_EQ(accepted.evidence.receipt_stamp_ns, kStart + 20);
  EXPECT_EQ(stored.rmw_source_timestamp_ns, kStart + 2);
  EXPECT_EQ(stored.rmw_received_timestamp_ns, kStart + 3);
  EXPECT_EQ(stored.callback_entry.steady_ns, 100);
  EXPECT_EQ(stored.callback_entry.thread_cpu_ns, 1000);
  EXPECT_EQ(stored.owner_end.steady_ns, 110);
  EXPECT_EQ(stored.gate_end.thread_cpu_ns, 1015);
  EXPECT_GT(stored.ingest_begin.steady_ns, 0);
  EXPECT_EQ(stored.ingest_commit.steady_ns - stored.ingest_begin.steady_ns,
            accepted.evidence.fk_wall_ns);
  EXPECT_EQ(stored.ingest_commit.thread_cpu_ns - stored.ingest_begin.thread_cpu_ns,
            accepted.evidence.fk_thread_cpu_ns);

  const auto snapshot = lookup_at(model, kStart, kStart + 21, 101);
  const auto full = snapshot.ingress_failure_detail("chest", 760);
  const auto evidence_detail = snapshot.evidence.ingress_detail();
  ASSERT_GE(full.size(), evidence_detail.size());
  EXPECT_EQ(full.substr(full.size() - evidence_detail.size()), evidence_detail);
  EXPECT_EQ(accepted.evidence.ingress_detail(), evidence_detail);
  EXPECT_NE(full.find("latest_joint_source_ns=" + std::to_string(kStart)), std::string::npos);
  EXPECT_NE(full.find("latest_callback_ros_ns=" + std::to_string(kStart + 20)), std::string::npos);
  EXPECT_NE(full.find("rmw_source_timestamp_ns=" + std::to_string(kStart + 2) +
                      " rmw_source_available=1"), std::string::npos);
  EXPECT_NE(full.find("callback_to_owner_wall_ns=10 callback_to_owner_thread_cpu_ns=10"), std::string::npos);
  EXPECT_NE(full.find("owner_to_gate_wall_ns=10 owner_to_gate_thread_cpu_ns=5"), std::string::npos);
  auto without_trace = snapshot;
  without_trace.evidence.ingress = {};
  EXPECT_EQ(snapshot.failure_detail("chest", 760, 123, 456),
            without_trace.failure_detail("chest", 760, 123, 456));

  trace.rmw_source_timestamp_ns = kStart + kStep + 2;
  trace.rmw_received_timestamp_ns = kStart + kStep + 3;
  trace.callback_entry.steady_ns += kStep;
  trace.owner_end.steady_ns += kStep;
  trace.gate_end.steady_ns += kStep;
  const auto next = model.ingest(sample(config, kStart + kStep), kStart + kStep + 20,
                                 100 + kStep, trace);
  ASSERT_TRUE(next.accepted);
  EXPECT_EQ(snapshot.ingress_failure_detail("chest", 760), full); // Frozen same-lock lookup.
  EXPECT_EQ(snapshot.evidence.ingress_detail(), evidence_detail); // Also usable without lookup context.
  auto bad = sample(config, kStart + 2 * kStep);
  bad.accepted = false;
  bad.reason = "fixture_rejected";
  trace.rmw_source_timestamp_ns = 999;
  const auto rejected = model.ingest(bad, kStart + 2 * kStep + 20, 100 + 2 * kStep, trace);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.evidence.source_stamp_ns, next.evidence.source_stamp_ns);
  EXPECT_EQ(rejected.evidence.ingress.rmw_source_timestamp_ns,
            next.evidence.ingress.rmw_source_timestamp_ns);
  EXPECT_EQ(model.evidence().ingress.ingest_commit.steady_ns,
            next.evidence.ingress.ingest_commit.steady_ns);

  const auto legacy = model.ingest(sample(config, kStart + 3 * kStep),
                                   kStart + 3 * kStep + 20, 100 + 3 * kStep);
  ASSERT_TRUE(legacy.accepted);
  EXPECT_FALSE(legacy.evidence.ingress.available);
  EXPECT_EQ(legacy.evidence.ingress.rmw_source_timestamp_ns, 0);
  EXPECT_EQ(legacy.evidence.ingress.callback_entry.steady_ns, -1);
  EXPECT_EQ(legacy.evidence.ingress.ingest_begin.thread_cpu_ns, -1);
}

TEST(JointKinematics, UnavailableIngressMetadataAndCpuAreDiagnosticOnlyAndNeverPretendToBeZeroWork) {
  const auto config = options();
  JointKinematics model(config);
  JointIngressTrace trace;
  trace.available = true;
  trace.rmw_source_timestamp_ns = 0;
  trace.rmw_received_timestamp_ns = -7;
  trace.callback_entry.steady_ns = 100;
  trace.owner_end.steady_ns = 110;
  trace.gate_end.steady_ns = 120;
  const auto result = model.ingest(sample(config, kStart), kStart + 20, 100, trace);
  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.evidence.ingress.callback_entry.thread_cpu_ns, -1);
  const auto snapshot = lookup_at(model, kStart, kStart + 21, 101);
  const auto full = snapshot.ingress_failure_detail("belly", 12);
  const auto evidence_detail = snapshot.evidence.ingress_detail();
  EXPECT_EQ(full.substr(full.size() - evidence_detail.size()), evidence_detail);
  EXPECT_NE(JointKinematicsEvidence{}.ingress_detail().find(
    "trace_available=0 rmw_source_timestamp_ns=0 rmw_source_available=0"), std::string::npos);
  EXPECT_NE(evidence_detail.find("callback_to_owner_thread_cpu_ns=-1"), std::string::npos);
  EXPECT_NE(full.find("trace_available=1 rmw_source_timestamp_ns=0 rmw_source_available=0"), std::string::npos);
  EXPECT_NE(full.find("rmw_received_timestamp_ns=-7 rmw_received_available=0"), std::string::npos);
  EXPECT_NE(full.find("callback_to_owner_wall_ns=10 callback_to_owner_thread_cpu_ns=-1"), std::string::npos);
  EXPECT_NE(full.find("owner_to_gate_wall_ns=10 owner_to_gate_thread_cpu_ns=-1"), std::string::npos);
  EXPECT_EQ(snapshot.status, JointKinematicsStatus::Ready);
}

TEST(JointKinematics, RejectsIncompleteUnsupportedAndNonfiniteModels) {
  auto config = options();
  config.expected_joint_names[0] = "missing";
  EXPECT_THROW(JointKinematics{config}, std::invalid_argument);
  config = options();
  const std::string revolute = "type=\"revolute\"";
  config.urdf_xml.replace(config.urdf_xml.find(revolute), revolute.size(), "type=\"floating\"");
  EXPECT_THROW(JointKinematics{config}, std::invalid_argument);
  config = options();
  const auto joint_end = config.urdf_xml.find("</joint>", config.urdf_xml.find("left_leg_pelvic_pitch_joint"));
  config.urdf_xml.insert(joint_end, "<mimic joint=\"right_leg_pelvic_pitch_joint\"/>");
  EXPECT_THROW(JointKinematics{config}, std::invalid_argument);
  config = options();
  const std::string origin = "xyz=\"0 0.061 0.0575\"";
  config.urdf_xml.replace(config.urdf_xml.find(origin), origin.size(), "xyz=\"nan 0 0\"");
  EXPECT_THROW(JointKinematics{config}, std::invalid_argument);
  config = options();
  config.expected_joint_names.pop_back();
  EXPECT_THROW(JointKinematics{config}, std::invalid_argument);
  config = options();
  config.root_frame = "wrong_root";
  EXPECT_THROW(JointKinematics{config}, std::invalid_argument);
}
TEST(JointKinematics, AttendedWaistPoseLimitCannotBecomeDynamicCalibrationPermission) {
  auto config=options();config.tf_mode="attended_direct";
  JointKinematics model(config);
  auto first=sample(config,kStart);
  const auto waist=std::find(first.names.begin(),first.names.end(),"waist_yaw");
  ASSERT_NE(waist,first.names.end());const auto index=std::distance(first.names.begin(),waist);
  first.positions[index]=.10;
  ASSERT_TRUE(model.ingest(first,kStart+10,100).accepted);
  auto violation=sample(config,kStart+kStep);violation.positions[index]=-.100001;
  const auto rejected=model.ingest(violation,kStart+kStep+10,100+kStep);
  EXPECT_FALSE(rejected.accepted);EXPECT_TRUE(rejected.fault_latched);
  EXPECT_EQ(rejected.reason,"attended_waist_forward_pose_contract_violated");
  EXPECT_EQ(model.evidence().source_stamp_ns,kStart);
  EXPECT_EQ(lookup_at(model,kStart,kStart+kStep+10,100+kStep).status,JointKinematicsStatus::Fault);
}
JointKinematicsOptions attended_options() {
  auto config=options();config.tf_mode="attended_direct";
  config.root_frame="base_link";config.attended_usage="attended_heading_correction";
  const std::string root_link="<link name=\"model_root\" />";
  config.urdf_xml.erase(config.urdf_xml.find(root_link),root_link.size());
  const auto start=config.urdf_xml.find("<joint name=\"root_fixed\"");
  config.urdf_xml.erase(start,config.urdf_xml.find("</joint>",start)+8-start);
  const auto asset=std::filesystem::path(__FILE__).parent_path().parent_path().parent_path()/
      "sunhorizon_nav2_bringup/config/robot_a_prototype_sensor_tf.attended-heading-correction.json";
  std::ifstream file(asset);
  if(!file.good())throw std::runtime_error("attended_test_asset_missing");
  nlohmann::json document;file>>document;
  config.attended_tf_asset_sha256="e298baf599c8c0757ba75a37269a3a30dcdfa64bb7065c718bf9e00d620f8dd5";
  config.attended_tf_receipt_sha256="0b09bbb96e03e94187f0cf636914cc06168c2110331465bccb6fac71b43cd9e4";
  for(const auto& edge:document.at("transforms")){
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id=edge.at("parent").get<std::string>();tf.child_frame_id=edge.at("child").get<std::string>();
    const auto xyz=edge.at("xyz_m").get<std::vector<double>>();const auto rpy=edge.at("rpy_rad").get<std::vector<double>>();
    tf.transform.translation.x=xyz[0];tf.transform.translation.y=xyz[1];tf.transform.translation.z=xyz[2];
    tf2::Quaternion q;q.setRPY(rpy[0],rpy[1],rpy[2]);
    tf.transform.rotation.x=q.x();tf.transform.rotation.y=q.y();tf.transform.rotation.z=q.z();tf.transform.rotation.w=q.w();
    config.attended_static_edges.push_back(std::move(tf));
  }
  return config;
}
TEST(JointKinematics, AttendedPublicCacheKeepsFixedSensorsWhileLegFkStaysPrivate) {
  auto config=attended_options();
  JointKinematics model(config);
  auto direct=std::make_shared<tf2::BufferCore>(tf2::Duration(config.cache_time_ns));
  std::vector<geometry_msgs::msg::TransformStamped> compatibility;
  size_t compatibility_calls=0;
  tf2::BufferCore oracle(tf2::Duration(config.cache_time_ns));
  model.mirror_into(direct,[&](const auto& batch,bool fixed){
    EXPECT_TRUE(fixed);++compatibility_calls;compatibility.insert(compatibility.end(),batch.begin(),batch.end());
  });
  ASSERT_EQ(compatibility_calls,1U);ASSERT_EQ(compatibility.size(),8U);
  for(int i=0;i<2;++i){auto input=sample(config,kStart+i*kStep,.2*i);input.positions.back()=i ? -.05 : .05;
    ASSERT_TRUE(model.ingest(input,input.source_stamp_ns+10,100+i*kStep).accepted);
    old_rsp_publish(oracle,config,input);}
  EXPECT_EQ(compatibility_calls,1U); // Twelve dynamic leg edges stay private.
  const auto query=kStart+kStep/2;
  const auto result=lookup_at(model,query,kStart+kStep+10,100+kStep);
  ASSERT_EQ(result.status,JointKinematicsStatus::Ready);
  ASSERT_EQ(result.transforms.size(),12U);
  for(size_t i=0;i<config.target_frames.size();++i){
    equal_transform(result.transforms[i],oracle.lookupTransform("base_link",config.target_frames[i],tf2::TimePoint(tf2::Duration(query))));
    EXPECT_FALSE(direct->canTransform("base_link",config.target_frames[i],tf2::TimePointZero));
  }
  for(const auto& edge:config.attended_static_edges)
    EXPECT_TRUE(direct->canTransform(edge.header.frame_id,edge.child_frame_id,tf2::TimePointZero));
  for(const auto time:{kStart,query,kStart+kStep}){
    const auto waist=direct->lookupTransform("base_link","waist_yaw_link",tf2::TimePoint(tf2::Duration(time)));
    EXPECT_DOUBLE_EQ(waist.transform.rotation.x,0.);EXPECT_DOUBLE_EQ(waist.transform.rotation.y,0.);
    EXPECT_DOUBLE_EQ(waist.transform.rotation.z,0.);EXPECT_DOUBLE_EQ(waist.transform.rotation.w,1.);
    EXPECT_DOUBLE_EQ(waist.transform.translation.z,.15424);
  }
}
TEST(JointKinematics, ProductionPublicCacheKeepsDynamicWaistAndStaticSensorsWithoutLegCopies) {
  auto config=options();
  config.tf_mode="production_direct";
  config.production_tf_receipt_sha256=std::string(64,'a');
  config.urdf_xml.insert(config.urdf_xml.find("</robot>"),
    R"(<link name="optical"/><joint name="camera_fixed" type="fixed"><parent link="waist_yaw_link"/><child link="optical"/><origin xyz="0.1 0.2 0.3" rpy="0.2 0.1 0.4"/></joint>)");
  JointKinematics model(config);
  auto direct=std::make_shared<tf2::BufferCore>(tf2::Duration(config.cache_time_ns));
  tf2::BufferCore oracle(tf2::Duration(config.cache_time_ns));
  std::vector<geometry_msgs::msg::TransformStamped> dynamic_compatibility;
  size_t static_calls=0;
  model.mirror_into(direct,[&](const auto& edges,bool fixed){
    if(fixed)++static_calls;
    else dynamic_compatibility.insert(dynamic_compatibility.end(),edges.begin(),edges.end());
  });
  ASSERT_EQ(static_calls,1U);
  for(int i=0;i<2;++i){
    auto input=sample(config,kStart+i*kStep,.2*i);
    ASSERT_TRUE(model.ingest(input,input.source_stamp_ns+10,100+i*kStep).accepted);
    old_rsp_publish(oracle,config,input);
  }
  ASSERT_EQ(dynamic_compatibility.size(),2U);
  for(const auto& edge:dynamic_compatibility)EXPECT_EQ(edge.child_frame_id,"waist_yaw_link");
  const auto query=kStart+kStep/2;
  const auto time=tf2::TimePoint(tf2::Duration(query));
  const auto result=lookup_at(model,query,kStart+kStep+10,100+kStep);
  ASSERT_EQ(result.status,JointKinematicsStatus::Ready)<<result.reason;
  ASSERT_EQ(result.transforms.size(),12U);
  for(size_t i=0;i<config.target_frames.size();++i){
    equal_transform(result.transforms[i],oracle.lookupTransform("base_link",config.target_frames[i],time));
    EXPECT_FALSE(direct->canTransform("base_link",config.target_frames[i],tf2::TimePointZero));
  }
  for(const auto& target:{"base_link","waist_yaw_link","optical"})
    equal_transform(direct->lookupTransform(config.root_frame,target,time),
      oracle.lookupTransform(config.root_frame,target,time));
  EXPECT_EQ(dynamic_compatibility.front().header.stamp.sec,kStart/1000000000LL);
  EXPECT_EQ(dynamic_compatibility.back().header.stamp.nanosec,(kStart+kStep)%1000000000LL);
}

TEST(JointKinematics, AttendedDirectRejectsFixedChildWritersBeforeAnyMirroring) {
  for(const bool dynamic:{false,true}){
    auto config=attended_options();
    if(dynamic){
      // The accepted fixed child collides with a retained dynamic leg edge.
      config.attended_static_edges.back().child_frame_id=config.target_frames.front();
    }else{
      // Constructor retains observable fixed descendants, even outside legs.
      const auto end=config.urdf_xml.find("</robot>");
      const auto child=config.attended_static_edges.back().child_frame_id;
      config.urdf_xml.insert(end,"<link name=\""+child+"\"/>"
        "<joint name=\"conflicting_sensor\" type=\"fixed\"><parent link=\"base_link\"/>"
        "<child link=\""+child+"\"/><origin xyz=\"1 2 3\" rpy=\"0 0 0\"/></joint>");
    }
    JointKinematics model(config);
    auto direct=std::make_shared<tf2::BufferCore>();
    size_t compatibility_calls=0;
    try{
      model.mirror_into(direct,[&](const auto&,bool){++compatibility_calls;});
      FAIL()<<"conflicting writer accepted";
    }catch(const std::logic_error& error){
      EXPECT_EQ(std::string(error.what()),dynamic ?
        "attended_direct_urdf_dynamic_child_conflict" : "attended_direct_urdf_static_child_conflict");
    }
    EXPECT_EQ(compatibility_calls,0U);
    // Even the first accepted edge was not installed before rejecting the set.
    const auto& first=config.attended_static_edges.front();
    EXPECT_FALSE(direct->canTransform(first.header.frame_id,first.child_frame_id,tf2::TimePointZero));
  }
}
TEST(JointKinematics, NumericGeometrySharesOnlyExactEdgesAndMatchesOriginalInterpolation) {
  auto config=options();JointKinematics model(config);
  tf2::BufferCore oracle(tf2::Duration(config.cache_time_ns));
  for(int i=0;i<4;++i){
    // Identical nonzero translations/rotations exercise tf2's real arithmetic;
    // then change multiple joints, including a shortest-arc wrap.
    auto input=sample(config,kStart+i*kStep,i<2?.17:.31*i);
    if(i>=2)input.positions[0]=i==2?3.1:-3.1;
    ASSERT_TRUE(model.ingest(input,input.source_stamp_ns+10,100+i*kStep).accepted);
    old_rsp_publish(oracle,config,input);
  }
  const auto clock=[&]{return JointKinematics::ClockSample{kStart+3*kStep+10,100+3*kStep};};
  std::shared_ptr<const JointNumericGeometry> first;
  for(const int64_t query:{kStart,kStart+kStep,kStart+kStep/3,kStart+kStep/2,
      kStart+4*kStep/3,kStart+5*kStep/2,kStart+3*kStep}){
    const auto actual=model.lookup_numeric(query,clock);
    const auto shared=model.lookup_shared_edges(query,clock);
    ASSERT_EQ(actual.status,JointKinematicsStatus::Ready)<<actual.reason;
    ASSERT_TRUE(actual.numeric_geometry);ASSERT_EQ(actual.numeric_geometry->transforms.size(),12U);
    EXPECT_TRUE(actual.transforms.empty());EXPECT_EQ(actual.query_source_stamp_ns,query);
    EXPECT_EQ(actual.evidence.source_stamp_ns,kStart+3*kStep);
    EXPECT_EQ(actual.evidence.receipt_steady_ns,100+3*kStep);
    for(std::size_t i=0;i<12;++i){
      // Exact equality to the previous shared-edge arithmetic; independent
      // RSP/BufferCore oracle also checks per-edge (not composed-pose) motion.
      const auto bits=[](const auto& tf){return std::array<double,7>{tf.translation.x,tf.translation.y,
        tf.translation.z,tf.rotation.x,tf.rotation.y,tf.rotation.z,tf.rotation.w};};
      const auto a=bits(actual.numeric_geometry->transforms[i]),b=bits(shared.transforms[i].transform);
      EXPECT_EQ(std::memcmp(a.data(),b.data(),sizeof(a)),0);
      equal_transform(shared.transforms[i],oracle.lookupTransform(config.base_frame,
        config.target_frames[i],tf2::TimePoint(tf2::Duration(query))));
    }
    if(query==kStart){first=actual.numeric_geometry;EXPECT_TRUE(actual.numeric_geometry_reused);}
    if(query==kStart+kStep){EXPECT_EQ(first,actual.numeric_geometry);EXPECT_TRUE(actual.numeric_geometry_reused);}
    if(query>kStart+kStep && query<kStart+3*kStep)EXPECT_FALSE(actual.numeric_geometry_reused);
  }
}
TEST(JointKinematics, NumericReuseNeverCachesClockHistoryOrFaultEligibility) {
  auto config=options();config.maximum_age_ns=kStep;config.max_history_samples=2;
  JointKinematics model(config);
  for(int i=0;i<3;++i)ASSERT_TRUE(model.ingest(sample(config,kStart+i*kStep),
    kStart+i*kStep+10,100+i*kStep).accepted);
  auto query=[&](int64_t source,int64_t ros,int64_t steady){return model.lookup_numeric(source,
    [=]{return JointKinematics::ClockSample{ros,steady};});};
  const auto good=query(kStart+2*kStep,kStart+2*kStep+10,100+2*kStep);
  ASSERT_EQ(good.status,JointKinematicsStatus::Ready);ASSERT_TRUE(good.numeric_geometry);
  const auto expired=query(kStart,kStart+2*kStep+10,100+2*kStep);
  EXPECT_EQ(expired.status,JointKinematicsStatus::QueryExpired);EXPECT_FALSE(expired.numeric_geometry);
  const auto stale_ros=query(kStart+2*kStep,kStart+3*kStep+1,100+2*kStep);
  EXPECT_EQ(stale_ros.status,JointKinematicsStatus::Stale);EXPECT_FALSE(stale_ros.numeric_geometry);
  const auto stale_steady=query(kStart+2*kStep,kStart+2*kStep+10,101+3*kStep);
  EXPECT_EQ(stale_steady.status,JointKinematicsStatus::Stale);EXPECT_FALSE(stale_steady.numeric_geometry);
  auto bad=sample(config,kStart+3*kStep);bad.positions[0]=std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(model.ingest(bad,kStart+3*kStep+10,100+3*kStep).fault_latched);
  const auto fault=query(kStart+2*kStep,kStart+3*kStep+10,100+3*kStep);
  EXPECT_EQ(fault.status,JointKinematicsStatus::Fault);EXPECT_FALSE(fault.numeric_geometry);
  // Releasing a prior result never re-qualifies this faulted owner.
  EXPECT_TRUE(good.numeric_geometry);EXPECT_FALSE(fault.numeric_geometry_reused);
}

TEST(JointKinematics, MeasuredCandidatePublicPrivateAndFullVendorChainRecomposeAtPlusMinusFiveDegrees) {
  auto config=attended_options();config.tf_mode="attended_measured_direct";
  JointKinematics model(config);
  auto direct=std::make_shared<tf2::BufferCore>(tf2::Duration(config.cache_time_ns));
  std::vector<geometry_msgs::msg::TransformStamped> fixed,dynamic;
  model.mirror_into(direct,[&](const auto& edges,bool is_fixed){
    auto& target=is_fixed?fixed:dynamic;target.insert(target.end(),edges.begin(),edges.end());
  });
  ASSERT_EQ(fixed.size(),6U);
  for(const auto& edge:fixed){EXPECT_NE(edge.child_frame_id,"base_link");EXPECT_NE(edge.child_frame_id,"waist_yaw_link");}
  tf2::BufferCore neutral(tf2::Duration(config.cache_time_ns));
  for(const auto& edge:config.attended_static_edges)ASSERT_TRUE(neutral.setTransform(edge,"neutral_asset_oracle",true));
  for(int i=0;i<3;++i){auto input=sample(config,kStart+i*kStep);
    input.positions.back()=(i-1)*5.*3.14159265358979323846/180.;
    ASSERT_TRUE(model.ingest(input,input.source_stamp_ns+10,100+i*kStep).accepted);
  }
  ASSERT_EQ(dynamic.size(),3U);
  const auto clock=[] {return JointKinematics::ClockSample{kStart+2*kStep+10,100+2*kStep};};
  const auto convert=[](const auto& message){tf2::Transform result;tf2::fromMsg(message,result);return result;};
  tf2::Quaternion q;q.setRPY(.07,-.05,.8);
  const tf2::Transform vendor_world_sensor(q,tf2::Vector3(1.2,-.4,.03));
  for(const auto stamp:{kStart,kStart+kStep/2,kStart+kStep,kStart+2*kStep}){
    const auto waist=model.lookup_measured_waist(stamp,clock);
    ASSERT_EQ(waist.status,JointKinematicsStatus::Ready)<<waist.reason;ASSERT_EQ(waist.transforms.size(),1U);
    const auto time=tf2::TimePoint(tf2::Duration(stamp));
    equal_transform(waist.transforms.front(),direct->lookupTransform("base_link","waist_yaw_link",time));
    // Airy and both RGBD chains use the SAME measured waist. Leg geometry stays private.
    for(const auto* sensor:{"airy_lidar","camera_chest_color_optical_frame","camera_belly_depth_optical_frame"}){
      const auto nominal=neutral.lookupTransform(sensor,"base_link",tf2::TimePointZero).transform;
      const auto source_base=source_from_base_at_measured_waist(nominal,model.neutral_base_from_waist(),waist.transforms.front().transform);
      const auto full=vendor_world_sensor*convert(source_base);
      const auto& rotation=full.getRotation();
      const auto yaw=std::atan2(2.*(rotation.w()*rotation.z()+rotation.x()*rotation.y()),
        1.-2.*(rotation.y()*rotation.y()+rotation.z()*rotation.z()));
      tf2::Quaternion planar_q;planar_q.setRPY(0.,0.,yaw);
      const tf2::Transform planar(planar_q,tf2::Vector3(full.getOrigin().x(),full.getOrigin().y(),full.getOrigin().z()));
      const auto residual=planar.inverse()*full;
      EXPECT_LT(residual.getOrigin().length(),1e-12);
      // Strict residual keeps original vendor z; no floor/foot-height inference.
      EXPECT_NEAR((planar*residual).getOrigin().z(),full.getOrigin().z(),1e-12);
      const auto public_base_sensor=convert(direct->lookupTransform("base_link",sensor,time).transform);
      const auto reconstructed=planar*residual*public_base_sensor;
      EXPECT_LT((reconstructed.getOrigin()-vendor_world_sensor.getOrigin()).length(),1e-12);
      EXPECT_LT(reconstructed.getRotation().angleShortestPath(vendor_world_sensor.getRotation()),1e-7);
      const auto identity=convert(source_base)*public_base_sensor;
      EXPECT_LT(identity.getOrigin().length(),1e-12);
      // Nonzero map gauge, including roll/pitch/z. The full correction must
      // recover the independent registered sensor datum, not add z twice.
      tf2::Quaternion map_q;map_q.setRPY(-.04,.03,-.6);
      const tf2::Transform registered_map_sensor(map_q,tf2::Vector3(3.,2.,.7));
      const auto map_body=registered_map_sensor*convert(source_base);
      const auto map_odom=map_body*full.inverse();
      const auto map_reconstructed=map_odom*planar*residual*public_base_sensor;
      EXPECT_LT((map_reconstructed.getOrigin()-registered_map_sensor.getOrigin()).length(),1e-12);
      EXPECT_LT(map_reconstructed.getRotation().angleShortestPath(registered_map_sensor.getRotation()),1e-7);
    }
    const auto legs=model.lookup_numeric(stamp,clock);ASSERT_EQ(legs.status,JointKinematicsStatus::Ready);
    EXPECT_FALSE(direct->canTransform("base_link",config.target_frames.front(),time));
  }
  EXPECT_EQ(model.lookup_measured_waist(kStart+3*kStep,clock).status,JointKinematicsStatus::FuturePending);
  const auto latest=model.evidence();
  // This fixture retains a five-second lease. Expire the actual accepted
  // source/receipt deadlines, rather than assuming the production default.
  const auto stale=model.lookup_measured_waist(kStart,[&]{return JointKinematics::ClockSample{
    latest.source_stamp_ns+config.maximum_age_ns+1,
    latest.receipt_steady_ns+config.maximum_age_ns+1};});
  EXPECT_EQ(stale.status,JointKinematicsStatus::Stale);EXPECT_TRUE(stale.transforms.empty());
}
}  // namespace
