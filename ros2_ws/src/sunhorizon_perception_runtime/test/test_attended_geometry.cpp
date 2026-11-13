#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <type_traits>
#include <tf2/LinearMath/Quaternion.h>

#include "sunhorizon_perception_runtime/attended_geometry.hpp"

namespace {
namespace runtime = sunhorizon_perception_runtime;
namespace dm = sunhorizon_rgbd_adapter::dynamic_self_model;
using Geometry = runtime::detail::AttendedGeometry;
const std::array<std::string, 2> kOptical{"belly_optical", "chest_optical"};

geometry_msgs::msg::TransformStamped edge(const std::string& parent, const std::string& child,
    double x, double y, double z, double roll, double pitch, double yaw) {
  geometry_msgs::msg::TransformStamped result;
  result.header.frame_id = parent; result.child_frame_id = child;
  result.transform.translation.x = x; result.transform.translation.y = y;
  result.transform.translation.z = z;
  tf2::Quaternion q; q.setRPY(roll, pitch, yaw);
  result.transform.rotation.x = q.x(); result.transform.rotation.y = q.y();
  result.transform.rotation.z = q.z(); result.transform.rotation.w = q.w();
  return result;
}

runtime::JointKinematicsOptions options() {
  runtime::JointKinematicsOptions result;
  result.tf_mode = "attended_direct";
  result.attended_usage = "attended_heading_correction";
  // Fixture identities stand in for the receipts validated by the runtime
  // loader; the helper does not independently grant asset acceptance.
  result.attended_tf_asset_sha256 = std::string(64, 'a');
  result.attended_tf_receipt_sha256 = std::string(64, 'b');
  result.attended_static_edges = {
    edge("base_footprint", "base_link", .1, -.2, .3, .02, -.03, .04),
    edge("base_link", "waist_yaw_link", .01, .02, .15, -.05, .06, .07),
    edge("waist_yaw_link", "belly_mount", .12, -.04, -.08, .11, -.12, .13),
    edge("belly_mount", kOptical[0], .02, .03, -.01, -.7, .2, -.4),
    edge("waist_yaw_link", "chest_mount", .21, .08, .19, -.15, .16, -.17),
    edge("chest_mount", kOptical[1], -.03, .01, .04, .6, -.3, .5),
    edge("waist_yaw_link", "lidar_mount", .2, .1, .3, .01, .02, .03),
    edge("lidar_mount", "rslidar", -.01, .02, -.03, .04, -.05, .06)};
  return result;
}

void equal_rigid(const dm::geometry::RigidTransform& actual,
    const dm::geometry::RigidTransform& expected) {
  for (std::size_t i = 0; i < actual.rotation.size(); ++i)
    EXPECT_NEAR(actual.rotation[i], expected.rotation[i], 1.e-12);
  EXPECT_NEAR(actual.translation.x, expected.translation.x, 1.e-12);
  EXPECT_NEAR(actual.translation.y, expected.translation.y, 1.e-12);
  EXPECT_NEAR(actual.translation.z, expected.translation.z, 1.e-12);
}

TEST(AttendedGeometry, BothRolesMatchOriginalExactTimeTfForNonidentityFixedTree) {
  auto input = options();
  const Geometry geometry(input, "base_footprint", kOptical);
  tf2::BufferCore original;
  for (const auto& transform : input.attended_static_edges)
    ASSERT_TRUE(original.setTransform(transform, "original_fixture", true));
  for (const int64_t stamp : {1LL, 1000000000LL, 1792126250000000000LL}) {
    for (std::size_t role = 0; role < kOptical.size(); ++role) {
      const auto* value = geometry.select("attended_direct", role, stamp, stamp);
      ASSERT_NE(value, nullptr);
      const tf2::TimePoint time{tf2::Duration(stamp)};
      equal_rigid(value->optical_from_base, dm::rigid(original.lookupTransform(
          kOptical[role], "base_link", time).transform));
      equal_rigid(value->common_from_optical, dm::rigid(original.lookupTransform(
          "base_footprint", kOptical[role], time).transform));
    }
  }
  for(const auto stamp:{1LL,1792126250000000000LL}){
    const auto* lidar=geometry.select_source("attended_direct","rslidar",stamp);
    ASSERT_NE(lidar,nullptr);
    equal_rigid(*lidar,dm::rigid(original.lookupTransform("rslidar","base_link",
      tf2::TimePoint(tf2::Duration(stamp))).transform));
  }
  // No references to the caller's mutable vector or a shared TF buffer survive.
  const auto before = *geometry.select("attended_direct", 0, 1, 1);
  input.attended_static_edges.clear();
  equal_rigid(geometry.select("attended_direct", 0, 2, 2)->optical_from_base,
      before.optical_from_base);
  static_assert(!std::is_copy_assignable_v<Geometry>);
}

TEST(AttendedGeometry, DynamicModesAndCrossTimeQueriesCannotSelectFixedGeometry) {
  const Geometry geometry(options(), "base_footprint", kOptical);
  EXPECT_EQ(geometry.select_source("production_direct","rslidar",10),nullptr);
  EXPECT_EQ(geometry.select_source("attended_global_tf","rslidar",10),nullptr);
  EXPECT_EQ(geometry.select_source("attended_direct","unknown_installation",10),nullptr);
  EXPECT_EQ(geometry.select_source("attended_direct","rslidar",0),nullptr);
  EXPECT_EQ(geometry.select("production_direct", 0, 10, 10), nullptr);
  EXPECT_EQ(geometry.select("attended_global_tf", 0, 10, 10), nullptr);
  EXPECT_EQ(geometry.select("attended_direct", 0, 10, 11), nullptr);
  EXPECT_EQ(geometry.select("attended_direct", 0, 0, 0), nullptr);
  EXPECT_EQ(geometry.select("attended_direct", 0, -1, -1), nullptr);
  EXPECT_EQ(geometry.select("attended_direct", 2, 10, 10), nullptr);
  auto input = options(); input.tf_mode = "production_direct";
  EXPECT_THROW((Geometry(input, "base_footprint", kOptical)), std::invalid_argument);
}

TEST(AttendedGeometry, MissingPathsDuplicateChildrenCyclesAndForestsAreRejected) {
  auto missing = options(); missing.attended_static_edges.erase(missing.attended_static_edges.begin());
  EXPECT_THROW((Geometry(missing, "base_footprint", kOptical)), std::invalid_argument);
  EXPECT_THROW((Geometry(options(), "missing_common", kOptical)), std::invalid_argument);
  const std::array<std::string, 2> absent{"belly_optical", "absent_optical"};
  EXPECT_THROW((Geometry(options(), "base_footprint", absent)), std::invalid_argument);
  auto duplicate = options(); duplicate.attended_static_edges.back() = duplicate.attended_static_edges.front();
  EXPECT_THROW((Geometry(duplicate, "base_footprint", kOptical)), std::invalid_argument);
  auto conflict = options(); conflict.attended_static_edges.back().child_frame_id = "base_link";
  EXPECT_THROW((Geometry(conflict, "base_footprint", kOptical)), std::invalid_argument);
  auto cycle = options(); cycle.attended_static_edges.front().header.frame_id = "waist_yaw_link";
  EXPECT_THROW((Geometry(cycle, "base_footprint", kOptical)), std::invalid_argument);
  auto forest = options(); forest.attended_static_edges.back().header.frame_id = "unconnected";
  EXPECT_THROW((Geometry(forest, "base_footprint", kOptical)), std::invalid_argument);
}

TEST(AttendedGeometry, InvalidRigidOrUnboundAssetCannotConstructConstants) {
  auto bad = options(); bad.attended_static_edges.front().transform.translation.x =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((Geometry(bad, "base_footprint", kOptical)), std::invalid_argument);
  bad = options(); bad.attended_static_edges.front().transform.rotation.w = 2.;
  EXPECT_THROW((Geometry(bad, "base_footprint", kOptical)), std::invalid_argument);
  bad = options(); bad.attended_tf_receipt_sha256.clear();
  EXPECT_THROW((Geometry(bad, "base_footprint", kOptical)), std::invalid_argument);
  bad = options(); bad.attended_usage = "production";
  EXPECT_THROW((Geometry(bad, "base_footprint", kOptical)), std::invalid_argument);
}
}  // namespace
