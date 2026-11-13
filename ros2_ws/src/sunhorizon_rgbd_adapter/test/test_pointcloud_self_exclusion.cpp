#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>
#include "sunhorizon_rgbd_adapter/pointcloud_self_exclusion.hpp"

using namespace sunhorizon_rgbd_adapter;
using Cloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;

namespace {
const ExclusionBox box{{.25, -.25, .50}, {.50, .25, 1.0}};

Cloud cloud(const std::vector<std::array<double, 3>>& points, bool big = false,
            bool doubles = false) {
  Cloud result;
  result.header.frame_id = "rslidar";
  result.header.stamp.sec = 123;
  result.header.stamp.nanosec = 456;
  result.height = 1;
  result.width = points.size();
  result.is_bigendian = big;
  result.is_dense = false;
  const uint32_t scalar = doubles ? 8 : 4;
  result.point_step = scalar * 3 + 8;
  result.row_step = result.width * result.point_step;
  const std::array<std::string, 4> names{"x", "y", "z", "timestamp"};
  for (std::size_t i = 0; i < names.size(); ++i) {
    Field field;
    field.name = names[i];
    field.offset = i * scalar;
    field.count = 1;
    field.datatype = i == 3 ? Field::FLOAT64 :
        (doubles ? Field::FLOAT64 : Field::FLOAT32);
    result.fields.push_back(field);
  }
  result.data.resize(result.row_step, 0xa5);
  const uint16_t probe = 1;
  const bool host_big = *reinterpret_cast<const uint8_t*>(&probe) == 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      uint8_t bytes[8];
      if (doubles) {
        const double value = points[i][axis];
        std::memcpy(bytes, &value, sizeof(value));
      } else {
        const float value = static_cast<float>(points[i][axis]);
        std::memcpy(bytes, &value, sizeof(value));
      }
      if (host_big != big) std::reverse(bytes, bytes + scalar);
      std::memcpy(result.data.data() + i * result.point_step + axis * scalar, bytes, scalar);
    }
    // Unique opaque metadata makes any byte loss/reordering apparent.
    result.data[i * result.point_step + scalar * 3] = static_cast<uint8_t>(i + 17);
  }
  return result;
}

std::vector<uint8_t> record(const Cloud& cloud, std::size_t index) {
  return {cloud.data.begin() + index * cloud.point_step,
          cloud.data.begin() + (index + 1) * cloud.point_step};
}
}  // namespace

TEST(PointcloudSelfExclusion, KeepsNearbyFloorAndObstacleOutsideBoundedRigVolume) {
  const auto input = cloud({{.375, 0., .75}, {.375, 0., -.9},
                           {.501, 0., .75}, {.375, .251, .75}, {.375, 0., 1.01}});
  const auto result = exclude_self_points(input, box, ExclusionTransform{});
  ASSERT_TRUE(result.cloud);
  EXPECT_EQ(result.input_count, 5U);
  EXPECT_EQ(result.removed_count, 1U);
  EXPECT_EQ(result.retained_count, 4U);
  EXPECT_EQ(result.cloud->header, input.header);
  EXPECT_EQ(result.cloud->fields, input.fields);
  EXPECT_EQ(result.cloud->point_step, input.point_step);
  EXPECT_EQ(result.cloud->is_bigendian, input.is_bigendian);
  for (std::size_t i = 0; i < 4; ++i) EXPECT_EQ(record(*result.cloud, i), record(input, i + 1));
}

TEST(PointcloudSelfExclusion, BoxBoundariesInclusiveAndNonfinitePointsRetainedByteForByte) {
  const auto input = cloud({{.25, -.25, .50}, {.50, .25, 1.0},
      {std::numeric_limits<double>::quiet_NaN(), 0., .75},
      {.375, std::numeric_limits<double>::infinity(), .75}});
  const auto result = exclude_self_points(input, box, ExclusionTransform{});
  ASSERT_TRUE(result.cloud);
  EXPECT_EQ(result.removed_count, 2U);
  ASSERT_EQ(result.retained_count, 2U);
  EXPECT_EQ(record(*result.cloud, 0), record(input, 2));
  EXPECT_EQ(record(*result.cloud, 1), record(input, 3));
}

TEST(PointcloudSelfExclusion, SupportsBothByteOrdersAndFloatWidths) {
  for (const bool big : {false, true}) {
    for (const bool doubles : {false, true}) {
      const auto input = cloud({{.375, 0., .75}, {1., 0., .75}}, big, doubles);
      const auto result = exclude_self_points(input, box, ExclusionTransform{});
      ASSERT_TRUE(result.cloud);
      EXPECT_EQ(result.removed_count, 1U);
      EXPECT_EQ(record(*result.cloud, 0), record(input, 1));
      EXPECT_EQ(result.cloud->fields, input.fields);
      EXPECT_EQ(result.cloud->is_bigendian, big);
    }
  }
}

TEST(PointcloudSelfExclusion, UsesFullRotationAndTranslationOnlyForMembership) {
  // 90 degree rotation around Z plus translation maps (0,-.25,.5) into the rig.
  ExclusionTransform transform;
  transform.rotation = {0., -1., 0., 1., 0., 0., 0., 0., 1.};
  transform.translation = {.125, 0., .25};
  const auto input = cloud({{0., -.25, .50}, {2., -.25, .50}});
  const auto result = exclude_self_points(input, box, transform);
  ASSERT_TRUE(result.cloud);
  EXPECT_EQ(result.removed_count, 1U);
  EXPECT_EQ(record(*result.cloud, 0), record(input, 1));
  EXPECT_EQ(result.cloud->header.frame_id, "rslidar");
}

TEST(PointcloudSelfExclusion, ReadsOrganizedCloudRowPaddingWithoutTreatingItAsPoints) {
  auto input = cloud({{.375, 0., .75}, {1., 0., .75}});
  const auto first = record(input, 0), second = record(input, 1);
  input.width = 1;
  input.height = 2;
  input.row_step = input.point_step + 7;
  input.data.assign(input.row_step * input.height, 0xff);
  std::copy(first.begin(), first.end(), input.data.begin());
  std::copy(second.begin(), second.end(), input.data.begin() + input.row_step);
  const auto result = exclude_self_points(input, box, ExclusionTransform{});
  ASSERT_TRUE(result.cloud);
  EXPECT_EQ(result.cloud->height, 1U);
  EXPECT_EQ(result.cloud->width, 1U);
  EXPECT_EQ(result.cloud->row_step, input.point_step);
  EXPECT_EQ(result.cloud->data, second);
}

TEST(PointcloudSelfExclusion, MissingOrNonfiniteTransformNeverPublishesRawFallback) {
  const auto input = cloud({{.375, 0., .75}});
  const auto missing = exclude_self_points(input, box, std::nullopt);
  EXPECT_FALSE(missing.cloud);
  EXPECT_EQ(missing.reason, "cloud_stamp_transform_unavailable");
  ExclusionTransform invalid;
  invalid.translation[0] = std::numeric_limits<double>::infinity();
  const auto failed = exclude_self_points(input, box, invalid);
  EXPECT_FALSE(failed.cloud);
  EXPECT_EQ(failed.reason, "invalid_transform");
}

TEST(PointcloudSelfExclusion, RejectsBroadReversedAndNonfiniteBounds) {
  EXPECT_TRUE(valid_exclusion_box(box));
  for (const auto invalid : {
      ExclusionBox{{-2., -2., -2.}, {2., 2., 2.}},
      ExclusionBox{{.5, -.25, .5}, {.25, .25, 1.}},
      ExclusionBox{{.25, -.25, .5}, {.50, .25, std::numeric_limits<double>::infinity()}},
      ExclusionBox{{2.1, 0., 0.}, {2.2, .1, .1}}}) {
    EXPECT_FALSE(valid_exclusion_box(invalid));
    EXPECT_FALSE(exclude_self_points(cloud({{0., 0., 0.}}), invalid, ExclusionTransform{}).cloud);
  }
}

TEST(PointcloudSelfExclusion, RejectsMalformedLayoutsAndAmbiguousXyzWithoutOutput) {
  const auto input = cloud({{.375, 0., .75}});
  auto truncated = input;
  truncated.data.pop_back();
  EXPECT_FALSE(exclude_self_points(truncated, box, ExclusionTransform{}).cloud);
  auto short_row = input;
  short_row.row_step = 1;
  EXPECT_FALSE(exclude_self_points(short_row, box, ExclusionTransform{}).cloud);
  auto out_of_bounds = input;
  out_of_bounds.fields[2].offset = input.point_step;
  EXPECT_FALSE(exclude_self_points(out_of_bounds, box, ExclusionTransform{}).cloud);
  auto duplicate = input;
  duplicate.fields.push_back(input.fields[0]);
  EXPECT_FALSE(exclude_self_points(duplicate, box, ExclusionTransform{}).cloud);
  auto absent = input;
  absent.fields.erase(absent.fields.begin());
  EXPECT_FALSE(exclude_self_points(absent, box, ExclusionTransform{}).cloud);
  auto overlapping = input;
  overlapping.fields[1].offset = 0;
  EXPECT_FALSE(exclude_self_points(overlapping, box, ExclusionTransform{}).cloud);
  EXPECT_FALSE(exclude_self_points(cloud({}), box, ExclusionTransform{}).cloud);
}

TEST(PointcloudSelfExclusion, RecordedRobotARigReturnsRemovedAndNearbyFloorRetained) {
  // Actual base-frame rows from nearbody-rig-check-1100/0.npz, 2026-09-10.
  // This regression asserts spatial selectivity, not object identity acceptance.
  const auto input = cloud({
      {.3832916758060455, -.21901947259902954, .7978753674125671},
      {.34450232899188993, .07309065759181976, .7352165210342407},
      {.36149965143203733, .15928088128566742, .8200923431015015},
      {.3632995626926422, .19424141943454742, -.8765548241043091},
      {.3836901030540466, .19527651369571686, -.882401205291748}});
  const ExclusionBox attended_rig{{.28, -.25, .65}, {.43, .25, .90}};
  const auto result = exclude_self_points(input, attended_rig, ExclusionTransform{});
  ASSERT_TRUE(result.cloud);
  EXPECT_EQ(result.removed_count, 3U);
  EXPECT_EQ(result.retained_count, 2U);
  EXPECT_EQ(record(*result.cloud, 0), record(input, 3));
  EXPECT_EQ(record(*result.cloud, 1), record(input, 4));
}

TEST(PointcloudSelfExclusion, RemovalFractionGuardRejectsUnexpectedCropAndInvalidLimit) {
  EXPECT_TRUE(exclusion_fraction_allowed(25, 72598, .02));
  EXPECT_TRUE(exclusion_fraction_allowed(2, 100, .02));
  EXPECT_FALSE(exclusion_fraction_allowed(3, 100, .02));
  EXPECT_FALSE(exclusion_fraction_allowed(100, 100, .02));
  EXPECT_FALSE(exclusion_fraction_allowed(1, 0, .02));
  EXPECT_TRUE(exclusion_fraction_allowed(0, 0, .02));
  EXPECT_FALSE(exclusion_fraction_allowed(0, 100, 1.));
  EXPECT_FALSE(exclusion_fraction_allowed(0, 100, std::numeric_limits<double>::quiet_NaN()));
}

TEST(PointcloudSelfExclusion, DisjointSelfVolumesDoNotRemoveTheSpaceBetweenHands) {
  const ExclusionBox right{{.12, -.36, .01}, {.24, -.20, .22}};
  const ExclusionBox left{{.12, .20, .01}, {.24, .36, .22}};
  const auto input = cloud({{.18, -.28, .10}, {.18, .28, .10},
      {.18, 0., .10}, {.375, 0., .75}, {.18, -.28, -.8}, {.26, -.28, .10}});
  const auto result = exclude_self_volumes(input, {box, right, left}, ExclusionTransform{});
  ASSERT_TRUE(result.cloud);
  EXPECT_EQ(result.removed_count, 3U);
  EXPECT_EQ(result.retained_count, 3U);
  EXPECT_EQ(record(*result.cloud, 0), record(input, 2));
  EXPECT_EQ(record(*result.cloud, 1), record(input, 4));
  EXPECT_EQ(record(*result.cloud, 2), record(input, 5));
  EXPECT_EQ(result.cloud->header, input.header);
}

TEST(PointcloudSelfExclusion, OverlappingVolumesCountEachPointOnceAndInvalidUnionFailsClosed) {
  const auto input = cloud({{.375, 0., .75}, {1., 0., .75}});
  const auto result = exclude_self_volumes(input, {box, box}, ExclusionTransform{});
  ASSERT_TRUE(result.cloud);
  EXPECT_EQ(result.removed_count, 1U);
  EXPECT_EQ(result.retained_count, 1U);
  EXPECT_FALSE(exclude_self_volumes(input, {}, ExclusionTransform{}).cloud);
  EXPECT_FALSE(exclude_self_volumes(input,
      std::vector<ExclusionBox>(kMaxSelfExclusionBoxes + 1, box), ExclusionTransform{}).cloud);
  EXPECT_FALSE(exclude_self_volumes(input, {box, ExclusionBox{}}, ExclusionTransform{}).cloud);
  EXPECT_FALSE(exclude_self_volumes(input, {box}, std::nullopt).cloud);
}

TEST(PointcloudSelfExclusion, EightLocalVolumesPreserveEveryGapAndRejectNinth) {
  std::vector<ExclusionBox> volumes;
  std::vector<std::array<double, 3>> points;
  // Synthetic geometry tests bounded union semantics, not new robot masks.
  for (std::size_t i = 0; i < kMaxSelfExclusionBoxes; ++i) {
    const double x = -.8 + static_cast<double>(i) * .2;
    volumes.push_back({{x, -.02, .05}, {x + .05, .02, .10}});
    points.push_back({x + .025, 0., .075});
    points.push_back({x + .10, 0., .075});
  }
  const auto input = cloud(points);
  const auto result = exclude_self_volumes(input, volumes, ExclusionTransform{});
  ASSERT_TRUE(result.cloud);
  EXPECT_EQ(result.removed_count, kMaxSelfExclusionBoxes);
  EXPECT_EQ(result.retained_count, kMaxSelfExclusionBoxes);
  for (std::size_t i = 0; i < kMaxSelfExclusionBoxes; ++i) {
    EXPECT_EQ(record(*result.cloud, i), record(input, 2 * i + 1));
  }
  volumes.push_back(box);
  const auto overflow = exclude_self_volumes(input, volumes, ExclusionTransform{});
  EXPECT_FALSE(overflow.cloud);
  EXPECT_EQ(overflow.reason, "invalid_exclusion_box");
  volumes.pop_back();
  volumes.back().maximum[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(exclude_self_volumes(input, volumes, ExclusionTransform{}).cloud);
}

TEST(PointcloudSelfExclusion, RecordedGoal25And28UnassignedReturnsStayOutsideExistingVolumes) {
  const std::vector<ExclusionBox> current_volumes{
      {{.28, -.25, .65}, {.43, .25, .90}},
      {{.140, -.359, -.027}, {.204, -.203, .194}},
      {{.140, .276, -.028}, {.156, .322, .050}},
      {{.140, .069, -.089}, {.190, .149, .031}}};
  // Actual filtered Airy rows transformed at their cloud stamps to
  // base_footprint. Goal25 stamps 1789021443017023488, 1789021443217011200,
  // 1789021443316977152; goal28 cloud stamps
  // 1789022429129874944, 1789022446829817088, 1789022463430075136.
  // These are measured omissions, not proof of hand identity. Capacity alone
  // must not erase them by mirroring a box or constructing a combined hull.
  const auto input = cloud({
      {.15054792827367783, -.08393162488937378, .02153136611938475},
      {.1837712070941925, .32702943682670593, .020675860652923567},
      {.22497813320159912, .3099798262119293, .025985576519966108},
      {.2525480977296829, .32336923480033875, .03260945559501646},
      {.141359268784523, .2339428812265396, .19685792767465113},
      {.14136188766360283, -.16291959583759308, .029618971357345564},
      {.24187371051311493, .3357977867126465, .03788340867519377},
      {.1413636348247528, -.18017305433750153, .03179148615360258}});
  const auto result = exclude_self_volumes(input, current_volumes, ExclusionTransform{});
  ASSERT_TRUE(result.cloud);
  EXPECT_EQ(result.removed_count, 0U);
  EXPECT_EQ(result.retained_count, input.width);
  EXPECT_EQ(result.cloud->data, input.data);
  EXPECT_EQ(result.cloud->header, input.header);
}

TEST(PointcloudSelfExclusion, RecordedTrial13HandsRemovedAndUnassignedCentreReturnRetained) {
  const ExclusionBox right{{.140, -.359, -.027}, {.204, -.203, .194}};
  const ExclusionBox left_outer{{.140, .276, -.028}, {.156, .322, .050}};
  const ExclusionBox left_inner{{.140, .069, -.089}, {.190, .149, .031}};
  // Actual Airy rows at cloud stamps 1789011598.565553 and .765478,
  // base_footprint coordinates. The on-site operator identified the hands.
  const auto input = cloud({
      {.1413767189, .2882137001, .0484778211},
      {.1646377593, -.3024312854, .0253915563},
      {.1505479217, .0825478286, .0209496692},
      {.1413557827, -.0109666158, .4339931905}});
  const auto result = exclude_self_volumes(input,
      {box, right, left_outer, left_inner}, ExclusionTransform{});
  ASSERT_TRUE(result.cloud);
  EXPECT_EQ(result.removed_count, 3U);
  EXPECT_EQ(result.retained_count, 1U);
  EXPECT_EQ(record(*result.cloud, 0), record(input, 3));
}

TEST(PointcloudSelfExclusion, MovingOneLegPreservesOldLocationGapAndChair) {
  // Synthetic local leg bounds: motion must move only that volume, never a
  // base-frame hull joining both legs or retaining a swept exclusion trail.
  const ExclusionBox leg{{-.04, -.03, -.2}, {.04, .03, .2}};
  ExclusionTransform left, right;
  left.translation = {-.1, -.15, .4};
  right.translation = {-.1, .15, .4};
  const auto input = cloud({{.1, .15, -.4}, {.1, -.15, -.4},
                           {.1, 0., -.4}, {2., 0., -.4}, {.3, .15, -.4}});
  auto result = exclude_self_framed_volumes(input, {leg, leg}, {left, right});
  ASSERT_TRUE(result.cloud);
  EXPECT_EQ(result.removed_count, 2U);
  for (std::size_t i = 0; i < 3; ++i) EXPECT_EQ(record(*result.cloud, i), record(input, i + 2));
  left.translation[0] = -.3;
  result = exclude_self_framed_volumes(input, {leg, leg}, {left, right});
  ASSERT_TRUE(result.cloud);
  ASSERT_EQ(result.retained_count, 3U);
  EXPECT_EQ(record(*result.cloud, 0), record(input, 0));
  EXPECT_EQ(record(*result.cloud, 1), record(input, 2));
  EXPECT_EQ(record(*result.cloud, 2), record(input, 3));
  EXPECT_EQ(result.cloud->header, input.header);
}

TEST(PointcloudSelfExclusion, AnyMissingOrInvalidLegTransformRejectsEntireCloud) {
  const auto input = cloud({{.375, 0., .75}, {2., 0., .75}});
  const auto missing = exclude_self_framed_volumes(
      input, {box, box}, {ExclusionTransform{}, std::nullopt});
  EXPECT_FALSE(missing.cloud);
  EXPECT_EQ(missing.reason, "cloud_stamp_transform_unavailable");
  EXPECT_FALSE(exclude_self_framed_volumes(input, {box, box}, {ExclusionTransform{}}).cloud);
  ExclusionTransform invalid;
  invalid.translation[1] = std::numeric_limits<double>::quiet_NaN();
  const auto bad = exclude_self_framed_volumes(input, {box, box}, {ExclusionTransform{}, invalid});
  EXPECT_FALSE(bad.cloud);
  EXPECT_EQ(bad.reason, "invalid_transform");
}

TEST(PointcloudSelfExclusion, LocalVolumeRotatesWithItsOwnLink) {
  const ExclusionBox leg{{-.02, -.02, -.3}, {.02, .02, .3}};
  ExclusionTransform turned;
  turned.rotation = {0., 0., -1., 0., 1., 0., 1., 0., 0.};
  const auto input = cloud({{.2, 0., 0.}, {0., 0., .2}, {2., 0., 0.}});
  const auto result = exclude_self_framed_volumes(input, {leg}, {turned});
  ASSERT_TRUE(result.cloud);
  ASSERT_EQ(result.removed_count, 1U);
  ASSERT_EQ(result.retained_count, 2U);
  EXPECT_EQ(record(*result.cloud, 0), record(input, 1));
  EXPECT_EQ(record(*result.cloud, 1), record(input, 2));
}

TEST(PointcloudSelfExclusion, BoundedShadowKeepsFrontSideAndFarObstaclesByteExact) {
  const ExclusionBox leg{{.4, -.04, -.2}, {.5, .04, .2}};
  // Behind the leg, in front, beside it, far behind it, and on the leg.
  const auto input = cloud({{.8, 0., 0.}, {.3, 0., 0.}, {.8, .3, 0.},
                           {2., 0., 0.}, {.45, 0., 0.}});
  const auto off = exclude_self_framed_volumes(input, {leg}, {ExclusionTransform{}});
  ASSERT_TRUE(off.cloud); EXPECT_EQ(off.removed_count, 1U);
  const auto on = exclude_self_framed_volumes(input, {leg}, {ExclusionTransform{}}, 1.);
  ASSERT_TRUE(on.cloud); ASSERT_EQ(on.removed_count, 2U);
  ASSERT_EQ(on.retained_count, 3U);
  for (std::size_t i = 0; i < 3; ++i) EXPECT_EQ(record(*on.cloud, i), record(input, i + 1));
  EXPECT_EQ(on.cloud->header, input.header);
  EXPECT_FALSE(exclude_self_framed_volumes(input, {leg}, {ExclusionTransform{}}, -1.).cloud);
}

TEST(PointcloudSelfExclusion, ShadowUsesTransformedSensorOriginAndMovingLink) {
  const ExclusionBox leg{{-.05, -.04, -.2}, {.05, .04, .2}};
  ExclusionTransform tf;
  tf.translation = {-.45, 0., 0.};
  const auto input = cloud({{.8, 0., 0.}, {.8, .3, 0.}});
  auto result = exclude_self_framed_volumes(input, {leg}, {tf}, 1.);
  ASSERT_TRUE(result.cloud); EXPECT_EQ(result.removed_count, 1U);
  EXPECT_EQ(record(*result.cloud, 0), record(input, 1));
  tf.translation[1] = -.5;
  result = exclude_self_framed_volumes(input, {leg}, {tf}, 1.);
  ASSERT_TRUE(result.cloud); EXPECT_EQ(result.cloud->data, input.data);
  EXPECT_FALSE(exclude_self_framed_volumes(input, {leg}, {std::nullopt}, 1.).cloud);
}

TEST(PointcloudSelfExclusion, SphereBoundKeepsLegacyNonRigidCoreInputBehavior) {
  // Production TF is rigid. The lower-level API historically also accepts
  // finite non-rigid matrices, for which a rigid sphere shortcut is invalid.
  ExclusionTransform scaled;
  scaled.rotation = {.01, 0., 0., 0., .01, 0., 0., 0., .01};
  const ExclusionBox local{{.09, -.01, -.01}, {.11, .01, .01}};
  const auto input = cloud({{10., 0., 0.}, {20., 0., 0.}});
  const auto result = exclude_self_framed_volumes(input, {local}, {scaled});
  ASSERT_TRUE(result.cloud); EXPECT_EQ(result.removed_count, 1U);
  ASSERT_EQ(result.retained_count, 1U);
  EXPECT_EQ(record(*result.cloud, 0), record(input, 1));
}

TEST(PointcloudSelfExclusion, PreservesOriginalRayIdentityAcrossAabbAndNanCompaction) {
  const auto input=cloud({{.375,0.,.75},{NAN,0.,.75},{1.,0.,.75}});
  const std::vector<uint8_t> blocked{1,1,0};
  const std::vector<std::array<float,3>> origins{{0.f,.1f,.2f},{1.f,1.1f,1.2f},{2.f,2.1f,2.2f}};
  const auto result=exclude_self_framed_volumes(input,{box},{ExclusionTransform{}},0.,&blocked,&origins);
  ASSERT_TRUE(result.cloud);ASSERT_EQ(result.retained_count,2U);
  EXPECT_EQ(result.ray_origins,(std::vector<std::array<float,3>>{origins[1],origins[2]}));
  EXPECT_EQ(result.ground_blocked,(std::vector<uint8_t>{1,0}));
  auto short_origins=origins;short_origins.pop_back();
  EXPECT_FALSE(exclude_self_framed_volumes(input,{box},{ExclusionTransform{}},0.,&blocked,&short_origins).cloud);
}
