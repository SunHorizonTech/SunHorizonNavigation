#include <gtest/gtest.h>
#include <cstring>
#include <limits>
#include "sunhorizon_rgbd_adapter/depth_projection.hpp"

using namespace sunhorizon_rgbd_adapter;
namespace {
sensor_msgs::msg::Image image(bool big = false) {
  sensor_msgs::msg::Image d;
  d.header.frame_id = "camera"; d.header.stamp.sec = 123;
  d.width = 3; d.height = 2; d.step = 8; d.encoding = "16UC1";
  d.is_bigendian = big; d.data.resize(16, 0xaa);
  const uint16_t values[] = {1000, 2000, 0, 3000, 100, 9000};
  for (std::size_t i = 0; i < 6; ++i) {
    auto offset = i / 3 * 8 + i % 3 * 2;
    d.data[offset + (big ? 1 : 0)] = values[i] & 255;
    d.data[offset + (big ? 0 : 1)] = values[i] >> 8;
  }
  return d;
}
sensor_msgs::msg::CameraInfo info() {
  sensor_msgs::msg::CameraInfo c; c.header.frame_id = "camera";
  c.width = 3; c.height = 2; c.k = {2., 0., 1., 0., 2., .5, 0., 0., 1.}; return c;
}
float coordinate(const sensor_msgs::msg::PointCloud2& c, std::size_t offset) {
  float f; std::memcpy(&f, c.data.data() + offset, sizeof(f)); return f;
}
}
TEST(DepthProjection, PreservesStampAndProjectsValidPixelsWithRowPaddingAndByteOrder) {
  for (const bool big : {false, true}) {
    const auto d = image(big); const auto c = project_depth(d, info(), .001, .2, 8., 1);
    ASSERT_EQ(c.width, 3U); EXPECT_EQ(c.header, d.header); EXPECT_FALSE(c.is_bigendian);
    EXPECT_FLOAT_EQ(coordinate(c, 0), -.5f); EXPECT_FLOAT_EQ(coordinate(c, 4), -.25f);
    EXPECT_FLOAT_EQ(coordinate(c, 8), 1.f); EXPECT_FLOAT_EQ(coordinate(c, 12), 0.f);
    EXPECT_FLOAT_EQ(coordinate(c, 20), 2.f); EXPECT_FLOAT_EQ(coordinate(c, 24), -1.5f);
    EXPECT_FLOAT_EQ(coordinate(c, 28), .75f); EXPECT_FLOAT_EQ(coordinate(c, 32), 3.f);
  }
}
TEST(DepthProjection, MaskAndStrideDoNotInventOrMoveRetainedPoints) {
  const auto d = image(); const std::vector<uint8_t> keep{0, 1, 1, 1, 1, 1};
  const auto c = project_depth(d, info(), .001, .2, 8., 1, &keep);
  const auto full = project_depth(d, info(), .001, .2, 8., 1);
  EXPECT_EQ(c.data, std::vector<uint8_t>(full.data.begin() + 12, full.data.end()));
  EXPECT_EQ(project_depth(d, info(), .001, .2, 8., 2).width, 1U);
}
TEST(DepthProjection, MalformedCalibrationLayoutOrMaskDoesNotProduceACloud) {
  auto d = image(); auto c = info(); const std::vector<uint8_t> short_mask(5);
  EXPECT_THROW(project_depth(d, c, .001, .2, 8., 1, &short_mask), std::invalid_argument);
  EXPECT_THROW(project_depth(d, c, .001, .2, 8., 0), std::invalid_argument);
  c.k[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(project_depth(d, c, .001, .2, 8., 1), std::invalid_argument);
  c = info(); c.header.frame_id = "other_camera";
  EXPECT_THROW(project_depth(d, c, .001, .2, 8., 1), std::invalid_argument);
  c = info(); d.data.resize(9);
  EXPECT_THROW(project_depth(d, c, .001, .2, 8., 1), std::invalid_argument);
}

TEST(DepthProjection, BoundedComponentMaskPreservesEveryDistantPointByteExact) {
  const auto d = image();
  const std::vector<uint8_t> reject_all(6, 0);
  const auto full = project_depth(d, info(), .001, .2, 8., 1);
  const auto bounded = project_depth(d, info(), .001, .2, 8., 1, &reject_all, 1.5);
  // The near 1 m return is rejected. Isolated 2 m and 3 m returns are retained.
  ASSERT_EQ(bounded.width, 2U);
  EXPECT_EQ(bounded.data, std::vector<uint8_t>(full.data.begin() + 12, full.data.end()));
  EXPECT_EQ(bounded.header, d.header);
  EXPECT_EQ(project_depth(d, info(), .001, .2, 8., 1, &reject_all).width, 0U);
  EXPECT_THROW(project_depth(d, info(), .001, .2, 8., 1, &reject_all, -1.), std::invalid_argument);
}
