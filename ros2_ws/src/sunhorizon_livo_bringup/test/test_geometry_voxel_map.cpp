#include "sunhorizon_livo_bringup/geometry_voxel_map.hpp"

#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>
#include <unistd.h>

namespace
{

namespace fs = std::filesystem;
using Json = nlohmann::json;
using Point = pcl::PointXYZI;
constexpr char kReleaseSha[] = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    root_ = fs::temp_directory_path() /
      ("sunhorizon-livo-geometry-test-" +
      std::to_string(static_cast<long long>(::getpid())) + "-XXXXXX");
    auto value = root_.string();
    value.push_back('\0');
    char * created = ::mkdtemp(value.data());
    if (created == nullptr) {
      throw std::runtime_error("mkdtemp_failed");
    }
    root_ = created;
  }

  ~TemporaryDirectory()
  {
    std::error_code ignored;
    fs::remove_all(root_, ignored);
  }

  const fs::path & path() const {return root_;}

private:
  fs::path root_;
};

fs::path write_raw_map(const fs::path & root)
{
  pcl::PointCloud<Point> cloud;
  Point first;
  first.x = 0.01F;
  first.y = 0.01F;
  first.z = 0.01F;
  first.intensity = 1.0F;
  cloud.push_back(first);
  Point second;
  second.x = 0.03F;
  second.y = 0.02F;
  second.z = 0.01F;
  second.intensity = 2.0F;
  cloud.push_back(second);
  Point third;
  third.x = 1.01F;
  third.y = 1.01F;
  third.z = 1.01F;
  third.intensity = 3.0F;
  cloud.push_back(third);
  Point invalid;
  invalid.y = 0.0F;
  invalid.z = 0.0F;
  invalid.intensity = 4.0F;
  invalid.x = std::numeric_limits<float>::quiet_NaN();
  cloud.push_back(invalid);
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1U;
  cloud.is_dense = false;
  const auto path = root / "map_3d_raw.pcd";
  if (pcl::io::savePCDFileBinary(path.string(), cloud) != 0) {
    throw std::runtime_error("fixture_pcd_write_failed");
  }
  return path;
}

Json load_json(const fs::path & path)
{
  std::ifstream input(path);
  return Json::parse(input);
}

TEST(GeometryVoxelMap, CreatesDeterministicAtomicMapAndReceipt)
{
  TemporaryDirectory temporary;
  const auto input = write_raw_map(temporary.path());
  const auto first = sunhorizon_livo_bringup::create_geometry_voxel_map(
    input, temporary.path() / "first", 0.2, kReleaseSha);
  const auto second = sunhorizon_livo_bringup::create_geometry_voxel_map(
    input, temporary.path() / "second", 0.2, kReleaseSha);

  EXPECT_EQ(first.loaded_point_count, 4U);
  EXPECT_EQ(first.finite_point_count, 3U);
  EXPECT_EQ(first.output_point_count, 2U);
  EXPECT_EQ(first.map_sha256, second.map_sha256);
  EXPECT_TRUE(fs::is_regular_file(first.map_path));
  EXPECT_TRUE(fs::is_regular_file(first.receipt_path));

  const Json receipt = load_json(first.receipt_path);
  EXPECT_EQ(receipt.at("schema_version"), 1);
  EXPECT_EQ(
    receipt.at("kind"),
    sunhorizon_livo_bringup::kGeometryVoxelReceiptKind);
  EXPECT_EQ(
    receipt.at("source").at("role"),
    sunhorizon_livo_bringup::kRawGeometrySourceRole);
  EXPECT_EQ(receipt.at("source").at("loaded_point_count"), 4);
  EXPECT_EQ(receipt.at("source").at("finite_point_count"), 3);
  EXPECT_EQ(receipt.at("source").at("rejected_nonfinite_point_count"), 1);
  EXPECT_EQ(
    receipt.at("algorithm").at("voxel_size_m"),
    Json::array({0.2, 0.2, 0.2}));
  EXPECT_EQ(
    receipt.at("algorithm").at("output_fields"),
    Json::array({"x", "y", "z"}));
  EXPECT_EQ(receipt.at("output").at("point_count"), 2);
  EXPECT_EQ(receipt.at("output").at("sha256"), first.map_sha256);
}

TEST(GeometryVoxelMap, RefusesExistingOutputWithoutModification)
{
  TemporaryDirectory temporary;
  const auto input = write_raw_map(temporary.path());
  const auto output = temporary.path() / "existing";
  fs::create_directory(output);
  const auto marker = output / "marker";
  std::ofstream(marker) << "preserve";

  EXPECT_THROW(
    sunhorizon_livo_bringup::create_geometry_voxel_map(
      input, output, 0.2, kReleaseSha),
    std::runtime_error);
  EXPECT_TRUE(fs::is_regular_file(marker));
  EXPECT_FALSE(fs::exists(output / "map.pcd"));
  EXPECT_FALSE(fs::exists(output / "receipt.json"));
}

TEST(GeometryVoxelMap, FailsClosedOnWrongSourceNameAndVoxelSize)
{
  TemporaryDirectory temporary;
  const auto input = write_raw_map(temporary.path());
  const auto renamed = temporary.path() / "colored_subset.pcd";
  fs::copy_file(input, renamed);

  EXPECT_THROW(
    sunhorizon_livo_bringup::create_geometry_voxel_map(
      renamed, temporary.path() / "wrong-source", 0.2, kReleaseSha),
    std::runtime_error);
  EXPECT_THROW(
    sunhorizon_livo_bringup::create_geometry_voxel_map(
      input, temporary.path() / "wrong-voxel", 0.0, kReleaseSha),
    std::runtime_error);
  EXPECT_FALSE(fs::exists(temporary.path() / "wrong-source"));
  EXPECT_FALSE(fs::exists(temporary.path() / "wrong-voxel"));
}

}  // namespace
