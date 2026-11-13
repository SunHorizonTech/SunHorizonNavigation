#include "sunhorizon_livo_bringup/geometry_voxel_map.hpp"

#include <openssl/evp.h>

#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <tuple>

#include <nlohmann/json.hpp>
#include <unistd.h>

namespace sunhorizon_livo_bringup
{
namespace
{

namespace fs = std::filesystem;
using Json = nlohmann::json;
using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

constexpr double kMinimumVoxelSizeM = 0.01;
constexpr double kMaximumVoxelSizeM = 5.0;
constexpr char kExpectedInputName[] = "map_3d_raw.pcd";
constexpr char kOutputMapName[] = "map.pcd";
constexpr char kOutputReceiptName[] = "receipt.json";

bool path_entry_exists(const fs::path & path)
{
  std::error_code error;
  const auto status = fs::symlink_status(path, error);
  if (error == std::errc::no_such_file_or_directory) {
    return false;
  }
  if (error) {
    throw std::runtime_error("path_status_failed:" + path.string());
  }
  return status.type() != fs::file_type::not_found;
}

void require_regular_file(const fs::path & path, const std::string & label)
{
  std::error_code error;
  const auto link_status = fs::symlink_status(path, error);
  if (error || link_status.type() == fs::file_type::symlink ||
    link_status.type() != fs::file_type::regular)
  {
    throw std::runtime_error("invalid_regular_file:" + label);
  }
  if (fs::file_size(path) == 0U) {
    throw std::runtime_error("empty_file:" + label);
  }
}

fs::path make_staging_directory(const fs::path & output)
{
  const auto parent = output.parent_path();
  const auto base = "." + output.filename().string() + ".tmp." +
    std::to_string(static_cast<long long>(::getpid()));
  for (int attempt = 0; attempt < 100; ++attempt) {
    const auto candidate = parent / (base + "." + std::to_string(attempt));
    std::error_code error;
    if (fs::create_directory(candidate, error)) {
      return candidate;
    }
    if (error && error != std::errc::file_exists) {
      throw std::runtime_error("staging_directory_create_failed:" + error.message());
    }
  }
  throw std::runtime_error("staging_directory_exhausted");
}

std::string bytes_to_hex(const unsigned char * data, std::size_t size)
{
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < size; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(data[index]);
  }
  return output.str();
}

void write_receipt(const fs::path & path, const Json & receipt)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("receipt_open_failed");
  }
  output << std::setw(2) << receipt << '\n';
  output.flush();
  if (!output) {
    throw std::runtime_error("receipt_write_failed");
  }
}

std::string pcl_version()
{
  return std::to_string(PCL_MAJOR_VERSION) + "." +
         std::to_string(PCL_MINOR_VERSION) + "." +
         std::to_string(PCL_REVISION_VERSION);
}

void validate_release_sha(const std::string & release_sha)
{
  static const std::regex pattern("^[0-9a-f]{40}$");
  if (!std::regex_match(release_sha, pattern)) {
    throw std::runtime_error("tool_release_sha_invalid");
  }
}

}  // namespace

std::string sha256_file(const fs::path & path)
{
  require_regular_file(path, "sha256_input");
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("sha256_open_failed");
  }

  using Context = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;
  Context context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("sha256_init_failed");
  }

  std::array<char, 1024 * 1024> buffer{};
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = input.gcount();
    if (count > 0 && EVP_DigestUpdate(
        context.get(), buffer.data(), static_cast<std::size_t>(count)) != 1)
    {
      throw std::runtime_error("sha256_update_failed");
    }
  }
  if (!input.eof()) {
    throw std::runtime_error("sha256_read_failed");
  }

  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1 ||
    digest_size != 32U)
  {
    throw std::runtime_error("sha256_final_failed");
  }
  return bytes_to_hex(digest.data(), digest_size);
}

GeometryVoxelResult create_geometry_voxel_map(
  const fs::path & input_raw_geometry,
  const fs::path & output_directory,
  double voxel_size_m,
  const std::string & tool_release_sha)
{
  validate_release_sha(tool_release_sha);
  if (!std::isfinite(voxel_size_m) || voxel_size_m < kMinimumVoxelSizeM ||
    voxel_size_m > kMaximumVoxelSizeM)
  {
    throw std::runtime_error("voxel_size_m_out_of_range");
  }

  const fs::path input = fs::absolute(input_raw_geometry).lexically_normal();
  const fs::path output = fs::absolute(output_directory).lexically_normal();
  require_regular_file(input, "input_raw_geometry");
  if (input.filename() != kExpectedInputName) {
    throw std::runtime_error("input_must_be_map_3d_raw_pcd");
  }
  if (path_entry_exists(output)) {
    throw std::runtime_error("output_directory_exists");
  }

  fs::create_directories(output.parent_path());
  if (!fs::is_directory(output.parent_path())) {
    throw std::runtime_error("output_parent_not_directory");
  }

  const auto source_size_before = fs::file_size(input);
  const auto source_time_before = fs::last_write_time(input);
  const auto source_sha256 = sha256_file(input);

  PointCloud::Ptr cloud(new PointCloud());
  if (pcl::io::loadPCDFile<Point>(input.string(), *cloud) != 0 || cloud->empty()) {
    throw std::runtime_error("raw_geometry_pcd_load_failed");
  }
  const auto loaded_point_count = static_cast<std::uint64_t>(cloud->size());
  cloud->points.erase(
    std::remove_if(
      cloud->points.begin(), cloud->points.end(),
      [](const Point & point) {return !pcl::isFinite(point);}),
    cloud->points.end());
  if (cloud->empty()) {
    throw std::runtime_error("raw_geometry_has_no_finite_points");
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1U;
  cloud->is_dense = true;
  const auto finite_point_count = static_cast<std::uint64_t>(cloud->size());

  PointCloud filtered;
  pcl::VoxelGrid<Point> voxel;
  const auto leaf = static_cast<float>(voxel_size_m);
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(filtered);
  if (filtered.empty()) {
    throw std::runtime_error("voxel_map_is_empty");
  }
  std::sort(
    filtered.points.begin(), filtered.points.end(),
    [](const Point & left, const Point & right) {
      return std::tie(left.x, left.y, left.z) < std::tie(right.x, right.y, right.z);
    });
  filtered.width = static_cast<std::uint32_t>(filtered.size());
  filtered.height = 1U;
  filtered.is_dense = true;

  if (fs::file_size(input) != source_size_before ||
    fs::last_write_time(input) != source_time_before)
  {
    throw std::runtime_error("raw_geometry_changed_during_processing");
  }

  const fs::path staging = make_staging_directory(output);
  try {
    const fs::path map_path = staging / kOutputMapName;
    if (pcl::io::savePCDFileBinary(map_path.string(), filtered) != 0) {
      throw std::runtime_error("voxel_map_write_failed");
    }
    require_regular_file(map_path, "staged_voxel_map");

    PointCloud verification;
    if (pcl::io::loadPCDFile<Point>(map_path.string(), verification) != 0 ||
      verification.size() != filtered.size() || verification.empty())
    {
      throw std::runtime_error("voxel_map_reload_mismatch");
    }
    if (std::any_of(
        verification.begin(), verification.end(),
        [](const Point & point) {return !pcl::isFinite(point);}))
    {
      throw std::runtime_error("voxel_map_contains_nonfinite_points");
    }

    const auto map_sha256 = sha256_file(map_path);
    const auto map_size_bytes = fs::file_size(map_path);
    const Json receipt = {
      {"schema_version", 1},
      {"kind", kGeometryVoxelReceiptKind},
      {"tool_release_sha", tool_release_sha},
      {"source", {
          {"role", kRawGeometrySourceRole},
          {"path", kExpectedInputName},
          {"sha256", source_sha256},
          {"size_bytes", source_size_before},
          {"loaded_point_count", loaded_point_count},
          {"finite_point_count", finite_point_count},
          {"rejected_nonfinite_point_count", loaded_point_count - finite_point_count},
        }},
      {"algorithm", {
          {"implementation", "pcl::VoxelGrid<pcl::PointXYZ>"},
          {"pcl_version", pcl_version()},
          {"voxel_size_m", {voxel_size_m, voxel_size_m, voxel_size_m}},
          {"output_fields", {"x", "y", "z"}},
          {"output_format", "pcd_binary"},
          {"deterministic_output_order", "lexicographic_xyz"},
        }},
      {"output", {
          {"role", kNdtGeometryMapRole},
          {"path", kOutputMapName},
          {"sha256", map_sha256},
          {"size_bytes", map_size_bytes},
          {"point_count", filtered.size()},
        }},
    };
    write_receipt(staging / kOutputReceiptName, receipt);
    require_regular_file(staging / kOutputReceiptName, "staged_receipt");

    fs::rename(staging, output);
    return GeometryVoxelResult{
      output,
      output / kOutputMapName,
      output / kOutputReceiptName,
      source_sha256,
      map_sha256,
      source_size_before,
      map_size_bytes,
      loaded_point_count,
      finite_point_count,
      static_cast<std::uint64_t>(filtered.size()),
      voxel_size_m,
    };
  } catch (...) {
    std::error_code ignored;
    fs::remove_all(staging, ignored);
    throw;
  }
}

}  // namespace sunhorizon_livo_bringup
