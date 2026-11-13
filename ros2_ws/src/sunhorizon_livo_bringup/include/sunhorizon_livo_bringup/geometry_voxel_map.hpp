#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

namespace sunhorizon_livo_bringup
{

inline constexpr char kGeometryVoxelReceiptKind[] =
  "sunhorizon_livo_raw_geometry_voxel_receipt";
inline constexpr char kRawGeometrySourceRole[] =
  "fast_livo2_raw_geometry";
inline constexpr char kNdtGeometryMapRole[] =
  "ndt_fixed_map_geometry";

struct GeometryVoxelResult
{
  std::filesystem::path output_directory;
  std::filesystem::path map_path;
  std::filesystem::path receipt_path;
  std::string source_sha256;
  std::string map_sha256;
  std::uintmax_t source_size_bytes{0};
  std::uintmax_t map_size_bytes{0};
  std::uint64_t loaded_point_count{0};
  std::uint64_t finite_point_count{0};
  std::uint64_t output_point_count{0};
  double voxel_size_m{0.0};
};

std::string sha256_file(const std::filesystem::path & path);

GeometryVoxelResult create_geometry_voxel_map(
  const std::filesystem::path & input_raw_geometry,
  const std::filesystem::path & output_directory,
  double voxel_size_m,
  const std::string & tool_release_sha);

}  // namespace sunhorizon_livo_bringup
