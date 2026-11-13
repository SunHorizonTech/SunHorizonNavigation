#include "sunhorizon_livo_bringup/geometry_voxel_map.hpp"

#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace
{

struct Arguments
{
  std::filesystem::path input;
  std::filesystem::path output_directory;
  double voxel_size_m{0.0};
  std::string release_sha;
};

[[noreturn]] void usage(const char * executable, const std::string & reason = "")
{
  if (!reason.empty()) {
    std::cerr << reason << '\n';
  }
  std::cerr << "usage: " << executable
            << " --input MAP_3D_RAW.PCD --output-directory DIRECTORY"
            << " --voxel-size-m METERS --release-sha FULL_GIT_SHA\n";
  std::exit(2);
}

Arguments parse_arguments(int argc, char ** argv)
{
  Arguments result;
  bool have_input = false;
  bool have_output = false;
  bool have_voxel = false;
  bool have_release = false;
  for (int index = 1; index < argc; ++index) {
    const std::string name(argv[index]);
    if (name == "--help" || name == "-h") {
      usage(argv[0]);
    }
    if (index + 1 >= argc) {
      usage(argv[0], "missing_value:" + name);
    }
    const std::string value(argv[++index]);
    if (name == "--input" && !have_input) {
      result.input = value;
      have_input = true;
    } else if (name == "--output-directory" && !have_output) {
      result.output_directory = value;
      have_output = true;
    } else if (name == "--voxel-size-m" && !have_voxel) {
      std::size_t consumed = 0;
      try {
        result.voxel_size_m = std::stod(value, &consumed);
      } catch (const std::exception &) {
        usage(argv[0], "invalid_voxel_size_m");
      }
      if (consumed != value.size()) {
        usage(argv[0], "invalid_voxel_size_m");
      }
      have_voxel = true;
    } else if (name == "--release-sha" && !have_release) {
      result.release_sha = value;
      have_release = true;
    } else {
      usage(argv[0], "unknown_or_duplicate_argument:" + name);
    }
  }
  if (!have_input || !have_output || !have_voxel || !have_release) {
    usage(argv[0], "required_argument_missing");
  }
  return result;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto arguments = parse_arguments(argc, argv);
    const auto result = sunhorizon_livo_bringup::create_geometry_voxel_map(
      arguments.input, arguments.output_directory, arguments.voxel_size_m,
      arguments.release_sha);
    const nlohmann::json summary = {
      {"map", result.map_path.string()},
      {"map_sha256", result.map_sha256},
      {"output_point_count", result.output_point_count},
      {"receipt", result.receipt_path.string()},
      {"voxel_size_m", result.voxel_size_m},
    };
    std::cout << summary.dump() << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "livo_geometry_voxelizer_failed:" << error.what() << '\n';
    return 1;
  }
}
