#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "sunhorizon_3d_localizer/ndt_backend.hpp"

namespace fs = std::filesystem;

namespace {

struct Pose {
  Eigen::Vector3f translation;
  Eigen::Quaternionf rotation;
};

struct ScanRecord {
  std::size_t scan_index{};
  fs::path path;
  Pose lidar_pose;
};

struct Trial {
  const char* name;
  float x;
  float y;
  float z;
  float yaw_deg;
  bool absolute;
};

constexpr std::array<Trial, 3> kTrials{{
    {"nominal", 0.0F, 0.0F, 0.0F, 0.0F, false},
    {"tracking", 0.20F, -0.10F, 0.0F, 5.0F, false},
    {"relocalization", 0.50F, 0.30F, 0.0F, 15.0F, false},
}};

constexpr std::size_t kMinimumBenchmarkPoints = 500;

struct CloudWithCounts {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::size_t full_points{};
  std::size_t selected_points{};
};

std::vector<std::string> split(const std::string& line, char delimiter) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, delimiter)) fields.push_back(field);
  return fields;
}

Eigen::Matrix3f axis_rotation() {
  Eigen::Matrix3f rotation;
  rotation << 0.0F, 0.0F, 1.0F,
             -1.0F, 0.0F, 0.0F,
              0.0F, -1.0F, 0.0F;
  return rotation;
}

Eigen::Matrix4f expected_transform(const Pose& pose) {
  const Eigen::Matrix3f axis = axis_rotation();
  Eigen::Matrix4f output = Eigen::Matrix4f::Identity();
  output.block<3, 3>(0, 0) = axis * pose.rotation.toRotationMatrix() * axis.transpose();
  output.block<3, 1>(0, 3) = axis * pose.translation;
  return output;
}

std::vector<ScanRecord> load_index(const fs::path& path, const fs::path& scan_root) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("scan_index_unreadable");
  std::vector<ScanRecord> records;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') continue;
    const auto fields = split(line, '\t');
    if (fields.size() != 9) throw std::runtime_error("scan_index_field_count_invalid");
    const auto parse_index_float = [&fields](std::size_t field_index) {
      std::size_t consumed = 0;
      const float value = std::stof(fields[field_index], &consumed);
      if (consumed != fields[field_index].size() || !std::isfinite(value)) {
        throw std::runtime_error("scan_index_number_invalid");
      }
      return value;
    };
    ScanRecord record;
    std::size_t scan_index_consumed = 0;
    record.scan_index = std::stoul(fields[0], &scan_index_consumed);
    if (scan_index_consumed != fields[0].size()) {
      throw std::runtime_error("scan_index_number_invalid");
    }
    const fs::path relative(fields[1]);
    if (relative.is_absolute() || relative.string().find("..") != std::string::npos) {
      throw std::runtime_error("scan_index_path_invalid");
    }
    record.path = scan_root / relative;
    record.lidar_pose.translation = Eigen::Vector3f(
        parse_index_float(2), parse_index_float(3), parse_index_float(4));
    record.lidar_pose.rotation = Eigen::Quaternionf(
        parse_index_float(8), parse_index_float(5), parse_index_float(6),
        parse_index_float(7));
    if (record.lidar_pose.rotation.norm() <= std::numeric_limits<float>::epsilon()) {
      throw std::runtime_error("scan_index_quaternion_invalid");
    }
    record.lidar_pose.rotation.normalize();
    records.push_back(record);
  }
  if (records.size() < 3) throw std::runtime_error("scan_index_too_short");
  return records;
}

CloudWithCounts load_and_filter(
    const fs::path& path, float leaf_size,
    const std::optional<std::pair<float, float>>& radial_range = std::nullopt,
    const std::optional<std::array<float, 3>>& xy_crop = std::nullopt) {
  auto input = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile(path.string(), *input) != 0 || input->empty()) {
    throw std::runtime_error("pcd_load_failed:" + path.string());
  }
  const std::size_t full_points = input->size();
  auto selected = input;
  const float minimum_range_squared = radial_range
      ? radial_range->first * radial_range->first : 0.0F;
  const float maximum_range_squared = radial_range
      ? radial_range->second * radial_range->second
      : std::numeric_limits<float>::infinity();
  const float crop_radius_squared = xy_crop
      ? (*xy_crop)[2] * (*xy_crop)[2] : std::numeric_limits<float>::infinity();
  if (radial_range || xy_crop) {
    selected.reset(new pcl::PointCloud<pcl::PointXYZ>());
    selected->reserve(std::min<std::size_t>(input->size(), 1000000));
    for (const auto& point : *input) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const float range_squared = point.x * point.x + point.y * point.y + point.z * point.z;
      if (radial_range &&
          (range_squared < minimum_range_squared || range_squared > maximum_range_squared)) {
        continue;
      }
      if (xy_crop) {
        const float dx = point.x - (*xy_crop)[0];
        const float dy = point.y - (*xy_crop)[1];
        if (dx * dx + dy * dy > crop_radius_squared) continue;
      }
      selected->push_back(point);
    }
  }
  if (selected->empty()) throw std::runtime_error("pcd_selection_empty:" + path.string());
  if ((radial_range || xy_crop) && selected->size() < kMinimumBenchmarkPoints) {
    throw std::runtime_error(
        std::string(radial_range ? "scan_range_too_sparse:" : "map_crop_too_sparse:") +
        path.string());
  }
  auto output = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.setInputCloud(selected);
  voxel.filter(*output);
  if (output->empty()) throw std::runtime_error("pcd_filter_empty:" + path.string());
  if ((radial_range || xy_crop) && output->size() < kMinimumBenchmarkPoints) {
    throw std::runtime_error(
        std::string(radial_range ? "scan_voxel_too_sparse:" : "map_voxel_too_sparse:") +
        path.string());
  }
  return {output, full_points, selected->size()};
}

float yaw(const Eigen::Matrix3f& rotation) {
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

float wrap(float angle) {
  constexpr float kPi = 3.14159265358979323846F;
  while (angle > kPi) angle -= 2.0F * kPi;
  while (angle < -kPi) angle += 2.0F * kPi;
  return angle;
}

Eigen::Matrix4f perturbation(const Trial& trial) {
  constexpr float kPi = 3.14159265358979323846F;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = Eigen::AngleAxisf(
      trial.yaw_deg * kPi / 180.0F, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  transform(0, 3) = trial.x;
  transform(1, 3) = trial.y;
  transform(2, 3) = trial.z;
  return transform;
}

struct Arguments {
  fs::path map;
  fs::path scan_root;
  fs::path scan_index;
  fs::path output;
  float map_leaf{0.20F};
  float scan_leaf{0.20F};
  float resolution{1.0F};
  int iterations{50};
  std::size_t maximum_scans{0};
  std::optional<float> minimum_scan_range_m;
  std::optional<float> maximum_scan_range_m;
  std::optional<float> map_crop_center_x;
  std::optional<float> map_crop_center_y;
  std::optional<float> map_crop_radius_m;
  std::optional<float> initial_x;
  std::optional<float> initial_y;
  std::optional<float> initial_z;
  std::optional<float> initial_yaw_deg;
  std::string backend{"pcl"};
  std::optional<int> threads;
  std::optional<std::string> neighborhood;
};

float parse_finite_float(const std::string& key, const std::string& value) {
  std::size_t consumed = 0;
  const float parsed = std::stof(value, &consumed);
  if (consumed != value.size()) throw std::runtime_error("argument_invalid:" + key);
  if (!std::isfinite(parsed)) throw std::runtime_error("argument_not_finite:" + key);
  return parsed;
}

int parse_int(const std::string& key, const std::string& value) {
  std::size_t consumed = 0;
  const int parsed = std::stoi(value, &consumed);
  if (consumed != value.size()) throw std::runtime_error("argument_invalid:" + key);
  return parsed;
}

Arguments parse_arguments(int argc, char** argv) {
  Arguments output;
  for (int index = 1; index < argc; ++index) {
    const std::string key(argv[index]);
    if (index + 1 >= argc) throw std::runtime_error("argument_value_missing:" + key);
    const std::string value(argv[++index]);
    if (key == "--map") output.map = value;
    else if (key == "--scan-root") output.scan_root = value;
    else if (key == "--scan-index") output.scan_index = value;
    else if (key == "--output") output.output = value;
    else if (key == "--map-leaf") output.map_leaf = parse_finite_float(key, value);
    else if (key == "--scan-leaf") output.scan_leaf = parse_finite_float(key, value);
    else if (key == "--resolution") output.resolution = parse_finite_float(key, value);
    else if (key == "--iterations") output.iterations = std::stoi(value);
    else if (key == "--max-scans") output.maximum_scans = std::stoul(value);
    else if (key == "--backend") output.backend = value;
    else if (key == "--threads") output.threads = parse_int(key, value);
    else if (key == "--neighborhood") output.neighborhood = value;
    else if (key == "--minimum-scan-range-m") {
      output.minimum_scan_range_m = parse_finite_float(key, value);
    } else if (key == "--maximum-scan-range-m") {
      output.maximum_scan_range_m = parse_finite_float(key, value);
    } else if (key == "--map-crop-center-x") {
      output.map_crop_center_x = parse_finite_float(key, value);
    } else if (key == "--map-crop-center-y") {
      output.map_crop_center_y = parse_finite_float(key, value);
    } else if (key == "--map-crop-radius-m") {
      output.map_crop_radius_m = parse_finite_float(key, value);
    } else if (key == "--initial-x") {
      output.initial_x = parse_finite_float(key, value);
    } else if (key == "--initial-y") {
      output.initial_y = parse_finite_float(key, value);
    } else if (key == "--initial-z") {
      output.initial_z = parse_finite_float(key, value);
    } else if (key == "--initial-yaw-deg") {
      output.initial_yaw_deg = parse_finite_float(key, value);
    }
    else throw std::runtime_error("unknown_argument:" + key);
  }
  if (output.map.empty() || output.scan_root.empty() || output.scan_index.empty() ||
      output.output.empty()) throw std::runtime_error("required_argument_missing");
  if (output.map_leaf <= 0.0F || output.scan_leaf <= 0.0F ||
      output.resolution <= 0.0F || output.iterations < 1) {
    throw std::runtime_error("ndt_parameter_invalid");
  }
  const bool any_scan_range = output.minimum_scan_range_m || output.maximum_scan_range_m;
  if (any_scan_range && (!output.minimum_scan_range_m || !output.maximum_scan_range_m)) {
    throw std::runtime_error("scan_range_requires_minimum_and_maximum");
  }
  if (any_scan_range && (*output.minimum_scan_range_m < 0.0F ||
      *output.maximum_scan_range_m <= *output.minimum_scan_range_m)) {
    throw std::runtime_error("scan_range_invalid");
  }
  const bool any_map_crop = output.map_crop_center_x || output.map_crop_center_y ||
      output.map_crop_radius_m;
  if (any_map_crop && (!output.map_crop_center_x || !output.map_crop_center_y ||
      !output.map_crop_radius_m)) {
    throw std::runtime_error("map_crop_requires_center_and_radius");
  }
  if (any_map_crop && *output.map_crop_radius_m <= 0.0F) {
    throw std::runtime_error("map_crop_radius_invalid");
  }
  const bool any_fixed_initial = output.initial_x || output.initial_y || output.initial_z ||
      output.initial_yaw_deg;
  if (any_fixed_initial && (!output.initial_x || !output.initial_y || !output.initial_z ||
      !output.initial_yaw_deg)) {
    throw std::runtime_error("fixed_initial_requires_xyz_and_yaw");
  }
  if (fs::exists(output.output)) throw std::runtime_error("output_exists");
  return output;
}

}  // namespace

int main(int argc, char** argv) {
  fs::path created_output;
  try {
    const auto arguments = parse_arguments(argc, argv);
    const std::optional<std::array<float, 3>> map_crop = arguments.map_crop_radius_m
        ? std::make_optional(std::array<float, 3>{
              *arguments.map_crop_center_x, *arguments.map_crop_center_y,
              *arguments.map_crop_radius_m})
        : std::nullopt;
    const auto target = load_and_filter(
        arguments.map, arguments.map_leaf, std::nullopt, map_crop);
    const auto records = load_index(arguments.scan_index, arguments.scan_root);
    const std::size_t scan_count = arguments.maximum_scans == 0
        ? records.size() : std::min(arguments.maximum_scans, records.size());
    const auto backend_options =
        sunhorizon_3d_localizer::resolve_ndt_backend_options(
            arguments.backend, arguments.threads, arguments.neighborhood);
    sunhorizon_3d_localizer::NdtRegistration ndt(backend_options);
    ndt.set_transformation_epsilon(0.005);
    ndt.set_step_size(0.1);
    ndt.set_resolution(arguments.resolution);
    ndt.set_maximum_iterations(arguments.iterations);
    // The default path remains pcl::NormalDistributionsTransform semantics;
    // the common wrapper performs ndt.setInputTarget(target.cloud); exactly once.
    ndt.set_input_target(target.cloud);
    const std::string backend_name =
        sunhorizon_3d_localizer::to_string(backend_options.backend);
    const std::string neighborhood_name =
        sunhorizon_3d_localizer::to_string(backend_options.neighborhood);
    std::cerr << "ndt_backend=" << backend_name
              << " threads=" << backend_options.threads
              << " neighborhood=" << neighborhood_name << '\n';
    std::vector<Trial> trials(kTrials.begin(), kTrials.end());
    if (arguments.initial_x) {
      trials = {{"fixed", *arguments.initial_x, *arguments.initial_y, *arguments.initial_z,
                 *arguments.initial_yaw_deg, true}};
    }
    std::ofstream output(arguments.output, std::ios::out | std::ios::trunc);
    if (!output) throw std::runtime_error("output_open_failed");
    created_output = arguments.output;
    output << "scan_index,trial,converged,iterations,fitness,translation_error_m,yaw_error_deg,";
    output << "result_x,result_y,result_z,result_yaw_deg,runtime_ms,";
    output << "source_raw_points,source_range_points,source_filtered_points,";
    output << "target_full_points,target_cropped_points,target_filtered_points,";
    output << "minimum_scan_range_m,maximum_scan_range_m,map_crop_enabled,";
    output << "map_crop_center_x,map_crop_center_y,map_crop_radius_m,";
    output << "initial_x,initial_y,initial_z,initial_yaw_deg,";
    output << "initial_to_result_translation_m,initial_to_result_yaw_deg,";
    output << "backend,threads,neighborhood\n";
    output << std::setprecision(9);
    for (std::size_t record_index = 0; record_index < scan_count; ++record_index) {
      const auto& record = records[record_index];
      const std::optional<std::pair<float, float>> radial_range =
          arguments.minimum_scan_range_m
          ? std::make_optional(std::pair<float, float>{
                *arguments.minimum_scan_range_m, *arguments.maximum_scan_range_m})
          : std::nullopt;
      const auto source = load_and_filter(
          record.path, arguments.scan_leaf, radial_range, std::nullopt);
      ndt.set_input_source(source.cloud);
      const Eigen::Matrix4f expected = expected_transform(record.lidar_pose);
      for (const auto& trial : trials) {
        pcl::PointCloud<pcl::PointXYZ> aligned;
        const Eigen::Matrix4f initial = trial.absolute
            ? perturbation(trial) : perturbation(trial) * expected;
        const auto started = std::chrono::steady_clock::now();
        ndt.align(aligned, initial);
        const auto runtime_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started).count();
        const Eigen::Matrix4f result = ndt.final_transformation();
        const Eigen::Matrix4f error = expected.inverse() * result;
        const float translation_error = error.block<3, 1>(0, 3).norm();
        constexpr float kDegrees = 57.29577951308232F;
        const float yaw_error = std::abs(wrap(yaw(error.block<3, 3>(0, 0)))) * kDegrees;
        const Eigen::Matrix4f initial_to_result = initial.inverse() * result;
        const float initial_to_result_translation =
            initial_to_result.block<3, 1>(0, 3).norm();
        const float initial_to_result_yaw =
            std::abs(wrap(yaw(initial_to_result.block<3, 3>(0, 0)))) * kDegrees;
        output << record.scan_index << ',' << trial.name << ','
               << (ndt.has_converged() ? 1 : 0) << ',' << ndt.final_iteration_count() << ','
               << ndt.fitness_score() << ',' << translation_error << ',' << yaw_error << ','
               << result(0, 3) << ',' << result(1, 3) << ',' << result(2, 3) << ','
               << yaw(result.block<3, 3>(0, 0)) * kDegrees << ',' << runtime_ms << ','
               << source.full_points << ',' << source.selected_points << ','
               << source.cloud->size() << ',' << target.full_points << ','
               << target.selected_points << ',' << target.cloud->size() << ','
               << (arguments.minimum_scan_range_m
                       ? *arguments.minimum_scan_range_m : 0.0F) << ','
               << (arguments.maximum_scan_range_m
                       ? *arguments.maximum_scan_range_m
                       : std::numeric_limits<float>::infinity()) << ','
               << (map_crop ? 1 : 0) << ','
               << (arguments.map_crop_center_x ? *arguments.map_crop_center_x : 0.0F) << ','
               << (arguments.map_crop_center_y ? *arguments.map_crop_center_y : 0.0F) << ','
               << (arguments.map_crop_radius_m ? *arguments.map_crop_radius_m : 0.0F) << ','
               << initial(0, 3) << ',' << initial(1, 3) << ',' << initial(2, 3) << ','
               << yaw(initial.block<3, 3>(0, 0)) * kDegrees << ','
               << initial_to_result_translation << ',' << initial_to_result_yaw << ','
               << backend_name << ',' << backend_options.threads << ','
               << neighborhood_name << '\n';
        output.flush();
        if (!output) throw std::runtime_error("output_write_failed");
        std::cerr << "scan=" << record.scan_index << " trial=" << trial.name
                  << " backend=" << backend_name
                  << " threads=" << backend_options.threads
                  << " neighborhood=" << neighborhood_name
                  << " runtime_ms=" << runtime_ms << " converged="
                  << (ndt.has_converged() ? 1 : 0) << '\n';
      }
    }
    output.flush();
    if (!output) throw std::runtime_error("output_write_failed");
    created_output.clear();
    return 0;
  } catch (const std::exception& error) {
    if (!created_output.empty()) {
      std::error_code ignored;
      fs::remove(created_output, ignored);
    }
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}
