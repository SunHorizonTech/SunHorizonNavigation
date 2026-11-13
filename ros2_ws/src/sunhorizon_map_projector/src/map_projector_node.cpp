#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <openssl/evp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sunhorizon_navigation_core/projection_grid.hpp>

namespace {

namespace fs = std::filesystem;
using sunhorizon_navigation_core::Point3D;
using sunhorizon_navigation_core::ProjectionConfig;
using sunhorizon_navigation_core::ProjectionGrid;

std::string sha256_file(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot open for hashing: " + path.string());
  }
  EVP_MD_CTX* context = EVP_MD_CTX_new();
  if (context == nullptr || EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1) {
    if (context != nullptr) EVP_MD_CTX_free(context);
    throw std::runtime_error("cannot initialize SHA-256");
  }
  std::array<char, 65536> buffer{};
  while (stream) {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    if (stream.gcount() > 0) {
      EVP_DigestUpdate(context, buffer.data(), static_cast<std::size_t>(stream.gcount()));
    }
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int length = 0;
  EVP_DigestFinal_ex(context, digest.data(), &length);
  EVP_MD_CTX_free(context);
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < length; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return output.str();
}

void atomic_write(const fs::path& path, const std::string& content, bool binary = false) {
  const fs::path temporary = path.string() + ".tmp";
  std::ofstream stream(temporary, binary ? std::ios::binary : std::ios::out);
  if (!stream) throw std::runtime_error("cannot write: " + temporary.string());
  stream.write(content.data(), static_cast<std::streamsize>(content.size()));
  stream.close();
  if (!stream) throw std::runtime_error("write failed: " + temporary.string());
  fs::rename(temporary, path);
}

void atomic_copy(const fs::path& source, const fs::path& destination) {
  const fs::path temporary = destination.string() + ".tmp";
  fs::copy_file(source, temporary, fs::copy_options::overwrite_existing);
  fs::rename(temporary, destination);
}

bool safe_map_id(const std::string& value) {
  if (value.empty() || value == "." || value == "..") return false;
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isalnum(character) != 0 || character == '-' || character == '_' ||
        character == '.';
  });
}

class MapProjector final : public rclcpp::Node {
 public:
  MapProjector() : Node("sunhorizon_map_projector") {
    input_pcd_ = declare_parameter("input_pcd", "");
    output_directory_ = declare_parameter("output_directory", "/data/navigation/maps");
    map_id_ = declare_parameter("map_id", "");
    project_on_startup_ = declare_parameter("project_on_startup", false);
    config_.resolution = declare_parameter("resolution", 0.05);
    config_.min_ground_z = declare_parameter("min_ground_z", -0.25);
    config_.max_ground_z = declare_parameter("max_ground_z", 0.10);
    config_.min_obstacle_z = declare_parameter("min_obstacle_z", 0.10);
    config_.max_obstacle_z = declare_parameter("max_obstacle_z", 1.80);
    config_.padding_m = declare_parameter("padding_m", 1.0);
    config_.minimum_ground_points_per_cell = static_cast<unsigned int>(
        declare_parameter("minimum_ground_points_per_cell", 1));
    config_.minimum_points_per_cell = static_cast<unsigned int>(
        declare_parameter("minimum_obstacle_points_per_cell", 2));
    config_.free_space_dilation_radius_cells = static_cast<unsigned int>(
        declare_parameter("free_space_dilation_radius_cells", 1));
    config_.wall_closing_radius_cells = static_cast<unsigned int>(
        declare_parameter("wall_closing_radius_cells", 1));
    config_.unknown_border_cells = static_cast<unsigned int>(
        declare_parameter("unknown_border_cells", 1));
    service_ = create_service<std_srvs::srv::Trigger>(
        "/sunhorizon/map/project",
        std::bind(&MapProjector::on_project, this, std::placeholders::_1, std::placeholders::_2));
    if (project_on_startup_) {
      startup_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        startup_timer_->cancel();
        try {
          RCLCPP_INFO(get_logger(), "%s", project().c_str());
        } catch (const std::exception& error) {
          RCLCPP_ERROR(get_logger(), "Projection failed: %s", error.what());
        }
      });
    }
  }

 private:
  std::string project() const {
    if (input_pcd_.empty() || map_id_.empty()) {
      throw std::runtime_error("input_pcd and map_id are required");
    }
    if (!safe_map_id(map_id_)) {
      throw std::runtime_error("map_id may contain only letters, digits, dot, dash and underscore");
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(input_pcd_, cloud) < 0) {
      throw std::runtime_error("cannot load PCD: " + input_pcd_);
    }
    std::vector<Point3D> points;
    points.reserve(cloud.size());
    for (const auto& point : cloud) points.push_back({point.x, point.y, point.z});
    const ProjectionGrid grid = sunhorizon_navigation_core::project_points(points, config_);

    const fs::path map_directory = fs::path(output_directory_) / map_id_;
    fs::create_directories(map_directory);
    const fs::path pcd = map_directory / "map.pcd";
    const fs::path image = map_directory / "map.pgm";
    const fs::path yaml = map_directory / "map.yaml";
    const fs::path manifest = map_directory / "manifest.json";

    if (!fs::exists(pcd) || !fs::equivalent(input_pcd_, pcd)) {
      atomic_copy(input_pcd_, pcd);
    }

    std::ostringstream pgm;
    pgm << "P5\n" << grid.width << " " << grid.height << "\n255\n";
    for (unsigned int row = 0; row < grid.height; ++row) {
      const unsigned int y = grid.height - row - 1U;
      for (unsigned int x = 0; x < grid.width; ++x) {
        const int value = grid.cells[static_cast<std::size_t>(y) * grid.width + x];
        pgm.put(static_cast<char>(value >= 100 ? 0 : (value < 0 ? 205 : 254)));
      }
    }
    atomic_write(image, pgm.str(), true);

    std::ostringstream yaml_text;
    yaml_text << std::setprecision(17)
              << "image: map.pgm\n"
              << "mode: trinary\n"
              << "resolution: " << grid.resolution << "\n"
              << "origin: [" << grid.origin_x << ", " << grid.origin_y << ", 0.0]\n"
              << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
    atomic_write(yaml, yaml_text.str());

    const std::string pcd_hash = sha256_file(pcd);
    const std::string pgm_hash = sha256_file(image);
    const std::string yaml_hash = sha256_file(yaml);
    std::ostringstream manifest_text;
    manifest_text << std::setprecision(17)
                  << "{\n"
                  << "  \"schema_version\": 2,\n"
                  << "  \"map_id\": \"" << map_id_ << "\",\n"
                  << "  \"map_pcd_sha256\": \"" << pcd_hash << "\",\n"
                  << "  \"map_pgm_sha256\": \"" << pgm_hash << "\",\n"
                  << "  \"map_yaml_sha256\": \"" << yaml_hash << "\",\n"
                  << "  \"resolution\": " << config_.resolution << ",\n"
                  << "  \"ground_z\": [" << config_.min_ground_z << ", "
                  << config_.max_ground_z << "],\n"
                  << "  \"obstacle_z\": [" << config_.min_obstacle_z << ", "
                  << config_.max_obstacle_z << "],\n"
                  << "  \"padding_m\": " << config_.padding_m << ",\n"
                  << "  \"minimum_ground_points_per_cell\": "
                  << config_.minimum_ground_points_per_cell << ",\n"
                  << "  \"minimum_obstacle_points_per_cell\": "
                  << config_.minimum_points_per_cell << ",\n"
                  << "  \"free_space_dilation_radius_cells\": "
                  << config_.free_space_dilation_radius_cells << ",\n"
                  << "  \"wall_closing_radius_cells\": "
                  << config_.wall_closing_radius_cells << ",\n"
                  << "  \"unknown_border_cells\": "
                  << config_.unknown_border_cells << ",\n"
                  << "  \"width\": " << grid.width << ",\n"
                  << "  \"height\": " << grid.height << "\n"
                  << "}\n";
    atomic_write(manifest, manifest_text.str());
    return "projected map " + map_id_ + " manifest_sha256=" + sha256_file(manifest);
  }

  void on_project(
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
    try {
      response->message = project();
      response->success = true;
    } catch (const std::exception& error) {
      response->success = false;
      response->message = error.what();
    }
  }

  std::string input_pcd_;
  std::string output_directory_;
  std::string map_id_;
  bool project_on_startup_{false};
  ProjectionConfig config_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapProjector>());
  rclcpp::shutdown();
  return 0;
}
