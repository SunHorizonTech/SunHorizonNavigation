#pragma once

#include <filesystem>
#include <string>

namespace sunhorizon_navigation_server {

struct MapBundleValidationResult {
  bool valid{false};
  std::string manifest_sha256;
  std::string reason;
};

MapBundleValidationResult validate_map_bundle(
    const std::filesystem::path& map_root,
    const std::string& map_id,
    const std::string& expected_profile);

}  // namespace sunhorizon_navigation_server
