#include "sunhorizon_navigation_server/map_bundle_validator.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <json-c/json.h>
#include <openssl/evp.h>

namespace sunhorizon_navigation_server {
namespace {

namespace fs = std::filesystem;

constexpr std::uintmax_t kMaximumJsonBytes = 1024U * 1024U;
constexpr std::array<const char*, 8> kCommonAssets{{
    "map.pgm",
    "map.yaml",
    "keepout.pgm",
    "keepout.yaml",
    "robot.urdf",
    "calibration.yaml",
    "trajectory_report.json",
    "metadata.json",
}};
constexpr std::array<const char*, 3> kVisualAssets{{
    "rtabmap.db",
    "map.pcd",
    "backend.yaml",
}};
constexpr std::array<const char*, 2> kLidarAssets{{
    "map.pcd",
    "backend.yaml",
}};

struct JsonDeleter {
  void operator()(json_object* value) const {
    if (value != nullptr) json_object_put(value);
  }
};
using JsonPointer = std::unique_ptr<json_object, JsonDeleter>;

bool regular_nonempty_file(const fs::path& path) {
  std::error_code error;
  const auto status = fs::symlink_status(path, error);
  if (error || fs::is_symlink(status) || !fs::is_regular_file(status)) return false;
  const auto size = fs::file_size(path, error);
  return !error && size > 0U;
}

std::string sha256_file(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("cannot_open:" + path.string());
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(
      EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("sha256_init_failed");
  }
  std::array<char, 65536> buffer{};
  while (stream) {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    if (stream.gcount() > 0 &&
        EVP_DigestUpdate(
            context.get(), buffer.data(), static_cast<std::size_t>(stream.gcount())) != 1) {
      throw std::runtime_error("sha256_update_failed");
    }
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int length = 0;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &length) != 1) {
    throw std::runtime_error("sha256_final_failed");
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < length; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return output.str();
}

bool safe_map_id(const std::string& value) {
  if (value.empty() || value.size() > 128U ||
      std::isalnum(static_cast<unsigned char>(value.front())) == 0) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isalnum(character) != 0 || character == '-' || character == '_' ||
        character == '.';
  });
}

bool supported_profile(const std::string& value) {
  return value == "visual" || value == "lidar";
}

std::string read_bounded_file(const fs::path& path) {
  if (!regular_nonempty_file(path)) {
    throw std::runtime_error("missing_or_empty:" + path.filename().string());
  }
  std::error_code error;
  const auto size = fs::file_size(path, error);
  if (error || size > kMaximumJsonBytes) {
    throw std::runtime_error("json_file_too_large:" + path.filename().string());
  }
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("cannot_open:" + path.string());
  return std::string(
      std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}

JsonPointer parse_json_object(const fs::path& path) {
  const std::string document = read_bounded_file(path);
  std::unique_ptr<json_tokener, decltype(&json_tokener_free)> tokener(
      json_tokener_new_ex(16), json_tokener_free);
  if (!tokener) throw std::runtime_error("json_parser_allocation_failed");
  json_tokener_set_flags(
      tokener.get(), JSON_TOKENER_STRICT | JSON_TOKENER_VALIDATE_UTF8);
  json_object* parsed = json_tokener_parse_ex(
      tokener.get(), document.data(), static_cast<int>(document.size()));
  const auto parse_error = json_tokener_get_error(tokener.get());
  const std::size_t parse_end = json_tokener_get_parse_end(tokener.get());
  if (parse_error != json_tokener_success || parsed == nullptr) {
    if (parsed != nullptr) json_object_put(parsed);
    throw std::runtime_error("invalid_json:" + path.filename().string());
  }
  for (std::size_t index = parse_end; index < document.size(); ++index) {
    if (std::isspace(static_cast<unsigned char>(document[index])) == 0) {
      json_object_put(parsed);
      throw std::runtime_error("invalid_json_trailing_data:" + path.filename().string());
    }
  }
  JsonPointer output(parsed);
  if (!json_object_is_type(output.get(), json_type_object)) {
    throw std::runtime_error("json_root_must_be_object:" + path.filename().string());
  }
  return output;
}

json_object* member(json_object* object, const char* key, json_type expected_type) {
  json_object* value = nullptr;
  if (!json_object_object_get_ex(object, key, &value) ||
      !json_object_is_type(value, expected_type)) {
    throw std::runtime_error("invalid_json_field:" + std::string(key));
  }
  return value;
}

std::string string_value(json_object* object, const char* key) {
  json_object* value = member(object, key, json_type_string);
  return std::string(
      json_object_get_string(value),
      static_cast<std::size_t>(json_object_get_string_len(value)));
}

bool exact_keys(json_object* object, const std::set<std::string>& expected) {
  if (!json_object_is_type(object, json_type_object) ||
      static_cast<std::size_t>(json_object_object_length(object)) != expected.size()) {
    return false;
  }
  json_object_object_foreach(object, key, value) {
    (void)value;
    if (expected.count(key) == 0U) return false;
  }
  return true;
}

template<std::size_t Size>
std::set<std::string> names(const std::array<const char*, Size>& values) {
  std::set<std::string> output;
  for (const char* value : values) output.emplace(value);
  return output;
}

bool lowercase_sha256(const std::string& value) {
  return value.size() == 64U &&
      std::all_of(value.begin(), value.end(), [](unsigned char character) {
        return std::isdigit(character) != 0 ||
            (character >= static_cast<unsigned char>('a') &&
             character <= static_cast<unsigned char>('f'));
      });
}

bool path_is_within(const fs::path& root, const fs::path& candidate) {
  auto root_part = root.begin();
  auto candidate_part = candidate.begin();
  while (root_part != root.end() && candidate_part != candidate.end()) {
    if (*root_part != *candidate_part) return false;
    ++root_part;
    ++candidate_part;
  }
  return root_part == root.end();
}

fs::path validated_asset_path(
    const fs::path& root,
    const std::string& logical_name,
    const std::string& relative_value) {
  if (relative_value.empty() || relative_value.find('\\') != std::string::npos) {
    throw std::runtime_error("invalid_asset_path:" + logical_name);
  }
  const fs::path relative(relative_value);
  if (relative.is_absolute()) {
    throw std::runtime_error("invalid_asset_path:" + logical_name);
  }
  for (const auto& part : relative) {
    if (part.empty() || part == "." || part == "..") {
      throw std::runtime_error("invalid_asset_path:" + logical_name);
    }
  }
  const fs::path candidate = root / relative;
  if (!regular_nonempty_file(candidate)) {
    throw std::runtime_error("missing_or_empty:" + relative_value);
  }
  std::error_code error;
  const fs::path canonical_root = fs::canonical(root, error);
  if (error) throw std::runtime_error("invalid_bundle_root");
  const fs::path canonical_candidate = fs::canonical(candidate, error);
  if (error || !path_is_within(canonical_root, canonical_candidate)) {
    throw std::runtime_error("asset_path_escapes_bundle:" + logical_name);
  }
  return candidate;
}

template<std::size_t Size>
void validate_asset_section(
    const fs::path& root,
    const std::string& section_name,
    json_object* section,
    const std::array<const char*, Size>& expected_assets,
    std::set<std::string>* seen_paths,
    fs::path* metadata_path) {
  if (!exact_keys(section, names(expected_assets))) {
    throw std::runtime_error("manifest_" + section_name + "_asset_set_mismatch");
  }
  for (const char* logical_name : expected_assets) {
    json_object* record = member(section, logical_name, json_type_object);
    if (!exact_keys(record, {"path", "sha256", "size_bytes"})) {
      throw std::runtime_error(
          "invalid_asset_record:" + section_name + ":" + logical_name);
    }
    const std::string relative_path = string_value(record, "path");
    if (!seen_paths->insert(relative_path).second) {
      throw std::runtime_error("duplicate_asset_path:" + relative_path);
    }
    const fs::path path = validated_asset_path(root, logical_name, relative_path);
    const std::string expected_hash = string_value(record, "sha256");
    if (!lowercase_sha256(expected_hash)) {
      throw std::runtime_error("invalid_asset_sha256:" + std::string(logical_name));
    }
    json_object* size_object = member(record, "size_bytes", json_type_int);
    const std::int64_t expected_size = json_object_get_int64(size_object);
    std::error_code error;
    const auto actual_size = fs::file_size(path, error);
    if (expected_size <= 0 || error ||
        static_cast<std::uintmax_t>(expected_size) != actual_size) {
      throw std::runtime_error("asset_integrity_mismatch:" + path.filename().string());
    }
    if (sha256_file(path) != expected_hash) {
      throw std::runtime_error("asset_integrity_mismatch:" + path.filename().string());
    }
    if (std::string(logical_name) == "metadata.json" && metadata_path != nullptr) {
      *metadata_path = path;
    }
  }
}

void validate_empty_section(const std::string& name, json_object* section) {
  if (!exact_keys(section, {})) {
    throw std::runtime_error("manifest_" + name + "_asset_set_mismatch");
  }
}

void validate_metadata(
    const fs::path& path,
    const std::string& map_id,
    const std::string& profile) {
  JsonPointer metadata = parse_json_object(path);
  json_object* schema = member(metadata.get(), "schema_version", json_type_int);
  if (json_object_get_int64(schema) != 3) {
    throw std::runtime_error("metadata_schema_mismatch");
  }
  if (string_value(metadata.get(), "map_id") != map_id) {
    throw std::runtime_error("metadata_map_id_mismatch");
  }
  if (string_value(metadata.get(), "profile") != profile) {
    throw std::runtime_error("metadata_profile_mismatch");
  }
  json_object* motion = member(
      metadata.get(), "motion_output_connected", json_type_boolean);
  if (json_object_get_boolean(motion) != 0) {
    throw std::runtime_error("mapping_motion_output_must_be_disconnected");
  }
}

}  // namespace

MapBundleValidationResult validate_map_bundle(
    const fs::path& map_root,
    const std::string& map_id,
    const std::string& expected_profile) {
  MapBundleValidationResult output;
  if (!safe_map_id(map_id)) {
    output.reason = "invalid_map_id";
    return output;
  }
  if (!supported_profile(expected_profile)) {
    output.reason = "unsupported_profile:" + expected_profile;
    return output;
  }
  const fs::path root = map_root / map_id;
  const fs::path manifest_path = root / "manifest.json";
  if (!regular_nonempty_file(manifest_path)) {
    output.reason = "missing_or_empty:manifest.json";
    return output;
  }
  try {
    output.manifest_sha256 = sha256_file(manifest_path);
    JsonPointer manifest = parse_json_object(manifest_path);
    json_object* schema = member(manifest.get(), "schema_version", json_type_int);
    if (json_object_get_int64(schema) != 3) {
      throw std::runtime_error("unsupported_manifest_schema");
    }
    if (string_value(manifest.get(), "map_id") != map_id) {
      throw std::runtime_error("manifest_map_id_mismatch");
    }
    const std::string profile = string_value(manifest.get(), "profile");
    if (!supported_profile(profile)) {
      throw std::runtime_error("invalid_manifest_profile");
    }
    if (profile != expected_profile) {
      throw std::runtime_error("bundle_profile_mismatch");
    }
    if (string_value(manifest.get(), "created_at").empty()) {
      throw std::runtime_error("invalid_manifest_created_at");
    }
    json_object* assets = member(manifest.get(), "assets", json_type_object);
    if (!exact_keys(assets, {"common", "visual", "lidar"})) {
      throw std::runtime_error("manifest_asset_sections_mismatch");
    }
    json_object* common = member(assets, "common", json_type_object);
    json_object* visual = member(assets, "visual", json_type_object);
    json_object* lidar = member(assets, "lidar", json_type_object);
    std::set<std::string> seen_paths;
    fs::path metadata_path;
    validate_asset_section(
        root, "common", common, kCommonAssets, &seen_paths, &metadata_path);
    if (profile == "visual") {
      validate_asset_section(
          root, "visual", visual, kVisualAssets, &seen_paths, nullptr);
      validate_empty_section("lidar", lidar);
    } else {
      validate_empty_section("visual", visual);
      validate_asset_section(
          root, "lidar", lidar, kLidarAssets, &seen_paths, nullptr);
    }
    validate_metadata(metadata_path, map_id, profile);
    output.valid = true;
    output.reason = "ok";
  } catch (const std::exception& error) {
    output.reason = error.what();
  }
  return output;
}

}  // namespace sunhorizon_navigation_server
