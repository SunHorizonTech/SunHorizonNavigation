#pragma once

// Shared immutable assets and exact-time transform construction. No ROS
// subscriptions, parameters, private camera TF, or inferred asset identity.
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <filesystem>
#include <openssl/evp.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include "sunhorizon_obstacle_coverage/dynamic_self_geometry.hpp"

namespace sunhorizon_rgbd_adapter::dynamic_self_model {
namespace geometry = sunhorizon_obstacle_coverage::dynamic_self_geometry;
inline int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline bool sha_valid(const std::string& value) {
  return value.size() == 64 && value.find_first_not_of("0123456789abcdef") == std::string::npos;
}
inline std::string sha256(const std::string& bytes) {
  unsigned char digest[EVP_MAX_MD_SIZE]; unsigned int length = 0;
  if (EVP_Digest(bytes.data(), bytes.size(), digest, &length, EVP_sha256(), nullptr) != 1 || length != 32)
    throw std::runtime_error("model_sha_failure");
  std::ostringstream out;
  for (unsigned int i = 0; i < length; ++i)
    out << std::hex << std::setfill('0') << std::setw(2) << unsigned(digest[i]);
  return out.str();
}
struct AssetSpec {
  std::string path, sha256;
  std::size_t triangle_count{0};
  std::string link_name, parent_frame;
  std::size_t boundary_triangle_begin{0},boundary_triangle_count{0};
  double boundary_tolerance_m{0.};
  std::size_t measured_triangle_begin{0},measured_triangle_count{0};
};
inline bool measured_policy_valid(const AssetSpec& s) {
  return s.measured_triangle_begin<=s.triangle_count &&
    s.measured_triangle_count<=s.triangle_count-s.measured_triangle_begin &&
    (s.measured_triangle_count || s.measured_triangle_begin==0);
}
inline bool boundary_policy_valid(const AssetSpec& s) {
  return std::isfinite(s.boundary_tolerance_m) && s.boundary_tolerance_m>=0. &&
    s.boundary_tolerance_m<=.003 && s.boundary_triangle_begin<=s.triangle_count &&
    s.boundary_triangle_count<=s.triangle_count-s.boundary_triangle_begin &&
    (s.boundary_triangle_count ? s.boundary_tolerance_m>0. :
      s.boundary_triangle_begin==0 && s.boundary_tolerance_m==0.);
}
inline geometry::RigidTransform rigid(const geometry_msgs::msg::Transform& t) {
  const auto& q = t.rotation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (!std::isfinite(norm) || std::abs(norm - 1.) > 1.e-6 ||
      !std::isfinite(t.translation.x) || !std::isfinite(t.translation.y) || !std::isfinite(t.translation.z))
    throw std::runtime_error("model_invalid_rigid_tf");
  tf2::Matrix3x3 r(tf2::Quaternion(q.x, q.y, q.z, q.w));
  geometry::RigidTransform result;
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j) result.rotation[i*3+j] = r[i][j];
  result.translation = {t.translation.x, t.translation.y, t.translation.z};
  return result;
}
inline geometry::RigidTransform multiply(const geometry::RigidTransform& a, const geometry::RigidTransform& b) {
  geometry::RigidTransform out;
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      out.rotation[i*3+j] = 0;
      for (std::size_t k = 0; k < 3; ++k) out.rotation[i*3+j] += a.rotation[i*3+k]*b.rotation[k*3+j];
    }
  }
  const double v[3]{b.translation.x,b.translation.y,b.translation.z};
  double t[3]{a.translation.x,a.translation.y,a.translation.z};
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t k = 0; k < 3; ++k) t[i] += a.rotation[i*3+k]*v[k];
  out.translation = {t[0],t[1],t[2]}; return out;
}
inline bool rigid_valid(const geometry::RigidTransform& transform) {
  const auto& r = transform.rotation;
  for (double value : r) if (!std::isfinite(value)) return false;
  if (!std::isfinite(transform.translation.x) || !std::isfinite(transform.translation.y) ||
      !std::isfinite(transform.translation.z)) return false;
  for (std::size_t i = 0; i < 3; ++i) for (std::size_t j = 0; j < 3; ++j) {
    double dot = 0.;
    for (std::size_t k = 0; k < 3; ++k) dot += r[k*3+i]*r[k*3+j];
    if (!std::isfinite(dot) || std::abs(dot - (i == j ? 1. : 0.)) > 1.e-6) return false;
  }
  const double determinant = r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6]);
  return std::isfinite(determinant) && std::abs(determinant-1.) <= 1.e-6;
}
// All lookups in an evaluation share this deadline; each call receives only
// its remaining allowance. TF2 can interpolate at source time, never latest.
inline rclcpp::Duration remaining(int64_t deadline) {
  const int64_t left = deadline - steady_ns();
  if (left <= 0) throw tf2::TimeoutException("model_tf_budget_exhausted");
  return rclcpp::Duration::from_nanoseconds(left);
}
class Model {
 public:
  explicit Model(std::vector<AssetSpec> specs) : specs_(std::move(specs)) {
    if (specs_.size() != 12) throw std::invalid_argument("model_requires_12_links");
    std::set<std::string> links;
    std::size_t total = 0;
    meshes_.resize(specs_.size());
    for (std::size_t i = 0; i < specs_.size(); ++i) {
      const auto& spec = specs_[i];
      if (!sha_valid(spec.sha256) || spec.link_name.empty() || spec.link_name.size()>128 ||
          spec.parent_frame.empty() || spec.parent_frame.size()>128 || spec.link_name == spec.parent_frame ||
          !links.insert(spec.link_name).second || spec.triangle_count == 0 ||
          spec.triangle_count > 1000000 || total > 1000000-spec.triangle_count ||
          !boundary_policy_valid(spec) || !measured_policy_valid(spec))
        throw std::invalid_argument("invalid_model_manifest");
      total += spec.triangle_count;
      const std::filesystem::path path(spec.path);
      if (!path.is_absolute() || std::filesystem::canonical(path) != path ||
          !std::filesystem::is_regular_file(std::filesystem::symlink_status(path)))
        throw std::invalid_argument("model_path_not_canonical_regular");
      const auto size = std::filesystem::file_size(path);
      const auto time = std::filesystem::last_write_time(path);
      if (size != 84 + 50*spec.triangle_count) throw std::invalid_argument("model_stl_size");
      std::ifstream input(path, std::ios::binary);
      std::string bytes(static_cast<std::size_t>(size), '\0');
      if (!input.read(bytes.data(), bytes.size()) || input.peek() != std::char_traits<char>::eof() ||
          std::filesystem::file_size(path) != size || std::filesystem::last_write_time(path) != time ||
          sha256(bytes) != spec.sha256) throw std::runtime_error("model_asset_changed_or_sha_mismatch");
      std::string error;
      if (!meshes_[i].load_binary_stl(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size(),
          spec.triangle_count, geometry::BuildLimits{}, &error))
        throw std::invalid_argument("model_stl_invalid:" + error);
      if(spec.boundary_triangle_count && !meshes_[i].configure_boundary_patch(
          spec.boundary_triangle_begin,spec.boundary_triangle_count,spec.boundary_tolerance_m,&error))
        throw std::invalid_argument("model_boundary_patch_invalid:"+error);
      if(spec.measured_triangle_count && !meshes_[i].configure_measured_surface(
          spec.measured_triangle_begin,spec.measured_triangle_count,&error))
        throw std::invalid_argument("model_measured_surface_invalid:"+error);
    }
  }
  const std::vector<AssetSpec>& specs() const { return specs_; }
  double mesh_origin_radius_bound(std::size_t index) const {return meshes_.at(index).origin_radius_bound();}
  // Direct same-source-time FK path. The owner must qualify camera and every
  // base_from_link for source_ns before calling; legs are in specs() order.
  // No private TF listener, DDS lookup, or model preparation occurs here.
  std::vector<geometry::LinkObservation> observations(
      const geometry::RigidTransform& optical_from_base,
      const std::vector<geometry::RigidTransform>& base_from_links,
      int64_t source_ns) const {
    if (source_ns <= 0) throw std::runtime_error("invalid_model_source_time");
    if (base_from_links.size() != specs_.size())
      throw std::invalid_argument("model_requires_12_source_time_fk_transforms");
    if (!rigid_valid(optical_from_base)) throw std::invalid_argument("model_invalid_camera_rigid");
    std::vector<geometry::LinkObservation> result; result.reserve(specs_.size());
    for (std::size_t i = 0; i < specs_.size(); ++i) {
      if (!meshes_[i].ready()) throw std::runtime_error("model_not_ready");
      if (!rigid_valid(base_from_links[i])) throw std::invalid_argument("model_invalid_fk_rigid");
      result.push_back({&meshes_[i], specs_[i].link_name,
          multiply(optical_from_base, base_from_links[i]), true, true, true});
    }
    return result;
  }
  std::vector<geometry::LinkObservation> observations(tf2_ros::Buffer& global,
      tf2_ros::Buffer& private_legs, const std::string& optical_frame,
      int64_t source_ns, rcl_clock_type_t clock_type, int64_t deadline) const {
    if (source_ns <= 0) throw std::runtime_error("invalid_model_source_time");
    const rclcpp::Time stamp(source_ns, clock_type);
    const auto camera = rigid(global.lookupTransform(optical_frame, "base_link", stamp, remaining(deadline)).transform);
    std::vector<geometry::LinkObservation> result; result.reserve(12);
    // This runs even for an all-invalid image. Invalid depth must never bypass
    // complete model identity and all twelve exact-source TF qualifications.
    for (std::size_t i = 0; i < specs_.size(); ++i) {
      if (!meshes_[i].ready()) throw std::runtime_error("model_not_ready");
      const auto leg = rigid(private_legs.lookupTransform("base_link", specs_[i].link_name,
          stamp, remaining(deadline)).transform);
      result.push_back({&meshes_[i], specs_[i].link_name, multiply(camera, leg), true, true, true});
    }
    return result;
  }
 private:
  std::vector<AssetSpec> specs_;
  std::vector<geometry::Mesh> meshes_;
};
}  // namespace sunhorizon_rgbd_adapter::dynamic_self_model
