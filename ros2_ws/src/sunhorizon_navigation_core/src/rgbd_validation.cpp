#include "sunhorizon_navigation_core/rgbd_validation.hpp"

#include <algorithm>
#include <cmath>

namespace sunhorizon_navigation_core {

namespace {

bool finite_positive(double value) {
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

bool valid_camera_intrinsics(const CameraIntrinsics& info, std::string* reason) {
  std::string failure;
  if (info.width == 0U || info.height == 0U) {
    failure = "camera_info_empty_dimensions";
  } else if (!finite_positive(info.fx) || !finite_positive(info.fy)) {
    failure = "camera_info_invalid_focal_length";
  } else if (!std::isfinite(info.cx) || !std::isfinite(info.cy) || info.cx < 0.0 ||
             info.cy < 0.0 || info.cx >= static_cast<double>(info.width) ||
             info.cy >= static_cast<double>(info.height)) {
    failure = "camera_info_invalid_principal_point";
  } else if (info.frame_id.empty()) {
    failure = "camera_info_empty_frame";
  }
  if (reason != nullptr) *reason = failure.empty() ? "ok" : failure;
  return failure.empty();
}

RgbdPairValidator::RgbdPairValidator(RgbdValidationLimits limits) : limits_(limits) {}

RgbdPairDecision RgbdPairValidator::evaluate(
    const ImageMetadata& color, const ImageMetadata& depth,
    const CameraIntrinsics& color_info, const CameraIntrinsics& depth_info,
    bool alignment_calibration_accepted) {
  RgbdPairDecision output;
  output.pair_delta_sec = std::abs(color.stamp_sec - depth.stamp_sec);
  if (!finite_positive(color.stamp_sec) || !finite_positive(depth.stamp_sec)) {
    output.reason = "invalid_timestamp";
    return output;
  }
  if ((last_color_stamp_sec_ > 0.0 && color.stamp_sec <= last_color_stamp_sec_) ||
      (last_depth_stamp_sec_ > 0.0 && depth.stamp_sec <= last_depth_stamp_sec_)) {
    output.reason = "non_monotonic_timestamp";
    return output;
  }
  if (output.pair_delta_sec > limits_.max_pair_delta_sec) {
    output.reason = "pair_delta_exceeded";
    return output;
  }
  if (color.width == 0U || color.height == 0U || color.width != depth.width ||
      color.height != depth.height) {
    output.reason = "aligned_image_dimensions_mismatch";
    return output;
  }
  if (color.encoding != "rgb8" && color.encoding != "bgr8") {
    output.reason = "unsupported_color_encoding";
    return output;
  }
  if (depth.encoding != "16UC1" && depth.encoding != "mono16") {
    output.reason = "unsupported_depth_encoding";
    return output;
  }
  std::string intrinsics_reason;
  if (!valid_camera_intrinsics(color_info, &intrinsics_reason)) {
    output.reason = "color_" + intrinsics_reason;
    return output;
  }
  if (!valid_camera_intrinsics(depth_info, &intrinsics_reason)) {
    output.reason = "depth_" + intrinsics_reason;
    return output;
  }
  if (color_info.width != color.width || color_info.height != color.height) {
    output.reason = "color_camera_info_dimensions_mismatch";
    return output;
  }

  last_color_stamp_sec_ = color.stamp_sec;
  last_depth_stamp_sec_ = depth.stamp_sec;
  output.stream_accepted = true;
  output.pointcloud_allowed = alignment_calibration_accepted;
  output.reason = alignment_calibration_accepted ? "ok" : "alignment_calibration_not_accepted";
  return output;
}

bool depth_sample_within_tolerance(
    uint16_t raw_depth, double known_distance_m, double depth_scale_m,
    double absolute_tolerance_m, double relative_tolerance) {
  if (raw_depth == 0U || !finite_positive(known_distance_m) ||
      !finite_positive(depth_scale_m) || absolute_tolerance_m < 0.0 ||
      relative_tolerance < 0.0) {
    return false;
  }
  const double measured = static_cast<double>(raw_depth) * depth_scale_m;
  const double tolerance = std::max(absolute_tolerance_m, known_distance_m * relative_tolerance);
  return std::abs(measured - known_distance_m) <= tolerance;
}

}  // namespace sunhorizon_navigation_core
