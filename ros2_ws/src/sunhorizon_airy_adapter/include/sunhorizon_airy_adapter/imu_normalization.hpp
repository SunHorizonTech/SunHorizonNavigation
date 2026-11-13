#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace sunhorizon_airy_adapter {

inline constexpr double kDefaultLinearAccelerationScale = 1.0;

enum class ImuNormalizationStatus {
  kOk,
  kCalibrationNotAccepted,
  kInvalidAccelerationScale,
  kMissingTargetFrame,
  kTargetFrameMatchesPointCloudFrame,
  kNonFiniteInput,
  kInvalidOrientation,
  kNonFiniteOutput,
};

struct ImuNormalizationResult {
  ImuNormalizationStatus status;
  std::optional<sensor_msgs::msg::Imu> message;
};

inline const char* imu_normalization_status_reason(ImuNormalizationStatus status) {
  switch (status) {
    case ImuNormalizationStatus::kOk:
      return "ok";
    case ImuNormalizationStatus::kCalibrationNotAccepted:
      return "imu_axis_calibration_not_accepted";
    case ImuNormalizationStatus::kInvalidAccelerationScale:
      return "imu_acceleration_scale_invalid";
    case ImuNormalizationStatus::kMissingTargetFrame:
      return "imu_target_frame_missing";
    case ImuNormalizationStatus::kTargetFrameMatchesPointCloudFrame:
      return "imu_target_frame_matches_point_cloud_frame";
    case ImuNormalizationStatus::kNonFiniteInput:
      return "imu_input_non_finite";
    case ImuNormalizationStatus::kInvalidOrientation:
      return "imu_orientation_invalid";
    case ImuNormalizationStatus::kNonFiniteOutput:
      return "imu_output_non_finite";
  }
  return "imu_normalization_status_unknown";
}

inline ImuNormalizationStatus validate_imu_normalization_config(
    double linear_acceleration_scale,
    const std::string& output_imu_frame,
    const std::string& output_points_frame) {
  if (!std::isfinite(linear_acceleration_scale) || linear_acceleration_scale <= 0.0 ||
      linear_acceleration_scale > std::sqrt(std::numeric_limits<double>::max())) {
    return ImuNormalizationStatus::kInvalidAccelerationScale;
  }
  if (output_imu_frame.empty()) {
    return ImuNormalizationStatus::kMissingTargetFrame;
  }
  if (output_imu_frame == output_points_frame) {
    return ImuNormalizationStatus::kTargetFrameMatchesPointCloudFrame;
  }
  return ImuNormalizationStatus::kOk;
}

inline bool finite_vector(const geometry_msgs::msg::Vector3& vector) {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

inline bool finite_quaternion(const geometry_msgs::msg::Quaternion& quaternion) {
  return std::isfinite(quaternion.x) && std::isfinite(quaternion.y) &&
         std::isfinite(quaternion.z) && std::isfinite(quaternion.w);
}

inline bool finite_covariance(const std::array<double, 9>& covariance) {
  for (const double value : covariance) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

inline geometry_msgs::msg::Vector3 rotate_vector(
    const geometry_msgs::msg::Vector3& input, const tf2::Matrix3x3& rotation) {
  const tf2::Vector3 output = rotation * tf2::Vector3(input.x, input.y, input.z);
  geometry_msgs::msg::Vector3 result;
  result.x = output.x();
  result.y = output.y();
  result.z = output.z();
  return result;
}

inline std::array<double, 9> rotate_covariance(
    const std::array<double, 9>& covariance, const tf2::Matrix3x3& rotation) {
  tf2::Matrix3x3 input(
      covariance[0], covariance[1], covariance[2],
      covariance[3], covariance[4], covariance[5],
      covariance[6], covariance[7], covariance[8]);
  const tf2::Matrix3x3 output = rotation * input * rotation.transpose();
  return {
      output[0][0], output[0][1], output[0][2],
      output[1][0], output[1][1], output[1][2],
      output[2][0], output[2][1], output[2][2]};
}

inline ImuNormalizationResult normalize_imu(
    const sensor_msgs::msg::Imu& input,
    const tf2::Quaternion& imu_calibration,
    const tf2::Matrix3x3& imu_rotation,
    bool calibration_valid,
    bool input_orientation_available,
    double linear_acceleration_scale,
    const std::string& output_imu_frame,
    const std::string& output_points_frame) {
  if (!calibration_valid) {
    return {ImuNormalizationStatus::kCalibrationNotAccepted, std::nullopt};
  }

  const auto config_status = validate_imu_normalization_config(
      linear_acceleration_scale, output_imu_frame, output_points_frame);
  if (config_status != ImuNormalizationStatus::kOk) {
    return {config_status, std::nullopt};
  }

  if (!finite_quaternion(input.orientation) || !finite_vector(input.angular_velocity) ||
      !finite_vector(input.linear_acceleration) ||
      !finite_covariance(input.orientation_covariance) ||
      !finite_covariance(input.angular_velocity_covariance) ||
      !finite_covariance(input.linear_acceleration_covariance)) {
    return {ImuNormalizationStatus::kNonFiniteInput, std::nullopt};
  }

  sensor_msgs::msg::Imu output = input;
  output.header.frame_id = output_imu_frame;
  output.angular_velocity = rotate_vector(input.angular_velocity, imu_rotation);
  output.linear_acceleration = rotate_vector(input.linear_acceleration, imu_rotation);
  output.linear_acceleration.x *= linear_acceleration_scale;
  output.linear_acceleration.y *= linear_acceleration_scale;
  output.linear_acceleration.z *= linear_acceleration_scale;
  output.angular_velocity_covariance =
      rotate_covariance(input.angular_velocity_covariance, imu_rotation);
  output.linear_acceleration_covariance =
      rotate_covariance(input.linear_acceleration_covariance, imu_rotation);
  const double acceleration_covariance_scale =
      linear_acceleration_scale * linear_acceleration_scale;
  for (double& value : output.linear_acceleration_covariance) {
    value *= acceleration_covariance_scale;
  }

  if (input_orientation_available && input.orientation_covariance[0] >= 0.0) {
    tf2::Quaternion orientation(
        input.orientation.x, input.orientation.y, input.orientation.z, input.orientation.w);
    if (!std::isfinite(orientation.length2()) || orientation.length2() < 1e-12) {
      return {ImuNormalizationStatus::kInvalidOrientation, std::nullopt};
    }
    orientation = imu_calibration * orientation;
    orientation.normalize();
    output.orientation.x = orientation.x();
    output.orientation.y = orientation.y();
    output.orientation.z = orientation.z();
    output.orientation.w = orientation.w();
    output.orientation_covariance =
        rotate_covariance(input.orientation_covariance, imu_rotation);
  } else {
    output.orientation_covariance.fill(0.0);
    output.orientation_covariance[0] = -1.0;
  }

  if (!finite_quaternion(output.orientation) || !finite_vector(output.angular_velocity) ||
      !finite_vector(output.linear_acceleration) ||
      !finite_covariance(output.orientation_covariance) ||
      !finite_covariance(output.angular_velocity_covariance) ||
      !finite_covariance(output.linear_acceleration_covariance)) {
    return {ImuNormalizationStatus::kNonFiniteOutput, std::nullopt};
  }

  return {ImuNormalizationStatus::kOk, std::move(output)};
}

}  // namespace sunhorizon_airy_adapter
