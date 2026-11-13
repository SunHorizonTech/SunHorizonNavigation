#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "sunhorizon_airy_adapter/imu_normalization.hpp"

namespace {

using sunhorizon_airy_adapter::ImuNormalizationStatus;

int failures = 0;

void expect(bool condition, const std::string& description) {
  if (!condition) {
    std::cerr << "FAIL: " << description << '\n';
    ++failures;
  }
}

void expect_near(double actual, double expected, const std::string& description) {
  expect(std::abs(actual - expected) < 1e-9, description);
}

sensor_msgs::msg::Imu valid_imu() {
  sensor_msgs::msg::Imu input;
  input.orientation.w = 1.0;
  input.angular_velocity.x = 0.1;
  input.angular_velocity.y = -0.2;
  input.angular_velocity.z = 0.3;
  input.linear_acceleration.x = 1.0;
  input.linear_acceleration.y = -2.0;
  input.linear_acceleration.z = 0.5;
  input.orientation_covariance = {1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0};
  input.angular_velocity_covariance = {4.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 6.0};
  input.linear_acceleration_covariance = {7.0, 0.0, 0.0, 0.0, 8.0, 0.0, 0.0, 0.0, 9.0};
  return input;
}

sunhorizon_airy_adapter::ImuNormalizationResult normalize(
    const sensor_msgs::msg::Imu& input,
    bool calibration_valid = true,
    double scale = sunhorizon_airy_adapter::kDefaultLinearAccelerationScale,
    bool input_orientation_available = true,
    const std::string& imu_frame = "airy_imu_normalized",
    const std::string& point_frame = "rslidar") {
  const tf2::Quaternion identity_quaternion(0.0, 0.0, 0.0, 1.0);
  const tf2::Matrix3x3 identity_rotation(identity_quaternion);
  return sunhorizon_airy_adapter::normalize_imu(
      input, identity_quaternion, identity_rotation, calibration_valid,
      input_orientation_available, scale,
      imu_frame, point_frame);
}

void test_default_scale_is_identity() {
  expect_near(
      sunhorizon_airy_adapter::kDefaultLinearAccelerationScale, 1.0,
      "default linear acceleration scale is 1.0");
  const auto result = normalize(valid_imu());
  expect(result.status == ImuNormalizationStatus::kOk, "default scale is accepted");
  expect(result.message.has_value(), "default scale produces normalized IMU");
  if (result.message.has_value()) {
    expect_near(result.message->linear_acceleration.x, 1.0, "default scale preserves x");
    expect_near(result.message->linear_acceleration.y, -2.0, "default scale preserves y");
    expect_near(result.message->linear_acceleration.z, 0.5, "default scale preserves z");
  }
}

void test_scale_applies_to_acceleration_and_covariance() {
  constexpr double scale = 9.80665;
  const auto result = normalize(valid_imu(), true, scale);
  expect(result.status == ImuNormalizationStatus::kOk, "measured positive scale is accepted");
  expect(result.message.has_value(), "scaled IMU is produced");
  if (result.message.has_value()) {
    expect_near(result.message->linear_acceleration.x, scale, "acceleration uses configured scale");
    expect_near(
        result.message->linear_acceleration_covariance[0], 7.0 * scale * scale,
        "acceleration covariance uses scale squared");
  }
}

void test_uncalibrated_is_fail_closed() {
  const auto result = normalize(valid_imu(), false);
  expect(
      result.status == ImuNormalizationStatus::kCalibrationNotAccepted,
      "uncalibrated input is rejected");
  expect(!result.message.has_value(), "uncalibrated input cannot produce normalized IMU");
}

void test_target_frame_is_explicit_and_independent() {
  auto status = sunhorizon_airy_adapter::validate_imu_normalization_config(1.0, "", "rslidar");
  expect(status == ImuNormalizationStatus::kMissingTargetFrame, "empty IMU target frame is rejected");
  status = sunhorizon_airy_adapter::validate_imu_normalization_config(
      1.0, "rslidar", "rslidar");
  expect(
      status == ImuNormalizationStatus::kTargetFrameMatchesPointCloudFrame,
      "point cloud frame cannot be reused as IMU target frame");

  const auto result = normalize(valid_imu());
  expect(result.message.has_value(), "independent IMU target frame is accepted");
  if (result.message.has_value()) {
    expect(
        result.message->header.frame_id == "airy_imu_normalized",
        "normalized message uses explicit IMU target frame");
  }
}

void test_orientation_placeholder_is_marked_unavailable() {
  const auto result = normalize(valid_imu(), true, 1.0, false);
  expect(result.status == ImuNormalizationStatus::kOk, "missing attitude is accepted explicitly");
  expect(result.message.has_value(), "missing attitude preserves usable inertial vectors");
  if (result.message.has_value()) {
    expect_near(
        result.message->orientation_covariance[0], -1.0,
        "missing attitude is marked unavailable using the ROS IMU convention");
    expect_near(
        result.message->angular_velocity.x, 0.1,
        "missing attitude does not discard angular velocity");
  }
}

void test_invalid_scales_are_rejected() {
  const std::vector<double> invalid_scales = {
      0.0,
      -1.0,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::max(),
  };
  for (const double scale : invalid_scales) {
    const auto status = sunhorizon_airy_adapter::validate_imu_normalization_config(
        scale, "airy_imu_normalized", "rslidar");
    expect(
        status == ImuNormalizationStatus::kInvalidAccelerationScale,
        "invalid acceleration scale is rejected deterministically");
    const auto result = normalize(valid_imu(), true, scale);
    expect(!result.message.has_value(), "invalid acceleration scale produces no IMU");
  }
}

void test_non_finite_inputs_are_rejected() {
  auto acceleration = valid_imu();
  acceleration.linear_acceleration.x = std::numeric_limits<double>::quiet_NaN();
  auto angular_velocity = valid_imu();
  angular_velocity.angular_velocity.z = std::numeric_limits<double>::infinity();
  auto covariance = valid_imu();
  covariance.linear_acceleration_covariance[4] =
      std::numeric_limits<double>::quiet_NaN();
  auto orientation = valid_imu();
  orientation.orientation.y = std::numeric_limits<double>::infinity();

  for (const auto* input : {&acceleration, &angular_velocity, &covariance, &orientation}) {
    const auto result = normalize(*input);
    expect(
        result.status == ImuNormalizationStatus::kNonFiniteInput,
        "non-finite IMU input is rejected deterministically");
    expect(!result.message.has_value(), "non-finite IMU input produces no normalized message");
  }
}

}  // namespace

int main() {
  test_default_scale_is_identity();
  test_scale_applies_to_acceleration_and_covariance();
  test_uncalibrated_is_fail_closed();
  test_target_frame_is_explicit_and_independent();
  test_orientation_placeholder_is_marked_unavailable();
  test_invalid_scales_are_rejected();
  test_non_finite_inputs_are_rejected();

  if (failures != 0) {
    std::cerr << failures << " assertion(s) failed\n";
    return 1;
  }
  std::cout << "All Airy IMU normalization tests passed\n";
  return 0;
}
