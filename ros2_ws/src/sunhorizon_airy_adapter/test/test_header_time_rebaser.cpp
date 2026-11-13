#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "sunhorizon_airy_adapter/header_time_rebaser.hpp"

namespace {

using sunhorizon_airy_adapter::HeaderTimeMode;
using sunhorizon_airy_adapter::HeaderTimeRebaser;

int failures = 0;

void expect(bool condition, const std::string& description) {
  if (!condition) {
    std::cerr << "FAIL: " << description << '\n';
    ++failures;
  }
}

void expect_near(
    double actual, double expected, double tolerance,
    const std::string& description) {
  expect(std::abs(actual - expected) <= tolerance, description);
}

void test_modes_parse_explicitly() {
  using sunhorizon_airy_adapter::parse_header_time_mode;
  expect(
      parse_header_time_mode("tracking_imu_to_ros") ==
          HeaderTimeMode::kTrackingImuToRos,
      "live tracking mode parses explicitly");
  bool rejected = false;
  try {
    static_cast<void>(parse_header_time_mode("arrival_time"));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  expect(rejected, "unknown timestamp mode is rejected");
}

void test_tracking_initializes_behind_arrival() {
  HeaderTimeRebaser rebaser(
      HeaderTimeMode::kTrackingImuToRos, 0.0, 0.1, 0.02, 0.002, 0.0);
  const auto waiting = rebaser.rebase(50000.0, 1788190000.0, false);
  expect(!waiting.accepted, "point cloud cannot initialize the shared clock");
  expect(
      waiting.reason == "clock_offset_waiting_for_first_imu",
      "point cloud reports the first-IMU gate");

  const auto initialized = rebaser.rebase(50000.0, 1788190000.0, true);
  expect(initialized.accepted, "first IMU initializes live tracking");
  expect(initialized.initialized_now, "initialization is observable");
  expect_near(
      initialized.shifted_stamp_sec, 1788189999.98, 1e-6,
      "first normalized IMU is deliberately 20 ms behind arrival");
}

void test_tracking_follows_device_drift_without_future_or_regression() {
  HeaderTimeRebaser rebaser(
      HeaderTimeMode::kTrackingImuToRos, 0.0, 0.1, 0.02, 0.002, 0.0);
  double raw = 50000.0;
  double now = 1788190000.0;
  auto result = rebaser.rebase(raw, now, true);
  double previous = result.shifted_stamp_sec;
  constexpr double host_step_sec = 0.005;
  constexpr double device_rate = 1.0003;
  for (int index = 0; index < 120000; ++index) {
    now += host_step_sec;
    raw += host_step_sec * device_rate;
    result = rebaser.rebase(raw, now, true);
    expect(result.accepted, "300 ppm drift remains accepted for ten minutes");
    if (!result.accepted) {
      break;
    }
    expect(
        result.shifted_stamp_sec > previous,
        "bounded tracking preserves strict timestamp monotonicity");
    expect(
        result.shifted_stamp_sec <= now + 0.1,
        "tracked timestamp never exceeds the future guard");
    previous = result.shifted_stamp_sec;
  }
  expect_near(
      now - result.shifted_stamp_sec, 0.02, 5e-4,
      "tracking maintains the requested host-clock delay");
}

void test_one_offset_is_shared_with_points() {
  HeaderTimeRebaser rebaser(
      HeaderTimeMode::kTrackingImuToRos, 0.0, 0.1, 0.02, 0.002, 0.0);
  const auto imu = rebaser.rebase(50000.0, 1788190000.0, true);
  const double offset = rebaser.offset_sec();
  const auto point = rebaser.rebase(49999.9, 1788190000.01, false);
  expect(imu.accepted && point.accepted, "IMU and point headers are both accepted");
  expect_near(
      point.shifted_stamp_sec, 49999.9 + offset, 1e-6,
      "point cloud uses the exact IMU-derived epoch offset");
  expect_near(
      rebaser.offset_sec(), offset, 1e-12,
      "point arrival cannot perturb the shared clock offset");
}

void test_invalid_and_non_monotonic_tracking_samples_fail_closed() {
  HeaderTimeRebaser rebaser(
      HeaderTimeMode::kTrackingImuToRos, 0.0, 0.1, 0.02, 0.002, 0.0);
  expect(
      !rebaser.rebase(
          std::numeric_limits<double>::quiet_NaN(), 1788190000.0, true).accepted,
      "non-finite raw stamp is rejected");
  expect(rebaser.rebase(50000.0, 1788190000.0, true).accepted,
         "valid first sample initializes tracking");
  const auto duplicate = rebaser.rebase(50000.0, 1788190000.005, true);
  expect(!duplicate.accepted, "duplicate IMU header is rejected");
  expect(
      duplicate.reason == "tracking_imu_header_not_strictly_increasing",
      "duplicate rejection is explicit");
}

void test_tracking_warmup_uses_a_current_sample_for_initialization() {
  HeaderTimeRebaser rebaser(
      HeaderTimeMode::kTrackingImuToRos, 0.0, 0.1, 0.02, 0.002, 0.5);
  double raw = 50000.0;
  double now = 1788190000.0;
  auto result = rebaser.rebase(raw, now, true);
  expect(!result.accepted, "first queued IMU starts the clock warmup");
  expect(
      result.reason == "clock_offset_warming_up",
      "warmup rejection is explicit");

  for (int index = 0; index < 101 && !result.accepted; ++index) {
    raw += 0.005;
    now += 0.005;
    result = rebaser.rebase(raw, now, true);
  }
  expect(result.accepted, "current IMU initializes after the warmup window");
  expect(result.initialized_now, "post-warmup initialization is observable");
  expect_near(
      now - result.shifted_stamp_sec, 0.02, 1e-6,
      "post-warmup initialization retains the target delay");
}

}  // namespace

int main() {
  test_modes_parse_explicitly();
  test_tracking_initializes_behind_arrival();
  test_tracking_follows_device_drift_without_future_or_regression();
  test_one_offset_is_shared_with_points();
  test_invalid_and_non_monotonic_tracking_samples_fail_closed();
  test_tracking_warmup_uses_a_current_sample_for_initialization();

  if (failures != 0) {
    std::cerr << failures << " assertion(s) failed\n";
    return 1;
  }
  std::cout << "All Airy header-time rebaser tests passed\n";
  return 0;
}
