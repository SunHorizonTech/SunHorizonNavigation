#include <stdexcept>
#include <string>
#include <vector>

#include "sunhorizon_camera_runtime/runtime_contract.hpp"

using sunhorizon_camera_runtime::FrameContract;
using sunhorizon_camera_runtime::CapturePairDecision;
using sunhorizon_camera_runtime::PixelContract;
using sunhorizon_camera_runtime::RoleRequest;
using sunhorizon_camera_runtime::StreamFreshnessDecision;
using sunhorizon_camera_runtime::StreamFreshnessObservation;
using sunhorizon_camera_runtime::StrictTimestampGate;
using sunhorizon_camera_runtime::TimestampDecision;
using sunhorizon_camera_runtime::TimestampRecoveryDecision;
using sunhorizon_camera_runtime::bind_enabled_roles_by_serial;
using sunhorizon_camera_runtime::expected_frame_bytes;
using sunhorizon_camera_runtime::evaluate_capture_pair;
using sunhorizon_camera_runtime::evaluate_sdk_system_timestamp;
using sunhorizon_camera_runtime::evaluate_stream_freshness;
using sunhorizon_camera_runtime::valid_capture_pair_thresholds;
using sunhorizon_camera_runtime::valid_timestamp_startup_grace;
using sunhorizon_camera_runtime::valid_stream_timeouts;
using sunhorizon_camera_runtime::valid_static_transform;
using sunhorizon_camera_runtime::validate_frame_shape;

namespace
{

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main()
{
  for (const auto fps : {int64_t{15}, int64_t{30}}) {
    const auto belly = sunhorizon_camera_runtime::belly_capture_contract(fps);
    require(belly.width == 640 && belly.height == 480 && belly.fps == fps &&
      belly.pixels == PixelContract::kDepth16, "belly FPS must preserve geometry/format");
    require(expected_frame_bytes(belly) == 614400U, "FPS must not change frame bytes");
  }
  for (const auto fps : {int64_t{-1}, int64_t{0}, int64_t{14}, int64_t{16}, int64_t{29}, int64_t{31}, INT64_MAX}) {
    bool rejected = false;
    try {(void)sunhorizon_camera_runtime::belly_capture_contract(fps);}
    catch (const std::invalid_argument&) {rejected = true;}
    require(rejected, "unsupported belly FPS must fail before SDK startup");
  }
  struct Calibration {
    std::string distortion_model;
    std::vector<double> d;
    std::vector<double> k;
  };
  const Calibration color{"rational_polynomial", {-.027, .034, -.0004, .0007, -.013, 0, 0, 0},
    {365.54, 0, 321.03, 0, 365.58, 233.25, 0, 0, 1}};
  const auto aligned = sunhorizon_camera_runtime::gemini_hw_d2c_depth_info(color);
  require(aligned.d == std::vector<double>(5, 0.), "D2C depth retained color distortion");
  require(aligned.distortion_model == "plumb_bob", "D2C depth model invalid");
  require(aligned.k == color.k, "D2C changed the matched color intrinsic matrix");
  require(color.d[0] == -.027 && color.distortion_model == "rational_polynomial",
    "D2C modified the raw RGB calibration");
  require(
    valid_static_transform(
      "camera_link", "camera_optical", {0.0, 0.0, 0.0}, {0.5, -0.5, 0.5, -0.5}),
    "valid static transform was rejected");
  require(
    !valid_static_transform(
      "camera_link", "camera_link", {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}),
    "self transform was accepted");
  require(
    !valid_static_transform(
      "camera_link", "camera_optical", {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.5}),
    "non-unit quaternion was accepted");
  const FrameContract rgb{640U, 480U, 30U, PixelContract::kRgb8};
  const FrameContract depth{640U, 480U, 30U, PixelContract::kDepth16};
  require(expected_frame_bytes(rgb) == 921600U, "RGB byte contract mismatch");
  require(expected_frame_bytes(depth) == 614400U, "depth byte contract mismatch");
  require(
    !validate_frame_shape(depth, 640U, 480U, PixelContract::kDepth16, 614400U),
    "valid depth frame was rejected");
  require(
    validate_frame_shape(depth, 640U, 360U, PixelContract::kDepth16, 460800U).has_value(),
    "wrong depth dimensions were accepted");
  require(
    validate_frame_shape(depth, 640U, 480U, PixelContract::kRgb8, 921600U).has_value(),
    "wrong depth format was accepted");

  StrictTimestampGate gate;
  require(gate.observe(0U) == TimestampDecision::kZero, "zero timestamp was accepted");
  require(gate.observe(100U) == TimestampDecision::kAccepted, "first timestamp was rejected");
  require(gate.observe(100U) == TimestampDecision::kDuplicate, "duplicate was accepted");
  require(gate.observe(99U) == TimestampDecision::kRegression, "regression was accepted");
  require(gate.observe(101U) == TimestampDecision::kAccepted, "recovery timestamp failed");
  gate.reanchor(10U);
  require(gate.observe(11U) == TimestampDecision::kAccepted, "timestamp reanchor failed");
  bool zero_reanchor_failed = false;
  try {
    gate.reanchor(0U);
  } catch (const std::invalid_argument &) {
    zero_reanchor_failed = true;
  }
  require(zero_reanchor_failed, "zero timestamp reanchor was accepted");

  constexpr double timestamp_startup_grace_sec = 60.0;
  require(
    valid_timestamp_startup_grace(timestamp_startup_grace_sec),
    "valid timestamp startup grace was rejected");
  require(
    !valid_timestamp_startup_grace(std::numeric_limits<double>::infinity()),
    "infinite timestamp startup grace was accepted");
  require(
    evaluate_sdk_system_timestamp(
      TimestampDecision::kRegression, 40.0, timestamp_startup_grace_sec) ==
    TimestampRecoveryDecision::kStartupReanchor,
    "cold-boot SDK system timestamp correction was not recoverable");
  require(
    evaluate_sdk_system_timestamp(
      TimestampDecision::kRegression, 60.01, timestamp_startup_grace_sec) ==
    TimestampRecoveryDecision::kFatal,
    "steady-state SDK system timestamp regression was accepted");
  require(
    evaluate_sdk_system_timestamp(
      TimestampDecision::kDuplicate, 1.0, timestamp_startup_grace_sec) ==
    TimestampRecoveryDecision::kFatal,
    "duplicate SDK system timestamp was accepted during startup");
  require(
    valid_capture_pair_thresholds(5000U, 20000U, 33333U),
    "split Robot A pair thresholds were rejected");
  require(
    !valid_capture_pair_thresholds(20001U, 20000U, 33333U),
    "device threshold wider than global threshold was accepted");
  require(
    !valid_capture_pair_thresholds(5000U, 33333U, 33333U),
    "global threshold of one complete frame period was accepted");
  for (const auto observed_global_pair_delta_us : {6278U, 8908U, 5058U}) {
    require(
      evaluate_capture_pair(
        19U, observed_global_pair_delta_us, 5000U, 20000U, 60.01,
        timestamp_startup_grace_sec) == CapturePairDecision::kAccepted,
      "observed healthy Robot A hardware-D2C pair was rejected");
  }
  require(
    evaluate_capture_pair(18U, 20001U, 5000U, 20000U, 1.0,
      timestamp_startup_grace_sec) ==
    CapturePairDecision::kStartupDrop,
    "cold-start global timestamp convergence frame was not dropped");
  require(
    evaluate_capture_pair(5001U, 20001U, 5000U, 20000U, 1.0,
      timestamp_startup_grace_sec) ==
    CapturePairDecision::kFatalDeviceMismatch,
    "hardware D2C device mismatch was accepted during startup");
  require(
    evaluate_capture_pair(18U, 20001U, 5000U, 20000U, 60.01,
      timestamp_startup_grace_sec) ==
    CapturePairDecision::kFatalGlobalMismatch,
    "steady-state global D2C mismatch was accepted");
  require(
    evaluate_capture_pair(18U, 20000U, 5000U, 20000U, 60.01,
      timestamp_startup_grace_sec) ==
    CapturePairDecision::kAccepted,
    "global D2C pair at the configured limit was rejected");

  constexpr double startup_timeout_sec = 3.0;
  constexpr double frame_timeout_sec = 0.5;
  constexpr double publication_timeout_sec = 2.0;
  require(
    valid_stream_timeouts(
      startup_timeout_sec, frame_timeout_sec, publication_timeout_sec),
    "valid stream timeouts were rejected");
  require(
    !valid_stream_timeouts(startup_timeout_sec, frame_timeout_sec, 0.49),
    "publication timeout shorter than frame timeout was accepted");
  require(
    !valid_stream_timeouts(
      startup_timeout_sec, frame_timeout_sec,
      std::numeric_limits<double>::quiet_NaN()),
    "non-finite publication timeout was accepted");
  auto freshness = [](StreamFreshnessObservation observation) {
      return evaluate_stream_freshness(
        observation, startup_timeout_sec, frame_timeout_sec,
        publication_timeout_sec);
    };
  require(
    freshness({false, false, false, 0U, 100.0, 0.0, 0.0, 0.0}) ==
    StreamFreshnessDecision::kHealthy,
    "disabled stream was rejected");
  require(
    freshness({true, false, false, 0U, 3.0, 0.0, 0.0, 0.0}) ==
    StreamFreshnessDecision::kHealthy,
    "startup timeout boundary was rejected early");
  require(
    freshness({true, false, false, 0U, 3.01, 0.0, 0.0, 0.0}) ==
    StreamFreshnessDecision::kStartupTimeout,
    "missing first SDK callback did not time out");
  require(
    freshness({true, true, true, 0U, 10.0, 10.0, 0.51, 0.1}) ==
    StreamFreshnessDecision::kFrameArrivalStale,
    "idle stale SDK callback stream was accepted");
  require(
    freshness({true, true, true, 1U, 10.0, 10.0, 0.70, 0.70}) ==
    StreamFreshnessDecision::kHealthy,
    "bounded in-flight publication was mislabeled as a capture outage");
  require(
    freshness({true, true, true, 1U, 10.0, 10.0, 2.01, 2.01}) ==
    StreamFreshnessDecision::kPublicationStale,
    "stuck in-flight publication was not bounded");
  require(
    freshness({true, true, false, 1U, 2.01, 2.01, 0.01, 0.0}) ==
    StreamFreshnessDecision::kPublicationStale,
    "first publication was allowed to stall indefinitely");
  require(
    freshness({true, true, true, 0U, 10.0, 10.0, 0.10, 2.01}) ==
    StreamFreshnessDecision::kPublicationStale,
    "stale publication completion clock was accepted");

  const std::vector<RoleRequest> roles{
    {"chest", "CHEST", true},
    {"belly", "BELLY", true},
    {"head", "HEAD", false},
  };
  const auto bindings = bind_enabled_roles_by_serial(roles, {"HEAD", "BELLY", "CHEST"});
  require(bindings.size() == 2U, "disabled head was bound");
  require(bindings.at("chest") == 2U, "chest serial binding mismatch");
  require(bindings.at("belly") == 1U, "belly serial binding mismatch");

  bool missing_failed = false;
  try {
    (void)bind_enabled_roles_by_serial(roles, {"HEAD", "CHEST"});
  } catch (const std::runtime_error &) {
    missing_failed = true;
  }
  require(missing_failed, "missing required camera was accepted");

  bool duplicate_device_failed = false;
  try {
    (void)bind_enabled_roles_by_serial(roles, {"CHEST", "BELLY", "BELLY"});
  } catch (const std::runtime_error &) {
    duplicate_device_failed = true;
  }
  require(duplicate_device_failed, "duplicate discovered serial was accepted");

  bool duplicate_config_failed = false;
  try {
    (void)bind_enabled_roles_by_serial(
      {{"chest", "SAME", true}, {"belly", "SAME", true}}, {"SAME"});
  } catch (const std::invalid_argument &) {
    duplicate_config_failed = true;
  }
  require(duplicate_config_failed, "duplicate configured serial was accepted");
  return 0;
}
