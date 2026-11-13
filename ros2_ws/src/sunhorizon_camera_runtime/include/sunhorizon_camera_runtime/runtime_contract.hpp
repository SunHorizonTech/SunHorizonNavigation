#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace sunhorizon_camera_runtime
{

// Gemini 330 hardware D2C uses color K but produces undistorted depth.
// Keep the original color calibration intact; its RGB pixels remain distorted.
template<typename CameraInfo>
CameraInfo gemini_hw_d2c_depth_info(const CameraInfo & color)
{
  CameraInfo depth = color;
  depth.distortion_model = "plumb_bob";
  depth.d.assign(5, 0.0);
  return depth;
}

enum class PixelContract
{
  kRgb8,
  kDepth16,
};

struct FrameContract
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t fps{0};
  PixelContract pixels{PixelContract::kDepth16};
};

// FPS changes acquisition cadence only; geometry, format and deadlines stay fixed.
// An exact SDK depth/IR profile must still exist; no nearest-profile fallback.
inline FrameContract belly_capture_contract(std::int64_t fps)
{
  if (fps != 15 && fps != 30) {
    throw std::invalid_argument("belly_capture_fps must be 15 or 30");
  }
  return FrameContract{640U, 480U, static_cast<std::uint32_t>(fps), PixelContract::kDepth16};
}

inline std::size_t bytes_per_pixel(PixelContract pixels)
{
  switch (pixels) {
    case PixelContract::kRgb8:
      return 3U;
    case PixelContract::kDepth16:
      return 2U;
  }
  throw std::logic_error("unsupported pixel contract");
}

inline std::size_t expected_frame_bytes(const FrameContract & contract)
{
  if (contract.width == 0U || contract.height == 0U || contract.fps == 0U) {
    throw std::invalid_argument("frame contract dimensions and fps must be non-zero");
  }
  const auto pixels = static_cast<std::size_t>(contract.width) * contract.height;
  if (pixels > std::numeric_limits<std::size_t>::max() / bytes_per_pixel(contract.pixels)) {
    throw std::overflow_error("frame byte count overflow");
  }
  return pixels * bytes_per_pixel(contract.pixels);
}

inline std::optional<std::string> validate_frame_shape(
  const FrameContract & expected,
  std::uint32_t actual_width,
  std::uint32_t actual_height,
  PixelContract actual_pixels,
  std::size_t actual_bytes)
{
  if (actual_width != expected.width || actual_height != expected.height) {
    return "frame_dimensions_mismatch";
  }
  if (actual_pixels != expected.pixels) {
    return "frame_format_mismatch";
  }
  if (actual_bytes != expected_frame_bytes(expected)) {
    return "frame_byte_count_mismatch";
  }
  return std::nullopt;
}

enum class TimestampDecision
{
  kAccepted,
  kZero,
  kDuplicate,
  kRegression,
};

enum class TimestampRecoveryDecision
{
  kAccepted,
  kStartupReanchor,
  kFatal,
};

enum class CapturePairDecision
{
  kAccepted,
  kStartupDrop,
  kFatalDeviceMismatch,
  kFatalGlobalMismatch,
};

enum class StreamFreshnessDecision
{
  kHealthy,
  kStartupTimeout,
  kFrameArrivalStale,
  kPublicationStale,
};

struct StreamFreshnessObservation
{
  bool started{false};
  bool frame_arrival_seen{false};
  bool publication_seen{false};
  std::uint32_t callbacks_in_flight{0U};
  double startup_age_sec{0.0};
  double first_frame_arrival_age_sec{0.0};
  double last_frame_arrival_age_sec{0.0};
  double last_publication_age_sec{0.0};
};

inline bool valid_stream_timeouts(
  double startup_timeout_sec,
  double frame_timeout_sec,
  double publication_timeout_sec)
{
  return std::isfinite(startup_timeout_sec) && startup_timeout_sec > 0.0 &&
         std::isfinite(frame_timeout_sec) && frame_timeout_sec > 0.0 &&
         std::isfinite(publication_timeout_sec) &&
         publication_timeout_sec >= frame_timeout_sec;
}

inline StreamFreshnessDecision evaluate_stream_freshness(
  const StreamFreshnessObservation & observation,
  double startup_timeout_sec,
  double frame_timeout_sec,
  double publication_timeout_sec)
{
  if (!observation.started) {
    return StreamFreshnessDecision::kHealthy;
  }
  if (!observation.frame_arrival_seen) {
    return observation.startup_age_sec > startup_timeout_sec ?
           StreamFreshnessDecision::kStartupTimeout :
           StreamFreshnessDecision::kHealthy;
  }

  // A callback already inside the process proves that the SDK delivered a
  // frame.  Do not relabel time spent copying or publishing that frame as a
  // hardware capture outage; the independent publication deadline below still
  // bounds a stuck callback.
  if (observation.callbacks_in_flight == 0U &&
    observation.last_frame_arrival_age_sec > frame_timeout_sec)
  {
    return StreamFreshnessDecision::kFrameArrivalStale;
  }

  const double publication_age_sec = observation.publication_seen ?
    observation.last_publication_age_sec :
    observation.first_frame_arrival_age_sec;
  if (publication_age_sec > publication_timeout_sec) {
    return StreamFreshnessDecision::kPublicationStale;
  }
  return StreamFreshnessDecision::kHealthy;
}

inline const char * stream_freshness_reason(StreamFreshnessDecision decision)
{
  switch (decision) {
    case StreamFreshnessDecision::kHealthy:
      return "ok";
    case StreamFreshnessDecision::kStartupTimeout:
      return "startup_timeout";
    case StreamFreshnessDecision::kFrameArrivalStale:
      return "frame_arrival_stale";
    case StreamFreshnessDecision::kPublicationStale:
      return "publication_stale";
  }
  return "unknown_stream_freshness_error";
}

class StrictTimestampGate
{
public:
  TimestampDecision observe(std::uint64_t timestamp_us)
  {
    if (timestamp_us == 0U) {
      return TimestampDecision::kZero;
    }
    if (!last_timestamp_us_.has_value()) {
      last_timestamp_us_ = timestamp_us;
      return TimestampDecision::kAccepted;
    }
    if (timestamp_us == *last_timestamp_us_) {
      return TimestampDecision::kDuplicate;
    }
    if (timestamp_us < *last_timestamp_us_) {
      return TimestampDecision::kRegression;
    }
    last_timestamp_us_ = timestamp_us;
    return TimestampDecision::kAccepted;
  }

  [[nodiscard]] std::optional<std::uint64_t> last_timestamp_us() const
  {
    return last_timestamp_us_;
  }

  void reanchor(std::uint64_t timestamp_us)
  {
    if (timestamp_us == 0U) {
      throw std::invalid_argument("timestamp reanchor must be non-zero");
    }
    last_timestamp_us_ = timestamp_us;
  }

private:
  std::optional<std::uint64_t> last_timestamp_us_;
};

inline bool valid_timestamp_startup_grace(double startup_grace_sec)
{
  return std::isfinite(startup_grace_sec) && startup_grace_sec > 0.0;
}

inline bool valid_capture_pair_thresholds(
  std::uint64_t maximum_device_pair_delta_us,
  std::uint64_t maximum_global_pair_delta_us,
  std::uint64_t frame_period_us)
{
  return maximum_device_pair_delta_us > 0U &&
         maximum_device_pair_delta_us <= maximum_global_pair_delta_us &&
         maximum_global_pair_delta_us < frame_period_us;
}

inline TimestampRecoveryDecision evaluate_sdk_system_timestamp(
  TimestampDecision decision,
  double startup_age_sec,
  double startup_grace_sec)
{
  if (decision == TimestampDecision::kAccepted) {
    return TimestampRecoveryDecision::kAccepted;
  }
  // Orbbec's SDK system timestamp is the host-arrival clock, not the capture
  // clock used in the ROS header.  A host wall-clock correction during cold
  // boot may therefore move it backwards while device/global capture clocks
  // remain valid.  Recovery is deliberately limited to that one condition and
  // to a bounded monotonic-clock startup window.
  if (decision == TimestampDecision::kRegression &&
    std::isfinite(startup_age_sec) && startup_age_sec >= 0.0 &&
    startup_age_sec <= startup_grace_sec)
  {
    return TimestampRecoveryDecision::kStartupReanchor;
  }
  return TimestampRecoveryDecision::kFatal;
}

inline CapturePairDecision evaluate_capture_pair(
  std::uint64_t device_pair_delta_us,
  std::uint64_t global_pair_delta_us,
  std::uint64_t maximum_device_pair_delta_us,
  std::uint64_t maximum_global_pair_delta_us,
  double startup_age_sec,
  double startup_grace_sec)
{
  if (device_pair_delta_us > maximum_device_pair_delta_us) {
    return CapturePairDecision::kFatalDeviceMismatch;
  }
  if (global_pair_delta_us <= maximum_global_pair_delta_us) {
    return CapturePairDecision::kAccepted;
  }
  // The SDK derives each stream's global epoch independently after enabling
  // global timestamps.  During cold-start convergence, discard a frame whose
  // hardware/device pair is valid but whose derived global pair is not.  The
  // independent publication watchdog still makes repeated drops fail closed.
  if (std::isfinite(startup_age_sec) && startup_age_sec >= 0.0 &&
    startup_age_sec <= startup_grace_sec)
  {
    return CapturePairDecision::kStartupDrop;
  }
  return CapturePairDecision::kFatalGlobalMismatch;
}

inline const char * timestamp_decision_reason(TimestampDecision decision)
{
  switch (decision) {
    case TimestampDecision::kAccepted:
      return "ok";
    case TimestampDecision::kZero:
      return "zero_timestamp";
    case TimestampDecision::kDuplicate:
      return "duplicate_timestamp";
    case TimestampDecision::kRegression:
      return "regressing_timestamp";
  }
  return "unknown_timestamp_error";
}

inline bool valid_static_transform(
  const std::string & parent,
  const std::string & child,
  const std::vector<double> & translation_xyz,
  const std::vector<double> & rotation_xyzw)
{
  if (parent.empty() || child.empty() || parent == child ||
    translation_xyz.size() != 3U || rotation_xyzw.size() != 4U)
  {
    return false;
  }
  for (double value : translation_xyz) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  double squared_norm = 0.0;
  for (double value : rotation_xyzw) {
    if (!std::isfinite(value)) {
      return false;
    }
    squared_norm += value * value;
  }
  return std::abs(std::sqrt(squared_norm) - 1.0) <= 1e-6;
}

struct RoleRequest
{
  std::string role;
  std::string serial;
  bool enabled{false};
};

inline std::map<std::string, std::size_t> bind_enabled_roles_by_serial(
  const std::vector<RoleRequest> & roles,
  const std::vector<std::string> & discovered_serials)
{
  std::unordered_set<std::string> configured;
  for (const auto & role : roles) {
    if (role.role.empty() || role.serial.empty()) {
      throw std::invalid_argument("camera role and serial must be non-empty");
    }
    if (!configured.insert(role.serial).second) {
      throw std::invalid_argument("configured camera serial is assigned to multiple roles: " + role.serial);
    }
  }

  std::unordered_map<std::string, std::size_t> discovered;
  for (std::size_t index = 0; index < discovered_serials.size(); ++index) {
    const auto & serial = discovered_serials[index];
    if (serial.empty()) {
      throw std::runtime_error("camera enumeration returned an empty serial");
    }
    if (!discovered.emplace(serial, index).second) {
      throw std::runtime_error("duplicate connected camera serial: " + serial);
    }
  }

  std::map<std::string, std::size_t> bindings;
  for (const auto & role : roles) {
    if (!role.enabled) {
      continue;
    }
    const auto found = discovered.find(role.serial);
    if (found == discovered.end()) {
      throw std::runtime_error(
              "required camera not found for role " + role.role + ": " + role.serial);
    }
    bindings.emplace(role.role, found->second);
  }
  return bindings;
}

}  // namespace sunhorizon_camera_runtime
