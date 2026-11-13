#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace sunhorizon_camera_runtime {

// Diagnostic copies only. These frames have no navigation acceptance or TF.
inline constexpr char kBellyLeftIrOpticalFrame[] = "camera_belly_ir_left_optical_frame";
inline constexpr char kBellyRightIrOpticalFrame[] = "camera_belly_ir_right_optical_frame";

struct StereoSourceTime {
  std::uint64_t device_us{0}, global_us{0}, system_us{0};
};

inline bool valid_ir_global_time(std::uint64_t global_us) {
  return global_us > 0 && global_us / 1000000ULL <=
    static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max());
}

struct StereoObservationTimes {
  // Depth, left, right; device equality never implies global epoch equality.
  std::array<StereoSourceTime, 3> source;
  std::int64_t left_minus_depth_global_us{0}, right_minus_depth_global_us{0};
};

inline StereoObservationTimes stereo_observation_times(
    const std::array<StereoSourceTime, 3> & source) {
  for (const auto & time : source) {
    if (!valid_ir_global_time(time.global_us) || time.device_us == 0 ||
        time.device_us != source[0].device_us) {
      throw std::invalid_argument("invalid exact stereo observation timestamps");
    }
  }
  // ROS-representable global epochs are below INT64_MAX, so signed subtraction
  // is safe. Preserve negative deltas and never re-stamp IR with depth time.
  return {source,
    static_cast<std::int64_t>(source[1].global_us) -
      static_cast<std::int64_t>(source[0].global_us),
    static_cast<std::int64_t>(source[2].global_us) -
      static_cast<std::int64_t>(source[0].global_us)};
}

struct NativeIrObservation {
  std::uint32_t width{0}, height{0};
  std::uint64_t global_us{0};
  std::string frame_id;
  std::vector<std::uint8_t> bytes;
};

inline NativeIrObservation copy_native_ir_observation(
    const void * data, std::size_t size, std::uint32_t width, std::uint32_t height,
    std::uint64_t global_us, const char * frame_id) {
  // The already acquired Y8 streams are exact 640x480, never resized/normalized.
  if (!data || width != 640U || height != 480U || size != 640U * 480U ||
      !valid_ir_global_time(global_us) || !frame_id ||
      (std::string(frame_id) != kBellyLeftIrOpticalFrame &&
       std::string(frame_id) != kBellyRightIrOpticalFrame)) {
    throw std::invalid_argument("invalid native IR observation layout or source");
  }
  const auto * bytes = static_cast<const std::uint8_t *>(data);
  return {width, height, global_us, frame_id, {bytes, bytes + size}};
}

}  // namespace sunhorizon_camera_runtime
