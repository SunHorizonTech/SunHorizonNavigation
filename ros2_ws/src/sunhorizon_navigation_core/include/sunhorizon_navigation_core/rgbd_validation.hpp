#pragma once

#include <cstdint>
#include <string>

namespace sunhorizon_navigation_core {

struct ImageMetadata {
  double stamp_sec{0.0};
  uint32_t width{0};
  uint32_t height{0};
  std::string encoding;
  std::string frame_id;
};

struct CameraIntrinsics {
  uint32_t width{0};
  uint32_t height{0};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  std::string frame_id;
};

struct RgbdValidationLimits {
  double max_pair_delta_sec{0.005};
  double depth_scale_m{0.001};
  double min_depth_m{0.20};
  double max_depth_m{8.0};
};

struct RgbdPairDecision {
  bool stream_accepted{false};
  bool pointcloud_allowed{false};
  double pair_delta_sec{0.0};
  std::string reason;
};

class RgbdPairValidator {
 public:
  explicit RgbdPairValidator(RgbdValidationLimits limits = {});

  RgbdPairDecision evaluate(
      const ImageMetadata& color, const ImageMetadata& depth,
      const CameraIntrinsics& color_info, const CameraIntrinsics& depth_info,
      bool alignment_calibration_accepted);

 private:
  RgbdValidationLimits limits_;
  double last_color_stamp_sec_{0.0};
  double last_depth_stamp_sec_{0.0};
};

bool valid_camera_intrinsics(const CameraIntrinsics& info, std::string* reason = nullptr);
bool depth_sample_within_tolerance(
    uint16_t raw_depth, double known_distance_m, double depth_scale_m = 0.001,
    double absolute_tolerance_m = 0.02, double relative_tolerance = 0.02);

}  // namespace sunhorizon_navigation_core
