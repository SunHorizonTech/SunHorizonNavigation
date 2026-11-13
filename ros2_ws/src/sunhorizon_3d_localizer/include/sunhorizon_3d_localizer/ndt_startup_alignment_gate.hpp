#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace sunhorizon_3d_localizer {

struct NdtStartupAlignmentLimits {
  double maximum_seed_translation_m{1.0};
  double maximum_seed_yaw_rad{0.2617993877991494};
  double maximum_candidate_translation_spread_m{0.08};
  double maximum_candidate_yaw_spread_rad{0.03490658503988659};
  double maximum_sample_gap_sec{0.30};
  std::size_t required_consecutive_samples{10U};
};

struct NdtStartupAlignmentSample {
  std::int64_t stamp_ns{0};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double seed_translation_m{0.0};
  double seed_yaw_rad{0.0};
};

struct NdtStartupAlignmentDecision {
  bool accepted{false};
  std::size_t consecutive_samples{0U};
  std::string reason;
};

class NdtStartupAlignmentGate {
 public:
  NdtStartupAlignmentDecision observe(
      const NdtStartupAlignmentSample& sample,
      const NdtStartupAlignmentLimits& limits);

  void reset();

 private:
  bool has_anchor_{false};
  std::int64_t previous_stamp_ns_{0};
  double anchor_x_m_{0.0};
  double anchor_y_m_{0.0};
  double anchor_yaw_rad_{0.0};
  std::size_t consecutive_samples_{0U};
};

}  // namespace sunhorizon_3d_localizer
