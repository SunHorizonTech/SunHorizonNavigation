#include "sunhorizon_camera_runtime/stereo_ir_observation.hpp"
#include <stdexcept>

using namespace sunhorizon_camera_runtime;
void require(bool value) { if (!value) throw std::runtime_error("IR observation failed"); }
template<typename Callback> void rejects(Callback callback) {
  bool rejected = false;
  try { callback(); } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected);
}

int main() {
  const std::array<StereoSourceTime, 3> source{{
    {100, 1000000, 2000000}, {100, 1000019, 2000050}, {100, 999993, 2000090}}};
  const auto times = stereo_observation_times(source);
  require(times.left_minus_depth_global_us == 19);
  require(times.right_minus_depth_global_us == -7);
  require(times.source[1].global_us == 1000019 && times.source[2].global_us == 999993);
  require(times.source[2].system_us == 2000090);
  for (std::size_t channel = 0; channel < source.size(); ++channel) {
    auto mismatch = source;
    mismatch[channel].device_us = 101;
    rejects([&]() { stereo_observation_times(mismatch); });
    mismatch = source;
    mismatch[channel].global_us = 0;
    rejects([&]() { stereo_observation_times(mismatch); });
  }
  auto overflow = source;
  overflow[1].global_us = 2147483648ULL * 1000000ULL;
  rejects([&]() { stereo_observation_times(overflow); });
  auto extreme = source;
  extreme[0].global_us = 1;
  extreme[1].global_us = 2147483647ULL * 1000000ULL + 999999;
  require(stereo_observation_times(extreme).left_minus_depth_global_us ==
    2147483647999998LL);

  std::vector<std::uint8_t> raw(640 * 480);
  for (std::size_t i = 0; i < raw.size(); ++i) raw[i] = static_cast<std::uint8_t>(i % 256);
  const auto left = copy_native_ir_observation(raw.data(), raw.size(), 640, 480,
    times.source[1].global_us, kBellyLeftIrOpticalFrame);
  const auto right = copy_native_ir_observation(raw.data(), raw.size(), 640, 480,
    times.source[2].global_us, kBellyRightIrOpticalFrame);
  require(left.bytes == raw && right.bytes == raw);
  require(left.frame_id != right.frame_id && left.global_us != right.global_us);
  require(left.width == 640 && left.height == 480);
  raw[0] = 255;  // Copy owns immutable evidence after SDK callback storage changes.
  require(left.bytes[0] == 0);
  rejects([&]() { copy_native_ir_observation(nullptr, raw.size(), 640, 480,
    1, kBellyLeftIrOpticalFrame); });
  rejects([&]() { copy_native_ir_observation(raw.data(), raw.size() - 1, 640, 480,
    1, kBellyLeftIrOpticalFrame); });
  rejects([&]() { copy_native_ir_observation(raw.data(), raw.size(), 1280, 480,
    1, kBellyLeftIrOpticalFrame); });
  rejects([&]() { copy_native_ir_observation(raw.data(), raw.size(), 640, 480,
    1, "camera_belly_depth_optical_frame"); });
  rejects([&]() { copy_native_ir_observation(raw.data(), raw.size(), 640, 480,
    0, kBellyLeftIrOpticalFrame); });
}
