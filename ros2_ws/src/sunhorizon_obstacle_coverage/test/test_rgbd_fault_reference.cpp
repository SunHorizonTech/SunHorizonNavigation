#include "sunhorizon_obstacle_coverage/rgbd_fault_reference.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>

#define CHECK(value) do { if (!(value)) { std::cerr << "CHECK failed: " #value << '\n'; std::exit(1); } } while (false)

struct Status {
  struct { struct { int32_t sec{0}; uint32_t nanosec{0}; } stamp; } header;
  double chest_pair_age_sec{0}, chest_pointcloud_age_sec{0};
  double belly_depth_age_sec{0}, belly_pointcloud_age_sec{0};
  bool chest_pair_fresh{false}, chest_camera_info_valid{false};
  bool chest_alignment_accepted{false}, chest_depth_scale_accepted{false};
  bool chest_pointcloud_healthy{false}, belly_depth_fresh{false};
  bool belly_camera_info_valid{false}, belly_depth_scale_accepted{false};
  bool belly_pointcloud_healthy{false};
};

int main() {
  using namespace sunhorizon_obstacle_coverage;
  RgbdFaultReference missing;
  CHECK(!missing.input_seen && missing.callback_sequence == 0);
  CHECK(rgbd_fault_reference_diagnostics(missing, 1791940420563864153LL)
      .find("D_ns=1791940420563864153,H_ns=0,C_ns=0") != std::string::npos);
  RgbdFaultReferenceState state;
  Status a;
  a.header.stamp = {1791940420, 563864153};
  a.chest_pair_age_sec = -0.0;
  a.chest_pointcloud_age_sec = std::numeric_limits<double>::infinity();
  a.belly_depth_age_sec = .313944817;
  a.belly_pointcloud_age_sec = std::numeric_limits<double>::quiet_NaN();
  a.chest_pair_fresh = a.belly_depth_fresh = a.belly_pointcloud_healthy = true;
  const auto snapshot = state.capture(a, 1791940420547272682LL);
  CHECK(snapshot.header_ns == 1791940420563864153LL);
  CHECK(snapshot.receipt_ns == 1791940420547272682LL);
  CHECK(snapshot.callback_sequence == 1 && snapshot.input_seen);
  CHECK(snapshot.header_valid && !snapshot.header_ambiguous);
  CHECK(snapshot.ages == upstream_diagnostics::age_bits(a));
  CHECK(snapshot.health == ((1U << 0) | (1U << 5) | (1U << 8)));
  auto b = a;
  b.header.stamp.nanosec += 1;
  b.belly_depth_age_sec = .01;
  const auto newer = state.capture(b, 1791940420547272683LL);
  CHECK(newer.callback_sequence == 2 && !newer.header_ambiguous);
  CHECK(snapshot.callback_sequence == 1 && snapshot.ages != newer.ages);
  CHECK(snapshot.header_ns + 1 == newer.header_ns);  // Snapshot does not track latest.
  b.belly_depth_age_sec = .02;
  const auto duplicate = state.capture(b, 123);
  CHECK(duplicate.callback_sequence == 3 && duplicate.header_ambiguous);
  CHECK(duplicate.ages != newer.ages);
  b.header.stamp.nanosec -= 2;
  CHECK(state.capture(b, 124).header_ambiguous);
  b.header.stamp = {2000000000, 1000000000U};
  const auto malformed = state.capture(b, 125);
  CHECK(!malformed.header_valid && malformed.header_ambiguous);
  b.header.stamp = {1791940420, 563864155};
  CHECK(!state.capture(b, 126).header_ambiguous);  // Invalid H did not poison high water.
  b.header.stamp = {-1, 999999999};
  CHECK(!state.capture(b, 127).header_valid);
  b.header.stamp = {0, 0};
  CHECK(state.capture(b, 128).header_ambiguous);
  const auto log = rgbd_fault_reference_diagnostics(snapshot, 1791940420563864153LL);
  CHECK(log.find("D_ns=1791940420563864153,H_ns=1791940420563864153,C_ns=1791940420547272682") != std::string::npos);
  CHECK(log.find("callback_seq=1,input_seen=1,header_valid=1,header_ambiguous=0") != std::string::npos);
  CHECK(log.find("health_bits=289") != std::string::npos);
  CHECK(log.size() < 512);
  std::cout << "rgbd fault reference checks passed; fixed reference bytes=" << sizeof(snapshot) << '\n';
}
