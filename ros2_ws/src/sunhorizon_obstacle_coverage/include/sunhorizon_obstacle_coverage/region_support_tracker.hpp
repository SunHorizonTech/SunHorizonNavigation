#pragma once
#include "sunhorizon_obstacle_coverage/region_depth_support.hpp"
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace sunhorizon_obstacle_coverage {
struct RegionSupportConfig {
  std::string status_name, region_id, base_frame_id, image_frame_id, contract;
  bool typed_evaluation_required{false};
};
struct RegionSupportReport {
  std::string name;
  std::uint8_t level{0};
  std::int64_t header_ros_ns{0};
  std::vector<std::pair<std::string, std::string>> values;
};
struct RegionSupportDecision {
  bool healthy{false};
  std::string reason{"region_missing"};
  std::uint64_t frame_sequence{0}, evaluation_sequence{0};
  std::int64_t source_stamp_ns{0}, local_receipt_ros_ns{0}, local_receipt_steady_ns{0};
  bool evaluation_complete{false};
  RegionEvaluationCode evaluation_code{RegionEvaluationCode::kRejected};
  RegionDisposition disposition{RegionDisposition::kHardFault};
};
// Pure, no ROS/clock/I/O. Serialize observe/evaluate/invalidate on ONE tracker.
// The caller must reject duplicate matching statuses in a DiagnosticArray.
// A matching malformed report, ordering violation or clock rollback is sticky:
// construct a new tracker only as part of an explicitly verified owner restart.
class RegionSupportTracker {
 public:
  static constexpr std::int64_t kFreshnessNs = 300000000;
  explicit RegionSupportTracker(RegionSupportConfig config);
  // Sample both local clocks inside the caller mutex immediately before observe.
  // Nonmatching names are ignored. true means parsed/accepted, NOT healthy.
  bool observe(const RegionSupportReport& report, std::int64_t receipt_ros_ns,
               std::int64_t receipt_steady_ns);
  // Mutates clock high-water marks; evaluating a disposable copy loses rollback detection.
  RegionSupportDecision evaluate(std::int64_t now_ros_ns, std::int64_t now_steady_ns);
  void invalidate(const std::string& reason);
 private:
  bool clock_valid(std::int64_t ros_ns, std::int64_t steady_ns);
  void reset_decision_evidence();
  RegionSupportConfig config_;
  RegionSupportDecision decision_;
  bool fault_{false}, seen_{false}, pending_{false}, clear_{false};
  // Parsed evidence stays private until evaluate rechecks both local clocks/freshness.
  bool evaluation_complete_{false};
  RegionEvaluationCode evaluation_code_{RegionEvaluationCode::kRejected};
  RegionDisposition disposition_{RegionDisposition::kHardFault};
  std::int64_t last_ros_{0}, last_steady_{0};
  std::int64_t image_receipt_ros_{0}, image_receipt_steady_{0}, evaluation_ros_{0}, evaluation_steady_{0};
};
}  // namespace sunhorizon_obstacle_coverage
