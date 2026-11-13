#include "sunhorizon_obstacle_coverage/region_support_tracker.hpp"
#include <limits>
#include <map>
#include <stdexcept>
#include <string_view>

namespace sunhorizon_obstacle_coverage {
namespace {
constexpr std::size_t kMaxContract = 16384, kMaxValues = 64, kMaxTotal = 32768;
constexpr std::uint64_t kMaxPixels = 4194304;
using Fields = std::map<std::string_view, std::string_view>;
std::string_view get(const Fields& fields, const char* key) {
  auto it = fields.find(key);
  if (it == fields.end()) throw std::invalid_argument("missing_field");
  return it->second;
}
std::uint64_t number(std::string_view value) {
  if (value.empty() || value.size() > 20 || (value.size() > 1 && value.front() == '0'))
    throw std::invalid_argument("invalid_integer");
  std::uint64_t result = 0;
  for (char c : value) {
    if (c < '0' || c > '9' || result > (std::numeric_limits<std::uint64_t>::max() - (c-'0')) / 10)
      throw std::invalid_argument("integer_overflow_or_syntax");
    result = result * 10 + (c-'0');
  }
  return result;
}
bool has_typed_canonical_suffix(std::string_view contract) {
  // Parse field boundaries: text that only happens to end with the marker is
  // not the required final length-prefixed canonical field.
  constexpr std::string_view marker = "typed-region-evaluation-diagnostics-v1";
  std::string_view last;
  while (!contract.empty()) {
    const auto colon = contract.find(':');
    if (colon == std::string_view::npos) return false;
    const auto length = number(contract.substr(0,colon));
    contract.remove_prefix(colon+1);
    if (length > 8192 || length > contract.size()) return false;
    last = contract.substr(0,static_cast<std::size_t>(length));
    contract.remove_prefix(static_cast<std::size_t>(length));
  }
  return last == marker;
}

std::int64_t ns(const Fields& f, const char* key, bool zero = false) {
  const auto n = number(get(f, key));
  if ((!zero && n == 0) || n > std::uint64_t(std::numeric_limits<std::int64_t>::max()))
    throw std::invalid_argument("invalid_timestamp");
  return static_cast<std::int64_t>(n);
}
struct Parsed {
  std::uint64_t frame, evaluation;
  std::int64_t source, received_ros, received_steady, evaluated_ros, evaluated_steady;
  std::string state, reason;
  bool complete{false};
  RegionEvaluationCode code{RegionEvaluationCode::kRejected};
  RegionDisposition disposition{RegionDisposition::kHardFault};
};
Parsed parse(const RegionSupportConfig& config, const RegionSupportReport& report) {
  if (report.values.size() > kMaxValues) throw std::invalid_argument("too_many_fields");
  Fields f; std::size_t total = 0;
  for (const auto& kv : report.values) {
    if (kv.first.empty() || kv.first.size() > 64 || kv.second.size() > kMaxContract)
      throw std::invalid_argument("field_bounds");
    total += kv.first.size() + kv.second.size();
    if (total > kMaxTotal || !f.emplace(kv.first, kv.second).second)
      throw std::invalid_argument("field_total_or_duplicate");
  }
  if (get(f,"diagnostic_event") != "region_depth_support" || get(f,"schema_version") != "1" ||
      get(f,"contract_encoding") != "length-prefixed-ascii-v1" ||
      get(f,"contract") != config.contract || get(f,"region_id") != config.region_id ||
      get(f,"base_frame_id") != config.base_frame_id || get(f,"image_frame_id") != config.image_frame_id)
    throw std::invalid_argument("identity_or_contract_mismatch");
  Parsed p{number(get(f,"region_frame_sequence")), number(get(f,"region_evaluation_sequence")),
      ns(f,"source_stamp_ns"), ns(f,"image_receipt_ros_ns"), ns(f,"image_receipt_steady_ns"),
      ns(f,"evaluation_ros_ns"), ns(f,"evaluation_steady_ns"),
      std::string(get(f,"state")), std::string(get(f,"reason"))};
  if (!p.frame || !p.evaluation || p.reason.empty() || p.reason.size() > 256 ||
      p.evaluated_ros != report.header_ros_ns || p.received_ros < p.source ||
      p.evaluated_ros < p.received_ros || p.evaluated_steady < p.received_steady)
    throw std::invalid_argument("invalid_source_timeline");
  if (p.state != "Unknown" && p.state != "ClearSupported" && p.state != "ObstacleSupported")
    throw std::invalid_argument("invalid_state");
  if (report.level != (p.state == "ClearSupported" ? 0 : 1))
    throw std::invalid_argument("state_level_mismatch");
  const auto paired = ns(f,"paired_info_stamp_ns",true), tf = ns(f,"tf_requested_stamp_ns",true);
  if ((paired != 0 && paired != p.source) || (tf != 0 && tf != p.source) ||
      (p.state != "Unknown" && (paired != p.source || tf != p.source)))
    throw std::invalid_argument("pair_or_tf_source_mismatch");
  std::uint64_t counts[7]; std::size_t i = 0;
  for (const auto* key : {"tested_pixels","intersecting_rays","clear_rays","obstacle_rays",
                         "invalid_depth_rays","front_occluded_rays","margin_band_rays"}) {
    counts[i] = number(get(f,key));
    if (counts[i++] > kMaxPixels) throw std::invalid_argument("ray_count_bound");
  }
  if (counts[1] > counts[0] || counts[2]+counts[3]+counts[4]+counts[5]+counts[6] > counts[1])
    throw std::invalid_argument("ray_count_inconsistent");
  if (p.state == "ClearSupported" && (!counts[1] || counts[2] != counts[1] ||
       counts[3] || counts[4] || counts[5] || counts[6]))
    throw std::invalid_argument("clear_support_inconsistent");
  if (p.state == "ObstacleSupported" && (!counts[3] || counts[2]+counts[3] != counts[1] ||
       counts[4] || counts[5] || counts[6]))
    throw std::invalid_argument("obstacle_support_inconsistent");
  if (config.typed_evaluation_required) {
    const auto complete = get(f,"evaluation_complete");
    if (complete != "true" && complete != "false")
      throw std::invalid_argument("invalid_evaluation_complete");
    p.complete = complete == "true";
    const auto code = number(get(f,"evaluation_code"));
    const auto disposition = number(get(f,"disposition"));
    if (code > 3 || disposition > 2) throw std::invalid_argument("invalid_evaluation_type");
    p.code = static_cast<RegionEvaluationCode>(code);
    p.disposition = static_cast<RegionDisposition>(disposition);
    if (!p.complete) {
      if (p.state != "Unknown" || code != 0 || disposition != 0)
        throw std::invalid_argument("incomplete_evaluation_type_mismatch");
    } else {
      if (!counts[1] || counts[2]+counts[3]+counts[4]+counts[5]+counts[6] != counts[1] ||
          paired != p.source || tf != p.source)
        throw std::invalid_argument("complete_evaluation_unclosed_or_unpaired");
      if (code == 1) {
        if (p.state != "ClearSupported" || disposition != 1)
          throw std::invalid_argument("typed_clear_mismatch");
      } else if (code == 2) {
        if (p.state != "ObstacleSupported" || disposition != 0)
          throw std::invalid_argument("typed_obstacle_mismatch");
      } else if (code == 3) {
        const bool pure_margin = counts[6] > 0 && !counts[3] && !counts[4] && !counts[5];
        if (p.state != "Unknown" || !(counts[4] || counts[5] || counts[6]) ||
            disposition != (pure_margin ? 2U : 0U))
          throw std::invalid_argument("typed_incomplete_rays_mismatch");
      } else {
        throw std::invalid_argument("complete_evaluation_rejected_code");
      }
    }
  }
  (void)ns(f,"region_wall_ns",true);
  if (get(f,"region_thread_cpu_ns") != "-1") (void)ns(f,"region_thread_cpu_ns",true);
  return p;
}
}  // namespace

RegionSupportTracker::RegionSupportTracker(RegionSupportConfig config) : config_(std::move(config)) {
  for (const auto* field : {&config_.status_name,&config_.region_id,&config_.base_frame_id,&config_.image_frame_id})
    if (field->empty() || field->size() > 256) throw std::invalid_argument("invalid_region_identity_pin");
  if (config_.contract.empty() || config_.contract.size() > kMaxContract)
    throw std::invalid_argument("invalid_region_contract_pin");
  if (config_.typed_evaluation_required && !has_typed_canonical_suffix(config_.contract))
    throw std::invalid_argument("typed_region_contract_suffix_missing");
}
void RegionSupportTracker::reset_decision_evidence() {
  decision_.evaluation_complete = false;
  decision_.evaluation_code = RegionEvaluationCode::kRejected;
  decision_.disposition = RegionDisposition::kHardFault;
}
void RegionSupportTracker::invalidate(const std::string& reason) {
  reset_decision_evidence();
  evaluation_complete_ = false; evaluation_code_ = RegionEvaluationCode::kRejected;
  disposition_ = RegionDisposition::kHardFault;
  fault_ = true; clear_ = false; pending_ = false; decision_.healthy = false;
  decision_.reason = "region_" + reason.substr(0,256);
}
bool RegionSupportTracker::clock_valid(std::int64_t ros, std::int64_t steady) {
  if (ros <= 0 || steady <= 0 || ros < last_ros_ || steady < last_steady_) {
    invalidate("clock_rollback_or_invalid"); return false;
  }
  last_ros_ = ros; last_steady_ = steady; return true;
}
bool RegionSupportTracker::observe(const RegionSupportReport& report, std::int64_t ros, std::int64_t steady) {
  if (report.name != config_.status_name) return false;
  if (fault_ || !clock_valid(ros,steady)) return false;
  try {
    const auto p = parse(config_,report);
    if (p.evaluated_ros > ros) throw std::invalid_argument("future_report");
    if (seen_ && (p.evaluation <= decision_.evaluation_sequence || p.frame < decision_.frame_sequence))
      throw std::invalid_argument("sequence_regression_or_repeat");
    if (seen_ && (p.received_ros < image_receipt_ros_ || p.received_steady < image_receipt_steady_ ||
                  p.evaluated_ros < evaluation_ros_ || p.evaluated_steady < evaluation_steady_))
      throw std::invalid_argument("producer_clock_rollback");
    const bool same = seen_ && p.frame == decision_.frame_sequence;
    if (same) {
      if (p.source != decision_.source_stamp_ns || p.received_ros != image_receipt_ros_ ||
          p.received_steady != image_receipt_steady_ || p.evaluated_steady < evaluation_steady_)
        throw std::invalid_argument("same_frame_timeline_changed");
      // Unknown may revoke a completed result; it can never re-arm a terminal frame.
      if (!pending_ && (p.state != "Unknown" || (config_.typed_evaluation_required && p.complete)))
        throw std::invalid_argument("repeated_terminal_result");
    } else {
      if (seen_ && p.source <= decision_.source_stamp_ns) throw std::invalid_argument("source_not_increasing");
      decision_.local_receipt_ros_ns = ros; decision_.local_receipt_steady_ns = steady;
      image_receipt_ros_ = p.received_ros; image_receipt_steady_ = p.received_steady;
    }
    pending_ = (!same || pending_) && p.state == "Unknown" && p.reason == "pending_exact_info" &&
        (!config_.typed_evaluation_required || !p.complete);
    evaluation_complete_ = p.complete; evaluation_code_ = p.code; disposition_ = p.disposition;
    reset_decision_evidence();
    clear_ = p.state == "ClearSupported";
    seen_ = true; evaluation_ros_ = p.evaluated_ros; evaluation_steady_ = p.evaluated_steady;
    decision_.frame_sequence = p.frame; decision_.evaluation_sequence = p.evaluation;
    decision_.source_stamp_ns = p.source;
    decision_.healthy = false;  // evaluate at the timer's post-TF time.
    decision_.reason = clear_ ? "region_clear_supported" : "region_" + p.reason;
    return true;
  } catch (const std::exception& error) {
    invalidate(error.what()); return false;
  }
}
RegionSupportDecision RegionSupportTracker::evaluate(std::int64_t ros, std::int64_t steady) {
  reset_decision_evidence();
  if (fault_ || !clock_valid(ros,steady) || !seen_) return decision_;
  if (ros < decision_.source_stamp_ns || ros < decision_.local_receipt_ros_ns ||
      steady < decision_.local_receipt_steady_ns) {
    invalidate("future_source_or_receipt"); return decision_;
  }
  if (ros - decision_.source_stamp_ns > kFreshnessNs ||
      ros - decision_.local_receipt_ros_ns > kFreshnessNs ||
      steady - decision_.local_receipt_steady_ns > kFreshnessNs) {
    decision_.healthy = false; decision_.reason = "region_stale"; return decision_;
  }
  decision_.healthy = clear_;
  if (config_.typed_evaluation_required) {
    decision_.evaluation_complete = evaluation_complete_;
    decision_.evaluation_code = evaluation_code_;
    decision_.disposition = disposition_;
  }
  return decision_;
}
}  // namespace sunhorizon_obstacle_coverage
