#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

namespace sunhorizon_livo_bringup {
// Observation only, serialized by the adapter's existing state mutex. This
// stores original output receipts, never a new freshness lease or watchdog state.
struct PropagationAuditStamp {
  int64_t source_ns{0}, receipt_ros_ns{0}, receipt_steady_ns{0};
  uint64_t sequence{0};
  std::string identity;
  // enqueue, dequeue, geometry-wait begin/end; absent is not zero latency.
  std::optional<std::array<int64_t,4>> delivery_timing;
};
struct PropagationCausalEvent {
  uint64_t event{0};
  double horizon_sec{0.0};
  // The watchdog may install a recovery anchor while returning accepted=false.
  // Keep its actual horizon basis distinct from the last accepted input below.
  int64_t watchdog_anchor_source_ns{0};
  PropagationAuditStamp propagated;
  std::optional<PropagationAuditStamp> accepted_corrected, rejected_corrected;
  std::string corrected_rejection;
  std::string text() const {
    std::ostringstream out;
    out << std::setprecision(17) << "event=" << event << " horizon_sec=" << horizon_sec
        << " watchdog_anchor_source_ns=" << watchdog_anchor_source_ns;
    const auto stamp=[&](const char* prefix,const PropagationAuditStamp& value){
      out << ' ' << prefix << "_source_ns=" << value.source_ns
          << ' ' << prefix << "_sequence=" << value.sequence
          << ' ' << prefix << "_receipt_ros_ns=" << value.receipt_ros_ns
          << ' ' << prefix << "_receipt_steady_ns=" << value.receipt_steady_ns
          << ' ' << prefix << "_identity=" << std::quoted(value.identity);
      out << ' ' << prefix << "_delivery_timing_available=" << bool(value.delivery_timing);
      if(value.delivery_timing){
        const std::array<const char*,4> keys{{"output_lane_enqueue_steady_ns",
          "output_lane_dequeue_steady_ns","geometry_wait_begin_steady_ns","geometry_wait_end_steady_ns"}};
        for(std::size_t i=0;i<keys.size();++i)
          out << ' ' << prefix << '_' << keys[i] << '=' << (*value.delivery_timing)[i];
      }
    };
    stamp("propagated",propagated);
    out << " accepted_corrected_available=" << bool(accepted_corrected);
    if(accepted_corrected)stamp("accepted_corrected",*accepted_corrected);
    out << " rejected_corrected_available=" << bool(rejected_corrected);
    if(rejected_corrected)stamp("rejected_corrected",*rejected_corrected);
    out << " corrected_rejection=" << std::quoted(corrected_rejection);
    return out.str();
  }
};
class PropagationCausalAudit {
 public:
  void accepted_corrected(PropagationAuditStamp stamp) {accepted_=bounded(std::move(stamp));}
  void rejected_corrected(PropagationAuditStamp stamp,const std::string& reason) {
    rejected_=bounded(std::move(stamp));rejection_=reason.substr(0,256);
  }
  void propagated_accepted() {unanchored_=false;}
  std::optional<PropagationCausalEvent> unanchored(
      PropagationAuditStamp stamp,double horizon,int64_t watchdog_anchor_source_ns) {
    if(unanchored_)return std::nullopt;
    unanchored_=true;
    return PropagationCausalEvent{++events_,horizon,watchdog_anchor_source_ns,
      bounded(std::move(stamp)),accepted_,rejected_,rejection_};
  }
 private:
  static PropagationAuditStamp bounded(PropagationAuditStamp stamp) {
    stamp.identity.resize(std::min<std::size_t>(stamp.identity.size(),256));return stamp;
  }
  std::optional<PropagationAuditStamp> accepted_,rejected_;
  std::string rejection_;
  uint64_t events_{0};
  bool unanchored_{false};
};
}  // namespace sunhorizon_livo_bringup
