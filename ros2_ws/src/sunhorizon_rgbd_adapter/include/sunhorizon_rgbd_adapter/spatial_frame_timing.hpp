#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string_view>

namespace sunhorizon_rgbd_adapter {
inline bool spatial_expiry_timing_hint(std::string_view reason,int64_t decision_ns,int64_t first_valid_h) {
  if (first_valid_h<=0 || decision_ns<first_valid_h || reason.size()>4096) return false;
  constexpr std::string_view token="spatial_visibility:spatial_frozen_evidence_expired";
  for (std::size_t start=0;start<=reason.size();) {
    const auto end=reason.find(';',start);
    if (reason.substr(start,end==std::string_view::npos?end:end-start)==token) return true;
    if (end==std::string_view::npos) break;
    start=end+1;
  }
  return false;
}
// Required BELLY ingress only. Fixed memory; diagnostics never authorize data.
// All stage values are steady ns, with 0 meaning not observed. RMW timestamps
// remain exactly supplied (including 0 unknown); they are not clock conversions.
enum class SpatialTimingStage : std::size_t {
  LegCheckStart, LegCheckEnd, ModelStart, ModelEnd, CommonTfStart, CommonTfEnd,
  SamplingStart, SamplingEnd, FusionStart, FusionEnd, OwnersStart, OwnersEnd,
  PublishStart, PublishReturn, Count
};
struct SpatialFrameTiming {
  uint64_t sequence{0}, attempts{0};
  int64_t source_ns{0}, callback_ros_ns{0}, callback_steady_ns{0};
  int64_t rmw_source_ns{0}, rmw_received_ns{0};
  int64_t paired_ns{0}, first_evaluate_ns{0}, last_evaluate_ns{0};
  int64_t first_future_wait_ns{0}, last_future_wait_ns{0};
  // Historical query at last_future_wait_ns, not the current admission state.
  int last_future_link_index{-1};int64_t last_future_source_query_ns{0},last_future_latest_upper_ns{0};
  bool last_evaluate_timer{false}, dropped{false};
  std::array<int64_t,static_cast<std::size_t>(SpatialTimingStage::Count)> stages{};
};
class SpatialFrameTimingRing {
 public:
  static constexpr std::size_t capacity=256;
  struct Snapshot {
    std::array<SpatialFrameTiming,capacity> records{};std::size_t count{0};bool claimed{false};
    uint64_t overwritten{0};int64_t trigger_d_ns{0},trigger_ros_ns{0},trigger_steady_ns{0};
  };
  void ingress(uint64_t seq,int64_t source,int64_t ros,int64_t steady,int64_t rmw_source,int64_t rmw_received) {
    std::lock_guard<std::mutex> lock(mutex_);
    SpatialFrameTiming value;value.sequence=seq;value.source_ns=source;
    value.callback_ros_ns=ros;value.callback_steady_ns=steady;
    value.rmw_source_ns=rmw_source;value.rmw_received_ns=rmw_received;
    records_[next_]=value;next_=(next_+1)%capacity;if (count_<capacity) ++count_;
    else if (overwritten_<std::numeric_limits<uint64_t>::max()) ++overwritten_;
  }
  void paired(uint64_t seq,int64_t steady) {update(seq,[&](auto& v){if (!v.paired_ns) v.paired_ns=steady;});}
  void dropped(uint64_t seq) {update(seq,[](auto& v){v.dropped=true;});}
  void evaluate(uint64_t seq,int64_t steady,bool timer) {
    update(seq,[&](auto& v){if (!v.first_evaluate_ns) v.first_evaluate_ns=steady;
      v.last_evaluate_ns=steady;v.last_evaluate_timer=timer;v.stages.fill(0);
      if (v.attempts<std::numeric_limits<uint64_t>::max()) ++v.attempts;});
  }
  void future_wait(uint64_t seq,int64_t steady,int link=-1,int64_t query=0,int64_t upper=0) {
    update(seq,[&](auto& v){if (!v.first_future_wait_ns) v.first_future_wait_ns=steady;v.last_future_wait_ns=steady;
      v.last_future_link_index=link;v.last_future_source_query_ns=query;v.last_future_latest_upper_ns=upper;});
  }
  void stage(uint64_t seq,SpatialTimingStage stage,int64_t steady) {
    const auto index=static_cast<std::size_t>(stage);
    if (index<static_cast<std::size_t>(SpatialTimingStage::Count)) update(seq,[&](auto& v){v.stages[index]=steady;});
  }
  Snapshot snapshot_once() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (dumped_) return {};
    dumped_=true;return copy_unlocked();
  }
  void freeze_hint(int64_t decision,int64_t ros,int64_t steady) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (hint_frozen_) return;
    hint_=copy_unlocked();hint_.trigger_d_ns=decision;hint_.trigger_ros_ns=ros;hint_.trigger_steady_ns=steady;
    hint_frozen_=true;
  }
  Snapshot hint_snapshot_once() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!hint_frozen_ || hint_dumped_) return {};
    hint_dumped_=true;return hint_;
  }
 private:
  Snapshot copy_unlocked() const {
    Snapshot result;result.claimed=true;result.count=count_;result.overwritten=overwritten_;
    const auto begin=(next_+capacity-count_)%capacity;
    for (std::size_t i=0;i<count_;++i) result.records[i]=records_[(begin+i)%capacity];
    return result;
  }
  template<class F> void update(uint64_t seq,F operation) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t i=0;i<count_;++i) {
      auto& value=records_[(next_+capacity-1-i)%capacity];
      if (value.sequence==seq) {operation(value);return;}
    }
  }
  std::array<SpatialFrameTiming,capacity> records_{};
  std::size_t next_{0},count_{0};uint64_t overwritten_{0};bool dumped_{false};std::mutex mutex_;
  Snapshot hint_;bool hint_frozen_{false},hint_dumped_{false};
};

// Separate TF callback group and data-group snapshots share only this short
// diagnostic mutex. Partial callbacks remain partial in immutable snapshots.
struct SpatialPrivateTfTiming {
  uint64_t sequence{0};bool is_static{false};std::size_t message_count{0},scanned_count{0},inserted_count{0};
  int64_t rmw_source_ns{0},rmw_received_ns{0},callback_ros_ns{0},callback_steady_ns{0};
  int64_t owner_start_ns{0},owner_end_ns{0},inserted_end_ns{0},callback_end_ns{0};
  // 0 in progress, 1 returned, 2 startup-owner pending, 3 rejected.
  int outcome{0};
  std::array<uint32_t,12> link_count{};
  std::array<int64_t,12> link_min_source_ns{},link_max_source_ns{};
  void edge(std::size_t index,int64_t source) {
    if (index>=link_count.size()) return;
    if (!link_count[index]) link_min_source_ns[index]=link_max_source_ns[index]=source;
    else {if (source<link_min_source_ns[index]) link_min_source_ns[index]=source;
      if (source>link_max_source_ns[index]) link_max_source_ns[index]=source;}
    if (link_count[index]<std::numeric_limits<uint32_t>::max()) ++link_count[index];
  }
};
class SpatialPrivateTfTimingRing {
 public:
  static constexpr std::size_t capacity=256;
  struct Snapshot {
    std::array<SpatialPrivateTfTiming,capacity> records{};std::size_t count{0};bool claimed{false};
    uint64_t overwritten{0};int64_t trigger_d_ns{0},trigger_ros_ns{0},trigger_steady_ns{0};
  };
  uint64_t ingress(SpatialPrivateTfTiming value) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Counter exhaustion is unknown diagnostic identity, never a production exception.
    value.sequence=sequence_==std::numeric_limits<uint64_t>::max()?0:++sequence_;
    records_[next_]=value;next_=(next_+1)%capacity;if (count_<capacity) ++count_;
    else if (overwritten_<std::numeric_limits<uint64_t>::max()) ++overwritten_;
    return value.sequence;
  }
  enum class Stage {OwnerStart,OwnerEnd};
  void stage(uint64_t seq,Stage stage,int64_t steady) {
    update(seq,[&](auto& v){if (stage==Stage::OwnerStart) v.owner_start_ns=steady;
      else v.owner_end_ns=steady;});
  }
  void outcome(uint64_t seq,int outcome) {update(seq,[&](auto& v){v.outcome=outcome;});}
  void inserted(uint64_t seq,std::size_t count,int64_t steady) {
    update(seq,[&](auto& v){v.inserted_count=count;v.inserted_end_ns=steady;});
  }
  void end(uint64_t seq,int64_t steady) {update(seq,[&](auto& v){v.callback_end_ns=steady;if (!v.outcome) v.outcome=1;});}
  void freeze_hint(int64_t decision,int64_t ros,int64_t steady) {
    std::lock_guard<std::mutex> lock(mutex_);if (hint_frozen_) return;
    hint_=copy_unlocked();hint_.trigger_d_ns=decision;hint_.trigger_ros_ns=ros;hint_.trigger_steady_ns=steady;hint_frozen_=true;
  }
  Snapshot hint_snapshot_once() {
    std::lock_guard<std::mutex> lock(mutex_);if (!hint_frozen_ || hint_dumped_) return {};
    hint_dumped_=true;return hint_;
  }
  Snapshot snapshot_once() {
    std::lock_guard<std::mutex> lock(mutex_);if (dumped_) return {};dumped_=true;return copy_unlocked();
  }
 private:
  Snapshot copy_unlocked() const {
    Snapshot result;result.claimed=true;result.count=count_;result.overwritten=overwritten_;
    for (std::size_t i=0;i<count_;++i) result.records[i]=records_[(next_+capacity-count_+i)%capacity];
    return result;
  }
  template<class F> void update(uint64_t seq,F operation) {
    if (!seq) return;
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t i=0;i<count_;++i) {auto& v=records_[(next_+capacity-1-i)%capacity];
      if (v.sequence==seq) {operation(v);return;}}
  }
  std::mutex mutex_;std::array<SpatialPrivateTfTiming,capacity> records_{};
  std::size_t next_{0},count_{0};uint64_t sequence_{0},overwritten_{0};bool dumped_{false};
  Snapshot hint_;bool hint_frozen_{false},hint_dumped_{false};
};
class SpatialPrivateTfTimingEnd {
 public:
  SpatialPrivateTfTimingEnd(SpatialPrivateTfTimingRing& ring,uint64_t seq,int64_t (*clock)())
      :ring_(ring),seq_(seq),clock_(clock) {}
  ~SpatialPrivateTfTimingEnd() noexcept {try {ring_.end(seq_,clock_());} catch (...) {}}
 private:
  SpatialPrivateTfTimingRing& ring_;uint64_t seq_;int64_t (*clock_)();
};
} // namespace sunhorizon_rgbd_adapter
