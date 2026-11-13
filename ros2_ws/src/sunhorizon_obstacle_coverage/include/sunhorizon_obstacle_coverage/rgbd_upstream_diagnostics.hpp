#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace sunhorizon_obstacle_coverage::upstream_diagnostics {

inline int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline uint64_t double_bits(double value) {
  static_assert(sizeof(double) == sizeof(uint64_t));
  uint64_t result;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}
template<class Status> std::array<uint64_t, 4> age_bits(const Status& s) {
  return {double_bits(s.chest_pair_age_sec), double_bits(s.chest_pointcloud_age_sec),
          double_bits(s.belly_depth_age_sec), double_bits(s.belly_pointcloud_age_sec)};
}
template<class Status> uint16_t health_bits(const Status& s) {
  const std::array<bool, 9> values{s.chest_pair_fresh, s.chest_camera_info_valid,
    s.chest_alignment_accepted, s.chest_depth_scale_accepted, s.chest_pointcloud_healthy,
    s.belly_depth_fresh, s.belly_camera_info_valid, s.belly_depth_scale_accepted,
    s.belly_pointcloud_healthy};
  uint16_t bits = 0;
  for (std::size_t i = 0; i < values.size(); ++i) if (values[i]) bits |= uint16_t{1} << i;
  return bits;
}

enum class Event : uint8_t { kEntry, kProjectionBegin, kProjectionEnd,
  kPointsPublishBegin, kPointsPublishReturn, kFinish,
  kStatusPublishBegin, kStatusPublishReturn, kRgbdReceipt,
  kDecisionPublishBegin, kDecisionPublishReturn, kExporterTerminal };
inline const char* event_name(Event e) {
  switch (e) {
    case Event::kEntry: return "entry";
    case Event::kProjectionBegin: return "projection_begin";
    case Event::kProjectionEnd: return "projection_end";
    case Event::kPointsPublishBegin: return "points_publish_begin";
    case Event::kPointsPublishReturn: return "points_publish_return";
    case Event::kFinish: return "finish";
    case Event::kStatusPublishBegin: return "status_publish_begin";
    case Event::kStatusPublishReturn: return "status_publish_return";
    case Event::kRgbdReceipt: return "rgbd_receipt";
    case Event::kDecisionPublishBegin: return "decision_publish_begin";
    case Event::kDecisionPublishReturn: return "decision_publish_return";
    case Event::kExporterTerminal: return "exporter_terminal";
  }
  return "unknown";
}
// Fixed POD records: no hot-path strings, file I/O or extra mutex.
struct Record {
  Event event{Event::kEntry};
  uint64_t sequence{0}, callback_sequence{0}, depth_callback_sequence{0}, points_callback_sequence{0};
  int64_t source_ns{0}, depth_source_ns{0}, points_source_ns{0};
  int64_t status_header_ns{0}, decision_header_ns{0}, receipt_ros_ns{0};
  int64_t event_steady_ns{0}, snapshot_begin_ns{0}, snapshot_end_ns{0};
  std::array<uint64_t, 4> ages{};
  uint16_t health{0};
  uint8_t outcome{0};  // Stage: 0 running, 1 published, 2 rejected, 3 exception/aborted.
  bool header_ambiguous{false}, input_seen{false};
};

// One writer (one original mutually exclusive callback group), one exporter.
// Records remain queued when a consumer is absent/full. Capacity is fixed.
template<std::size_t Capacity = 12800> class Queue {
 public:
  static_assert(std::atomic<uint64_t>::is_always_lock_free);
  bool try_push(const Record& record) noexcept {
    const auto write = write_.load(std::memory_order_relaxed);
    if (write - read_.load(std::memory_order_acquire) == Capacity) {
      drops_.fetch_add(1, std::memory_order_relaxed); return false;
    }
    records_[write % Capacity] = record;
    write_.store(write + 1, std::memory_order_release); return true;
  }
  const Record* front() const noexcept {
    const auto read = read_.load(std::memory_order_relaxed);
    return read == write_.load(std::memory_order_acquire) ? nullptr : &records_[read % Capacity];
  }
  void pop() noexcept { read_.fetch_add(1, std::memory_order_release); }
  uint64_t drops() const noexcept { return drops_.load(std::memory_order_relaxed); }
  uint64_t pending() const noexcept {
    // Read first: a concurrent pop must never underflow an earlier write sample.
    // Concurrent writer progress may make this conservative, capped at capacity.
    const auto read = read_.load(std::memory_order_acquire);
    const auto write = write_.load(std::memory_order_acquire);
    return std::min<uint64_t>(write - read, Capacity);
  }
 private:
  std::array<Record, Capacity> records_{};
  std::atomic<uint64_t> read_{0}, write_{0}, drops_{0};
};

// Rollback/duplicate H is an ambiguity, even while a recovered clock catches up.
class HeaderIdentity {
 public:
  bool observe(int64_t header) noexcept {
    const bool ambiguous = header <= 0 || (seen_ && header <= high_water_);
    if (header > high_water_) high_water_ = header;
    seen_ = true;
    return ambiguous;
  }
 private:
  int64_t high_water_{0}; bool seen_{false};
};

inline void validate_endpoint(const std::string& path) {
  constexpr const char* prefix = "/data/navigation/evidence/";
  if (path.compare(0, std::strlen(prefix), prefix) != 0 || path.size() >= sizeof(sockaddr_un::sun_path)
      || path.size() < 5 || path.substr(path.size() - 5) != ".sock"
      || path.find("/../") != std::string::npos || path.find("/./") != std::string::npos
      || path.find("//") != std::string::npos || path.find('\0') != std::string::npos)
    throw std::invalid_argument("invalid_upstream_diagnostic_endpoint");
}
inline std::string endpoint_from_environment() {
  const auto* value = std::getenv("SUNHORIZON_RGBD_TRACE_SOCKET");
  if (!value || !*value) return {};
  std::string endpoint(value); validate_endpoint(endpoint); return endpoint;
}

inline std::string encode(const Record& r, const std::string& role,
    const std::string& boot, int pid, const std::string& startticks, unsigned uid,
    int64_t epoch, std::size_t channel,
    uint64_t export_sequence, uint64_t dropped, uint64_t pending, uint64_t send_failures) {
  std::ostringstream o;
  o << "{\"schema_version\":1,\"role\":\"" << role << "\",\"boot_id\":\"" << boot
    << "\",\"pid\":" << pid << ",\"startticks\":\"" << startticks
    << "\",\"uid\":" << uid << ",\"epoch_steady_ns\":" << epoch
    << ",\"channel\":" << channel << ",\"export_sequence\":" << export_sequence
    << ",\"queue_drops\":" << dropped << ",\"queue_pending\":" << pending
    << ",\"send_failures\":" << send_failures
    << ",\"event\":\"" << event_name(r.event) << "\",\"sequence\":" << r.sequence
    << ",\"callback_sequence\":" << r.callback_sequence
    << ",\"depth_callback_sequence\":" << r.depth_callback_sequence
    << ",\"points_callback_sequence\":" << r.points_callback_sequence
    << ",\"source_ns\":" << r.source_ns << ",\"depth_source_ns\":" << r.depth_source_ns
    << ",\"points_source_ns\":" << r.points_source_ns
    << ",\"status_header_ns\":" << r.status_header_ns
    << ",\"decision_header_ns\":" << r.decision_header_ns
    << ",\"receipt_ros_ns\":" << r.receipt_ros_ns
    << ",\"event_steady_ns\":" << r.event_steady_ns
    << ",\"snapshot_begin_ns\":" << r.snapshot_begin_ns
    << ",\"snapshot_end_ns\":" << r.snapshot_end_ns
    << ",\"age_bits\":[" << r.ages[0] << ',' << r.ages[1] << ',' << r.ages[2] << ',' << r.ages[3]
    << "],\"health_bits\":" << r.health << ",\"outcome\":" << unsigned(r.outcome)
    << ",\"header_ambiguous\":" << (r.header_ambiguous ? "true" : "false")
    << ",\"input_seen\":" << (r.input_seen ? "true" : "false")
    << ",\"complete\":false}";
  return o.str();
}

// No DDS in this worker. Every send is one <=4096-byte nonblocking datagram.
// Shutdown has no external wait: wake, finish at most 64 nonblocking attempts,
// join. A missing/full receiver cannot block node callbacks or shutdown.
class Exporter {
 public:
  Exporter(Queue<>& first, Queue<>& second, std::string endpoint, std::string role)
      : queues_{&first, &second}, role_(std::move(role)), epoch_(steady_ns()), pid_(getpid()) {
    std::ifstream boot("/proc/sys/kernel/random/boot_id"); std::getline(boot, boot_);
    // Local non-Linux tests have no boot file; production Linux must have it.
    if (boot_.empty()) boot_ = "unavailable";
    std::ifstream process("/proc/self/stat"); std::string line; std::getline(process, line);
    const auto close_paren = line.rfind(')');
    if (close_paren != std::string::npos) {
      std::istringstream fields(line.substr(close_paren + 1));
      for (unsigned i = 0; i <= 19; ++i) fields >> startticks_;
    }
    if (startticks_.empty()) startticks_ = "unavailable";
    address_.sun_family = AF_UNIX;
    if (endpoint.size() >= sizeof(address_.sun_path)) throw std::invalid_argument("endpoint_too_long");
    std::memcpy(address_.sun_path, endpoint.c_str(), endpoint.size() + 1);
    fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd_ < 0) throw std::runtime_error("diagnostic_socket_failed");
    try { worker_ = std::thread([this] { run(); }); }
    catch (...) { close(fd_); fd_ = -1; throw; }
  }
  ~Exporter() { stop(); if (fd_ >= 0) close(fd_); }
  void stop() noexcept {
    stopped_.store(true); wake_.notify_all();
    if (worker_.joinable()) worker_.join();
  }
  Exporter(const Exporter&) = delete;
  Exporter& operator=(const Exporter&) = delete;
  std::array<uint64_t, 2> pending_counts() const noexcept {
    return {queues_[0]->pending(), queues_[1]->pending()};
  }
 private:
  bool send_one(std::size_t channel) noexcept {
    auto& queue = *queues_[channel];
    const auto* record = queue.front(); if (!record) return false;
    try {
      const auto text = encode(*record, role_, boot_, pid_, startticks_, geteuid(), epoch_, channel,
          exported_ + 1, queue.drops(), queue.pending(), failures_);
      if (text.size() > 4096 || sendto(fd_, text.data(), text.size(), MSG_DONTWAIT,
            reinterpret_cast<sockaddr*>(&address_), sizeof(address_)) != static_cast<ssize_t>(text.size())) {
        ++failures_; return false;
      }
      ++exported_; queue.pop(); return true;
    } catch (...) { ++failures_; return false; }
  }
  void run() noexcept {
    while (!stopped_.load()) {
      for (std::size_t channel = 0; channel < queues_.size(); ++channel) {
        for (unsigned i = 0; i < 32 && !stopped_.load(); ++i) {
          if (!send_one(channel)) break;  // Retain pending event on failed send.
        }
      }
      std::unique_lock<std::mutex> lock(wait_mutex_);
      wake_.wait_for(lock, std::chrono::milliseconds(20), [this] { return stopped_.load(); });
    }
    // One bounded final drain, never retry or sleep for a missing/full receiver.
    for (std::size_t channel = 0; channel < queues_.size(); ++channel)
      for (unsigned i = 0; i < 64; ++i) if (!send_one(channel)) break;
    // Best effort terminal counters; if these do not arrive, shutdown remainder
    // is unknown to the collector. The queues/counters still retain it locally.
    for (std::size_t channel = 0; channel < queues_.size(); ++channel) {
      try {
        Record terminal; terminal.event = Event::kExporterTerminal;
        terminal.event_steady_ns = steady_ns();
        auto& queue = *queues_[channel];
        const auto text = encode(terminal, role_, boot_, pid_, startticks_, geteuid(), epoch_, channel,
            exported_ + 1, queue.drops(), queue.pending(), failures_);
        if (text.size() <= 4096 && sendto(fd_, text.data(), text.size(), MSG_DONTWAIT,
            reinterpret_cast<sockaddr*>(&address_), sizeof(address_)) == static_cast<ssize_t>(text.size())) ++exported_;
      } catch (...) {}
    }
  }
  std::array<Queue<>*, 2> queues_;
  std::string role_, boot_, startticks_;
  int64_t epoch_; int pid_, fd_{-1}; sockaddr_un address_{};
  uint64_t exported_{0}, failures_{0};
  std::atomic<bool> stopped_{false}; std::mutex wait_mutex_;
  std::condition_variable wake_; std::thread worker_;
};

// Entry/stage begin events can leave the process before a production publish
// returns. Destruction cannot mask an original callback exception.
class Stage {
 public:
  Stage(Queue<>& queue, uint64_t sequence, int64_t source, int64_t receipt)
      : queue_(queue) {
    record_.callback_sequence = sequence; record_.source_ns = source;
    record_.receipt_ros_ns = receipt; emit(Event::kEntry);
  }
  ~Stage() noexcept { if (!finished_) finish(3); }
  void emit(Event event) noexcept {
    record_.event = event; record_.event_steady_ns = steady_ns(); queue_.try_push(record_);
  }
  void finish(uint8_t outcome) noexcept {
    record_.outcome = outcome; emit(Event::kFinish); finished_ = true;
  }
 private:
  Queue<>& queue_; Record record_; bool finished_{false};
};

}  // namespace sunhorizon_obstacle_coverage::upstream_diagnostics
