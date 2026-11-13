#pragma once
// Opt-in diagnostics; fixed-size samples, report after callback lock release.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <sys/syscall.h>
#include <unistd.h>

namespace sunhorizon_localization_manager {
class DeliveryTiming {
 public:
  static bool enabled() noexcept {
    static const bool value = [] {
      const char *value = std::getenv("SUNHORIZON_LOCALIZATION_DELIVERY_TIMING");
      return value && std::strcmp(value, "1") == 0;
    }();
    return value;
  }
  DeliveryTiming(const char *scope, const char *backend, int64_t source_ns,
                 int64_t *previous_entry = nullptr) noexcept
      : enabled_(enabled()), scope_(scope), backend_(backend), source_ns_(source_ns) {
    if (!enabled_) return;
    previous_ = current_;
    current_ = this;
    realtime_ = clock_ns(CLOCK_REALTIME);
    wall_ = last_wall_ = clock_ns(CLOCK_MONOTONIC);
    cpu_ = last_cpu_ = clock_ns(CLOCK_THREAD_CPUTIME_ID);
    if (previous_entry) {
      gap_ = *previous_entry ? wall_ - *previous_entry : 0;
      *previous_entry = wall_;
    }
  }
  static void mark(const char *stage) noexcept {
    if (current_) current_->sample(stage);
  }
  ~DeliveryTiming() noexcept {
    if (!enabled_) return;
    sample("tail");
    current_ = previous_;
    if (last_wall_ - wall_ < 30000000 && gap_ < 100000000) return;
    char line[3072];
    int used = std::snprintf(line, sizeof(line),
        "LOCALIZATION_DELIVERY_TIMING scope=%s backend=%s tid=%ld source_ns=%lld "
        "begin_ns=%lld callback_gap_ms=%.3f wall_ms=%.3f cpu_ms=%.3f",
        scope_, backend_, syscall(SYS_gettid), static_cast<long long>(source_ns_),
        static_cast<long long>(realtime_), gap_*1e-6,
        (last_wall_-wall_)*1e-6, (last_cpu_-cpu_)*1e-6);
    for (int i = 0; i < count_ && used > 0 && used < int(sizeof(line))-1; ++i) {
      const auto &s = samples_[i];
      const int n = std::snprintf(line+used, sizeof(line)-used,
          " %s_ms=%.3f %s_cpu_ms=%.3f", s.name, s.wall*1e-6, s.name, s.cpu*1e-6);
      if (n < 0 || n >= int(sizeof(line))-used) break;
      used += n;
    }
    if (used > 0 && used < int(sizeof(line))-1) {
      line[used++] = '\n';
      std::fwrite(line, 1, used, stderr);
    }
  }

 private:
  static int64_t clock_ns(clockid_t id) noexcept {
    timespec t{};
    if (clock_gettime(id, &t) != 0) return 0;
    return int64_t(t.tv_sec)*1000000000+t.tv_nsec;
  }
  void sample(const char *stage) noexcept {
    const auto wall = clock_ns(CLOCK_MONOTONIC);
    const auto cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
    if (count_ < 24) samples_[count_++] = {stage, wall-last_wall_, cpu-last_cpu_};
    last_wall_ = wall;
    last_cpu_ = cpu;
  }
  struct Sample { const char *name; int64_t wall, cpu; };
  inline static thread_local DeliveryTiming *current_{nullptr};
  DeliveryTiming *previous_{nullptr};
  bool enabled_;
  const char *scope_, *backend_;
  int64_t source_ns_, realtime_{0}, wall_{0}, cpu_{0}, last_wall_{0}, last_cpu_{0}, gap_{0};
  Sample samples_[24]{};
  int count_{0};
};
}  // namespace sunhorizon_localization_manager
