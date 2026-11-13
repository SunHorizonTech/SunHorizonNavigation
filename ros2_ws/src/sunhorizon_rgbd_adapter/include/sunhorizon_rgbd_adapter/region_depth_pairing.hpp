#pragma once
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace sunhorizon_rgbd_adapter {
// Versioned length-prefixed ASCII fields; this is a contract, NOT a hash.
class RegionCanonical {
 public:
  RegionCanonical& add(const std::string& value) {
    if (value.size() > 8192 || text_.size() + value.size() + 24 > 16384)
      throw std::invalid_argument("region_contract_too_large");
    text_ += std::to_string(value.size()) + ":" + value; return *this;
  }
  RegionCanonical& number(double value) {
    if (!std::isfinite(value)) throw std::invalid_argument("nonfinite_region_contract");
    std::ostringstream out; out.imbue(std::locale::classic());
    out << std::scientific << std::setprecision(17) << (value == 0. ? 0. : value);
    return add(out.str());
  }
  const std::string& str() const { return text_; }
 private:
  std::string text_;
};
struct RegionFrameKey {
  int64_t stamp_ns{0};
  std::string frame;
  uint32_t width{0}, height{0};
  bool operator==(const RegionFrameKey& other) const {
    return stamp_ns == other.stamp_ns && frame == other.frame &&
        width == other.width && height == other.height;
  }
};
inline bool region_key_valid(const RegionFrameKey& k) {
  return k.stamp_ns > 0 && !k.frame.empty() && k.frame.size() <= 256 &&
      k.width > 0 && k.height > 0 && k.width <= 4096 && k.height <= 4096 &&
      uint64_t(k.width) * k.height <= 4194304;
}
inline bool region_times_fresh(int64_t source, int64_t received_ros,
    int64_t received_steady, int64_t now_ros, int64_t now_steady, double max_age) {
  if (source <= 0 || received_ros < source || received_steady <= 0 ||
      now_ros < received_ros || now_steady < received_steady ||
      !std::isfinite(max_age) || max_age <= 0.) return false;
  // All ordered operands are positive, so subtraction cannot overflow.
  return (now_ros - source) / 1.e9 <= max_age &&
      (now_steady - received_steady) / 1.e9 <= max_age;
}
// A clock rollback cannot silently recover within this producer instance.
class RegionClockGuard {
 public:
  bool observe(int64_t ros, int64_t steady) {
    if (ros <= 0 || steady <= 0 || ros < ros_ || steady < steady_) valid_ = false;
    ros_ = ros; steady_ = steady; return valid_;
  }
 private:
  int64_t ros_{0}, steady_{0};
  bool valid_{true};
};
// No waits. At most eight immutable small Info objects and one image reference.
// Conflicting same-key Info stays poisoned until its bounded cache entry expires.
template<class Image, class Info>
class RegionExactPairing {
 public:
  struct Pending {
    RegionFrameKey key;
    std::shared_ptr<const Image> image;
    uint64_t sequence{0};
    int64_t receipt_ros_ns{0}, receipt_steady_ns{0};
  };
  struct Entry {
    RegionFrameKey key;
    std::string canonical;
    std::shared_ptr<const Info> info;
    bool conflict{false};
  };
  bool image(Pending next) {
    pending_.reset();  // Even an invalid new image cannot resurrect the old one.
    if (!region_key_valid(next.key) || next.key.stamp_ns <= latest_stamp_) return false;
    latest_stamp_ = next.key.stamp_ns;
    pending_ = std::make_shared<Pending>(std::move(next)); return true;
  }
  bool info(RegionFrameKey key, std::string canonical, std::shared_ptr<const Info> info) {
    if (!region_key_valid(key) || canonical.size() > 8192) return false;
    for (auto& entry : entries_) {
      if (!(entry.key == key)) continue;
      entry.conflict = entry.conflict || entry.canonical != canonical;
      return true;
    }
    if (entries_.size() == 8) entries_.pop_front();
    entries_.push_back({std::move(key), std::move(canonical), std::move(info), false});
    return true;
  }
  const Entry* match(const RegionFrameKey& key) const {
    for (const auto& entry : entries_) if (entry.key == key) return &entry;
    return nullptr;
  }
  const std::shared_ptr<Pending>& pending() const { return pending_; }
  void clear() { pending_.reset(); }
  std::size_t info_count() const { return entries_.size(); }
 private:
  int64_t latest_stamp_{0};
  std::deque<Entry> entries_;
  std::shared_ptr<Pending> pending_;
};
}  // namespace sunhorizon_rgbd_adapter
