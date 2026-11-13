#pragma once

#include <cmath>
#include <cstddef>
#include <optional>

namespace sunhorizon_sensor_rate_probe
{

struct RateSnapshot
{
  std::size_t samples{0};
  std::size_t receive_interval_count{0};
  double receive_span_sec{0.0};
  double receive_rate_hz{0.0};
  std::size_t header_samples{0};
  std::size_t header_interval_count{0};
  double header_span_sec{0.0};
  double header_rate_hz{0.0};
  std::size_t header_regressions{0};
  std::size_t header_duplicates{0};
};

class RateTracker
{
public:
  void observe(double received_at_sec, std::optional<double> header_sec)
  {
    if (samples_ == 0) {
      first_receive_sec_ = received_at_sec;
    }
    last_receive_sec_ = received_at_sec;
    ++samples_;

    if (!header_sec.has_value() || !std::isfinite(*header_sec)) {
      return;
    }
    if (header_samples_ == 0) {
      first_header_sec_ = *header_sec;
    } else if (*header_sec < last_header_sec_) {
      ++header_regressions_;
    } else if (*header_sec == last_header_sec_) {
      ++header_duplicates_;
    }
    last_header_sec_ = *header_sec;
    ++header_samples_;
  }

  [[nodiscard]] RateSnapshot snapshot() const
  {
    RateSnapshot result;
    result.samples = samples_;
    if (samples_ > 1) {
      result.receive_interval_count = samples_ - 1;
      result.receive_span_sec = last_receive_sec_ - first_receive_sec_;
      if (result.receive_span_sec > 0.0) {
        result.receive_rate_hz =
          static_cast<double>(result.receive_interval_count) / result.receive_span_sec;
      }
    }
    result.header_samples = header_samples_;
    if (header_samples_ > 1) {
      result.header_interval_count = header_samples_ - 1;
      result.header_span_sec = last_header_sec_ - first_header_sec_;
      if (result.header_span_sec > 0.0) {
        result.header_rate_hz =
          static_cast<double>(result.header_interval_count) / result.header_span_sec;
      }
    }
    result.header_regressions = header_regressions_;
    result.header_duplicates = header_duplicates_;
    return result;
  }

private:
  std::size_t samples_{0};
  double first_receive_sec_{0.0};
  double last_receive_sec_{0.0};
  std::size_t header_samples_{0};
  double first_header_sec_{0.0};
  double last_header_sec_{0.0};
  std::size_t header_regressions_{0};
  std::size_t header_duplicates_{0};
};

}  // namespace sunhorizon_sensor_rate_probe
