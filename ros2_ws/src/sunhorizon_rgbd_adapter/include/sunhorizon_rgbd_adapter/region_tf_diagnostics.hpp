#pragma once
#include <cstdint>
#include <string>
#include <tf2/exceptions.hpp>

namespace sunhorizon_rgbd_adapter {
// Audit only: never used to retry, select a transform, or authorize Clear.
struct RegionTfDiagnostics {
  static constexpr std::size_t kMaxErrorBytes = 512;
  bool error_truncated{false};
  std::string error_type{"none"}, error_detail;
  int64_t lookup_wall_ns{0};
  void finish(int64_t start_steady, int64_t end_steady) {
    lookup_wall_ns = start_steady >= 0 && end_steady >= start_steady ? end_steady - start_steady : -1;
  }
  void failure(const tf2::TransformException& error, int64_t start_steady, int64_t end_steady) {
    finish(start_steady, end_steady);
    if (dynamic_cast<const tf2::LookupException*>(&error)) error_type = "lookup";
    else if (dynamic_cast<const tf2::ConnectivityException*>(&error)) error_type = "connectivity";
    else if (dynamic_cast<const tf2::ExtrapolationException*>(&error)) error_type = "extrapolation";
    else if (dynamic_cast<const tf2::InvalidArgumentException*>(&error)) error_type = "invalid_argument";
    else if (dynamic_cast<const tf2::TimeoutException*>(&error)) error_type = "timeout";
    else error_type = "transform";
    const char* text = error.what();
    error_detail.clear(); error_truncated = false;
    // At most 513 bytes inspected; control/non-ASCII bytes do not enter wire text.
    for (std::size_t i = 0; text[i] != '\0'; ++i) {
      if (i == kMaxErrorBytes) { error_truncated = true; break; }
      const auto byte = static_cast<unsigned char>(text[i]);
      error_detail += byte >= 32 && byte <= 126 ? static_cast<char>(byte) : '?';
    }
  }
};
}  // namespace sunhorizon_rgbd_adapter
