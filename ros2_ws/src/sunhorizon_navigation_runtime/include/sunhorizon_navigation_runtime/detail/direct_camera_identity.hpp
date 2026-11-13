#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include "sunhorizon_camera_runtime/direct_source_identity.hpp"

namespace sunhorizon_navigation_runtime::detail {
struct DirectCameraSample {
  std::shared_ptr<const sunhorizon_camera_runtime::CameraSourceIdentity> owner;
  uint64_t sequence{0};
  int64_t depth_source_ns{0}, color_source_ns{0}, receipt_ros_ns{0}, receipt_steady_ns{0};
};
// Diagnostic evidence only: no clocks, publication, image owners, or health
// authorization. Repeated observers cannot renew an original SDK receipt.
class DirectCameraIdentity {
 public:
  struct Snapshot {std::array<DirectCameraSample,2> samples; std::string failure;};
  void accepted(const std::string& role, DirectCameraSample sample) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.failure.empty()) return;
    if ((role!="belly" && role!="chest") || !sample.owner ||
        sample.owner->owner_fqn.empty() || sample.owner->opened_serial.empty() ||
        sample.owner->capture_epoch.empty() || !sample.sequence || sample.depth_source_ns<=0 ||
        sample.receipt_ros_ns<=0 || sample.receipt_steady_ns<=0) {
      state_.failure="direct_camera_identity_missing"; return;
    }
    auto& previous=state_.samples[role=="belly"?0:1];
    if (previous.owner && (sample.owner->owner_fqn!=previous.owner->owner_fqn ||
        sample.owner->opened_serial!=previous.owner->opened_serial ||
        sample.owner->capture_epoch!=previous.owner->capture_epoch)) {
      state_.failure="direct_camera_owner_changed"; return;
    }
    if (previous.sequence && (sample.sequence<=previous.sequence ||
        sample.depth_source_ns<=previous.depth_source_ns ||
        sample.receipt_ros_ns<previous.receipt_ros_ns || sample.receipt_steady_ns<previous.receipt_steady_ns)) {
      state_.failure="direct_camera_measurement_reordered"; return;
    }
    previous=std::move(sample);
  }
  Snapshot snapshot() const {std::lock_guard<std::mutex> lock(mutex_);return state_;}
 private:
  mutable std::mutex mutex_;
  Snapshot state_;
};
}  // namespace sunhorizon_navigation_runtime::detail
