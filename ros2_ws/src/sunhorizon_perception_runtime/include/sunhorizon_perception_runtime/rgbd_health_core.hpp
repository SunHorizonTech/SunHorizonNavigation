#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "sunhorizon_navigation_core/rgbd_validation.hpp"
#include "sunhorizon_rgbd_adapter/rgbd_status_snapshot.hpp"

namespace sunhorizon_perception_runtime {
struct RgbdHealthConfig {
  double target_rate_hz{15.0}, status_rate_hz{20.0}, max_pair_delta_sec{0.005};
  double freshness_timeout_sec{0.30}, statistics_window_sec{10.0};
  double minimum_healthy_pair_rate_hz{12.0}, maximum_rejected_pair_ratio{0.05};
  double depth_scale_m{0.001}, min_depth_m{0.20}, max_depth_m{8.0};
  double minimum_valid_depth_ratio{0.01};
  std::int64_t queue_size{2}, pointcloud_pixel_stride{2};
  bool chest_alignment_calibration_accepted{false}, chest_depth_scale_accepted{false};
  bool belly_projection_calibration_accepted{false}, belly_depth_scale_accepted{false};
  std::string calibration_report_sha256;
};
struct RgbdAdmission {
  bool accepted{false}, computation_allowed{false};
  std::int64_t source_stamp_ns{0};
  std::uint64_t sequence{0};
  std::string reason;
};
// ROS-free observation used by the deterministic contract tests. The adapter
// derives all metadata and validity from the actual immutable CameraFrame.
struct RgbdHealthObservation {
  std::string role;
  std::uint64_t sequence{0};
  std::int64_t depth_source_ns{0}, color_source_ns{0};
  std::int64_t receipt_ros_ns{-1}, receipt_steady_ns{-1};
  bool source_validated{false}, depth_shape_valid{false}, color_shape_valid{false};
  bool depth_info_matches{false}, color_info_matches{false};
  double depth_scale_m{0.0}, valid_depth_ratio{0.0};
  sunhorizon_navigation_core::ImageMetadata color, depth;
  sunhorizon_navigation_core::CameraIntrinsics color_info, depth_info;
};
struct RgbdHealthSnapshot {
  sunhorizon_rgbd_adapter::RgbdStatusState source;
  bool chest_pair_fresh{false}, belly_depth_fresh{false};
  bool chest_pointcloud_healthy{false}, belly_pointcloud_healthy{false};
  double chest_pair_age_sec{0.0}, belly_depth_age_sec{0.0};
  double chest_pointcloud_age_sec{0.0}, belly_pointcloud_age_sec{0.0};
  double accepted_pair_rate_hz{0.0}, rejected_pair_ratio{1.0};
  // Numeric values match the unchanged RgbdStatus wire contract.
  std::uint8_t state{1};
  std::string reason;
};

class RgbdHealthCore {
 public:
  explicit RgbdHealthCore(RgbdHealthConfig config)
  : config_(std::move(config)), validator_({config_.max_pair_delta_sec,
      config_.depth_scale_m, config_.min_depth_m, config_.max_depth_m}) {validate_config();}
  const RgbdHealthConfig & config() const noexcept {return config_;}

  RgbdAdmission admit(const RgbdHealthObservation & input,
    std::int64_t now_ros_ns, std::int64_t now_steady_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    initialize(now_steady_ns);
    RgbdAdmission output; output.source_stamp_ns=input.depth_source_ns; output.sequence=input.sequence;
    const bool chest=input.role=="chest";
    if (!chest && input.role!="belly") {output.reason="unsupported_camera_role";return output;}
    auto & progress=chest?chest_:belly_;
    const auto reject=[&](std::string reason) {
      output.reason=std::move(reason);
      progress.failure=output.reason;
      if (chest) {
        status_.rejected(seconds(now_steady_ns), config_.statistics_window_sec);
        status_.update([&](auto & s){s.last_rejection_reason=output.reason;});
      } else status_.update([&](auto & s){s.belly_reason=output.reason;});
      return output;
    };
    if (!fatal_reason_.empty()) return reject("camera_fault_latched:"+fatal_reason_);
    if (input.sequence==0 || input.sequence<=progress.last_seen_sequence ||
      input.depth_source_ns<=0 || input.depth_source_ns<=progress.last_seen_source_ns) {
      return reject(chest?"chest_non_monotonic_timestamp":"belly_non_monotonic_timestamp");
    }
    progress.last_seen_sequence=input.sequence;
    progress.last_seen_source_ns=input.depth_source_ns;
    if (!input.source_validated || !fresh(input.depth_source_ns, now_ros_ns) ||
      !receipt_fresh(input.receipt_steady_ns, now_steady_ns) || input.receipt_ros_ns<0 ||
      input.receipt_ros_ns>now_ros_ns) return reject("camera_source_invalid_or_stale");
    if (!std::isfinite(input.depth_scale_m) || input.depth_scale_m!=config_.depth_scale_m)
      return reject("depth_scale_mismatch");
    const bool depth_info_valid=sunhorizon_navigation_core::valid_camera_intrinsics(input.depth_info) &&
      input.depth_info_matches;
    const bool color_info_valid=sunhorizon_navigation_core::valid_camera_intrinsics(input.color_info) &&
      input.color_info_matches;
    status_.update([&](auto & s){
      if (chest) s.chest_info_valid=depth_info_valid && color_info_valid;
      else {s.belly_info_valid=depth_info_valid;s.belly_depth_stamp=seconds(input.depth_source_ns);
        s.belly_depth_source_ns=input.depth_source_ns;s.belly_depth_callback_sequence=input.sequence;}
    });
    if (chest) {
      // Rate selection is intentionally not a rejected pair, matching the
      // original adapter. Sequence/source high-water checks still run first.
      if (chest_.last_accepted_color_ns>0 && input.color_source_ns>chest_.last_accepted_color_ns &&
        seconds(input.color_source_ns-chest_.last_accepted_color_ns)<1.0/config_.target_rate_hz) {
        output.reason="target_rate_throttled";return output;
      }
      const auto decision=validator_.evaluate(input.color,input.depth,input.color_info,input.depth_info,
        config_.chest_alignment_calibration_accepted && config_.chest_depth_scale_accepted);
      status_.update([&](auto & s){s.last_pair_delta_sec=decision.pair_delta_sec;});
      if (!decision.stream_accepted) return reject(decision.reason);
      if (!input.color_shape_valid || !depth_info_valid || !color_info_valid)
        return reject("chest_image_or_camera_info_mismatch");
    } else if (!depth_info_valid) return reject("belly_camera_info_invalid");
    if (!input.depth_shape_valid || !std::isfinite(input.valid_depth_ratio) ||
      input.valid_depth_ratio<config_.minimum_valid_depth_ratio || input.valid_depth_ratio>1.)
      return reject(chest?"chest_depth_invalid_or_empty":"belly_depth_invalid_or_empty");
    progress.admitted_sequence=input.sequence;progress.admitted_source_ns=input.depth_source_ns;
    progress.receipt_steady_ns=input.receipt_steady_ns;
    progress.failure.clear();
    output.accepted=true;
    if (chest) {
      chest_.last_accepted_color_ns=input.color_source_ns;
      status_.accepted(seconds(now_steady_ns),seconds(input.color_source_ns),config_.statistics_window_sec);
      output.computation_allowed=config_.chest_alignment_calibration_accepted && config_.chest_depth_scale_accepted;
    } else {
      output.computation_allowed=config_.belly_projection_calibration_accepted && config_.belly_depth_scale_accepted;
      status_.update([&](auto & s){s.belly_reason=output.computation_allowed?"ok":"belly_calibration_not_accepted";});
    }
    output.reason=output.computation_allowed?"ok":"calibration_not_accepted";
    progress.computation_allowed=output.computation_allowed;
    // Admission and cloud egress are independently scheduled. Retain bounded
    // immutable identities so a newer source frame cannot invalidate a still
    // fresh older computation, or renew that computation's original receipt.
    if (output.computation_allowed) {
      if (progress.admitted.size()==64) progress.admitted.pop_front();
      progress.admitted.push_back({input.sequence,input.depth_source_ns,input.receipt_steady_ns,false});
    }
    return output;
  }

  bool complete(const std::string & role, std::uint64_t sequence, std::int64_t cloud_source_ns,
    std::int64_t now_ros_ns, std::int64_t now_steady_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (role!="chest" && role!="belly") return false;
    auto & p=role=="chest"?chest_:belly_;
    const auto found=std::find_if(p.admitted.begin(),p.admitted.end(),
      [sequence](const auto & token){return token.sequence==sequence;});
    if (!fatal_reason_.empty() || found==p.admitted.end() || found->failed ||
      cloud_source_ns!=found->source_ns || sequence<=p.completed_sequence ||
      !fresh(cloud_source_ns,now_ros_ns) || !receipt_fresh(found->receipt_steady_ns,now_steady_ns)) return false;
    p.completed_sequence=sequence;p.completion_steady_ns=now_steady_ns;
    p.completed_receipt_steady_ns=found->receipt_steady_ns;
    if (sequence==p.last_seen_sequence) p.failure.clear();
    if(sequence>p.upstream_failed_sequence)p.upstream_failure.clear();
    status_.update([&](auto & s){
      if (role=="chest") s.chest_points_stamp=seconds(cloud_source_ns);
      else {s.belly_points_stamp=seconds(cloud_source_ns);s.belly_points_source_ns=cloud_source_ns;
        s.belly_points_callback_sequence=sequence;s.belly_reason="ok";}
    });
    return true;
  }
  void reject_computation(const std::string & role, std::uint64_t sequence, const std::string & reason, bool upstream_until_completion=false) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (role!="chest" && role!="belly") return;
    auto & p=role=="chest"?chest_:belly_;
    const auto found=std::find_if(p.admitted.begin(),p.admitted.end(),
      [sequence](const auto & token){return token.sequence==sequence;});
    if (found==p.admitted.end()) return;
    found->failed=true;
    if(upstream_until_completion && sequence>p.completed_sequence && sequence>=p.upstream_failed_sequence) {
      p.upstream_failed_sequence=sequence;p.upstream_failure="computation:"+reason;
    }
    p.failure="computation:"+reason;p.computation_allowed=false;
    status_.update([&](auto & s){
      if (role=="chest") s.last_rejection_reason=p.failure;else s.belly_reason=p.failure;
    });
  }
  void fault(const std::string & reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fatal_reason_.empty()) fatal_reason_=reason.empty()?"unknown_camera_fault":reason;
  }
  RgbdHealthSnapshot snapshot(std::int64_t now_ros_ns,std::int64_t now_steady_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    initialize(now_steady_ns);
    RgbdHealthSnapshot o;o.source=status_.snapshot(seconds(now_steady_ns),config_.statistics_window_sec);
    o.chest_pair_age_sec=age_seconds(seconds(now_ros_ns),o.source.chest_pair_stamp);
    o.belly_depth_age_sec=age_seconds(seconds(now_ros_ns),o.source.belly_depth_stamp);
    o.chest_pointcloud_age_sec=age_seconds(seconds(now_ros_ns),o.source.chest_points_stamp);
    o.belly_pointcloud_age_sec=age_seconds(seconds(now_ros_ns),o.source.belly_points_stamp);
    o.chest_pair_fresh=age_fresh(o.chest_pair_age_sec) && receipt_fresh(chest_.receipt_steady_ns,now_steady_ns);
    o.belly_depth_fresh=age_fresh(o.belly_depth_age_sec) && receipt_fresh(belly_.receipt_steady_ns,now_steady_ns);
    o.chest_pointcloud_healthy=age_fresh(o.chest_pointcloud_age_sec) &&
      receipt_fresh(chest_.completion_steady_ns,now_steady_ns) &&
      receipt_fresh(chest_.completed_receipt_steady_ns,now_steady_ns) && chest_.failure.empty() && chest_.upstream_failure.empty();
    o.belly_pointcloud_healthy=age_fresh(o.belly_pointcloud_age_sec) &&
      receipt_fresh(belly_.completion_steady_ns,now_steady_ns) &&
      receipt_fresh(belly_.completed_receipt_steady_ns,now_steady_ns) && belly_.failure.empty() && belly_.upstream_failure.empty();
    if (!fatal_reason_.empty()) {
      o.chest_pair_fresh=false;o.belly_depth_fresh=false;
      o.chest_pointcloud_healthy=false;o.belly_pointcloud_healthy=false;
    }
    o.accepted_pair_rate_hz=double(o.source.window_accepted)/config_.statistics_window_sec;
    const auto attempts=o.source.window_accepted+o.source.window_rejected;
    o.rejected_pair_ratio=attempts?double(o.source.window_rejected)/double(attempts):1.;
    std::ostringstream reason;
    if (!fatal_reason_.empty()) reason<<"camera_fault_latched="<<fatal_reason_<<';';
    if (o.source.statistics_overflow) reason<<"pair_statistics_overflow;";
    if (!o.chest_pair_fresh) reason<<"chest_pair_stale;";
    if (!o.source.chest_info_valid) reason<<"chest_camera_info_invalid;";
    if (!config_.chest_alignment_calibration_accepted) reason<<"chest_alignment_not_accepted;";
    if (!config_.chest_depth_scale_accepted) reason<<"chest_depth_scale_not_accepted;";
    if (!o.chest_pointcloud_healthy) reason<<"chest_pointcloud_stale;";
    if (!o.belly_depth_fresh) reason<<"belly_depth_stale;";
    if (!o.source.belly_info_valid) reason<<"belly_camera_info_invalid;";
    if (!config_.belly_projection_calibration_accepted) reason<<"belly_projection_not_accepted;";
    if (!config_.belly_depth_scale_accepted) reason<<"belly_depth_scale_not_accepted;";
    if (!o.belly_pointcloud_healthy) reason<<"belly_pointcloud_stale;";
    if (o.accepted_pair_rate_hz<config_.minimum_healthy_pair_rate_hz) reason<<"pair_rate_low;";
    if (o.rejected_pair_ratio>config_.maximum_rejected_pair_ratio) reason<<"pair_rejection_ratio_high;";
    if (!o.source.last_rejection_reason.empty()) reason<<"last_pair_rejection="<<o.source.last_rejection_reason<<';';
    if (!o.source.belly_reason.empty() && o.source.belly_reason!="ok") reason<<"belly="<<o.source.belly_reason<<';';
    if(!chest_.upstream_failure.empty())reason<<"chest_upstream="<<chest_.upstream_failure<<';';
    if(!belly_.upstream_failure.empty())reason<<"belly_upstream="<<belly_.upstream_failure<<';';
    o.reason=reason.str().empty()?"ok":reason.str();
    o.state=o.reason=="ok"?2:(!fatal_reason_.empty()?4:
      (now_steady_ns>=startup_steady_ns_ && seconds(now_steady_ns-startup_steady_ns_)<config_.statistics_window_sec?1:4));
    return o;
  }
 private:
  struct AdmissionToken {
    std::uint64_t sequence;
    std::int64_t source_ns,receipt_steady_ns;
    bool failed;
  };
  struct Progress {
    std::deque<AdmissionToken> admitted;
    std::uint64_t last_seen_sequence{0}, admitted_sequence{0}, completed_sequence{0};
    std::int64_t last_seen_source_ns{0}, admitted_source_ns{0}, last_accepted_color_ns{0};
    std::int64_t receipt_steady_ns{-1},completion_steady_ns{-1},completed_receipt_steady_ns{-1};
    bool computation_allowed{false};std::string failure;
    uint64_t upstream_failed_sequence{0};std::string upstream_failure;
  };
  static double seconds(std::int64_t ns) {return double(ns)/1.e9;}
  static double age_seconds(double now,double stamp) {
    return stamp>0. && std::isfinite(stamp)?now-stamp:std::numeric_limits<double>::infinity();
  }
  bool age_fresh(double age) const {return std::isfinite(age) && age>=0. && age<=config_.freshness_timeout_sec;}
  bool fresh(std::int64_t source,std::int64_t now) const {return source>0 && now>=source && age_fresh(seconds(now-source));}
  bool receipt_fresh(std::int64_t receipt,std::int64_t now) const {return receipt>=0 && now>=receipt && age_fresh(seconds(now-receipt));}
  void initialize(std::int64_t now) {if (startup_steady_ns_<0) startup_steady_ns_=now;}
  void validate_config() const {
    for (const double v:{config_.target_rate_hz,config_.status_rate_hz,config_.max_pair_delta_sec,
      config_.freshness_timeout_sec,config_.statistics_window_sec,config_.minimum_healthy_pair_rate_hz,
      config_.depth_scale_m,config_.max_depth_m,config_.minimum_valid_depth_ratio})
      if (!std::isfinite(v) || v<=0.) throw std::invalid_argument("invalid RGB-D health positive parameter");
    if (!std::isfinite(config_.maximum_rejected_pair_ratio) || config_.maximum_rejected_pair_ratio<0. ||
      config_.maximum_rejected_pair_ratio>=1. || !std::isfinite(config_.min_depth_m) || config_.min_depth_m<0. ||
      config_.max_depth_m<=config_.min_depth_m || config_.minimum_valid_depth_ratio>1. ||
      config_.queue_size<2 || config_.queue_size>64 || config_.pointcloud_pixel_stride<1 ||
      config_.pointcloud_pixel_stride>std::numeric_limits<std::int32_t>::max()/2)
      throw std::invalid_argument("invalid RGB-D health parameter");
    if ((config_.chest_alignment_calibration_accepted || config_.chest_depth_scale_accepted ||
      config_.belly_projection_calibration_accepted || config_.belly_depth_scale_accepted) &&
      config_.calibration_report_sha256.size()!=64)
      throw std::invalid_argument("accepted RGB-D calibration requires a 64-character calibration_report_sha256");
  }
  const RgbdHealthConfig config_;
  sunhorizon_navigation_core::RgbdPairValidator validator_;
  sunhorizon_rgbd_adapter::RgbdStatusSnapshot status_;
  std::mutex mutex_;Progress chest_,belly_;
  std::string fatal_reason_;std::int64_t startup_steady_ns_{-1};
};
}  // namespace sunhorizon_perception_runtime
