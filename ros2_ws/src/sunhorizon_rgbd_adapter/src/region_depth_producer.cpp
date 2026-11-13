#include "sunhorizon_rgbd_adapter/region_depth_producer.hpp"
#include <algorithm>
#include <ctime>
#include <limits>
#include <vector>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace sunhorizon_rgbd_adapter {
namespace {
int64_t steady_now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
int64_t cpu_now() {
  timespec t{}; return clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t) == 0 ?
      int64_t(t.tv_sec) * 1000000000 + t.tv_nsec : -1;
}
int64_t source_ns(const builtin_interfaces::msg::Time& t) {
  if (t.sec < 0 || t.nanosec >= 1000000000) return 0;
  return int64_t(t.sec) * 1000000000 + t.nanosec;
}
template<class T> RegionFrameKey key(const T& message) {
  if (message.header.frame_id.size() > 256)
    return {0, "invalid_oversized_frame_id", 0, 0};
  return {source_ns(message.header.stamp), message.header.frame_id, message.width, message.height};
}
void text_valid(const std::string& value) {
  if (value.empty() || value.size() > 256 || value.find('\0') != std::string::npos)
    throw std::invalid_argument("invalid_region_identifier");
}
}
std::string region_calibration_contract(const sensor_msgs::msg::CameraInfo& info) {
  if (!region_key_valid({1, info.header.frame_id, info.width, info.height}) ||
      info.d.size() > 14 || info.distortion_model.size() > 64)
    throw std::invalid_argument("invalid_region_calibration_layout");
  RegionCanonical c;
  c.add("region-camera-v1").add(info.header.frame_id)
      .add(std::to_string(info.width)).add(std::to_string(info.height))
      .add(info.distortion_model).add(std::to_string(info.d.size()));
  for (double v : info.d) c.number(v);
  for (double v : info.k) c.number(v);
  for (double v : info.r) c.number(v);
  for (double v : info.p) c.number(v);
  c.add(std::to_string(info.binning_x)).add(std::to_string(info.binning_y));
  c.add(std::to_string(info.roi.x_offset)).add(std::to_string(info.roi.y_offset))
      .add(std::to_string(info.roi.width)).add(std::to_string(info.roi.height))
      .add(info.roi.do_rectify ? "true" : "false");
  return c.str();
}
std::string region_producer_contract(const RegionProducerOptions& o) {
  for (const auto* s : {&o.region_id, &o.base_frame, &o.optical_frame, &o.depth_topic, &o.info_topic, &o.diagnostics_topic})
    text_valid(*s);
  if (o.base_frame == o.optical_frame || o.depth_topic == o.info_topic ||
      o.diagnostics_topic == o.depth_topic || o.diagnostics_topic == o.info_topic ||
      o.expected_calibration.empty() || o.expected_calibration.size() > 8192 ||
      !std::isfinite(o.clear_margin_m) || o.clear_margin_m < 0. ||
      !std::isfinite(o.depth_scale_m) || o.depth_scale_m <= 0. ||
      !std::isfinite(o.minimum_depth_m) || o.minimum_depth_m < 0. ||
      !std::isfinite(o.maximum_depth_m) || o.maximum_depth_m <= o.minimum_depth_m ||
      !std::isfinite(o.max_age_sec) || o.max_age_sec <= 0. || o.max_age_sec > .3 ||
      !std::isfinite(o.tf_wait_sec) || o.tf_wait_sec < 0. || o.tf_wait_sec > .02)
    throw std::invalid_argument("invalid_region_profile");
  if (o.shadow_only_enabled && (o.accepted.rectified_pinhole_verified ||
      o.accepted.metric_axial_depth_verified || o.accepted.depth_intrinsics_registration_verified ||
      o.accepted.optical_to_base_extrinsics_verified || o.accepted.source_time_and_transform_verified))
    throw std::invalid_argument("region_shadow_requires_all_contract_flags_false");
  RegionCanonical c;
  c.add("region-producer-v1").add("region-depth-core-v1")
      .add(o.region_id).add(o.base_frame).add(o.optical_frame)
      .add(o.depth_topic).add(o.info_topic).add(o.diagnostics_topic).add("16UC1");
  for (std::size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(o.box_min[i]) || !std::isfinite(o.box_max[i]) || o.box_min[i] >= o.box_max[i])
      throw std::invalid_argument("invalid_region_box");
    c.number(o.box_min[i]).number(o.box_max[i]);
  }
  c.number(o.clear_margin_m).number(o.depth_scale_m).number(o.minimum_depth_m)
      .number(o.maximum_depth_m).number(o.max_age_sec);
  for (bool flag : {o.accepted.rectified_pinhole_verified, o.accepted.metric_axial_depth_verified,
      o.accepted.depth_intrinsics_registration_verified, o.accepted.optical_to_base_extrinsics_verified,
      o.accepted.source_time_and_transform_verified}) c.add(flag ? "true" : "false");
  c.add(o.expected_calibration).add("exact-source-info-tf;pending-unknown;info-cache=8;depth-pending=1");
  if (o.tf_wait_sec > 0.) c.add("bounded-exact-tf-wait-v1").number(o.tf_wait_sec);
  if (o.shadow_only_enabled) c.add("shadow-only-ray-diagnostics-v1");
  if (o.hold_complete_measurement_enabled) c.add("complete-measurement-hold-v1");
  if (o.typed_evaluation_diagnostics_enabled) c.add("typed-region-evaluation-diagnostics-v1");
  return c.str();
}
RegionDepthProducer::RegionDepthProducer(RegionProducerOptions options,
    rclcpp::Clock::SharedPtr clock, std::string node_name, Publish publish, Lookup lookup)
    : options_(std::move(options)), clock_(std::move(clock)), node_name_(std::move(node_name)),
      contract_(region_producer_contract(options_)), publish_(std::move(publish)), lookup_(std::move(lookup)) {
  if (contract_ != options_.expected_contract)
    throw std::invalid_argument("region_contract_mismatch");
  if (!lookup_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_, tf2::durationFromSec(2.));
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  }
}
void RegionDepthProducer::prepared(Image::ConstSharedPtr message, Info::ConstSharedPtr camera,
    const float* raw_depth_m, std::size_t elements, uint64_t sequence,
    int64_t receipt_ros_ns, int64_t receipt_steady_ns) {
  if (!lookup_ || !message || !camera || !raw_depth_m ||
      elements != std::size_t(message->width)*message->height || !sequence ||
      !(key(*message) == key(*camera)))
    throw std::invalid_argument("invalid_prepared_region_input");
  // Pairing/calibration contradictions still pass through the original gates.
  // Its clock guard sees actual clock reads, never a delayed receipt as 'now'.
  info(std::move(camera));
  prepared_depth_ = raw_depth_m; prepared_elements_ = elements;
  try { begin_image(std::move(message), sequence, receipt_ros_ns, receipt_steady_ns); }
  catch (...) { prepared_depth_ = nullptr; prepared_elements_ = 0; throw; }
  prepared_depth_ = nullptr; prepared_elements_ = 0;
}
void RegionDepthProducer::image(Image::ConstSharedPtr message) {
  begin_image(std::move(message), ++frame_sequence_, clock_->now().nanoseconds(), steady_now());
}
void RegionDepthProducer::begin_image(Image::ConstSharedPtr message, uint64_t sequence,
    int64_t receipt_ros_ns, int64_t receipt_steady_ns) {
  paired_stamp_ = tf_stamp_ = work_wall_ns_ = work_cpu_ns_ = 0;
  tf_diagnostics_ = {}; reset_shadow();
  Pairing::Pending next{key(*message), message, sequence, receipt_ros_ns, receipt_steady_ns};
  current_ = next; current_.image.reset();  // Metadata, never a second retained image.
  if (options_.hold_complete_measurement_enabled &&
      !report_clock_.observe(lookup_ ? clock_->now().nanoseconds() : current_.receipt_ros_ns,
                             lookup_ ? steady_now() : current_.receipt_steady_ns)) {
    pairs_.clear(); report("region_report_clock_rollback_or_invalid"); return;
  }
  if (!pairs_.image(std::move(next))) { report("invalid_or_nonmonotonic_image"); return; }
  const auto& k = current_.key;
  const uint64_t bytes = uint64_t(message->step) * message->height;
  if (message->encoding != "16UC1" || k.frame != options_.optical_frame ||
      message->step < uint64_t(k.width) * 2 || bytes > 16777216 || message->data.size() != bytes) {
    pairs_.clear(); report("invalid_region_image_layout"); return;
  }
  if (!region_times_fresh(k.stamp_ns, current_.receipt_ros_ns, current_.receipt_steady_ns,
                          clock_->now().nanoseconds(), steady_now(), options_.max_age_sec)) {
    pairs_.clear(); report("region_image_stale_or_clock_invalid"); return;
  }
  if (!pairs_.match(k)) {
    // No complete measurement yet. Do not publish the old result again: the
    // consumer retains its ORIGINAL source/receipt deadlines and expires it.
    if (!options_.hold_complete_measurement_enabled) report("pending_exact_info");
    return;
  }
  evaluate();
}
void RegionDepthProducer::info(Info::ConstSharedPtr message) {
  const auto k = key(*message);
  if (options_.hold_complete_measurement_enabled &&
      !report_clock_.observe(clock_->now().nanoseconds(), steady_now())) {
    pairs_.clear(); paired_stamp_ = tf_stamp_ = work_wall_ns_ = work_cpu_ns_ = 0;
    tf_diagnostics_ = {}; reset_shadow();
    report("region_report_clock_rollback_or_invalid"); return;
  }
  // A newer incomplete image must not hide a contradiction to the measurement
  // still held by the consumer. Revoke using that measurement's original identity.
  const bool held = options_.hold_complete_measurement_enabled &&
      last_reported_.sequence != 0 && k.stamp_ns == last_reported_.key.stamp_ns;
  const auto revoke_held = [this]() {
    current_ = last_reported_; pairs_.clear();
    paired_stamp_ = tf_stamp_ = work_wall_ns_ = work_cpu_ns_ = 0;
    tf_diagnostics_ = {}; reset_shadow();
  };
  std::string canonical;
  try { canonical = region_calibration_contract(*message); }
  catch (const std::invalid_argument&) {
    if (held) revoke_held();
    if (k.stamp_ns == current_.key.stamp_ns) {
      pairs_.clear(); paired_stamp_ = tf_stamp_ = work_wall_ns_ = work_cpu_ns_ = 0;
      tf_diagnostics_ = {}; reset_shadow();
      report("invalid_matching_camera_info");
    }
    return;
  }
  if (held && (!(k == last_reported_.key) || canonical != options_.expected_calibration)) {
    revoke_held(); report("conflicting_held_camera_info"); return;
  }
  if (k.stamp_ns == current_.key.stamp_ns &&
      (!(k == current_.key) || canonical != options_.expected_calibration)) {
    pairs_.info(k, canonical, message);
    pairs_.clear(); paired_stamp_ = tf_stamp_ = work_wall_ns_ = work_cpu_ns_ = 0;
    tf_diagnostics_ = {}; reset_shadow();
    report("camera_calibration_contract_mismatch"); return;
  }
  if (!pairs_.info(k, std::move(canonical), message)) return;
  if (!(k == current_.key)) return;
  const auto* entry = pairs_.match(k);
  if (entry && entry->conflict) {
    pairs_.clear(); paired_stamp_ = tf_stamp_ = work_wall_ns_ = work_cpu_ns_ = 0;
    tf_diagnostics_ = {}; reset_shadow();
    report("conflicting_exact_camera_info"); return;
  }
  if (pairs_.pending()) evaluate();
}
void RegionDepthProducer::evaluate() {
  auto pending = pairs_.pending();
  if (!pending) return;
  pairs_.clear();  // Each exact pair is evaluated at most once; no TF retry/hold.
  const auto* entry = pairs_.match(pending->key);
  if (!entry || entry->conflict) { report("conflicting_exact_camera_info"); return; }
  paired_stamp_ = entry->key.stamp_ns;
  if (entry->canonical != options_.expected_calibration) { report("camera_calibration_contract_mismatch"); return; }
  const auto& info = *entry->info;
  // The configured attestation cannot turn a non-pinhole payload into pinhole.
  bool pinhole = (info.distortion_model == "plumb_bob" || info.distortion_model == "rational_polynomial") &&
      info.binning_x <= 1 && info.binning_y <= 1 && !info.roi.do_rectify &&
      info.roi.x_offset == 0 && info.roi.y_offset == 0 && info.roi.width == 0 && info.roi.height == 0;
  for (double d : info.d) pinhole = pinhole && d == 0.;
  for (std::size_t i = 0; i < 9; ++i) pinhole = pinhole && info.r[i] == (i % 4 == 0 ? 1. : 0.);
  pinhole = pinhole && info.k[1] == 0. && info.k[3] == 0. && info.k[6] == 0. &&
      info.k[7] == 0. && info.k[8] == 1.;
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col)
      pinhole = pinhole && info.p[row * 4 + col] == info.k[row * 3 + col];
    pinhole = pinhole && info.p[row * 4 + 3] == 0.;
  }
  if (!pinhole) { report("camera_not_verified_pinhole_layout"); return; }
  const auto fresh = [this]() { return region_times_fresh(current_.key.stamp_ns,
      current_.receipt_ros_ns, current_.receipt_steady_ns, clock_->now().nanoseconds(),
      steady_now(), options_.max_age_sec); };
  if (!fresh()) { report("region_pair_stale_or_clock_invalid"); return; }
  const auto wall = steady_now(), cpu = cpu_now();
  const auto finish_work = [this, wall, cpu]() {
    work_wall_ns_ = steady_now() - wall;
    const auto cpu_end = cpu_now(); work_cpu_ns_ = cpu >= 0 && cpu_end >= cpu ? cpu_end - cpu : -1;
  };
  sunhorizon_obstacle_coverage::RegionDepthSupport result;
  int64_t lookup_start = 0;
  try {
    tf_stamp_ = current_.key.stamp_ns;
    lookup_start = steady_now();
    const auto tf = lookup_ ? lookup_(options_.base_frame, options_.optical_frame,
        current_.key.stamp_ns, options_.tf_wait_sec) : tf_buffer_->lookupTransform(
        options_.base_frame, options_.optical_frame,
        rclcpp::Time(current_.key.stamp_ns, clock_->get_clock_type()), rclcpp::Duration::from_seconds(options_.tf_wait_sec));
    if (lookup_ && (tf.header.frame_id != options_.base_frame || tf.child_frame_id != options_.optical_frame ||
        source_ns(tf.header.stamp) != current_.key.stamp_ns))
      throw std::invalid_argument("region_exact_tf_identity_mismatch");
    tf_diagnostics_.finish(lookup_start, steady_now());
    if (!fresh()) {
      finish_work(); reset_shadow(); report("region_tf_became_stale_or_clock_invalid"); return;
    }
    const auto& q = tf.transform.rotation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(norm) || std::abs(norm - 1.) > 1.e-6)
      throw std::invalid_argument("invalid_region_tf_quaternion");
    const tf2::Matrix3x3 rotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
    const auto& image = *pending->image;
    std::vector<float> raster;
    if (!prepared_depth_) raster.resize(std::size_t(image.width) * image.height);
    if (!prepared_depth_) for (uint32_t v = 0; v < image.height; ++v) for (uint32_t u = 0; u < image.width; ++u) {
      const auto offset = std::size_t(v) * image.step + std::size_t(u) * 2;
      const uint16_t raw = image.is_bigendian ? (uint16_t(image.data[offset]) << 8) | image.data[offset+1] :
          uint16_t(image.data[offset]) | (uint16_t(image.data[offset+1]) << 8);
      raster[std::size_t(v) * image.width + u] = static_cast<float>(raw * options_.depth_scale_m);
    }
    sunhorizon_obstacle_coverage::RegionDepthInput input;
    input.box_min = options_.box_min; input.box_max = options_.box_max;
    input.width = image.width; input.height = image.height;
    input.fx = info.k[0]; input.fy = info.k[4]; input.cx = info.k[2]; input.cy = info.k[5];
    input.depth_m = prepared_depth_ ? prepared_depth_ : raster.data();
    input.depth_elements = prepared_depth_ ? prepared_elements_ : raster.size();
    input.row_stride_elements = image.width;
    for (std::size_t i = 0; i < 9; ++i) input.optical_to_base_rotation[i] = rotation[i/3][i%3];
    input.optical_to_base_translation = {tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z};
    input.minimum_depth_m = options_.minimum_depth_m; input.maximum_depth_m = options_.maximum_depth_m;
    input.clear_margin_m = options_.clear_margin_m; input.contract = options_.accepted;
    if (options_.shadow_only_enabled) {
      // Explicit conditional geometry assumption on this local input only. Never
      // alters the pinned Options, production attestations or official support.
      input.contract = {true, true, true, true, true};
      shadow_support_ = sunhorizon_obstacle_coverage::evaluate_region_depth_support(input);
      shadow_evaluated_ = true;
      result.reason = "unverified_input_contract";
    } else {
      result = sunhorizon_obstacle_coverage::evaluate_region_depth_support(input);
    }
  } catch (const tf2::TransformException& error) {
    reset_shadow();
    tf_diagnostics_.failure(error, lookup_start, steady_now());
    result.reason = "exact_source_tf_unavailable";
  }
    catch (const std::invalid_argument& e) { reset_shadow(); result.reason = e.what(); }
  finish_work();
  if (!fresh()) { reset_shadow(); report("region_became_stale_or_clock_invalid"); return; }
  report(result.reason, result);
}
void RegionDepthProducer::report(const std::string& reason,
    sunhorizon_obstacle_coverage::RegionDepthSupport support) {
  using State = sunhorizon_obstacle_coverage::RegionSupportState;
  const auto evaluated_ros = clock_->now().nanoseconds(), evaluated_steady = steady_now();
  std::string actual_reason = reason;
  if (!report_clock_.observe(evaluated_ros, evaluated_steady)) {
    support = {}; reset_shadow(); actual_reason = "region_report_clock_rollback_or_invalid";
  } else if ((support.state != State::kUnknown ||
      (options_.typed_evaluation_diagnostics_enabled && support.evaluation_complete)) && !region_times_fresh(current_.key.stamp_ns,
      current_.receipt_ros_ns, current_.receipt_steady_ns, evaluated_ros, evaluated_steady, options_.max_age_sec)) {
    support = {}; actual_reason = "region_stale_at_report";
  }
  if (options_.shadow_only_enabled) {
    support = {};  // Defense in depth: this mode can never authorize production Clear.
    if (shadow_evaluated_ && !region_times_fresh(current_.key.stamp_ns,
        current_.receipt_ros_ns, current_.receipt_steady_ns, evaluated_ros, evaluated_steady, options_.max_age_sec)) {
      reset_shadow(); actual_reason = "region_stale_at_report";
    }
  }
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = rclcpp::Time(evaluated_ros, clock_->get_clock_type());
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = node_name_ + "/region_support"; status.message = actual_reason;
  status.level = support.state == State::kClearSupported ? status.OK : status.WARN;
  const auto add = [&status](const std::string& k, const std::string& v) {
    diagnostic_msgs::msg::KeyValue item; item.key = k; item.value = v; status.values.push_back(std::move(item));
  };
  add("diagnostic_event", "region_depth_support"); add("schema_version", "1");
  add("contract_encoding", "length-prefixed-ascii-v1"); add("contract", contract_);
  add("region_id", options_.region_id); add("base_frame_id", options_.base_frame);
  add("image_frame_id", current_.key.frame);
  add("region_frame_sequence", std::to_string(current_.sequence));
  add("region_evaluation_sequence", std::to_string(++evaluation_sequence_));
  add("source_stamp_ns", std::to_string(current_.key.stamp_ns));
  add("image_receipt_ros_ns", std::to_string(current_.receipt_ros_ns));
  add("image_receipt_steady_ns", std::to_string(current_.receipt_steady_ns));
  add("evaluation_ros_ns", std::to_string(evaluated_ros));
  add("evaluation_steady_ns", std::to_string(evaluated_steady));
  add("paired_info_stamp_ns", std::to_string(paired_stamp_)); add("tf_requested_stamp_ns", std::to_string(tf_stamp_));
  add("tf_error_type", tf_diagnostics_.error_type);
  add("tf_error_detail", tf_diagnostics_.error_detail);
  add("tf_error_truncated", tf_diagnostics_.error_truncated ? "true" : "false");
  add("tf_lookup_wall_ns", std::to_string(tf_diagnostics_.lookup_wall_ns));
  add("state", support.state == State::kClearSupported ? "ClearSupported" :
      support.state == State::kObstacleSupported ? "ObstacleSupported" : "Unknown");
  add("reason", actual_reason);
  if (options_.typed_evaluation_diagnostics_enabled) {
    add("evaluation_complete", support.evaluation_complete ? "true" : "false");
    add("evaluation_code", std::to_string(static_cast<int>(support.evaluation_code)));
    add("disposition", std::to_string(static_cast<int>(support.disposition)));
  }
  add("tested_pixels", std::to_string(support.tested_pixels));
  add("intersecting_rays", std::to_string(support.intersecting_rays));
  add("clear_rays", std::to_string(support.clear_rays));
  add("obstacle_rays", std::to_string(support.obstacle_rays));
  add("invalid_depth_rays", std::to_string(support.invalid_depth_rays));
  add("front_occluded_rays", std::to_string(support.front_occluded_rays));
  add("margin_band_rays", std::to_string(support.margin_band_rays));
  if (options_.shadow_only_enabled) {
    add("shadow_evaluated", shadow_evaluated_ ? "true" : "false");
    add("shadow_state", shadow_support_.state == State::kClearSupported ? "ClearSupported" :
        shadow_support_.state == State::kObstacleSupported ? "ObstacleSupported" : "Unknown");
    add("shadow_reason", shadow_evaluated_ ? shadow_support_.reason : actual_reason);
    add("shadow_tested_pixels", std::to_string(shadow_support_.tested_pixels));
    add("shadow_intersecting_rays", std::to_string(shadow_support_.intersecting_rays));
    add("shadow_clear_rays", std::to_string(shadow_support_.clear_rays));
    add("shadow_obstacle_rays", std::to_string(shadow_support_.obstacle_rays));
    add("shadow_invalid_depth_rays", std::to_string(shadow_support_.invalid_depth_rays));
    add("shadow_front_occluded_rays", std::to_string(shadow_support_.front_occluded_rays));
    add("shadow_margin_band_rays", std::to_string(shadow_support_.margin_band_rays));
  }
  add("region_wall_ns", std::to_string(work_wall_ns_)); add("region_thread_cpu_ns", std::to_string(work_cpu_ns_));
  last_reported_ = current_;
  array.status.push_back(std::move(status)); publish_(array);
}
}  // namespace sunhorizon_rgbd_adapter
