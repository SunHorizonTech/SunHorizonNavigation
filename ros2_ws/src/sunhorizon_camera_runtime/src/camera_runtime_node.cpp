#include <libobsensor/ObSensor.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "sunhorizon_camera_runtime/ae_target_transaction.hpp"
#include "sunhorizon_camera_runtime/direct_frame.hpp"
#include "sunhorizon_camera_runtime/runtime_contract.hpp"
#include "sunhorizon_camera_runtime/callback_timing.hpp"
#include "sunhorizon_camera_runtime/capture_thread_affinity.hpp"
#include "sunhorizon_camera_runtime/context_startup.hpp"
#include "sunhorizon_camera_runtime/latest_frame_worker.hpp"
#include "sunhorizon_camera_runtime/stereo_depth_quality.hpp"
#include "sunhorizon_camera_runtime/observer_stride.hpp"
#include "sunhorizon_camera_runtime/optional_shadow.hpp"
#include "sunhorizon_camera_runtime/exact_stereo_pairing.hpp"
#include "sunhorizon_camera_runtime/stereo_ir_observation.hpp"

namespace sunhorizon_camera_runtime
{
namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

constexpr char kChestColorTopic[] = "/sunhorizon/raw/rgbd/chest/color/image";
constexpr char kChestColorInfoTopic[] = "/sunhorizon/raw/rgbd/chest/color/camera_info";
constexpr char kChestDepthTopic[] = "/sunhorizon/raw/rgbd/chest/aligned_depth/image";
constexpr char kChestDepthInfoTopic[] =
  "/sunhorizon/raw/rgbd/chest/aligned_depth/camera_info";
constexpr char kBellyDepthTopic[] = "/sunhorizon/raw/rgbd/belly/depth/image";
constexpr char kBellyDepthInfoTopic[] = "/sunhorizon/raw/rgbd/belly/depth/camera_info";
constexpr char kBellyLeftIrTopic[] = "/sunhorizon/candidate/rgbd/belly/ir_left/image";
constexpr char kBellyRightIrTopic[] = "/sunhorizon/candidate/rgbd/belly/ir_right/image";
constexpr char kBellyIrObservationTopic[] =
  "/sunhorizon/candidate/rgbd/belly/stereo_observation/status";
constexpr char kDiagnosticsTopic[] = "/sunhorizon/raw/rgbd/status";
constexpr char kChestLinkFrame[] = "camera_chest_link";
constexpr char kChestOpticalFrame[] = "camera_chest_color_optical_frame";
constexpr char kBellyLinkFrame[] = "camera_belly_link";
constexpr char kBellyOpticalFrame[] = "camera_belly_depth_optical_frame";

constexpr FrameContract kColorContract{640U, 480U, 30U, PixelContract::kRgb8};
constexpr FrameContract kDepthContract{640U, 480U, 30U, PixelContract::kDepth16};
constexpr std::uint64_t kMicrosPerSecond = 1000000ULL;
constexpr std::uint64_t kChestFramePeriodUs = kMicrosPerSecond / kColorContract.fps;

builtin_interfaces::msg::Time ros_time_from_host_clock_us(std::uint64_t timestamp_us)
{
  const auto seconds = timestamp_us / kMicrosPerSecond;
  if (seconds > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max())) {
    throw std::range_error("SDK system timestamp exceeds ROS 2 time range");
  }
  builtin_interfaces::msg::Time output;
  output.sec = static_cast<std::int32_t>(seconds);
  output.nanosec = static_cast<std::uint32_t>(timestamp_us % kMicrosPerSecond) * 1000U;
  return output;
}

KeyValue key_value(std::string key, std::string value)
{
  KeyValue output;
  output.key = std::move(key);
  output.value = std::move(value);
  return output;
}

OBFormat expected_sdk_format(PixelContract pixels)
{
  switch (pixels) {
    case PixelContract::kRgb8:
      return OB_FORMAT_RGB;
    case PixelContract::kDepth16:
      return OB_FORMAT_Y16;
  }
  throw std::logic_error("unsupported pixel contract");
}

std::shared_ptr<ob::VideoStreamProfile> select_exact_profile(
  const std::shared_ptr<ob::StreamProfileList> & profiles,
  const FrameContract & expected,
  const std::string & role_and_stream)
{
  std::shared_ptr<ob::VideoStreamProfile> selected;
  for (std::uint32_t index = 0; index < profiles->getCount(); ++index) {
    auto candidate = profiles->getProfile(index)->as<ob::VideoStreamProfile>();
    if (candidate->getWidth() == expected.width &&
      candidate->getHeight() == expected.height &&
      candidate->getFps() == expected.fps &&
      candidate->getFormat() == expected_sdk_format(expected.pixels))
    {
      selected = std::move(candidate);
      break;
    }
  }
  if (!selected) {
    throw std::runtime_error(
            role_and_stream + " has no exact " + std::to_string(expected.width) + "x" +
            std::to_string(expected.height) + "@" + std::to_string(expected.fps) +
            " required profile");
  }
  return selected;
}

CameraInfo make_camera_info(
  const std::shared_ptr<ob::VideoStreamProfile> & profile,
  const FrameContract & expected,
  const std::string & frame_id)
{
  const auto intrinsic = profile->getIntrinsic();
  if (intrinsic.width != static_cast<std::int16_t>(expected.width) ||
    intrinsic.height != static_cast<std::int16_t>(expected.height) ||
    !std::isfinite(intrinsic.fx) || !std::isfinite(intrinsic.fy) ||
    !std::isfinite(intrinsic.cx) || !std::isfinite(intrinsic.cy) ||
    intrinsic.fx <= 0.0F || intrinsic.fy <= 0.0F)
  {
    throw std::runtime_error("invalid camera intrinsic for selected profile");
  }

  const auto distortion = profile->getDistortion();
  CameraInfo output;
  output.header.frame_id = frame_id;
  output.width = expected.width;
  output.height = expected.height;
  switch (distortion.model) {
    case OB_DISTORTION_NONE:
      output.distortion_model = "plumb_bob";
      output.d.assign(5U, 0.0);
      break;
    case OB_DISTORTION_BROWN_CONRADY:
      output.distortion_model = "plumb_bob";
      output.d = {distortion.k1, distortion.k2, distortion.p1, distortion.p2,
        distortion.k3};
      break;
    case OB_DISTORTION_BROWN_CONRADY_K6:
      output.distortion_model = "rational_polynomial";
      output.d = {distortion.k1, distortion.k2, distortion.p1, distortion.p2,
        distortion.k3, distortion.k4, distortion.k5, distortion.k6};
      break;
    case OB_DISTORTION_KANNALA_BRANDT4:
      output.distortion_model = "equidistant";
      output.d = {distortion.k1, distortion.k2, distortion.k3, distortion.k4};
      break;
    case OB_DISTORTION_MODIFIED_BROWN_CONRADY:
    case OB_DISTORTION_INVERSE_BROWN_CONRADY:
    default:
      throw std::runtime_error("SDK distortion model cannot be represented safely in CameraInfo");
  }
  output.k = {
    intrinsic.fx, 0.0, intrinsic.cx,
    0.0, intrinsic.fy, intrinsic.cy,
    0.0, 0.0, 1.0};
  output.r = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  output.p = {
    intrinsic.fx, 0.0, intrinsic.cx, 0.0,
    0.0, intrinsic.fy, intrinsic.cy, 0.0,
    0.0, 0.0, 1.0, 0.0};
  return output;
}

class PipelineOwner
{
public:
  explicit PipelineOwner(std::shared_ptr<ob::Pipeline> pipeline)
  : pipeline_(std::move(pipeline)) {}

  ~PipelineOwner() noexcept
  {
    stop_noexcept();
  }

  PipelineOwner(const PipelineOwner &) = delete;
  PipelineOwner & operator=(const PipelineOwner &) = delete;

  std::shared_ptr<ob::Pipeline> pipeline() const
  {
    return pipeline_;
  }

  void mark_start_attempted()
  {
    start_attempt_.mark();
  }

  void stop_noexcept() noexcept
  {
    if (!start_attempt_.take_stop()) {
      return;
    }
    try {
      pipeline_->stop();
    } catch (const ob::Error & error) {
      std::fprintf(stderr, "sunhorizon camera stop error: %s\n", error.what());
    } catch (const std::exception & error) {
      std::fprintf(stderr, "sunhorizon camera stop error: %s\n", error.what());
    } catch (...) {
      std::fprintf(stderr, "sunhorizon camera stop error: unknown exception\n");
    }
  }

private:
  std::shared_ptr<ob::Pipeline> pipeline_;
  CaptureStartAttempt start_attempt_;
};

struct RoleState
{
  std::string role;
  std::string serial;
  std::shared_ptr<const CameraSourceIdentity> direct_identity;
  bool enabled{false};
  bool started{false};
  std::uint64_t accepted_frames{0U};
  std::uint64_t rejected_frames{0U};
  std::uint64_t startup_system_clock_reanchors{0U};
  std::uint64_t startup_global_pair_drops{0U};
  std::uint64_t last_color_device_timestamp_us{0U};
  std::uint64_t last_color_global_timestamp_us{0U};
  std::uint64_t last_color_sdk_system_timestamp_us{0U};
  std::uint64_t last_depth_device_timestamp_us{0U};
  std::uint64_t last_depth_global_timestamp_us{0U};
  std::uint64_t last_depth_sdk_system_timestamp_us{0U};
  std::atomic<std::int64_t> last_host_receive_timestamp_ns{0};
  std::atomic<std::int64_t> first_frame_arrival_steady_ns{0};
  std::atomic<std::int64_t> last_frame_arrival_steady_ns{0};
  std::atomic<std::int64_t> last_publish_complete_steady_ns{0};
  std::atomic<std::uint32_t> callbacks_in_flight{0U};
  std::atomic<std::uint32_t> workers_in_flight{0U};
  double last_depth_value_scale_mm{0.0};
  std::string last_reason{"not_started"};
  std::string last_startup_transient_reason{"none"};
  CompletedCallbackTiming last_completed_timing;
  mutable std::mutex mutex;
};

std::int64_t steady_now_ns() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::int64_t system_now_ns() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

CallbackClockPoint callback_clock_now() noexcept
{
  const auto steady_ns = steady_now_ns();
  timespec cpu{};
  const auto cpu_ns = ::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu) == 0 ?
    static_cast<std::int64_t>(cpu.tv_sec) * 1000000000LL + cpu.tv_nsec : -1;
  return {steady_ns, cpu_ns};
}

CallbackFrameTiming callback_entry_now() noexcept
{
  CallbackFrameTiming timing;
  timing.entry_system_ns = system_now_ns();
  timing.entry = callback_clock_now();
  return timing;
}

double age_seconds(std::int64_t now_ns, std::int64_t timestamp_ns)
{
  return timestamp_ns <= 0 ? std::numeric_limits<double>::infinity() :
         std::max(0.0, static_cast<double>(now_ns - timestamp_ns) / 1.0e9);
}

class CallbackActivityGuard
{
public:
  CallbackActivityGuard(RoleState & state, std::int64_t host_receive_ns)
  : state_(state)
  {
    const auto arrival_steady_ns = steady_now_ns();
    state_.last_host_receive_timestamp_ns.store(
      host_receive_ns, std::memory_order_release);
    std::int64_t unseen = 0;
    state_.first_frame_arrival_steady_ns.compare_exchange_strong(
      unseen, arrival_steady_ns, std::memory_order_acq_rel);
    state_.last_frame_arrival_steady_ns.store(
      arrival_steady_ns, std::memory_order_release);
    state_.callbacks_in_flight.fetch_add(1U, std::memory_order_acq_rel);
  }

  ~CallbackActivityGuard()
  {
    state_.callbacks_in_flight.fetch_sub(1U, std::memory_order_acq_rel);
  }

  CallbackActivityGuard(const CallbackActivityGuard &) = delete;
  CallbackActivityGuard & operator=(const CallbackActivityGuard &) = delete;

private:
  RoleState & state_;
};

class CameraRuntimeNode final : public rclcpp::Node
{
public:
  CameraRuntimeNode(const rclcpp::NodeOptions & options, CameraFrameSink frame_sink,
    CameraFaultSink fault_sink)
  : Node("sunhorizon_camera_runtime", options), system_clock_(RCL_SYSTEM_TIME),
    frame_sink_(std::move(frame_sink)), fault_sink_(std::move(fault_sink))
  {
    rcl_interfaces::msg::ParameterDescriptor affinity_descriptor;
    affinity_descriptor.read_only=true;
    affinity_descriptor.description="Camera startup descendants only; empty inherits; Robot A ten-core mask excludes CPUs4/5";
    const auto capture_cpus=declare_parameter<std::vector<int64_t>>(
      "capture_thread_cpu_affinity",std::vector<int64_t>{},affinity_descriptor);
    capture_affinity::validate(capture_cpus);
    rcl_interfaces::msg::ParameterDescriptor network_enumeration_descriptor;
    network_enumeration_descriptor.read_only = true;
    network_enumeration_descriptor.description =
      "Enable SDK automatic network camera discovery (GVCP); USB discovery is independent";
    const auto network_device_enumeration_enabled = declare_parameter<bool>(
      "network_device_enumeration_enabled", false, network_enumeration_descriptor);
    if (frame_sink_ && !fault_sink_) {
      throw std::invalid_argument("direct camera delivery requires a fault sink");
    }
    raw_ros_publish_enabled_ = declare_parameter("raw_ros_publish_enabled", !bool(frame_sink_));
    if (!frame_sink_ && !raw_ros_publish_enabled_) {
      throw std::invalid_argument("camera runtime requires a direct sink or raw ROS publication");
    }
    chest_state_.role = "chest";
    chest_state_.enabled = declare_parameter("chest_enabled", true);
    chest_state_.serial = declare_parameter("chest_serial", "CPCAC53000NF");
    belly_state_.role = "belly";
    belly_state_.enabled = declare_parameter("belly_enabled", true);
    belly_state_.serial = declare_parameter("belly_serial", "CP9W463000ZW");
    rcl_interfaces::msg::ParameterDescriptor belly_fps_descriptor;
    belly_fps_descriptor.read_only = true;
    belly_fps_descriptor.description = "Exact belly depth and both IR capture FPS: 15 or 30; default30; no profile fallback";
    belly_contract_ = belly_capture_contract(declare_parameter<int64_t>(
      "belly_capture_fps", int64_t{30}, belly_fps_descriptor));
    head_state_.role = "head";
    head_state_.enabled = declare_parameter("head_enabled", false);
    head_state_.serial = declare_parameter("head_serial", "CP9W463000DY");
    startup_timeout_sec_ = declare_parameter("startup_timeout_sec", 3.0);
    frame_timeout_sec_ = declare_parameter("frame_timeout_sec", 1.0);
    publish_timeout_sec_ = declare_parameter("publish_timeout_sec", 2.0);
    timestamp_startup_grace_sec_ = declare_parameter(
      "timestamp_startup_grace_sec", 60.0);
    maximum_device_pair_delta_us_ = declare_parameter(
      "maximum_device_pair_delta_us", 5000);
    maximum_global_pair_delta_us_ = declare_parameter(
      "maximum_global_pair_delta_us", 20000);
    const auto legacy_maximum_pair_delta_us = declare_parameter(
      "maximum_pair_delta_us", -1);
    if (legacy_maximum_pair_delta_us > 0) {
      maximum_device_pair_delta_us_ = legacy_maximum_pair_delta_us;
      maximum_global_pair_delta_us_ = legacy_maximum_pair_delta_us;
      RCLCPP_WARN(
        get_logger(),
        "maximum_pair_delta_us is deprecated; its value is being applied to both "
        "maximum_device_pair_delta_us and maximum_global_pair_delta_us");
    } else if (legacy_maximum_pair_delta_us != -1) {
      throw std::invalid_argument(
              "deprecated maximum_pair_delta_us must be positive when configured");
    }
    diagnostics_period_ms_ = declare_parameter("diagnostics_period_ms", 250);
    belly_stereo_quality_shadow_ = declare_parameter("belly_stereo_quality_shadow", false);
    belly_stereo_ir_observation_ = declare_parameter("belly_stereo_ir_observation", false);
    // Independent observer on the authoritative pipeline; never a navigation input.
    belly_stereo_observation_v2_ = declare_parameter("belly_stereo_observation_v2", false);
    if(!belly_stereo_observation_stride_.set(declare_parameter("belly_stereo_observation_stride",6)))
      throw std::invalid_argument("invalid stereo observer stride");
    observer_stride_parameter_callback_=add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters){
        rcl_interfaces::msg::SetParametersResult result;result.successful=true;
        for(const auto& p:parameters)if(p.get_name()=="belly_stereo_observation_stride"&&parameters.size()!=1){
          result.successful=false;result.reason="observer stride must be updated alone";return result;}
        for(const auto& p:parameters)if(p.get_name()=="belly_stereo_observation_stride" &&
            (p.get_type()!=rclcpp::ParameterType::PARAMETER_INTEGER||!ObserverStride::valid(p.as_int()))){
          result.successful=false;result.reason="observer stride requires integer in [1,60]";return result;}
        for(const auto& p:parameters)if(p.get_name()=="belly_stereo_observation_stride")
          (void)belly_stereo_observation_stride_.set(p.as_int());
        return result;
      });
    if (belly_stereo_observation_stride_.get() < 1 || belly_stereo_observation_stride_.get() > 60 ||
      (belly_stereo_observation_v2_ &&
       (belly_stereo_quality_shadow_ || belly_stereo_ir_observation_))) {
      throw std::invalid_argument("invalid stereo observer stride or mixed legacy/v2 observers");
    }
    rcl_interfaces::msg::ParameterDescriptor ae_descriptor;ae_descriptor.read_only = true;
    belly_ir_ae_target_override_ = declare_parameter("belly_ir_ae_target_override", -1, ae_descriptor);
    belly_ir_ae_target_journal_directory_ = declare_parameter<std::string>(
      "belly_ir_ae_target_journal_directory", "/data/navigation/camera-property-journals", ae_descriptor);
    // A configurable alternative path could hide an unresolved transaction on
    // the next default-off startup. Keep this recovery location invariant.
    if (belly_ir_ae_target_override_ < -2 ||
      belly_ir_ae_target_journal_directory_ != "/data/navigation/camera-property-journals" ||
      (belly_ir_ae_target_override_ != -1 && !belly_stereo_observation_v2_)) {
      throw std::invalid_argument("AE target override requires v2 evidence and the fixed production journal directory");
    }
    rcl_interfaces::msg::ParameterDescriptor dispatch_descriptor;dispatch_descriptor.read_only=true;
    capture_dispatch_mode_=declare_parameter<std::string>("capture_dispatch_mode",frame_sink_?"latest-worker-v1":"inline-v1",dispatch_descriptor);
    if (frame_sink_ && (capture_dispatch_mode_ != "latest-worker-v1" ||
      belly_stereo_quality_shadow_ || belly_stereo_ir_observation_)) {
      throw std::invalid_argument("direct camera delivery requires latest-worker-v1; "
        "stereo quality shadow/IR are incompatible and must be explicitly disabled");
    }
    if (!valid_capture_dispatch_mode(capture_dispatch_mode_,belly_stereo_quality_shadow_,belly_stereo_ir_observation_))
      throw std::invalid_argument("unsupported or incompatible capture_dispatch_mode");
    if (belly_stereo_ir_observation_ && !belly_stereo_quality_shadow_) {
      throw std::invalid_argument("belly_stereo_ir_observation requires belly_stereo_quality_shadow");
    }
    publish_internal_optical_tf_ = declare_parameter("publish_internal_optical_tf", false);
    internal_optical_tf_receipt_sha256_ = declare_parameter(
      "internal_optical_tf_acceptance_receipt_sha256", "");

    if (!chest_state_.enabled || !belly_state_.enabled) {
      throw std::invalid_argument("camera runtime v1 requires chest and belly cameras enabled");
    }
    if (head_state_.enabled) {
      throw std::invalid_argument("head camera is deliberately unsupported by camera runtime v1");
    }
    if (!valid_stream_timeouts(
        startup_timeout_sec_, frame_timeout_sec_, publish_timeout_sec_) ||
      !valid_timestamp_startup_grace(timestamp_startup_grace_sec_) ||
      !valid_capture_pair_thresholds(
        static_cast<std::uint64_t>(maximum_device_pair_delta_us_),
        static_cast<std::uint64_t>(maximum_global_pair_delta_us_),
        kChestFramePeriodUs) ||
      diagnostics_period_ms_ <= 0)
    {
      throw std::invalid_argument(
              "camera watchdog/startup-grace timeouts must be finite and positive, and "
              "publish timeout must not be shorter than frame timeout; camera pair "
              "thresholds must be positive, device <= global, and global < one frame period");
    }

    // Camera capture must never wait for a downstream reliable reader. Publish
    // the owned raw streams as latest-only best-effort sensor data; the
    // LIVO adapter selects the newest epochs and owns the private reliable
    // 10 Hz vendor boundary. This keeps DDS backpressure out of the Orbbec SDK
    // callback and therefore out of the hardware freshness watchdog.
    if (raw_ros_publish_enabled_) {
      auto chest_color_qos = rclcpp::QoS(rclcpp::KeepLast(1));
      chest_color_qos.best_effort().durability_volatile();
      chest_color_pub_ = create_publisher<Image>(kChestColorTopic, chest_color_qos);
      chest_color_info_pub_ =
        create_publisher<CameraInfo>(kChestColorInfoTopic, chest_color_qos);
      chest_depth_pub_ = create_publisher<Image>(kChestDepthTopic, rclcpp::SensorDataQoS());
      chest_depth_info_pub_ =
        create_publisher<CameraInfo>(kChestDepthInfoTopic, rclcpp::SensorDataQoS());
      belly_depth_pub_ = create_publisher<Image>(kBellyDepthTopic, rclcpp::SensorDataQoS());
      belly_depth_info_pub_ =
        create_publisher<CameraInfo>(kBellyDepthInfoTopic, rclcpp::SensorDataQoS());
      if (belly_stereo_quality_shadow_) {
        belly_quality_depth_pub_ = create_publisher<Image>(
          "/sunhorizon/candidate/rgbd/belly/quality_depth/image", rclcpp::SensorDataQoS());
        belly_quality_info_pub_ = create_publisher<CameraInfo>(
          "/sunhorizon/candidate/rgbd/belly/quality_depth/camera_info", rclcpp::SensorDataQoS());
      }
      if (belly_stereo_ir_observation_) {
        auto ir_qos = rclcpp::QoS(rclcpp::KeepLast(1));
        ir_qos.best_effort().durability_volatile();
        belly_left_ir_pub_ = create_publisher<Image>(kBellyLeftIrTopic, ir_qos);
        belly_right_ir_pub_ = create_publisher<Image>(kBellyRightIrTopic, ir_qos);
        belly_ir_observation_pub_ = create_publisher<DiagnosticArray>(kBellyIrObservationTopic, ir_qos);
      }
    }
    if (belly_stereo_observation_v2_) {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
      qos.best_effort().durability_volatile();
      belly_quality_depth_pub_ = create_publisher<Image>(
        "/sunhorizon/candidate/rgbd/belly/quality_depth/image", qos);
      belly_quality_info_pub_ = create_publisher<CameraInfo>(
        "/sunhorizon/candidate/rgbd/belly/quality_depth/camera_info", qos);
      belly_left_ir_pub_ = create_publisher<Image>(kBellyLeftIrTopic, qos);
      belly_right_ir_pub_ = create_publisher<Image>(kBellyRightIrTopic, qos);
      belly_ir_observation_pub_ = create_publisher<DiagnosticArray>(kBellyIrObservationTopic, qos);
      belly_observed_depth_pub_ = create_publisher<Image>(
        "/sunhorizon/candidate/rgbd/belly/stereo_observation/depth/image", qos);
      belly_native_depth_pub_ = create_publisher<Image>(
        "/sunhorizon/candidate/rgbd/belly/stereo_observation/depth_native/image", qos);
      belly_observed_info_pub_ = create_publisher<CameraInfo>(
        "/sunhorizon/candidate/rgbd/belly/stereo_observation/depth/camera_info", qos);
      belly_quality_reason_pub_ = create_publisher<Image>(
        "/sunhorizon/candidate/rgbd/belly/stereo_observation/quality_reason", qos);
    }
    diagnostics_pub_ = create_publisher<DiagnosticArray>(kDiagnosticsTopic, rclcpp::QoS(10));

    configure_internal_optical_tf();

    verify_sdk_version();
    try {
      capture_affinity::with_affinity(capture_cpus,capture_affinity::NativeOps{},
        [this, network_device_enumeration_enabled] {
        context_ = create_context_with_network_enumeration(
          network_device_enumeration_enabled, [] {return std::make_shared<ob::Context>();});
        // SDK callbacks may run synchronously from Pipeline::start(). Establish
        // the cold-start epoch before either pipeline can invoke one.
        startup_steady_timestamp_ns_ = steady_now_ns();
        try {
          start_workers();
          bind_and_start();
          diagnostics_timer_ = create_wall_timer(
            std::chrono::milliseconds(diagnostics_period_ms_),
            [this]() {on_health_timer();});
        } catch (...) {
          // Stop callbacks before scope restoration and member unwinding.
          stop_all_noexcept();
          throw;
        }
      });
    } catch (...) {
      // Also cover affinity apply/readback/restore failures after successful
      // pipeline startup: a throwing constructor has no derived destructor.
      stop_all_noexcept();
      throw;
    }
  }

  ~CameraRuntimeNode() noexcept override
  {
    stop_all_noexcept();
  }

  void stop() noexcept {stop_all_noexcept();}

  [[nodiscard]] bool failed() const noexcept
  {
    return fatal_latched_.load();
  }

private:
  struct Capture {
    std::shared_ptr<ob::FrameSet> frameset;
    std::shared_ptr<ob::ColorFrame> color;
    std::shared_ptr<ob::DepthFrame> depth;
    uint64_t color_device_us{0},color_global_us{0},color_system_us{0};
    uint64_t depth_device_us{0},depth_global_us{0},depth_system_us{0};
    double depth_scale_mm{0.};
    CallbackFrameTiming timing;
  };
  using CaptureWorker=LatestFrameWorker<Capture>;
  using RawWorker = LatestFrameWorker<CameraFrame::ConstSharedPtr>;
  struct ObservedSdkFrame {
    std::shared_ptr<ob::Frame> frame;
    std::int64_t callback_ros_ns{0}, callback_steady_ns{0};
  };
  using ObservationPairing = ExactStereoPairing<std::shared_ptr<ObservedSdkFrame>>;
  struct ObservationWork { ObservationPairing::Triple frames; };
  using ObservationWorker = LatestFrameWorker<ObservationWork>;
  void publish_raw(const CameraFrame & frame) {
    if (frame.role == "chest") {
      chest_color_pub_->publish(*frame.color);
      chest_color_info_pub_->publish(*frame.color_info);
      chest_depth_pub_->publish(*frame.depth);
      chest_depth_info_pub_->publish(*frame.depth_info);
    } else {
      belly_depth_pub_->publish(*frame.depth);
      belly_depth_info_pub_->publish(*frame.depth_info);
    }
  }
  void start_workers() {
    if (belly_stereo_observation_v2_) {
      belly_observation_worker_ = std::make_unique<ObservationWorker>(
        [this](ObservationWork & work) {
          ExactStereoPairing<std::shared_ptr<ob::Frame>>::Triple frames;
          for (std::size_t i = 0; i < frames.size(); ++i) frames[i] = work.frames[i]->frame;
          publish_belly_quality(frames, &work);
        }, [this] {return accepting_frames_.load() && !fatal_latched_.load();},
        [this](std::exception_ptr) {
          belly_quality_last_frame_valid_.store(false);
          ++belly_quality_failed_frames_;
          RCLCPP_ERROR(get_logger(), "optional stereo evidence worker failed; navigation unchanged");
        });
    }
    if (frame_sink_ && raw_ros_publish_enabled_) {
      const auto allowed=[this]{return accepting_frames_.load() && !fatal_latched_.load();};
      // DDS runs on separate, bounded observer workers; its queue and errors
      // cannot block or poison direct delivery/source freshness.
      const auto failure=[this](std::exception_ptr) {
        ++raw_publish_errors_;
        RCLCPP_ERROR(get_logger(), "optional raw ROS publisher worker failed");
      };
      chest_raw_worker_=std::make_unique<RawWorker>(
        [this](CameraFrame::ConstSharedPtr & f){publish_raw(*f);},allowed,failure);
      belly_raw_worker_=std::make_unique<RawWorker>(
        [this](CameraFrame::ConstSharedPtr & f){publish_raw(*f);},allowed,failure);
    }
    if (capture_dispatch_mode_!="latest-worker-v1") return;
    const auto allowed=[this]{return !fatal_latched_.load() && accepting_frames_.load();};
    const auto worker=[this](bool chest,Capture& frame) {
      auto& state=chest?chest_state_:belly_state_;
      struct Activity {std::atomic<uint32_t>& count;explicit Activity(std::atomic<uint32_t>& c):count(c){++count;}~Activity(){--count;}} active(state.workers_in_flight);
      frame.timing.worker_start_steady_ns=steady_now_ns();
      if (chest) convert_publish_chest(frame,frame.timing);
      else convert_publish_belly(frame,frame.timing);
    };
    const auto failure=[this](const char* role,std::exception_ptr error) {
      try {std::rethrow_exception(error);}
      catch (const ob::Error& e) {latch_callback_failure(role,"sdk_callback_error",e.what());}
      catch (const std::exception& e) {latch_callback_failure(role,"callback_error",e.what());}
      catch (...) {latch_callback_failure(role,"callback_error","unknown exception");}
    };
    chest_worker_=std::make_unique<CaptureWorker>([worker](Capture& f){worker(true,f);},allowed,
        [failure](std::exception_ptr e){failure("chest",e);});
    belly_worker_=std::make_unique<CaptureWorker>([worker](Capture& f){worker(false,f);},allowed,
        [failure](std::exception_ptr e){failure("belly",e);});
  }
  void dispatch_capture(bool chest,const std::shared_ptr<ob::FrameSet>& frameset,CallbackFrameTiming& timing) {
    std::unique_ptr<Capture> retired;
    {
      timing.lock_wait.begin=callback_clock_now();
      std::lock_guard<std::mutex> ingress(chest?chest_callback_mutex_:belly_callback_mutex_);
      timing.lock_wait.end=callback_clock_now();timing.ingress_validate.begin=timing.lock_wait.end;
      if (!accepting_frames_.load()) return;
      std::optional<Capture> frame=chest?validate_chest(frameset):std::optional<Capture>(validate_belly(frameset));
      if (frame) {
        frame->depth_scale_mm=frame->depth->getValueScale();
        if (!std::isfinite(frame->depth_scale_mm) || frame->depth_scale_mm<=0.)
          throw std::runtime_error("invalid depth value scale");
        (void)ros_time_from_host_clock_us(frame->depth_global_us);
        if (chest) (void)ros_time_from_host_clock_us(frame->color_global_us);
      }
      timing.ingress_validate.end=callback_clock_now();if (!frame) return;
      auto& sequence=chest?chest_ingress_sequence_:belly_ingress_sequence_;
      if (sequence==std::numeric_limits<uint64_t>::max()) throw std::runtime_error("camera ingress sequence overflow");
      timing.ingress_sequence=++sequence;timing.worker_dispatch=true;
      // SDK/worker CPU clocks must never be subtracted across threads.
      timing.validate_convert=CallbackTimingPhase{};
      auto item=std::make_unique<Capture>(std::move(*frame));
      timing.queue_enter_steady_ns=steady_now_ns();item->timing=timing;
      retired=(chest?chest_worker_:belly_worker_)->replace(std::move(item));
    }
    // A replaced FrameSet can release SDK buffers: outside both locks.
    retired.reset();
  }
  void deliver_direct(bool chest, Capture & capture, const CallbackFrameTiming & timing,
    Image depth, CameraInfo depth_info, Image color = Image{}, CameraInfo color_info = CameraInfo{})
  {
    auto frame = std::make_shared<CameraFrame>();
    frame->role = chest ? "chest" : "belly";
    frame->source_identity = (chest ? chest_state_ : belly_state_).direct_identity;
    frame->depth = std::make_shared<const Image>(std::move(depth));
    frame->depth_info = std::make_shared<const CameraInfo>(std::move(depth_info));
    if (chest) {
      frame->color = std::make_shared<const Image>(std::move(color));
      frame->color_info = std::make_shared<const CameraInfo>(std::move(color_info));
    }
    frame->source_stamp = frame->depth->header.stamp;
    frame->sdk_depth_scale_m = capture.depth_scale_mm * 0.001;
    frame->first_sdk_callback_ros_ns = timing.entry_ros_ns;
    frame->first_sdk_callback_steady_ns = timing.entry.steady_ns;
    frame->sequence = timing.ingress_sequence;
    frame->depth_time = {capture.depth_device_us, capture.depth_global_us, capture.depth_system_us};
    frame->depth_frame_index=capture.depth->getIndex();
    frame->color_time = {capture.color_device_us, capture.color_global_us, capture.color_system_us};
    frame->source_validated = true;
    if(!chest)frame->measurement_quality=primary_depth_quality(capture,*frame);
    frame->timing = timing;
    CameraFrame::ConstSharedPtr immutable = std::move(frame);
    // Caller callback must only admit into its own bounded queue. Never run
    // fusion/filtering in this camera conversion worker.
    frame_sink_(immutable);
    if (raw_ros_publish_enabled_) {
      auto retired = (chest ? chest_raw_worker_ : belly_raw_worker_)->replace(
        std::make_unique<CameraFrame::ConstSharedPtr>(std::move(immutable)));
      retired.reset();
    }
  }
  DepthMeasurementQuality::ConstSharedPtr primary_depth_quality(const Capture& capture,const CameraFrame& frame) {
    auto q=std::make_shared<DepthMeasurementQuality>();q->role="belly";
    q->source_frame=frame.depth->header.frame_id;q->source_ns=capture.depth_global_us*1000;
    q->sequence=frame.sequence;q->image_identity=frame.depth.get();q->info_identity=frame.depth_info.get();
    q->opened_serial=frame.source_identity->opened_serial;q->capture_epoch=frame.source_identity->capture_epoch;
    q->width=frame.depth->width;q->height=frame.depth->height;q->sdk_depth_scale_m=frame.sdk_depth_scale_m;
    q->pixels.assign(q->width*q->height,uint8_t(DepthMeasurementState::Unassessed));
    // The original depth's geometric calibration is independent of whether
    // optional IR frames have reached their exact-pair mailbox yet. No waiting,
    // borrowing of adjacent images, or quality promotion follows this proof.
    try {
      const auto profile=capture.depth->getStreamProfile()->as<ob::VideoStreamProfile>();
      const auto k=profile->getIntrinsic();const auto distortion=profile->getDistortion();
      const std::array<double,9> sdk_k{{k.fx,0,k.cx,0,k.fy,k.cy,0,0,1}};
      q->geometry_depth_device_us=capture.depth->getTimeStampUs();
      q->geometry_depth_global_us=capture.depth->getGlobalTimeStampUs();
      q->geometry_depth_frame_index=capture.depth->getIndex();
      q->geometry_depth_scale_m=capture.depth->getValueScale()*.001;
      q->depth_pixel_type=static_cast<int>(capture.depth->getPixelType());
      q->stereo_baseline_m=belly_stereo_baseline_mm_*.001;
      q->right_from_depth_translation_m=belly_right_from_depth_translation_m_;
      q->rectified_depth_k=sdk_k;
      q->stereo_body_geometry_verified=belly_stereo_body_geometry_verified_ &&
        q->depth_pixel_type==static_cast<int>(OB_PIXEL_DEPTH) &&
        profile->getFormat()==OB_FORMAT_Y16 && k.width==q->width && k.height==q->height &&
        sdk_k==frame.depth_info->k && sdk_k==belly_depth_info_template_.k &&
        distortion.k1==0.&&distortion.k2==0.&&distortion.k3==0.&&distortion.k4==0.&&
        distortion.k5==0.&&distortion.k6==0.&&distortion.p1==0.&&distortion.p2==0.&&
        q->geometry_depth_device_us==capture.depth_device_us && q->geometry_depth_global_us==capture.depth_global_us &&
        q->geometry_depth_frame_index==frame.depth_frame_index && q->geometry_depth_scale_m==frame.sdk_depth_scale_m;
      q->stereo_body_geometry_reason=q->stereo_body_geometry_verified?"same_depth_sdk_rectified_profile":"depth_sdk_profile_or_identity_mismatch";
    }catch(...){q->stereo_body_geometry_reason="depth_sdk_geometry_read_failed";}
    std::shared_ptr<const ObservationWork> exact;
    {std::lock_guard<std::mutex> lock(belly_observation_mutex_);
      for(auto it=belly_primary_quality_pairs_.rbegin();it!=belly_primary_quality_pairs_.rend();++it)
        if(it->first==capture.depth_device_us){exact=it->second;break;}}
    // Never wait for IR, use a nearby frame, or update an already-delivered token.
    if(!exact)return q;
    try {
      auto depth=exact->frames[0]->frame->as<ob::DepthFrame>();
      auto left=exact->frames[1]->frame->as<ob::VideoFrame>();
      auto right=exact->frames[2]->frame->as<ob::VideoFrame>();
      const std::array<std::shared_ptr<ob::Frame>,3> frames{{depth,left,right}};
      for(std::size_t i=0;i<3;++i){q->device_us[i]=frames[i]->getTimeStampUs();q->global_us[i]=frames[i]->getGlobalTimeStampUs();q->frame_indices[i]=frames[i]->getIndex();}
      if(depth->getIndex()!=capture.depth->getIndex()||depth->getPixelType()!=OB_PIXEL_DEPTH||
          depth->getValueScale()*0.001!=frame.sdk_depth_scale_m||
          q->device_us[0]!=capture.depth_device_us||q->global_us[0]!=capture.depth_global_us||
          q->device_us[0]!=q->device_us[1]||q->device_us[0]!=q->device_us[2]||
          q->global_us[0]!=q->global_us[1]||q->global_us[0]!=q->global_us[2])return q;
      const auto count=q->width*q->height;
      for(const auto& ir:{left,right})if(!ir->getData()||ir->getFormat()!=OB_FORMAT_Y8||
          ir->getWidth()!=q->width||ir->getHeight()!=q->height||ir->getDataSize()!=count)return q;
      auto& mm=belly_primary_quality_workspace_.depth_mm;mm.resize(count);const auto& bytes=frame.depth->data;
      for(std::size_t i=0;i<count;++i)mm[i]=uint16_t(bytes[2*i])|(uint16_t(bytes[2*i+1])<<8);
      const RectifiedStereoSample sample{mm.data(),left->getData(),right->getData(),count,count,count,
        uint32_t(q->width),uint32_t(q->height),q->device_us[0],q->device_us[1],q->device_us[2],
        frame.depth_info->k[0],belly_stereo_baseline_mm_,true,frame.depth_info->k[4],frame.depth_info->k[2],frame.depth_info->k[5]};
      const auto assessed=assess_stereo_saturation(sample,&belly_primary_quality_workspace_);if(!assessed.valid)return q;
      q->depth_pixel_type=static_cast<int>(depth->getPixelType());
      q->stereo_match_complete=assessed.stereo_match_complete;q->stereo_match_reason=assessed.stereo_match_reason;q->stereo_match_checks=assessed.stereo_match_checks;
      q->component_extension_complete=assessed.component_extension_complete;q->component_extension_reason=assessed.component_extension_reason;q->component_checks=assessed.component_checks;
      q->exact_rectified_pair=true;q->reason="same_sdk_epoch_native_quality_rejection_only;component="+assessed.component_extension_reason+";stereo="+assessed.stereo_match_reason;
      for(std::size_t i=0;i<count;++i)q->pixels[i]=static_cast<uint8_t>(assessed.pixels[i]);
    }catch(...){q->reason="stereo_quality_unassessed";}
    return q;
  }

  static bool valid_sha256(const std::string & value)
  {
    return value.size() == 64U && std::all_of(
      value.begin(), value.end(), [](unsigned char character) {
        return (character >= '0' && character <= '9') ||
               (character >= 'a' && character <= 'f');
      });
  }

  geometry_msgs::msg::TransformStamped internal_optical_transform(
    const std::string & role,
    const char * expected_parent,
    const char * expected_child)
  {
    const auto parent = declare_parameter(role + "_internal_tf_parent_frame", "");
    const auto child = declare_parameter(role + "_internal_tf_child_frame", "");
    const auto translation = declare_parameter<std::vector<double>>(
      role + "_internal_tf_translation_xyz", {});
    const auto rotation = declare_parameter<std::vector<double>>(
      role + "_internal_tf_rotation_xyzw", {});
    if (parent != expected_parent || child != expected_child ||
      !valid_static_transform(parent, child, translation, rotation))
    {
      throw std::invalid_argument(role + " internal optical TF parameters are invalid");
    }
    geometry_msgs::msg::TransformStamped output;
    output.header.stamp = now();
    output.header.frame_id = parent;
    output.child_frame_id = child;
    output.transform.translation.x = translation[0];
    output.transform.translation.y = translation[1];
    output.transform.translation.z = translation[2];
    output.transform.rotation.x = rotation[0];
    output.transform.rotation.y = rotation[1];
    output.transform.rotation.z = rotation[2];
    output.transform.rotation.w = rotation[3];
    return output;
  }

  void configure_internal_optical_tf()
  {
    if (!publish_internal_optical_tf_) {
      if (!internal_optical_tf_receipt_sha256_.empty()) {
        throw std::invalid_argument(
                "camera internal optical TF receipt provided while publication is disabled");
      }
      return;
    }
    if (!valid_sha256(internal_optical_tf_receipt_sha256_)) {
      throw std::invalid_argument(
              "camera internal optical TF requires a validated production receipt SHA-256");
    }
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(internal_optical_transform(
      "chest", kChestLinkFrame, kChestOpticalFrame));
    transforms.push_back(internal_optical_transform(
      "belly", kBellyLinkFrame, kBellyOpticalFrame));
    if (transforms[0].child_frame_id == transforms[1].child_frame_id) {
      throw std::invalid_argument("camera internal optical TF child frames must be unique");
    }
    internal_optical_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    internal_optical_tf_broadcaster_->sendTransform(transforms);
  }

  void verify_sdk_version()
  {
    if (ob::Version::getMajor() != 2 || ob::Version::getMinor() != 8 ||
      ob::Version::getPatch() != 6)
    {
      throw std::runtime_error(
              "camera runtime requires OrbbecSDK 2.8.6 exactly, got " +
              std::to_string(ob::Version::getMajor()) + "." +
              std::to_string(ob::Version::getMinor()) + "." +
              std::to_string(ob::Version::getPatch()));
    }
    sdk_version_ = "2.8.6";
  }

  void bind_and_start()
  {
    auto device_list = context_->queryDeviceList();
    std::vector<std::string> serials;
    serials.reserve(device_list->getCount());
    for (std::uint32_t index = 0; index < device_list->getCount(); ++index) {
      serials.emplace_back(device_list->getSerialNumber(index));
    }
    const std::vector<RoleRequest> requests{
      {"chest", chest_state_.serial, chest_state_.enabled},
      {"belly", belly_state_.serial, belly_state_.enabled},
      {"head", head_state_.serial, head_state_.enabled},
    };
    const auto bindings = bind_enabled_roles_by_serial(requests, serials);

    auto chest_device = device_list->getDevice(bindings.at("chest"));
    chest_state_.direct_identity = opened_identity(chest_device, chest_state_.serial, "chest");
    auto belly_device = device_list->getDevice(bindings.at("belly"));
    belly_state_.direct_identity = opened_identity(belly_device, belly_state_.serial, "belly");
    configure_belly_ae_target(belly_device); // Recover before either pipeline captures.
    start_chest(chest_device);
    start_belly(belly_device);
  }

  std::shared_ptr<const CameraSourceIdentity> opened_identity(
    const std::shared_ptr<ob::Device> & device,
    const std::string & expected,
    const std::string & role)
  {
    const std::string opened = device->getDeviceInfo()->getSerialNumber();
    if (opened != expected) {
      throw std::runtime_error(
              role + " opened device serial mismatch: expected " + expected + ", got " + opened);
    }
    return std::make_shared<const CameraSourceIdentity>(CameraSourceIdentity{
      get_fully_qualified_name(), opened,
      std::string(get_fully_qualified_name()) + "#" + role + "#" + opened + "#" + std::to_string(steady_now_ns())});
  }

  void start_chest(const std::shared_ptr<ob::Device> & device)
  {
    require_global_timestamp(device, "chest");
    auto pipeline = std::make_shared<ob::Pipeline>(device);
    auto color_profile = select_exact_profile(
      pipeline->getStreamProfileList(OB_SENSOR_COLOR), kColorContract, "chest color");
    auto depth_profile = select_exact_profile(
      pipeline->getD2CDepthProfileList(color_profile, ALIGN_D2C_HW_MODE),
      kDepthContract, "chest hardware-D2C depth");

    chest_color_info_template_ = make_camera_info(
      color_profile, kColorContract, kChestOpticalFrame);
    chest_depth_info_template_ =
      sunhorizon_camera_runtime::gemini_hw_d2c_depth_info(chest_color_info_template_);

    auto config = std::make_shared<ob::Config>();
    config->enableStream(color_profile);
    config->enableStream(depth_profile);
    config->setAlignMode(ALIGN_D2C_HW_MODE);
    config->setDepthScaleRequire(true);
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
    pipeline->enableFrameSync();

    chest_pipeline_ = std::make_unique<PipelineOwner>(pipeline);
    chest_pipeline_->mark_start_attempted();
    pipeline->start(config, [this](std::shared_ptr<ob::FrameSet> frameset) noexcept {
      auto timing = callback_entry_now();
      CallbackActivityGuard activity(chest_state_, timing.entry_system_ns);
      callback_guard("chest", [this, &frameset, &timing]() {
        timing.entry_ros_ns = get_clock()->now().nanoseconds();
        if (capture_dispatch_mode_=="latest-worker-v1") dispatch_capture(true,frameset,timing);
        else publish_chest(frameset,timing);
      });
    });
    set_started(chest_state_);
  }

  void publish_ae_target_control_receipt() noexcept
  {
    if (!belly_ae_target_transaction_ || !belly_ir_observation_pub_) return;
    try {
      DiagnosticArray receipt;
      receipt.header.stamp = system_clock_.now();
      DiagnosticStatus status;
      status.name = "sunhorizon_camera_runtime/belly_ae_target_transaction";
      status.hardware_id = belly_state_.serial;
      status.level = DiagnosticStatus::OK;
      status.message = "property_control_receipt_not_a_source_frame_observation";
      for (const auto & item : belly_ae_target_transaction_->receipt()) {
        status.values.push_back(key_value(item.first, item.second));
        if (item.first == "ae_target_transaction_state" && item.second == "restore_failed") {
          status.level = DiagnosticStatus::ERROR;
        }
      }
      receipt.status.push_back(std::move(status));
      belly_ir_observation_pub_->publish(receipt);
    } catch (...) {} // Durable journal, rather than optional DDS, owns recovery.
  }

  void configure_belly_ae_target(const std::shared_ptr<ob::Device> & device)
  {
    if (device->getDeviceInfo()->getSerialNumber() != belly_state_.serial) {
      throw std::runtime_error("belly AE target device identity mismatch");
    }
    AeTargetDevice access;
    access.target_read_supported = [device] {
      return device->isPropertySupported(OB_PROP_IR_BRIGHTNESS_INT, OB_PERMISSION_READ);};
    access.target_write_supported = [device] {
      return device->isPropertySupported(OB_PROP_IR_BRIGHTNESS_INT, OB_PERMISSION_WRITE);};
    access.ae_read_supported = [device] {
      return device->isPropertySupported(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, OB_PERMISSION_READ);};
    access.read_target = [device] {return device->getIntProperty(OB_PROP_IR_BRIGHTNESS_INT);};
    access.range = [device] {
      const auto range = device->getIntPropertyRange(OB_PROP_IR_BRIGHTNESS_INT);
      return AeTargetRange{range.min, range.max, range.step};};
    access.read_ae = [device] {return device->getBoolProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL);};
    access.write_target = [device](int value) {device->setIntProperty(OB_PROP_IR_BRIGHTNESS_INT, value);};
    belly_ae_target_transaction_ = std::make_unique<AeTargetTransaction>(
      belly_ir_ae_target_journal_directory_, belly_state_.serial, std::move(access));
    try {
      belly_ae_target_transaction_->begin(belly_ir_ae_target_override_);
      for (const auto & item : belly_ae_target_transaction_->receipt()) {
        belly_observation_device_values_.push_back(key_value(item.first, item.second));
      }
      publish_ae_target_control_receipt();
    } catch (...) {publish_ae_target_control_receipt();throw;}
  }

  void start_belly(const std::shared_ptr<ob::Device> & device)
  {
    require_global_timestamp(device, "belly");
    auto pipeline = std::make_shared<ob::Pipeline>(device);
    auto depth_profile = select_exact_profile(
      pipeline->getStreamProfileList(OB_SENSOR_DEPTH), belly_contract_, "belly depth");
    belly_depth_info_template_ = make_camera_info(
      depth_profile, belly_contract_, kBellyOpticalFrame);

    auto config = std::make_shared<ob::Config>();
    config->enableStream(depth_profile);
    if (belly_stereo_quality_shadow_ || belly_stereo_observation_v2_) {
      const auto depth_k = depth_profile->getIntrinsic();
      const auto depth_d = depth_profile->getDistortion();
      const auto undistorted = [](const OBCameraDistortion & d) {
        return d.k1 == 0. && d.k2 == 0. && d.k3 == 0. && d.k4 == 0. &&
               d.k5 == 0. && d.k6 == 0. && d.p1 == 0. && d.p2 == 0.;
      };
      if (!undistorted(depth_d)) throw std::runtime_error("belly native depth not rectified");
      for (const auto sensor : {OB_SENSOR_IR_LEFT, OB_SENSOR_IR_RIGHT}) {
        const auto profile = pipeline->getStreamProfileList(sensor)->getVideoStreamProfile(
          belly_contract_.width, belly_contract_.height, OB_FORMAT_Y8, belly_contract_.fps);
        if (!profile || profile->getWidth() != belly_contract_.width ||
            profile->getHeight() != belly_contract_.height ||
            profile->getFps() != belly_contract_.fps || profile->getFormat() != OB_FORMAT_Y8) {
          throw std::runtime_error("belly IR exact capture profile unavailable");
        }
        const auto k = profile->getIntrinsic();
        const auto e = depth_profile->getExtrinsicTo(profile);
        bool verified = k.width == depth_k.width && k.height == depth_k.height &&
          std::abs(k.fx - depth_k.fx) < 1.e-5 && std::abs(k.fy - depth_k.fy) < 1.e-5 &&
          std::abs(k.cx - depth_k.cx) < 1.e-5 && std::abs(k.cy - depth_k.cy) < 1.e-5 &&
          undistorted(profile->getDistortion());
        for (int i = 0; i < 9; ++i) {
          verified = verified && std::abs(e.rot[i] - (i % 4 == 0 ? 1. : 0.)) < 1.e-6;
        }
        verified = verified && std::abs(e.trans[1]) < 1.e-5 && std::abs(e.trans[2]) < 1.e-5;
        if (sensor == OB_SENSOR_IR_LEFT) verified = verified && std::abs(e.trans[0]) < 1.e-5;
        else {
          verified = verified && std::isfinite(e.trans[0]) && e.trans[0] < 0.;
          belly_stereo_baseline_mm_ = -e.trans[0];
          belly_stereo_body_geometry_verified_=e.trans[1]==0.&&e.trans[2]==0.;
          for(int j=0;j<9;++j)belly_stereo_body_geometry_verified_ =
            belly_stereo_body_geometry_verified_ && e.rot[j]==(j%4==0?1.:0.);
          belly_right_from_depth_translation_m_={double(e.trans[0])*.001,double(e.trans[1])*.001,double(e.trans[2])*.001};
        }
        if (!verified) throw std::runtime_error("unsupported belly rectified stereo geometry");
        config->enableStream(profile);
      }
    }
    // Pipeline construction enables sync in this SDK; omission is insufficient.
    if (belly_stereo_quality_shadow_) pipeline->disableFrameSync();
    if (belly_stereo_observation_v2_) {
      pipeline->disableFrameSync();
      // Read support and effective values, never change depth/laser/filter properties.
      const auto read_int = [&](const char * name, OBPropertyID property) {
        try {
          const bool supported = device->isPropertySupported(property, OB_PERMISSION_READ);
          belly_observation_device_values_.push_back(key_value(std::string(name) + "_read_supported",
            supported ? "true" : "false"));
          if (supported) belly_observation_device_values_.push_back(
            key_value(name, std::to_string(device->getIntProperty(property))));
        } catch (...) {belly_observation_device_values_.push_back(key_value(name, "read_failed"));}
      };
      read_int("min_depth_mm", OB_PROP_MIN_DEPTH_INT);
      read_int("max_depth_mm", OB_PROP_MAX_DEPTH_INT);
      for (const auto item : {
        std::make_pair("depth_with_confidence_stream", OB_PROP_DEPTH_WITH_CONFIDENCE_STREAM_ENABLE_BOOL),
        std::make_pair("confidence_stream_filter", OB_PROP_CONFIDENCE_STREAM_FILTER_BOOL),
        std::make_pair("depth_postfilter", OB_PROP_DEPTH_POSTFILTER_BOOL),
        std::make_pair("depth_holefilter", OB_PROP_DEPTH_HOLEFILTER_BOOL),
        std::make_pair("software_noise_remove_filter", OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL),
        std::make_pair("hardware_noise_remove_filter", OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL)}) {
        try {
          const bool supported = device->isPropertySupported(item.second, OB_PERMISSION_READ);
          belly_observation_device_values_.push_back(key_value(std::string(item.first) + "_read_supported",
            supported ? "true" : "false"));
          if (supported) belly_observation_device_values_.push_back(key_value(item.first,
            device->getBoolProperty(item.second) ? "true" : "false"));
        } catch (...) {belly_observation_device_values_.push_back(key_value(item.first, "read_failed"));}
      }
      try {
        const auto info = device->getDeviceInfo();
        belly_observation_device_values_.push_back(key_value("device_name", info->getName()));
        belly_observation_device_values_.push_back(key_value("device_serial", info->getSerialNumber()));
        belly_observation_device_values_.push_back(key_value("firmware_version", info->getFirmwareVersion()));
      } catch (...) {belly_observation_device_values_.push_back(key_value("device_info", "read_failed"));}
      try {
        const auto sensors = device->getSensorList();
        std::string types;
        for (uint32_t i = 0; i < sensors->getCount(); ++i) {
          if (i) types += ",";
          types += std::to_string(static_cast<int>(sensors->getSensorType(i)));
        }
        belly_observation_device_values_.push_back(key_value("sensor_types", types));
      } catch (...) {belly_observation_device_values_.push_back(key_value("sensor_types", "read_failed"));}
    }
    // SDK aggregation must not make raw depth wait for optional IR streams.
    config->setFrameAggregateOutputMode((belly_stereo_quality_shadow_ || belly_stereo_observation_v2_) ?
      OB_FRAME_AGGREGATE_OUTPUT_DISABLE : OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

    belly_pipeline_ = std::make_unique<PipelineOwner>(pipeline);
    belly_pipeline_->mark_start_attempted();
    pipeline->start(config, [this](std::shared_ptr<ob::FrameSet> frameset) noexcept {
      auto timing = callback_entry_now();
      callback_guard("belly", [this, &frameset, &timing]() {
        timing.entry_ros_ns = get_clock()->now().nanoseconds();
        // Pair already-arrived SDK owners before the depth worker can run.
        // This bounded4-epoch cache never waits for a missing IR arrival.
        if (belly_stereo_observation_v2_) {
          try {observe_belly(frameset, timing);} catch (...) {
            belly_quality_last_frame_valid_.store(false);
            ++belly_quality_failed_frames_;
          }
        }
        if ((!belly_stereo_quality_shadow_ && !belly_stereo_observation_v2_) ||
          (frameset && frameset->getDepthFrame())) {
          CallbackActivityGuard activity(belly_state_, timing.entry_system_ns);
          if (capture_dispatch_mode_=="latest-worker-v1") dispatch_capture(false,frameset,timing);
          else route_belly(frameset, timing);
        } else if (belly_stereo_quality_shadow_) {
          // Optional IR arrivals must not refresh depth arrival or activity.
          route_belly(frameset, timing);
        }
        // No conversion, saturation calculation or DDS publication runs here.
      });
    });
    set_started(belly_state_);
  }

  static void require_global_timestamp(
    const std::shared_ptr<ob::Device> & device,
    const std::string & role)
  {
    if (!device->isGlobalTimestampSupported()) {
      throw std::runtime_error(role + " camera does not support SDK global timestamps");
    }
    device->enableGlobalTimestamp(true);
  }

  template<typename Callback>
  void callback_guard(const char * role, Callback && callback) noexcept
  {
    if (fatal_latched_.load()) {
      return;
    }
    try {
      callback();
    } catch (const ob::Error & error) {
      latch_callback_failure(role, "sdk_callback_error", error.what());
    } catch (const std::exception & error) {
      latch_callback_failure(role, "callback_error", error.what());
    } catch (...) {
      latch_callback_failure(role, "callback_error", "unknown exception");
    }
  }

  void latch_callback_failure(
    const char * role,
    const char * category,
    const char * detail) noexcept
  {
    try {
      RoleState & state = std::strcmp(role, "chest") == 0 ? chest_state_ : belly_state_;
      {
        std::lock_guard<std::mutex> lock(state.mutex);
        ++state.rejected_frames;
        state.last_reason = std::string(category) + ": " + detail;
      }
      latch_fatal(std::string(role) + "_" + category + ": " + detail);
    } catch (...) {
      fatal_latched_.store(true);
      std::fprintf(stderr, "sunhorizon camera callback failure could not be formatted\n");
    }
  }

  std::optional<Capture> validate_chest(const std::shared_ptr<ob::FrameSet>& frameset)
  {
    if (!frameset) {
      throw std::runtime_error("null frameset");
    }
    auto color = frameset->getColorFrame();
    auto depth = frameset->getDepthFrame();
    if (!color || !depth) {
      throw std::runtime_error("hardware-D2C frameset missing color or depth");
    }
    validate_sdk_frame(color, kColorContract, "chest_color");
    validate_sdk_frame(depth, kDepthContract, "chest_aligned_depth");

    const auto color_device_us = color->getTimeStampUs();
    const auto depth_device_us = depth->getTimeStampUs();
    const auto color_global_us = color->getGlobalTimeStampUs();
    const auto depth_global_us = depth->getGlobalTimeStampUs();
    const auto color_system_us = color->getSystemTimeStampUs();
    const auto depth_system_us = depth->getSystemTimeStampUs();
    const auto device_pair_delta_us = color_device_us > depth_device_us ?
      color_device_us - depth_device_us : depth_device_us - color_device_us;
    const auto global_pair_delta_us = color_global_us > depth_global_us ?
      color_global_us - depth_global_us : depth_global_us - color_global_us;
    const auto pair_decision = evaluate_capture_pair(
      device_pair_delta_us, global_pair_delta_us,
      static_cast<std::uint64_t>(maximum_device_pair_delta_us_),
      static_cast<std::uint64_t>(maximum_global_pair_delta_us_), startup_age_sec(),
      timestamp_startup_grace_sec_);
    if (pair_decision == CapturePairDecision::kStartupDrop) {
      record_startup_pair_drop(
        chest_state_, device_pair_delta_us, global_pair_delta_us,
        color_system_us > depth_system_us ?
        color_system_us - depth_system_us : depth_system_us - color_system_us);
      return std::nullopt;
    }
    if (pair_decision == CapturePairDecision::kFatalDeviceMismatch) {
      throw std::runtime_error(
              "chest hardware-D2C device timestamp delta exceeds "
              "maximum_device_pair_delta_us: device_us=" +
              std::to_string(device_pair_delta_us) + ", limit_us=" +
              std::to_string(maximum_device_pair_delta_us_) + ", global_us=" +
              std::to_string(global_pair_delta_us) + ", host_arrival_us=" +
              std::to_string(color_system_us > depth_system_us ?
              color_system_us - depth_system_us : depth_system_us - color_system_us));
    }
    if (pair_decision == CapturePairDecision::kFatalGlobalMismatch) {
      throw std::runtime_error(
              "chest SDK global mapping timestamp delta exceeds "
              "maximum_global_pair_delta_us outside startup grace: global_us=" +
              std::to_string(global_pair_delta_us) + ", limit_us=" +
              std::to_string(maximum_global_pair_delta_us_) + ", device_us=" +
              std::to_string(device_pair_delta_us) + ", host_arrival_us=" +
              std::to_string(color_system_us > depth_system_us ?
              color_system_us - depth_system_us : depth_system_us - color_system_us));
    }
    require_timestamp(
      chest_color_system_timestamp_gate_, color_system_us, "chest_color", "sdk_system",
      &chest_state_);
    require_timestamp(
      chest_depth_system_timestamp_gate_, depth_system_us, "chest_aligned_depth", "sdk_system",
      &chest_state_);
    require_timestamp(
      chest_color_device_timestamp_gate_, color_device_us, "chest_color", "device");
    require_timestamp(
      chest_depth_device_timestamp_gate_, depth_device_us, "chest_aligned_depth", "device");
    require_timestamp(
      chest_color_global_timestamp_gate_, color_global_us, "chest_color", "global");
    require_timestamp(
      chest_depth_global_timestamp_gate_, depth_global_us, "chest_aligned_depth", "global");

    Capture frame;frame.frameset=frameset;frame.color=color;frame.depth=depth;
    frame.color_device_us=color_device_us;
    frame.color_global_us=color_global_us;
    frame.color_system_us=color_system_us;
    frame.depth_device_us=depth_device_us;
    frame.depth_global_us=depth_global_us;
    frame.depth_system_us=depth_system_us;
    return frame;
  }

  void publish_chest(
    const std::shared_ptr<ob::FrameSet> & frameset, CallbackFrameTiming & timing)
  {
    timing.lock_wait.begin = callback_clock_now();
    std::lock_guard<std::mutex> callback_lock(chest_callback_mutex_);
    timing.lock_wait.end = callback_clock_now();
    timing.validate_convert.begin = timing.lock_wait.end;
    auto frame=validate_chest(frameset);if (!frame) return;
    convert_publish_chest(*frame,timing);
  }

  void convert_publish_chest(Capture& frame,CallbackFrameTiming& timing)
  {
    if (timing.worker_dispatch) timing.worker_convert.begin=callback_clock_now();
    const auto& color=frame.color;const auto& depth=frame.depth;
    const auto color_device_us=frame.color_device_us;
    const auto color_global_us=frame.color_global_us;
    const auto color_system_us=frame.color_system_us;
    const auto depth_device_us=frame.depth_device_us;
    const auto depth_global_us=frame.depth_global_us;
    const auto depth_system_us=frame.depth_system_us;
    auto color_image = make_color_image(color, kChestOpticalFrame);
    auto depth_image = make_depth_image(depth, kChestOpticalFrame,
      timing.worker_dispatch?std::optional<double>(frame.depth_scale_mm):std::nullopt);
    if (timing.worker_dispatch) {
      color_image.header.stamp=ros_time_from_host_clock_us(frame.color_global_us);
      depth_image.header.stamp=ros_time_from_host_clock_us(frame.depth_global_us);
    }
    auto color_info = chest_color_info_template_;
    auto depth_info = chest_depth_info_template_;
    color_info.header = color_image.header;
    depth_info.header = depth_image.header;

    const auto converted=callback_clock_now();
    if (timing.worker_dispatch) {timing.worker_convert.end=converted;}
    else timing.validate_convert.end=converted;
    timing.publish.begin=converted;
    if (timing.worker_dispatch && fatal_latched_.load()) return;
    if (frame_sink_) {
      deliver_direct(true, frame, timing, std::move(depth_image), std::move(depth_info),
        std::move(color_image), std::move(color_info));
    } else {
      chest_color_pub_->publish(color_image);
      chest_color_info_pub_->publish(color_info);
      chest_depth_pub_->publish(depth_image);
      chest_depth_info_pub_->publish(depth_info);
    }
    const auto publication_end = callback_clock_now();
    timing.finish_publication(publication_end, system_now_ns());
    record_accepted(
      chest_state_, color_device_us, color_global_us, color_system_us,
      depth_device_us, depth_global_us, depth_system_us, steady_now_ns(),
      timing.worker_dispatch?frame.depth_scale_mm:depth->getValueScale(), timing);
  }

  void route_belly(const std::shared_ptr<ob::FrameSet> & frameset, CallbackFrameTiming & timing)
  {
    if (!belly_stereo_quality_shadow_) { publish_belly(frameset, timing); return; }
    std::lock_guard<std::mutex> routing_lock(belly_routing_mutex_);
    if (!frameset) throw std::runtime_error("null belly frameset");
    auto depth = frameset->getDepthFrame();
    const auto result = publish_primary_then_shadow([&]() {
      // IR-only delivery is normal with aggregation disabled. It must not count
      // as a received depth frame; existing raw depth freshness gates remain.
      if (depth) publish_belly(frameset, timing);
    }, [&]() {
      const std::array<OBFrameType, 3> types{{OB_FRAME_DEPTH, OB_FRAME_IR_LEFT, OB_FRAME_IR_RIGHT}};
      for (std::size_t channel = 0; channel < types.size(); ++channel) {
        auto frame = frameset->getFrame(types[channel]);
        if (!frame) continue;
        const auto ready = belly_stereo_pairing_.insert(channel, frame->getTimeStampUs(), frame);
        if (ready[0]) publish_belly_quality(ready);
      }
      belly_quality_pairing_evictions_.store(belly_stereo_pairing_.evicted());
    });
    if (result == ShadowResult::kFailed) {
      belly_quality_last_frame_valid_.store(false);
      ++belly_quality_failed_frames_;
    }
  }

  void observe_belly(const std::shared_ptr<ob::FrameSet> & frameset,
    const CallbackFrameTiming & timing)
  {
    if (!frameset || !accepting_frames_.load()) return;
    std::unique_ptr<ObservationWork> retired;
    std::shared_ptr<const ObservationWork> retired_primary_quality;
    {
      std::lock_guard<std::mutex> lock(belly_observation_mutex_);
      const std::array<OBFrameType, 3> types{{OB_FRAME_DEPTH, OB_FRAME_IR_LEFT, OB_FRAME_IR_RIGHT}};
      for (std::size_t i = 0; i < types.size(); ++i) {
        auto frame = frameset->getFrame(types[i]);
        if (!frame) continue;
        auto observed = std::make_shared<ObservedSdkFrame>();
        observed->frame = frame;
        observed->callback_ros_ns = timing.entry_ros_ns;
        observed->callback_steady_ns = timing.entry.steady_ns;
        auto ready = belly_observation_pairing_.insert(i, frame->getTimeStampUs(), observed);
        if (ready[0]) {
          auto complete=std::make_shared<ObservationWork>();complete->frames=ready;
          belly_primary_quality_pairs_.push_back({ready[0]->frame->getTimeStampUs(),complete});
          if(belly_primary_quality_pairs_.size()>4) {
            retired_primary_quality=std::move(belly_primary_quality_pairs_.front().second);
            belly_primary_quality_pairs_.erase(belly_primary_quality_pairs_.begin());
          }
        }
        if (ready[0] && (++belly_observation_complete_ %
          static_cast<std::size_t>(belly_stereo_observation_stride_.get()) == 0)) {
          auto work = std::make_unique<ObservationWork>();
          work->frames = std::move(ready);
          retired = belly_observation_worker_->replace(std::move(work));
        }
      }
      belly_quality_pairing_evictions_.store(belly_observation_pairing_.evicted());
    }
    retired.reset();
  }

  void publish_belly_quality(const ExactStereoPairing<std::shared_ptr<ob::Frame>>::Triple & frames,
    const ObservationWork * observation = nullptr)
  {
    const auto depth = frames[0]->as<ob::DepthFrame>();
    const auto left = frames[1]->as<ob::VideoFrame>();
    const auto right = frames[2]->as<ob::VideoFrame>();
    auto depth_image = make_depth_image(depth, kBellyOpticalFrame);
    auto depth_info = belly_depth_info_template_;
    depth_info.header = depth_image.header;
    const auto device_us = depth->getTimeStampUs();
    Image quality_depth;
    const std::size_t count = std::size_t(depth_image.width) * depth_image.height;
    for (const auto & ir : {left, right}) {
      if (!ir->getData() || ir->getFormat() != OB_FORMAT_Y8 ||
          ir->getWidth() != depth_image.width || ir->getHeight() != depth_image.height ||
          ir->getDataSize() != count) throw std::runtime_error("invalid belly IR layout");
    }
    std::vector<uint16_t> millimetres(count);
    for (std::size_t i = 0; i < count; ++i) {
      millimetres[i] = uint16_t(depth_image.data[2 * i]) |
        (uint16_t(depth_image.data[2 * i + 1]) << 8);
    }
    const RectifiedStereoSample sample{millimetres.data(), left->getData(), right->getData(),
      count, count, count, depth_image.width, depth_image.height, device_us,
      left->getTimeStampUs(), right->getTimeStampUs(), depth_info.k[0],
      belly_stereo_baseline_mm_, true,depth_info.k[4],depth_info.k[2],depth_info.k[5]};
    const auto quality = assess_stereo_saturation(sample);
    if (!quality.valid) throw std::runtime_error(quality.reason);
    quality_depth = depth_image;
    for (std::size_t i = 0; i < count; ++i) {
      if (quality.pixels[i] != StereoQuality::kNotRejected) {
        quality_depth.data[2 * i] = quality_depth.data[2 * i + 1] = 0;
      }
    }
    if (observation) {
      Image reason;
      reason.header = depth_image.header;
      reason.width = depth_image.width; reason.height = depth_image.height;
      reason.encoding = "mono8"; reason.is_bigendian = false; reason.step = reason.width;
      reason.data.reserve(count);
      for (const auto value : quality.pixels) reason.data.push_back(static_cast<uint8_t>(value));
      // Preserve SDK units/bytes as well as the normal millimetre image. This
      // topic is evidence only: the receipt's pixel type and value scale are
      // mandatory to interpret it; it is never a ROS navigation depth input.
      Image native = depth_image;
      const auto * bytes = static_cast<const uint8_t *>(depth->getData());
      native.data.assign(bytes, bytes + depth->getDataSize());
      belly_native_depth_pub_->publish(native);
      belly_observed_depth_pub_->publish(depth_image);
      belly_observed_info_pub_->publish(depth_info);
      belly_quality_reason_pub_->publish(reason);
    }
    belly_quality_saturated_.store(quality.saturated);
    belly_quality_unassessed_.store(quality.unassessed);
    belly_quality_depth_pub_->publish(quality_depth);
    belly_quality_info_pub_->publish(depth_info);
    // This remains inside route_belly's optional-shadow exception boundary.
    // Observation failures invalidate only this candidate attempt; primary raw
    // depth was already published and cannot be stopped by these publishers.
    if (belly_stereo_ir_observation_ || observation) publish_belly_ir_observation(frames, observation);
    belly_quality_last_frame_valid_.store(true);
  }

  static Image make_native_ir_image(
    const std::shared_ptr<ob::VideoFrame> & frame, const char * optical_frame)
  {
    if (frame->getFormat() != OB_FORMAT_Y8) throw std::runtime_error("invalid native IR format");
    auto observation = copy_native_ir_observation(frame->getData(), frame->getDataSize(),
      frame->getWidth(), frame->getHeight(), frame->getGlobalTimeStampUs(), optical_frame);
    Image image;
    image.header.stamp = ros_time_from_host_clock_us(observation.global_us);
    image.header.frame_id = std::move(observation.frame_id);
    image.width = observation.width;
    image.height = observation.height;
    image.encoding = "mono8";
    image.is_bigendian = false;
    image.step = image.width;
    image.data = std::move(observation.bytes);
    return image;
  }

  void publish_belly_ir_observation(
    const ExactStereoPairing<std::shared_ptr<ob::Frame>>::Triple & frames,
    const ObservationWork * observation = nullptr)
  {
    std::array<StereoSourceTime, 3> source;
    for (std::size_t channel = 0; channel < frames.size(); ++channel) {
      source[channel] = {frames[channel]->getTimeStampUs(),
        frames[channel]->getGlobalTimeStampUs(), frames[channel]->getSystemTimeStampUs()};
    }
    const auto times = stereo_observation_times(source);
    auto left = make_native_ir_image(frames[1]->as<ob::VideoFrame>(), kBellyLeftIrOpticalFrame);
    auto right = make_native_ir_image(frames[2]->as<ob::VideoFrame>(), kBellyRightIrOpticalFrame);
    DiagnosticArray receipt;
    receipt.header.stamp = ros_time_from_host_clock_us(times.source[0].global_us);
    receipt.header.frame_id = kBellyOpticalFrame;
    DiagnosticStatus status;
    status.name = "sunhorizon_camera_runtime/belly_stereo_ir_observation";
    status.hardware_id = belly_state_.serial;
    status.level = DiagnosticStatus::OK;
    status.message = "native_ir_diagnostic_evidence_not_navigation_acceptance";
    const std::array<std::string, 3> channels{{"depth", "left", "right"}};
    for (std::size_t channel = 0; channel < channels.size(); ++channel) {
      status.values.push_back(key_value(channels[channel] + "_device_us",
        std::to_string(times.source[channel].device_us)));
      status.values.push_back(key_value(channels[channel] + "_global_us",
        std::to_string(times.source[channel].global_us)));
      status.values.push_back(key_value(channels[channel] + "_system_us",
        std::to_string(times.source[channel].system_us)));
    }
    if (observation) {
      status.values.insert(status.values.end(), belly_observation_device_values_.begin(),
        belly_observation_device_values_.end());
      status.values.push_back(key_value("observer_contract", "exact_device_epoch_v2_not_navigation_input"));
      status.values.push_back(key_value("quality_algorithm", kStereoQualityAlgorithm));
      status.values.push_back(key_value("quality_stereo_stage", "disabled_foreground_ambiguity"));
      status.values.push_back(key_value("quality_reason_encoding", "0=not_rejected,1=direct_dual_ir_saturated,2=unassessed,3=saturation_dominated_component,4=historical_stereo_mismatch_disabled"));
      status.values.push_back(key_value("rejected_depth_semantics", "unknown_not_free_not_self"));
      status.values.push_back(key_value("depth_native_units", "sdk_units_times_depth_value_scale_mm"));
      status.values.push_back(key_value("baseline_mm", std::to_string(belly_stereo_baseline_mm_)));
      status.values.push_back(key_value("rectified_geometry_verified", "true"));
      status.values.push_back(key_value("depth_value_scale_mm", std::to_string(frames[0]->as<ob::DepthFrame>()->getValueScale())));
      try {status.values.push_back(key_value("depth_pixel_type",
          std::to_string(static_cast<int>(frames[0]->as<ob::DepthFrame>()->getPixelType()))));}
      catch (...) {status.values.push_back(key_value("depth_pixel_type", "read_failed"));}
      for (std::size_t i = 0; i < frames.size(); ++i) {
        status.values.push_back(key_value(channels[i] + "_frame_index", std::to_string(frames[i]->getIndex())));
        status.values.push_back(key_value(channels[i] + "_callback_ros_ns", std::to_string(observation->frames[i]->callback_ros_ns)));
        status.values.push_back(key_value(channels[i] + "_callback_steady_ns", std::to_string(observation->frames[i]->callback_steady_ns)));
        for (const auto item : {std::make_pair("exposure", OB_FRAME_METADATA_TYPE_EXPOSURE),
          std::make_pair("gain", OB_FRAME_METADATA_TYPE_GAIN)}) {
          try {
            status.values.push_back(key_value(channels[i] + "_" + item.first,
              frames[i]->hasMetadata(item.second) ?
              std::to_string(frames[i]->getMetadataValue(item.second)) : "unsupported"));
          } catch (...) {status.values.push_back(key_value(channels[i] + "_" + item.first, "read_failed"));}
        }
      }
    }
    status.values.push_back(key_value("left_minus_depth_device_us",
      std::to_string(times.source[1].device_us - times.source[0].device_us)));
    status.values.push_back(key_value("right_minus_depth_device_us",
      std::to_string(times.source[2].device_us - times.source[0].device_us)));
    status.values.push_back(key_value("left_minus_depth_global_us",
      std::to_string(times.left_minus_depth_global_us)));
    status.values.push_back(key_value("right_minus_depth_global_us",
      std::to_string(times.right_minus_depth_global_us)));
    status.values.push_back(key_value("left_frame_id", left.header.frame_id));
    status.values.push_back(key_value("right_frame_id", right.header.frame_id));
    status.values.push_back(key_value("depth_frame_id", kBellyOpticalFrame));
    status.values.push_back(key_value("ros_header_time_source", "each_frame.getGlobalTimeStampUs"));
    status.values.push_back(key_value("saturated_pixels", std::to_string(belly_quality_saturated_.load())));
    status.values.push_back(key_value("unassessed_pixels", std::to_string(belly_quality_unassessed_.load())));
    status.values.push_back(key_value("pairing_evictions", std::to_string(belly_quality_pairing_evictions_.load())));
    if (observation && belly_observation_worker_) {
      const auto stats = belly_observation_worker_->stats();
      status.values.push_back(key_value("observer_admitted", std::to_string(stats.admitted)));
      status.values.push_back(key_value("observer_replaced", std::to_string(stats.replaced)));
      status.values.push_back(key_value("observer_failed", std::to_string(stats.errors)));
    }
    status.values.push_back(key_value("observation_publish_system_ns", std::to_string(system_now_ns())));
    receipt.status.push_back(std::move(status));
    // Three nontransactional publications. Recorders must require both image
    // source stamps named in this receipt; partial pairs cannot be retracted.
    belly_left_ir_pub_->publish(left);
    belly_right_ir_pub_->publish(right);
    belly_ir_observation_pub_->publish(receipt);
    ++belly_ir_observation_frames_;
  }

  Capture validate_belly(const std::shared_ptr<ob::FrameSet>& frameset)
  {
    if (!frameset) {
      throw std::runtime_error("null frameset");
    }
    auto depth = frameset->getDepthFrame();
    if (!depth) {
      throw std::runtime_error("frameset missing depth");
    }
    validate_sdk_frame(depth, belly_contract_, "belly_depth");
    const auto device_us = depth->getTimeStampUs();
    const auto global_us = depth->getGlobalTimeStampUs();
    const auto system_us = depth->getSystemTimeStampUs();
    require_timestamp(
      belly_depth_system_timestamp_gate_, system_us, "belly_depth", "sdk_system",
      &belly_state_);
    require_timestamp(
      belly_depth_device_timestamp_gate_, device_us, "belly_depth", "device");
    require_timestamp(
      belly_depth_global_timestamp_gate_, global_us, "belly_depth", "global");

    Capture frame;frame.frameset=frameset;frame.depth=depth;
    frame.depth_device_us=device_us;frame.depth_global_us=global_us;frame.depth_system_us=system_us;
    return frame;
  }

  void publish_belly(
    const std::shared_ptr<ob::FrameSet> & frameset, CallbackFrameTiming & timing)
  {
    timing.lock_wait.begin = callback_clock_now();
    std::lock_guard<std::mutex> callback_lock(belly_callback_mutex_);
    timing.lock_wait.end = callback_clock_now();
    timing.validate_convert.begin = timing.lock_wait.end;
    auto frame=validate_belly(frameset);convert_publish_belly(frame,timing);
  }

  void convert_publish_belly(Capture& frame,CallbackFrameTiming& timing)
  {
    if (timing.worker_dispatch) timing.worker_convert.begin=callback_clock_now();
    const auto& depth=frame.depth;
    const auto device_us=frame.depth_device_us,global_us=frame.depth_global_us,system_us=frame.depth_system_us;
    auto depth_image = make_depth_image(depth, kBellyOpticalFrame,
      timing.worker_dispatch?std::optional<double>(frame.depth_scale_mm):std::nullopt);
    if (timing.worker_dispatch) depth_image.header.stamp=ros_time_from_host_clock_us(frame.depth_global_us);
    auto depth_info = belly_depth_info_template_;
    depth_info.header = depth_image.header;
    const auto converted=callback_clock_now();
    if (timing.worker_dispatch) {timing.worker_convert.end=converted;}
    else timing.validate_convert.end=converted;
    timing.publish.begin=converted;
    if (timing.worker_dispatch && fatal_latched_.load()) return;
    if (frame_sink_) {
      deliver_direct(false, frame, timing, std::move(depth_image), std::move(depth_info));
    } else {
      belly_depth_pub_->publish(depth_image);
      belly_depth_info_pub_->publish(depth_info);
    }
    const auto publication_end = callback_clock_now();
    timing.finish_publication(publication_end, system_now_ns());
    record_accepted(
      belly_state_, 0U, 0U, 0U, device_us, global_us, system_us,
      steady_now_ns(), timing.worker_dispatch?frame.depth_scale_mm:depth->getValueScale(), timing);
  }

  static void validate_sdk_frame(
    const std::shared_ptr<ob::VideoFrame> & frame,
    const FrameContract & contract,
    const std::string & stream)
  {
    if (!frame->getData()) {
      throw std::runtime_error(stream + " has null frame data");
    }
    PixelContract pixels;
    if (frame->getFormat() == OB_FORMAT_RGB) {
      pixels = PixelContract::kRgb8;
    } else if (frame->getFormat() == OB_FORMAT_Y16) {
      pixels = PixelContract::kDepth16;
    } else {
      throw std::runtime_error(stream + " frame format is neither RGB nor Y16");
    }
    const auto error = validate_frame_shape(
      contract, frame->getWidth(), frame->getHeight(), pixels, frame->getDataSize());
    if (error.has_value()) {
      throw std::runtime_error(stream + " " + *error);
    }
  }

  void require_timestamp(
    StrictTimestampGate & gate,
    std::uint64_t timestamp_us,
    const std::string & stream,
    const std::string & clock_name,
    RoleState * startup_recovery_state = nullptr)
  {
    const auto decision = gate.observe(timestamp_us);
    if (decision == TimestampDecision::kAccepted) {
      return;
    }
    if (startup_recovery_state != nullptr && clock_name == "sdk_system" &&
      evaluate_sdk_system_timestamp(
        decision, startup_age_sec(), timestamp_startup_grace_sec_) ==
      TimestampRecoveryDecision::kStartupReanchor)
    {
      gate.reanchor(timestamp_us);
      record_startup_system_clock_reanchor(
        *startup_recovery_state, stream, timestamp_us);
      return;
    }
    throw std::runtime_error(
            stream + " " + clock_name + "_" + timestamp_decision_reason(decision));
  }

  double startup_age_sec() const
  {
    return age_seconds(steady_now_ns(), startup_steady_timestamp_ns_);
  }

  static void record_startup_system_clock_reanchor(
    RoleState & state,
    const std::string & stream,
    std::uint64_t timestamp_us)
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    ++state.startup_system_clock_reanchors;
    state.last_startup_transient_reason =
      stream + " sdk_system_regression_reanchored:" + std::to_string(timestamp_us);
  }

  static void record_startup_pair_drop(
    RoleState & state,
    std::uint64_t device_pair_delta_us,
    std::uint64_t global_pair_delta_us,
    std::uint64_t host_arrival_pair_delta_us)
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    ++state.rejected_frames;
    ++state.startup_global_pair_drops;
    state.last_reason = "startup_global_pair_settling";
    state.last_startup_transient_reason =
      "global_pair_drop:device_us=" + std::to_string(device_pair_delta_us) +
      ",global_us=" + std::to_string(global_pair_delta_us) +
      ",host_arrival_us=" + std::to_string(host_arrival_pair_delta_us);
  }

  static Image make_color_image(
    const std::shared_ptr<ob::ColorFrame> & frame,
    const std::string & frame_id)
  {
    Image output;
    output.header.stamp = ros_time_from_host_clock_us(frame->getGlobalTimeStampUs());
    output.header.frame_id = frame_id;
    output.height = frame->getHeight();
    output.width = frame->getWidth();
    output.encoding = "rgb8";
    output.is_bigendian = 0U;
    output.step = output.width * 3U;
    output.data.resize(frame->getDataSize());
    std::memcpy(output.data.data(), frame->getData(), output.data.size());
    return output;
  }

  static Image make_depth_image(
    const std::shared_ptr<ob::DepthFrame> & frame,
    const std::string & frame_id,std::optional<double> approved_scale=std::nullopt)
  {
    const double scale_mm = approved_scale?*approved_scale:frame->getValueScale();
    if (!std::isfinite(scale_mm) || scale_mm <= 0.0) {
      throw std::runtime_error("invalid depth value scale");
    }

    Image output;
    output.header.stamp = ros_time_from_host_clock_us(frame->getGlobalTimeStampUs());
    output.header.frame_id = frame_id;
    output.height = frame->getHeight();
    output.width = frame->getWidth();
    output.encoding = "16UC1";
    output.is_bigendian = 0U;
    output.step = output.width * 2U;
    output.data.resize(frame->getDataSize());

    const auto pixel_count = static_cast<std::size_t>(output.width) * output.height;
    const auto * source_bytes = static_cast<const std::uint8_t *>(frame->getData());
    for (std::size_t index = 0; index < pixel_count; ++index) {
      std::uint16_t raw_value = 0U;
      std::memcpy(&raw_value, source_bytes + index * sizeof(raw_value), sizeof(raw_value));
      const auto millimeters = raw_value == 0U ? 0.0 : std::round(raw_value * scale_mm);
      if (millimeters < 0.0 || millimeters > std::numeric_limits<std::uint16_t>::max()) {
        throw std::runtime_error("scaled depth value exceeds 16UC1 millimeter range");
      }
      const auto output_value = static_cast<std::uint16_t>(millimeters);
      std::memcpy(output.data.data() + index * sizeof(output_value),
        &output_value, sizeof(output_value));
    }
    return output;
  }

  static void set_started(RoleState & state)
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.started = true;
    if (state.accepted_frames == 0U) {
      state.last_reason = "waiting_for_first_frame";
    }
  }

  static void record_accepted(
    RoleState & state,
    std::uint64_t color_device_timestamp_us,
    std::uint64_t color_global_timestamp_us,
    std::uint64_t color_sdk_system_timestamp_us,
    std::uint64_t depth_device_timestamp_us,
    std::uint64_t depth_global_timestamp_us,
    std::uint64_t depth_sdk_system_timestamp_us,
    std::int64_t publish_complete_steady_ns,
    double depth_value_scale_mm,
    const CallbackFrameTiming & timing)
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    ++state.accepted_frames;
    state.last_color_device_timestamp_us = color_device_timestamp_us;
    state.last_color_global_timestamp_us = color_global_timestamp_us;
    state.last_color_sdk_system_timestamp_us = color_sdk_system_timestamp_us;
    state.last_depth_device_timestamp_us = depth_device_timestamp_us;
    state.last_depth_global_timestamp_us = depth_global_timestamp_us;
    state.last_depth_sdk_system_timestamp_us = depth_sdk_system_timestamp_us;
    state.last_publish_complete_steady_ns.store(
      publish_complete_steady_ns, std::memory_order_release);
    state.last_depth_value_scale_mm = depth_value_scale_mm;
    state.last_reason = "ok";
    state.last_completed_timing.accept(state.accepted_frames, timing);
  }

  void latch_fatal(std::string reason) noexcept
  {
    bool expected = false;
    if (!fatal_latched_.compare_exchange_strong(expected, true)) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(fatal_mutex_);
      fatal_reason_ = std::move(reason);
    }
    RCLCPP_ERROR(get_logger(), "camera runtime fail-closed: %s", fatal_reason().c_str());
    if (fault_sink_) {
      try {fault_sink_(CameraFault{fatal_reason(), get_clock()->now().nanoseconds(), steady_now_ns(), true});}
      catch (...) {std::fprintf(stderr, "camera fault sink threw after fatal latch\n");}
    }
  }

  std::string fatal_reason() const
  {
    std::lock_guard<std::mutex> lock(fatal_mutex_);
    return fatal_reason_;
  }

  void on_health_timer()
  {
    if (!fatal_latched_.load()) {
      check_role_freshness(chest_state_);
      check_role_freshness(belly_state_);
    }
    publish_diagnostics();
    if (fatal_latched_.load() && !shutdown_started_.exchange(true)) {
      stop_all_noexcept();
      // A composed owner controls the process/context lifecycle itself.
      if (!frame_sink_) rclcpp::shutdown(get_node_base_interface()->get_context());
    }
  }

  void check_role_freshness(const RoleState & state)
  {
    std::string role;
    bool enabled = false;
    bool started = false;
    {
      std::lock_guard<std::mutex> lock(state.mutex);
      role = state.role;
      enabled = state.enabled;
      started = state.started;
    }
    if (!enabled || !started) {
      return;
    }

    const auto now_ns = steady_now_ns();
    const auto first_arrival_ns = state.first_frame_arrival_steady_ns.load(
      std::memory_order_acquire);
    const auto last_arrival_ns = state.last_frame_arrival_steady_ns.load(
      std::memory_order_acquire);
    const auto last_publish_ns = state.last_publish_complete_steady_ns.load(
      std::memory_order_acquire);
    const auto callbacks_in_flight = state.callbacks_in_flight.load(
      std::memory_order_acquire);
    const StreamFreshnessObservation observation{
      started,
      first_arrival_ns > 0,
      last_publish_ns > 0,
      callbacks_in_flight,
      age_seconds(now_ns, startup_steady_timestamp_ns_),
      age_seconds(now_ns, first_arrival_ns),
      age_seconds(now_ns, last_arrival_ns),
      age_seconds(now_ns, last_publish_ns),
    };
    const auto decision = evaluate_stream_freshness(
      observation, startup_timeout_sec_, frame_timeout_sec_, publish_timeout_sec_);
    if (decision == StreamFreshnessDecision::kStartupTimeout) {
      latch_fatal(role + " did not receive a frame before startup timeout");
    } else if (decision == StreamFreshnessDecision::kFrameArrivalStale) {
      latch_fatal(
        role + " frame stream became stale: frame_arrival_age_sec=" +
        std::to_string(observation.last_frame_arrival_age_sec) +
        ", publish_complete_age_sec=" +
        std::to_string(observation.last_publication_age_sec) +
        ", callbacks_in_flight=" + std::to_string(callbacks_in_flight));
    } else if (decision == StreamFreshnessDecision::kPublicationStale) {
      latch_fatal(
        role + " frame publication became stale: frame_arrival_age_sec=" +
        std::to_string(observation.last_frame_arrival_age_sec) +
        ", publish_complete_age_sec=" +
        std::to_string(observation.publication_seen ?
        observation.last_publication_age_sec :
        observation.first_frame_arrival_age_sec) +
        ", callbacks_in_flight=" + std::to_string(callbacks_in_flight));
    }
  }

  void publish_diagnostics()
  {
    DiagnosticArray output;
    output.header.stamp = system_clock_.now();

    DiagnosticStatus runtime;
    runtime.name = "sunhorizon_camera_runtime";
    runtime.hardware_id = "robot_a_orbbec_rgbd";
    runtime.level = fatal_latched_.load() ? DiagnosticStatus::ERROR : DiagnosticStatus::OK;
    runtime.message = fatal_latched_.load() ? fatal_reason() : "owned_camera_runtime_active";
    runtime.values.push_back(key_value("ros_header_time_source", "frame.getGlobalTimeStampUs"));
    runtime.values.push_back(key_value("host_arrival_time_source", "frame.getSystemTimeStampUs"));
    runtime.values.push_back(key_value("device_time_source", "frame.getTimeStampUs"));
    runtime.values.push_back(key_value("orbbec_sdk_version", sdk_version_));
    runtime.values.push_back(key_value(
      "frame_timeout_sec", std::to_string(frame_timeout_sec_)));
    runtime.values.push_back(key_value(
      "publish_timeout_sec", std::to_string(publish_timeout_sec_)));
    runtime.values.push_back(key_value("belly_capture_fps",std::to_string(belly_contract_.fps)));
    runtime.values.push_back(key_value("capture_dispatch_mode",capture_dispatch_mode_));
    runtime.values.push_back(key_value("direct_frame_delivery",frame_sink_?"true":"false"));
    runtime.values.push_back(key_value("raw_ros_publish_enabled",raw_ros_publish_enabled_?"true":"false"));
    runtime.values.push_back(key_value("raw_publish_errors",std::to_string(raw_publish_errors_.load())));
    runtime.values.push_back(key_value(
      "timestamp_startup_grace_sec", std::to_string(timestamp_startup_grace_sec_)));
    runtime.values.push_back(key_value(
      "maximum_device_pair_delta_us", std::to_string(maximum_device_pair_delta_us_)));
    runtime.values.push_back(key_value(
      "maximum_global_pair_delta_us", std::to_string(maximum_global_pair_delta_us_)));
    runtime.values.push_back(key_value("firmware_upgrade", "disabled_not_invoked"));
    runtime.values.push_back(key_value("head_v1", "disabled"));
    runtime.values.push_back(key_value("belly_stereo_quality_shadow",
      belly_stereo_quality_shadow_ ? "true" : "false"));
    runtime.values.push_back(key_value("belly_stereo_ir_observation",
      belly_stereo_ir_observation_ ? "true" : "false"));
    runtime.values.push_back(key_value("belly_ir_observation_frames",
      std::to_string(belly_ir_observation_frames_.load())));
    runtime.values.push_back(key_value("belly_stereo_observation_v2",
      belly_stereo_observation_v2_ ? "true" : "false"));
    runtime.values.push_back(key_value("belly_stereo_observation_stride",
      std::to_string(belly_stereo_observation_stride_.get())));
    if (belly_observation_worker_) {
      const auto stats = belly_observation_worker_->stats();
      runtime.values.push_back(key_value("stereo_observer_admitted", std::to_string(stats.admitted)));
      runtime.values.push_back(key_value("stereo_observer_replaced", std::to_string(stats.replaced)));
      runtime.values.push_back(key_value("stereo_observer_errors", std::to_string(stats.errors)));
      runtime.values.push_back(key_value("stereo_observer_closed", stats.closed ? "true" : "false"));
    }
    runtime.values.push_back(key_value("belly_quality_last_frame_valid",
      (belly_stereo_quality_shadow_ || belly_stereo_observation_v2_) &&
      belly_quality_last_frame_valid_.load() ? "true" : "false"));
    runtime.values.push_back(key_value("belly_quality_pairing_evictions",
      std::to_string(belly_quality_pairing_evictions_.load())));
    runtime.values.push_back(key_value("belly_quality_failed_frames",
      std::to_string(belly_quality_failed_frames_.load())));
    runtime.values.push_back(key_value("belly_quality_saturated_pixels",
      std::to_string(belly_quality_saturated_.load())));
    runtime.values.push_back(key_value("belly_quality_unassessed_pixels",
      std::to_string(belly_quality_unassessed_.load())));
    runtime.values.push_back(key_value(
      "internal_optical_tf_published", publish_internal_optical_tf_ ? "true" : "false"));
    runtime.values.push_back(key_value(
      "internal_optical_tf_receipt_sha256", internal_optical_tf_receipt_sha256_));
    output.status.push_back(std::move(runtime));
    output.status.push_back(role_diagnostic(chest_state_));
    output.status.push_back(role_diagnostic(belly_state_));
    output.status.push_back(role_diagnostic(head_state_));
    diagnostics_pub_->publish(output);
  }

  DiagnosticStatus role_diagnostic(const RoleState & state) const
  {
    const auto now_ns = steady_now_ns();
    const auto last_arrival_ns = state.last_frame_arrival_steady_ns.load(
      std::memory_order_acquire);
    const auto last_publish_ns = state.last_publish_complete_steady_ns.load(
      std::memory_order_acquire);
    const auto callbacks_in_flight = state.callbacks_in_flight.load(
      std::memory_order_acquire);
    std::lock_guard<std::mutex> lock(state.mutex);
    DiagnosticStatus output;
    output.name = "sunhorizon_camera_runtime/" + state.role;
    output.hardware_id = state.serial;
    if (!state.enabled) {
      output.level = DiagnosticStatus::OK;
      output.message = "disabled_by_configuration";
    } else if (fatal_latched_.load()) {
      output.level = DiagnosticStatus::ERROR;
      output.message = fatal_reason();
    } else if (state.accepted_frames == 0U) {
      output.level = DiagnosticStatus::WARN;
      output.message = state.last_reason;
    } else {
      output.level = DiagnosticStatus::OK;
      output.message = state.last_reason;
    }
    output.values.push_back(key_value("serial", state.serial));
    output.values.push_back(key_value("enabled", state.enabled ? "true" : "false"));
    output.values.push_back(key_value("started", state.started ? "true" : "false"));
    output.values.push_back(key_value("accepted_frames", std::to_string(state.accepted_frames)));
    output.values.push_back(key_value("rejected_frames", std::to_string(state.rejected_frames)));
    output.values.push_back(key_value(
      "startup_system_clock_reanchors",
      std::to_string(state.startup_system_clock_reanchors)));
    output.values.push_back(key_value(
      "startup_global_pair_drops", std::to_string(state.startup_global_pair_drops)));
    output.values.push_back(key_value(
      "last_startup_transient_reason", state.last_startup_transient_reason));
    output.values.push_back(key_value(
      "last_color_device_timestamp_us", std::to_string(state.last_color_device_timestamp_us)));
    output.values.push_back(key_value(
      "last_color_global_timestamp_us", std::to_string(state.last_color_global_timestamp_us)));
    output.values.push_back(key_value(
      "last_color_sdk_system_timestamp_us",
      std::to_string(state.last_color_sdk_system_timestamp_us)));
    output.values.push_back(key_value(
      "last_depth_device_timestamp_us", std::to_string(state.last_depth_device_timestamp_us)));
    output.values.push_back(key_value(
      "last_depth_global_timestamp_us", std::to_string(state.last_depth_global_timestamp_us)));
    output.values.push_back(key_value(
      "last_depth_sdk_system_timestamp_us",
      std::to_string(state.last_depth_sdk_system_timestamp_us)));
    output.values.push_back(key_value(
      "last_host_receive_timestamp_ns",
      std::to_string(state.last_host_receive_timestamp_ns.load(std::memory_order_acquire))));
    output.values.push_back(key_value(
      "frame_arrival_age_sec", std::to_string(age_seconds(now_ns, last_arrival_ns))));
    output.values.push_back(key_value(
      "publish_complete_age_sec", std::to_string(age_seconds(now_ns, last_publish_ns))));
    output.values.push_back(key_value(
      "callbacks_in_flight", std::to_string(callbacks_in_flight)));
    output.values.push_back(key_value("workers_in_flight",std::to_string(state.workers_in_flight.load())));
    if (capture_dispatch_mode_=="latest-worker-v1" && (state.role=="chest" || state.role=="belly")) {
      const auto& worker=state.role=="chest"?chest_worker_:belly_worker_;
      const auto queue=worker?worker->stats():CaptureWorker::Stats{};
      output.values.push_back(key_value("worker_pending",queue.pending?"true":"false"));
      output.values.push_back(key_value("worker_admitted",std::to_string(queue.admitted)));
      output.values.push_back(key_value("worker_replaced",std::to_string(queue.replaced)));
      output.values.push_back(key_value("worker_errors",std::to_string(queue.errors)));
    }
    output.values.push_back(key_value(
      "source_depth_value_scale_mm", std::to_string(state.last_depth_value_scale_mm)));
    // These source timestamps and the accepted sequence above share this role
    // lock with the timing snapshot. Arrival atomics can describe a newer frame.
    const auto & completed = state.last_completed_timing;
    const auto & timing = completed.frame;
    output.values.push_back(key_value("last_completed_dispatch_mode",timing.worker_dispatch?"latest-worker-v1":"inline-v1"));
    output.values.push_back(key_value("last_completed_ingress_sequence",std::to_string(timing.ingress_sequence)));
    output.values.push_back(key_value("last_completed_ingress_validate_wall_ns",std::to_string(timing.ingress_validate.wall_ns())));
    output.values.push_back(key_value("last_completed_ingress_validate_thread_cpu_ns",std::to_string(timing.ingress_validate.thread_cpu_ns())));
    output.values.push_back(key_value("last_completed_queue_wait_wall_ns",std::to_string(timing.queue_wait_wall_ns())));
    output.values.push_back(key_value("last_completed_worker_convert_wall_ns",std::to_string(timing.worker_convert.wall_ns())));
    output.values.push_back(key_value("last_completed_worker_convert_thread_cpu_ns",std::to_string(timing.worker_convert.thread_cpu_ns())));
    output.values.push_back(key_value(
      "last_completed_timing_valid", completed.valid ? "true" : "false"));
    output.values.push_back(key_value(
      "last_completed_timing_accepted_frames", std::to_string(completed.accepted_frames)));
    output.values.push_back(key_value(
      "last_completed_callback_entry_system_ns", std::to_string(timing.entry_system_ns)));
    output.values.push_back(key_value(
      "last_completed_callback_entry_steady_ns", std::to_string(timing.entry.steady_ns)));
    output.values.push_back(key_value(
      "last_completed_callback_entry_thread_cpu_ns", std::to_string(timing.entry.thread_cpu_ns)));
    output.values.push_back(key_value(
      "last_completed_publish_complete_system_ns", std::to_string(timing.publish_complete_system_ns)));
    output.values.push_back(key_value(
      "last_completed_publish_complete_steady_ns", std::to_string(timing.publish.end.steady_ns)));
    output.values.push_back(key_value(
      "last_completed_callback_lock_wait_wall_ns", std::to_string(timing.lock_wait.wall_ns())));
    output.values.push_back(key_value(
      "last_completed_callback_lock_wait_thread_cpu_ns", std::to_string(timing.lock_wait.thread_cpu_ns())));
    output.values.push_back(key_value(
      "last_completed_validate_convert_wall_ns", std::to_string(timing.validate_convert.wall_ns())));
    output.values.push_back(key_value(
      "last_completed_validate_convert_thread_cpu_ns", std::to_string(timing.validate_convert.thread_cpu_ns())));
    output.values.push_back(key_value(
      "last_completed_publish_wall_ns", std::to_string(timing.publish.wall_ns())));
    output.values.push_back(key_value(
      "last_completed_publish_thread_cpu_ns", std::to_string(timing.publish.thread_cpu_ns())));
    return output;
  }

  void stop_all_noexcept() noexcept
  {
    std::lock_guard<std::mutex> shutdown_lock(stop_mutex_);
    accepting_frames_.store(false);
    if (diagnostics_timer_) {
      try {diagnostics_timer_->cancel();} catch (...) {} // Context may already be shutting down.
    }
    // Cancel pending admission first; no SDK/state/gate lock across stop/join.
    if (belly_observation_worker_) {auto retired=belly_observation_worker_->request_stop();retired.reset();}
    if (belly_worker_) {auto retired=belly_worker_->request_stop();retired.reset();}
    if (chest_worker_) {auto retired=chest_worker_->request_stop();retired.reset();}
    if (belly_raw_worker_) {auto retired=belly_raw_worker_->request_stop();retired.reset();}
    if (chest_raw_worker_) {auto retired=chest_raw_worker_->request_stop();retired.reset();}
    if (belly_pipeline_) {
      belly_pipeline_->stop_noexcept();
    }
    if (chest_pipeline_) {
      chest_pipeline_->stop_noexcept();
    }
    if (belly_observation_worker_) belly_observation_worker_->join();
    if (belly_worker_) belly_worker_->join();
    if (chest_worker_) chest_worker_->join();
    if (belly_raw_worker_) belly_raw_worker_->join();
    if (chest_raw_worker_) chest_raw_worker_->join();
    // Admission is closed and workers have joined before the SDK restore call.
    if (belly_ae_target_transaction_) {
      if (!belly_ae_target_transaction_->restore_noexcept()) {
        RCLCPP_ERROR(get_logger(), "AE target restoration failed; durable journal remains unresolved and next capture must refuse startup");
      }
      publish_ae_target_control_receipt();
    }
  }

  rclcpp::Clock system_clock_;
  CameraFrameSink frame_sink_;
  CameraFaultSink fault_sink_;
  bool raw_ros_publish_enabled_{true};
  std::unique_ptr<RawWorker> chest_raw_worker_, belly_raw_worker_;
  std::atomic<std::uint64_t> raw_publish_errors_{0};
  std::shared_ptr<ob::Context> context_;
  std::unique_ptr<PipelineOwner> chest_pipeline_;
  std::unique_ptr<PipelineOwner> belly_pipeline_;
  std::unique_ptr<CaptureWorker> chest_worker_,belly_worker_;
  std::mutex stop_mutex_;
  std::atomic<bool> accepting_frames_{true};
  uint64_t chest_ingress_sequence_{0},belly_ingress_sequence_{0};
  FrameContract belly_contract_{belly_capture_contract(30)};
  std::string capture_dispatch_mode_{"inline-v1"};

  RoleState chest_state_;
  RoleState belly_state_;
  RoleState head_state_;
  StrictTimestampGate chest_color_system_timestamp_gate_;
  StrictTimestampGate chest_depth_system_timestamp_gate_;
  StrictTimestampGate belly_depth_system_timestamp_gate_;
  StrictTimestampGate chest_color_device_timestamp_gate_;
  StrictTimestampGate chest_depth_device_timestamp_gate_;
  StrictTimestampGate belly_depth_device_timestamp_gate_;
  StrictTimestampGate chest_color_global_timestamp_gate_;
  StrictTimestampGate chest_depth_global_timestamp_gate_;
  StrictTimestampGate belly_depth_global_timestamp_gate_;
  std::mutex chest_callback_mutex_;
  std::mutex belly_callback_mutex_;

  CameraInfo chest_color_info_template_;
  CameraInfo chest_depth_info_template_;
  CameraInfo belly_depth_info_template_;

  rclcpp::Publisher<Image>::SharedPtr chest_color_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr chest_color_info_pub_;
  rclcpp::Publisher<Image>::SharedPtr chest_depth_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr chest_depth_info_pub_;
  rclcpp::Publisher<Image>::SharedPtr belly_depth_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr belly_depth_info_pub_;
  rclcpp::Publisher<Image>::SharedPtr belly_quality_depth_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr belly_quality_info_pub_;
  rclcpp::Publisher<Image>::SharedPtr belly_left_ir_pub_, belly_right_ir_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr belly_ir_observation_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> internal_optical_tf_broadcaster_;

  double startup_timeout_sec_{3.0};
  double frame_timeout_sec_{1.0};
  double publish_timeout_sec_{2.0};
  double timestamp_startup_grace_sec_{60.0};
  int maximum_device_pair_delta_us_{5000};
  int maximum_global_pair_delta_us_{20000};
  int diagnostics_period_ms_{250};
  int belly_ir_ae_target_override_{-1};
  std::string belly_ir_ae_target_journal_directory_;
  std::unique_ptr<AeTargetTransaction> belly_ae_target_transaction_;
  bool belly_stereo_observation_v2_{false};
  ObserverStride belly_stereo_observation_stride_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr observer_stride_parameter_callback_;
  std::unique_ptr<ObservationWorker> belly_observation_worker_;
  ObservationPairing belly_observation_pairing_;
  std::mutex belly_observation_mutex_;
  std::size_t belly_observation_complete_{0};
  std::vector<KeyValue> belly_observation_device_values_;
  rclcpp::Publisher<Image>::SharedPtr belly_observed_depth_pub_, belly_native_depth_pub_, belly_quality_reason_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr belly_observed_info_pub_;
  bool belly_stereo_quality_shadow_{false};
  bool belly_stereo_ir_observation_{false};
  bool belly_stereo_body_geometry_verified_{false};
  std::array<double,3> belly_right_from_depth_translation_m_{};
  std::atomic<std::size_t> belly_ir_observation_frames_{0};
  double belly_stereo_baseline_mm_{0.};
  std::mutex belly_routing_mutex_;
  std::vector<std::pair<uint64_t,std::shared_ptr<const ObservationWork>>> belly_primary_quality_pairs_;
  // Only the serialized primary belly conversion owner uses this scratch:
  // the single capture worker, or inline callback under belly_callback_mutex_.
  // The IR observer and every delivered quality token own separate storage.
  StereoQualityWorkspace belly_primary_quality_workspace_;
  ExactStereoPairing<std::shared_ptr<ob::Frame>> belly_stereo_pairing_;
  std::atomic<std::size_t> belly_quality_pairing_evictions_{0};
  std::atomic<bool> belly_quality_last_frame_valid_{false};
  std::atomic<std::size_t> belly_quality_failed_frames_{0};
  std::atomic<std::size_t> belly_quality_saturated_{0}, belly_quality_unassessed_{0};
  bool publish_internal_optical_tf_{false};
  std::string internal_optical_tf_receipt_sha256_;
  std::int64_t startup_steady_timestamp_ns_{0};
  std::atomic<bool> fatal_latched_{false};
  std::atomic<bool> shutdown_started_{false};
  mutable std::mutex fatal_mutex_;
  std::string fatal_reason_{"none"};
  std::string sdk_version_{"unverified"};
};

}  // namespace
rclcpp::Node::SharedPtr create_camera_runtime(const rclcpp::NodeOptions & options,
  CameraFrameSink frame_sink, CameraFaultSink fault_sink)
{
  try {
    return std::make_shared<CameraRuntimeNode>(options, std::move(frame_sink), std::move(fault_sink));
  } catch (const ob::Error & error) {
    throw std::runtime_error(std::string("SDK startup failure: ") + error.what());
  }
}
void stop_camera_runtime(const rclcpp::Node::SharedPtr & node) noexcept
{
  if (auto owner = std::dynamic_pointer_cast<CameraRuntimeNode>(node)) owner->stop();
}
bool camera_runtime_failed(const rclcpp::Node::SharedPtr & node) noexcept
{
  const auto owner = std::dynamic_pointer_cast<CameraRuntimeNode>(node);
  return !owner || owner->failed();
}
}  // namespace sunhorizon_camera_runtime
