from pathlib import Path
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE / "src" / "camera_runtime_node.cpp"
PROFILE_PROBE = PACKAGE / "src" / "camera_profile_probe.cpp"
CONFIG = PACKAGE / "config" / "robot_a.yaml"
CMAKE = PACKAGE / "CMakeLists.txt"
PACKAGE_XML = PACKAGE / "package.xml"
LAUNCH = PACKAGE / "launch" / "camera_runtime.launch.py"
CONTRACT = PACKAGE / "internal_optical_tf_contract.py"


class CameraRuntimeSourceContractTest(unittest.TestCase):
    def test_network_discovery_policy_is_read_only_and_applied_before_capture(self):
        source = SOURCE.read_text()
        self.assertIn('network_enumeration_descriptor.read_only = true', source)
        self.assertIn('"network_device_enumeration_enabled", false, network_enumeration_descriptor', source)
        startup = source.split('    verify_sdk_version();', 1)[1].split('  ~CameraRuntimeNode()', 1)[0]
        self.assertLess(startup.index('create_context_with_network_enumeration('),
                        startup.index('start_workers();'))
        self.assertLess(startup.index('create_context_with_network_enumeration('),
                        startup.index('bind_and_start();'))
        self.assertIn('network_device_enumeration_enabled, [] {return std::make_shared<ob::Context>();}', startup)
        self.assertIn('stop_all_noexcept();', startup)
        self.assertIn('network_device_enumeration_enabled: false', CONFIG.read_text())

    def test_direct_identity_is_the_opened_device_and_is_frozen_before_capture(self):
        source=SOURCE.read_text()
        binding=source.split('  void bind_and_start()',1)[1].split('  void start_chest(',1)[0]
        for role in ('chest','belly'):
            self.assertLess(binding.index(role+'_state_.direct_identity = opened_identity('),
                            binding.index('start_'+role+'('+role+'_device)'))
        self.assertIn('device->getDeviceInfo()->getSerialNumber()',binding)
        self.assertIn('if (opened != expected)',binding)
        delivery=source.split('  void deliver_direct(',1)[1].split('  static bool valid_sha256',1)[0]
        self.assertIn('frame->source_identity = (chest ? chest_state_ : belly_state_).direct_identity',delivery)
        self.assertIn('frame->first_sdk_callback_ros_ns = timing.entry_ros_ns',delivery)
        self.assertNotIn('getDeviceInfo()',delivery)
        self.assertNotIn('ifstream',delivery)

    def test_latest_worker_is_opt_in_and_keeps_admission_and_publication_separate(self):
        source=SOURCE.read_text()
        self.assertIn('declare_parameter<std::string>("capture_dispatch_mode",frame_sink_?"latest-worker-v1":"inline-v1",dispatch_descriptor)',source)
        self.assertIn('dispatch_descriptor.read_only=true',source)
        self.assertIn('valid_capture_dispatch_mode(capture_dispatch_mode_,belly_stereo_quality_shadow_,belly_stereo_ir_observation_)',source)
        admission=source.split('  void dispatch_capture(',1)[1].split('  static bool valid_sha256',1)[0]
        self.assertLess(admission.index('validate_chest(frameset)'),admission.index('->replace('))
        self.assertLess(admission.index('invalid depth value scale'),admission.index('->replace('))
        self.assertIn('retired.reset();',admission)
        self.assertNotIn('->publish(',admission)
        for role in ('chest','belly'):
            converted=source.split('  void convert_publish_'+role+'(',1)[1].split('\n  '+('void route_belly' if role=='chest' else 'static void validate_sdk_frame'),1)[0]
            self.assertNotIn('callback_mutex_',converted)
            self.assertNotIn('require_timestamp(',converted)
            self.assertIn('frame.depth_global_us',converted)
            self.assertIn('frame.depth_scale_mm',converted)
        health=source.split('  void check_role_freshness(',1)[1].split('  void publish_diagnostics()',1)[0]
        self.assertNotIn('workers_in_flight',health)
        stop=source.split('  void stop_all_noexcept() noexcept',1)[1].split('  rclcpp::Clock system_clock_',1)[0]
        self.assertLess(stop.index('request_stop()'),stop.index('belly_pipeline_->stop_noexcept()'))
        self.assertLess(stop.index('chest_pipeline_->stop_noexcept()'),stop.index('belly_worker_->join()'))
        for role in ('chest','belly'):
            start=source.split('  void start_'+role+'(',1)[1].split('\n  '+('void start_belly' if role=='chest' else 'static void require_global_timestamp'),1)[0]
            self.assertLess(start.index(role+'_pipeline_->mark_start_attempted()'),start.index('pipeline->start('))
        self.assertIn('if (!start_attempt_.take_stop())',source)
    def test_direct_factory_owns_sdk_and_keeps_raw_dds_on_observer_workers(self):
        source = SOURCE.read_text()
        header = (PACKAGE / "include/sunhorizon_camera_runtime/direct_frame.hpp").read_text()
        self.assertIn("rclcpp::Node::SharedPtr create_camera_runtime(", header)
        self.assertIn("CameraFrameSink frame_sink = {}", header)
        self.assertIn("using ConstSharedPtr = std::shared_ptr<const CameraFrame>", header)
        self.assertIn("sensor_msgs::msg::Image::ConstSharedPtr depth", header)
        self.assertIn("depth_scale_m{0.001}", header)
        self.assertIn("first_sdk_callback_ros_ns", header)
        self.assertIn("first_sdk_callback_steady_ns", header)
        self.assertIn('declare_parameter("raw_ros_publish_enabled", !bool(frame_sink_))', source)
        self.assertIn('direct camera delivery requires latest-worker-v1', source)
        self.assertIn('stereo quality shadow/IR are incompatible', source)
        self.assertNotIn('create_subscription', source)
        delivery = source.split('  void deliver_direct(', 1)[1].split('  static bool valid_sha256', 1)[0]
        self.assertIn('std::make_shared<const Image>(std::move(depth))', delivery)
        self.assertLess(delivery.index('frame_sink_(immutable)'), delivery.index('->replace('))
        self.assertNotIn('->publish(', delivery)
        self.assertIn('frame->source_stamp = frame->depth->header.stamp', delivery)
        self.assertIn('frame->sdk_depth_scale_m = capture.depth_scale_mm * 0.001', delivery)
        stop = source.split('  void stop_all_noexcept() noexcept', 1)[1].split('  rclcpp::Clock system_clock_', 1)[0]
        for role in ('chest', 'belly'):
            self.assertIn(role + '_raw_worker_->request_stop()', stop)
            self.assertIn(role + '_raw_worker_->join()', stop)
        cmake = CMAKE.read_text()
        self.assertIn('add_library(camera_runtime SHARED src/camera_runtime_node.cpp)', cmake)
        self.assertIn('ament_export_targets(export_camera_runtime HAS_LIBRARY_TARGET)', cmake)
        self.assertIn('src/camera_runtime_main.cpp', cmake)
        self.assertNotIn('int main(', source)

    def test_ir_observation_is_default_off_latest_only_and_requires_existing_shadow(self):
        source = SOURCE.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")
        self.assertIn('declare_parameter("belly_stereo_ir_observation", false)', source)
        self.assertIn("belly_stereo_ir_observation: false", config)
        self.assertIn("belly_stereo_ir_observation_ && !belly_stereo_quality_shadow_", source)
        publishers = source.split("    if (belly_stereo_ir_observation_) {", 1)[1].split(
            "    diagnostics_pub_", 1)[0]
        self.assertIn("rclcpp::QoS(rclcpp::KeepLast(1))", publishers)
        self.assertIn("ir_qos.best_effort().durability_volatile()", publishers)
        for publisher in ("belly_left_ir_pub_", "belly_right_ir_pub_", "belly_ir_observation_pub_"):
            self.assertIn(publisher, publishers)
        start = source.split("  void start_belly(", 1)[1].split(
            "  static void require_global_timestamp", 1)[0]
        self.assertNotIn("belly_stereo_ir_observation_", start)

    def test_ir_images_keep_independent_source_headers_and_raw_bytes(self):
        source = SOURCE.read_text(encoding="utf-8")
        conversion = source.split("  static Image make_native_ir_image(", 1)[1].split(
            "  void publish_belly_ir_observation(", 1)[0]
        self.assertIn("frame->getFormat() != OB_FORMAT_Y8", conversion)
        self.assertIn("frame->getGlobalTimeStampUs(), optical_frame", conversion)
        self.assertIn("ros_time_from_host_clock_us(observation.global_us)", conversion)
        self.assertIn('image.encoding = "mono8"', conversion)
        self.assertIn("image.step = image.width", conversion)
        self.assertIn("image.data = std::move(observation.bytes)", conversion)
        observation = source.split("  void publish_belly_ir_observation(", 1)[1].split(
            "  void publish_belly(\n", 1)[0]
        for field in ("getTimeStampUs()", "getGlobalTimeStampUs()", "getSystemTimeStampUs()",
                      "left_minus_depth_global_us", "right_minus_depth_global_us"):
            self.assertIn(field, observation)
        self.assertIn("frames[1]->as<ob::VideoFrame>(), kBellyLeftIrOpticalFrame", observation)
        self.assertIn("frames[2]->as<ob::VideoFrame>(), kBellyRightIrOpticalFrame", observation)
        self.assertNotIn("left.header =", observation)
        self.assertNotIn("right.header =", observation)
        self.assertNotIn("sendTransform", observation)
        self.assertLess(observation.index("belly_left_ir_pub_->publish(left)"),
                        observation.index("belly_ir_observation_pub_->publish(receipt)"))
        self.assertLess(observation.index("belly_right_ir_pub_->publish(right)"),
                        observation.index("belly_ir_observation_pub_->publish(receipt)"))

    def test_ir_observation_publication_remains_inside_optional_failure_boundary(self):
        source = SOURCE.read_text(encoding="utf-8")
        route = source.split("  void route_belly(", 1)[1].split(
            "  void publish_belly_quality", 1)[0]
        self.assertIn("if (ready[0]) publish_belly_quality(ready)", route)
        self.assertIn("publish_primary_then_shadow", route)
        self.assertIn("++belly_quality_failed_frames_", route)
        candidate = source.split("  void publish_belly_quality(", 1)[1].split(
            "  static Image make_native_ir_image", 1)[0]
        self.assertLess(candidate.index("belly_quality_info_pub_->publish(depth_info)"),
                        candidate.index("publish_belly_ir_observation(frames, observation)"))
        self.assertIn("if (belly_stereo_ir_observation_ || observation) publish_belly_ir_observation(frames, observation)", candidate)
        self.assertLess(candidate.index("publish_belly_ir_observation(frames, observation)"),
                        candidate.index("belly_quality_last_frame_valid_.store(true)"))
        raw = source.split("  void publish_belly(\n", 1)[1].split(
            "  static void validate_sdk_frame", 1)[0]
        self.assertNotIn("ir_observation", raw)

    def test_completed_frame_timing_is_committed_with_source_under_role_lock(self):
        source = SOURCE.read_text(encoding="utf-8")
        accepted = source.split("  static void record_accepted(", 1)[1].split(
            "  void latch_fatal(", 1)[0]
        self.assertLess(accepted.index("lock(state.mutex)"),
                        accepted.index("++state.accepted_frames"))
        self.assertLess(accepted.index("state.last_depth_global_timestamp_us ="),
                        accepted.index("state.last_completed_timing.accept("))
        self.assertEqual(source.count("state.last_completed_timing.accept("), 1)
        for role, last_publisher in (
            ("chest", "chest_depth_info_pub_"),
            ("belly", "belly_depth_info_pub_"),
        ):
            callback = source.split(f"  void convert_publish_{role}(", 1)[1].split(
                "\n  static void" if role == "belly" else "\n  void route_belly(", 1)[0]
            self.assertLess(callback.index(f"{last_publisher}->publish("),
                            callback.index("timing.finish_publication("))
            self.assertLess(callback.index("timing.finish_publication("),
                            callback.index("record_accepted("))
        entry = source.split("CallbackFrameTiming callback_entry_now() noexcept", 1)[1].split(
            "double age_seconds", 1)[0]
        self.assertIn("timing.entry_system_ns = system_now_ns()", entry)
        self.assertIn("timing.entry = callback_clock_now()", entry)
        self.assertIn("CallbackClockPoint callback_clock_now() noexcept", source)
        self.assertIn("CLOCK_THREAD_CPUTIME_ID", source)

    def test_fixed_owned_topics_are_present(self):
        source = SOURCE.read_text(encoding="utf-8")
        for topic in (
            "/sunhorizon/raw/rgbd/chest/color/image",
            "/sunhorizon/raw/rgbd/chest/color/camera_info",
            "/sunhorizon/raw/rgbd/chest/aligned_depth/image",
            "/sunhorizon/raw/rgbd/chest/aligned_depth/camera_info",
            "/sunhorizon/raw/rgbd/belly/depth/image",
            "/sunhorizon/raw/rgbd/belly/depth/camera_info",
        ):
            self.assertIn(topic, source)

    def test_robot_a_defaults_and_head_gate(self):
        config = CONFIG.read_text(encoding="utf-8")
        self.assertIn("chest_serial: CPCAC53000NF", config)
        self.assertIn("belly_serial: CP9W463000ZW", config)
        self.assertIn("head_serial: CP9W463000DY", config)
        self.assertIn("chest_enabled: true", config)
        self.assertIn("belly_enabled: true", config)
        self.assertIn("head_enabled: false", config)
        self.assertIn("head camera is deliberately unsupported", SOURCE.read_text(encoding="utf-8"))

    def test_no_vendor_ros_or_motion_dependencies(self):
        source = SOURCE.read_text(encoding="utf-8")
        manifest = PACKAGE_XML.read_text(encoding="utf-8")
        for forbidden in (
            "/camera_head/",
            "/camera_chest/",
            "/camera_belly/",
            "nav2",
            "cmd_vel",
            "WebRTC",
            "ZMQ",
        ):
            self.assertNotIn(forbidden, source)
            self.assertNotIn(forbidden, manifest)

    def test_internal_optical_tf_is_receipt_gated_and_single_owner(self):
        source = SOURCE.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")
        manifest = PACKAGE_XML.read_text(encoding="utf-8")
        self.assertIn("publish_internal_optical_tf: false", config)
        self.assertNotIn("internal_tf_translation_xyz:", config)
        self.assertIn('declare_parameter("publish_internal_optical_tf", false)', source)
        self.assertIn("internal_optical_tf_acceptance_receipt_sha256", source)
        self.assertEqual(source.count("StaticTransformBroadcaster>(this)"), 1)
        self.assertIn("camera_chest_link", source)
        self.assertIn("camera_chest_color_optical_frame", source)
        self.assertIn("camera_belly_link", source)
        self.assertIn("camera_belly_depth_optical_frame", source)
        self.assertIn("geometry_msgs", manifest)
        self.assertIn("tf2_ros", manifest)
        launch = LAUNCH.read_text(encoding="utf-8")
        contract = CONTRACT.read_text(encoding="utf-8")
        self.assertIn("internal_optical_tf_receipt", launch)
        self.assertIn(".validate(", launch)
        self.assertIn('EXPECTED_USAGE = ["motion_acceptance"]', contract)
        self.assertIn('"vendor_orbbec_tf_publishers_allowed": False', contract)

    def test_no_firmware_update_api_and_sdk_root_is_explicit(self):
        source = SOURCE.read_text(encoding="utf-8")
        cmake = CMAKE.read_text(encoding="utf-8")
        for forbidden in ("updateFirmware", "firmwareUpdate", "upgradeFirmware"):
            self.assertNotIn(forbidden, source)
        self.assertIn("ORBBEC_SDK_ROOT", cmake)
        self.assertIn("find_package(OrbbecSDK CONFIG", cmake)
        self.assertIn('string(FIND "${_orbbec_root}" "/workspace/embed_fusion"', cmake)

    def test_callback_boundary_and_idempotent_stop_are_explicit(self):
        source = SOURCE.read_text(encoding="utf-8")
        self.assertIn("callback_guard", source)
        self.assertIn("catch (...)", source)
        self.assertIn("if (!start_attempt_.take_stop())", source)
        self.assertIn("duplicate_timestamp", (PACKAGE / "include" / "sunhorizon_camera_runtime" / "runtime_contract.hpp").read_text(encoding="utf-8"))
        self.assertIn('"sdk_system"', source)
        self.assertIn('"device"', source)
        self.assertIn('"global"', source)
        self.assertIn("device->enableGlobalTimestamp(true)", source)
        self.assertIn("frame->getGlobalTimeStampUs()", source)

    def test_profile_probe_enumerates_without_starting_streams(self):
        source = PROFILE_PROBE.read_text(encoding="utf-8")
        self.assertIn("getD2CDepthProfileList", source)
        self.assertIn("getStreamProfileList", source)
        self.assertIn("expected exactly one device for serial", source)
        self.assertIn("isGlobalTimestampSupported", source)
        for forbidden in (
            "pipeline->start",
            "waitForFrameset",
            "updateFirmware",
            "firmwareUpdate",
        ):
            self.assertNotIn(forbidden, source)
        self.assertIn("camera_profile_probe", CMAKE.read_text(encoding="utf-8"))

    def test_robot_a_frame_contract_matches_probed_hardware_d2c_profile(self):
        source = SOURCE.read_text(encoding="utf-8")
        self.assertIn(
            "constexpr FrameContract kColorContract{640U, 480U, 30U, PixelContract::kRgb8}",
            source,
        )
        self.assertIn(
            "constexpr FrameContract kDepthContract{640U, 480U, 30U, PixelContract::kDepth16}",
            source,
        )
        self.assertNotIn("has no exact 1280x720@30", source)

    def test_belly_fps_is_one_readonly_contract_for_depth_and_ir(self):
        source = SOURCE.read_text(encoding="utf-8")
        self.assertIn("belly_fps_descriptor.read_only = true", source)
        self.assertIn('"belly_capture_fps", int64_t{30}, belly_fps_descriptor', source)
        belly_start = source.split("  void start_belly(", 1)[1].split("  static void require_global_timestamp", 1)[0]
        self.assertIn('belly_contract_, "belly depth"', belly_start)
        self.assertIn("belly_contract_.width, belly_contract_.height, OB_FORMAT_Y8, belly_contract_.fps", belly_start)
        self.assertIn("profile->getFps() != belly_contract_.fps", belly_start)
        self.assertIn('validate_sdk_frame(depth, belly_contract_, "belly_depth")', source)
        self.assertIn('validate_sdk_frame(depth, kDepthContract, "chest_aligned_depth")', source)
        self.assertIn("belly_capture_fps: 30", CONFIG.read_text(encoding="utf-8"))

    def test_owned_chest_rgb_is_latest_only_best_effort(self):
        source = SOURCE.read_text(encoding="utf-8")
        self.assertIn("rclcpp::QoS(rclcpp::KeepLast(1))", source)
        self.assertIn("chest_color_qos.best_effort().durability_volatile()", source)
        self.assertNotIn("chest_color_qos.reliable()", source)
        self.assertIn(
            "create_publisher<Image>(kChestColorTopic, chest_color_qos)", source)
        self.assertIn(
            "create_publisher<CameraInfo>(kChestColorInfoTopic, chest_color_qos)",
            source,
        )

    def test_capture_and_publication_watchdogs_are_independent_and_bounded(self):
        source = SOURCE.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")
        contract = (
            PACKAGE / "include" / "sunhorizon_camera_runtime" /
            "runtime_contract.hpp"
        ).read_text(encoding="utf-8")

        self.assertIn("frame_timeout_sec: 1.0", config)
        self.assertIn("publish_timeout_sec: 2.0", config)
        self.assertIn("timestamp_startup_grace_sec: 60.0", config)
        self.assertIn("maximum_device_pair_delta_us: 5000", config)
        self.assertIn("maximum_global_pair_delta_us: 20000", config)
        self.assertNotIn("\n    maximum_pair_delta_us:", config)
        self.assertIn(
            'declare_parameter("frame_timeout_sec", 1.0)', source)
        self.assertIn(
            'declare_parameter("publish_timeout_sec", 2.0)', source)
        self.assertIn(
            '"timestamp_startup_grace_sec", 60.0', source)
        self.assertIn(
            '"maximum_device_pair_delta_us", 5000', source)
        self.assertIn(
            '"maximum_global_pair_delta_us", 20000', source)
        self.assertIn(
            '"maximum_pair_delta_us", -1', source)
        self.assertIn("maximum_pair_delta_us is deprecated", source)
        self.assertIn(
            "valid_stream_timeouts(", source)
        self.assertIn("valid_capture_pair_thresholds(", source)
        self.assertIn("publication_timeout_sec >= frame_timeout_sec", contract)
        self.assertIn("StreamFreshnessObservation", contract)
        self.assertIn("callbacks_in_flight == 0U", contract)
        self.assertIn("StreamFreshnessDecision::kPublicationStale", contract)

        chest_callback = source.index(
            "CallbackActivityGuard activity(chest_state_")
        chest_publish = source.index("chest_color_pub_->publish(color_image)")
        chest_complete = source.index(
            "record_accepted(\n      chest_state_", chest_publish)
        self.assertLess(chest_callback, chest_publish)
        self.assertLess(chest_publish, chest_complete)
        self.assertIn("last_publish_complete_steady_ns.store", source)
        self.assertIn('"frame_arrival_age_sec"', source)
        self.assertIn('"publish_complete_age_sec"', source)

    def test_optional_quality_runs_after_primary_publication_with_separate_failure(self):
        source = SOURCE.read_text(encoding="utf-8")
        route = source.split("  void route_belly(", 1)[1].split(
            "  void publish_belly_quality", 1)[0]
        self.assertLess(route.index("publish_primary_then_shadow([&]()"),
                        route.index("if (depth) publish_belly(frameset, timing)"))
        self.assertLess(route.index("if (depth) publish_belly(frameset, timing)"),
                        route.index("belly_stereo_pairing_.insert"))
        self.assertIn("++belly_quality_failed_frames_", route)
        self.assertNotIn("latch_fatal", route)
        start = source.split("  void start_belly(", 1)[1].split(
            "  static void require_global_timestamp", 1)[0]
        self.assertIn("OB_FRAME_AGGREGATE_OUTPUT_DISABLE", start)
        self.assertNotIn("enableFrameSync", start)
        self.assertIn("if (belly_stereo_quality_shadow_) pipeline->disableFrameSync()", start)
        depth_guard = start.index(
            "if ((!belly_stereo_quality_shadow_ && !belly_stereo_observation_v2_) ||")
        activity = start.index("CallbackActivityGuard activity(belly_state_")
        self.assertLess(depth_guard, activity)
        self.assertEqual(start.count("CallbackActivityGuard activity(belly_state_"), 1)
        raw = source.split("  void publish_belly(\n", 1)[1].split(
            "  static void validate_sdk_frame", 1)[0]
        validation = source.split("  Capture validate_belly(", 1)[1].split("  void publish_belly(\n", 1)[0]
        self.assertIn("validate_sdk_frame(depth", validation)
        self.assertIn("require_timestamp(", validation)
        self.assertIn("validate_belly(frameset)", raw)
        self.assertIn("convert_publish_belly(frame,timing)", raw)
        converted = source.split("  void convert_publish_belly(", 1)[1].split("  static void validate_sdk_frame", 1)[0]
        self.assertIn("record_accepted(", converted)
        self.assertNotIn("belly_stereo_pairing_", raw)

    def test_v2_observer_keeps_direct_depth_admission_and_shutdown_independent(self):
        source = SOURCE.read_text(encoding="utf-8")
        self.assertIn('declare_parameter("belly_stereo_observation_v2", false)', source)
        start = source.split("  void start_belly(", 1)[1].split(
            "  static void require_global_timestamp", 1)[0]
        self.assertLess(start.index("observe_belly(frameset, timing)"),
                        start.index("dispatch_capture(false,frameset,timing)"))
        self.assertNotIn("wait_for", start)
        observer = source.split("  void observe_belly(", 1)[1].split("  void publish_belly_quality", 1)[0]
        self.assertIn("frame->getTimeStampUs()", observer)
        self.assertIn("timing.entry_ros_ns", observer)
        self.assertIn("timing.entry.steady_ns", observer)
        self.assertNotIn("publish(", observer)
        self.assertNotIn("assess_stereo_saturation", observer)
        workers = source.split("  void start_workers()", 1)[1].split("  void dispatch_capture", 1)[0]
        self.assertIn("publish_belly_quality(frames, &work)", workers)
        self.assertNotIn("latch_callback_failure", workers.split('if (frame_sink_', 1)[0])
        stop = source.split("  void stop_all_noexcept()", 1)[1]
        self.assertLess(stop.index("belly_observation_worker_->request_stop()"),
                        stop.index("belly_pipeline_->stop_noexcept()"))
        self.assertLess(stop.index("belly_pipeline_->stop_noexcept()"),
                        stop.index("belly_observation_worker_->join()"))

    def test_ae_target_recovery_precedes_capture_and_restores_after_join(self):
        source = SOURCE.read_text(encoding="utf-8")
        self.assertIn('declare_parameter("belly_ir_ae_target_override", -1, ae_descriptor)', source)
        self.assertIn('"/data/navigation/camera-property-journals"', source)
        bind = source.split("  void bind_and_start()", 1)[1].split("  std::shared_ptr<const CameraSourceIdentity>", 1)[0]
        self.assertLess(bind.index("configure_belly_ae_target(belly_device)"), bind.index("start_chest(chest_device)"))
        self.assertLess(bind.index("configure_belly_ae_target(belly_device)"), bind.index("start_belly(belly_device)"))
        configure = source.split("  void configure_belly_ae_target(", 1)[1].split("  void start_belly(", 1)[0]
        self.assertIn("OB_PERMISSION_READ", configure)
        self.assertIn("OB_PERMISSION_WRITE", configure)
        self.assertIn("getIntPropertyRange(OB_PROP_IR_BRIGHTNESS_INT)", configure)
        self.assertEqual(configure.count("setIntProperty("), 1)
        self.assertIn("setIntProperty(OB_PROP_IR_BRIGHTNESS_INT, value)", configure)
        self.assertNotIn("setBoolProperty(", configure)
        self.assertNotIn("setStructuredData(", configure)
        stop = source.split("  void stop_all_noexcept()", 1)[1]
        self.assertLess(stop.index("belly_pipeline_->stop_noexcept()"), stop.index("belly_ae_target_transaction_->restore_noexcept()"))
        self.assertLess(stop.index("belly_observation_worker_->join()"), stop.index("belly_ae_target_transaction_->restore_noexcept()"))

    def test_partial_constructor_stops_callbacks_before_member_destruction(self):
        source = SOURCE.read_text(encoding="utf-8")
        start = source.index("        try {\n          start_workers();")
        destructor = source.index("  ~CameraRuntimeNode()")
        guarded_start = source[start:destructor]
        self.assertIn("diagnostics_timer_ = create_wall_timer", guarded_start)
        self.assertIn("} catch (...) {", guarded_start)
        self.assertLess(guarded_start.index("stop_all_noexcept();"),
                        guarded_start.index("      throw;"))

    def test_cold_boot_timestamp_recovery_does_not_weaken_capture_clocks(self):
        source = SOURCE.read_text(encoding="utf-8")
        contract = (
            PACKAGE / "include" / "sunhorizon_camera_runtime" /
            "runtime_contract.hpp"
        ).read_text(encoding="utf-8")
        baseline = source.index("startup_steady_timestamp_ns_ = steady_now_ns()")
        pipeline_start = source.index("bind_and_start()", baseline)
        self.assertLess(baseline, pipeline_start)
        self.assertIn("evaluate_sdk_system_timestamp", source)
        self.assertIn("evaluate_capture_pair", source)
        self.assertIn("TimestampDecision::kRegression", contract)
        self.assertIn("TimestampRecoveryDecision::kStartupReanchor", contract)
        self.assertIn("CapturePairDecision::kFatalDeviceMismatch", contract)
        self.assertIn("CapturePairDecision::kFatalGlobalMismatch", contract)
        self.assertIn("startup_system_clock_reanchors", source)
        self.assertIn("startup_global_pair_drops", source)
        self.assertIn('"maximum_device_pair_delta_us"', source)
        self.assertIn('"maximum_global_pair_delta_us"', source)


if __name__ == "__main__":
    unittest.main()
