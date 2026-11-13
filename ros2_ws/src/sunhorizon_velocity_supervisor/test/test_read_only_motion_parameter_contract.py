from pathlib import Path
import re
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_SOURCE = (PACKAGE_ROOT / "src" / "velocity_supervisor_node.cpp").read_text()
AUTHORITY_SOURCE = (
    PACKAGE_ROOT
    / "include/sunhorizon_velocity_supervisor/motion_authority_contract.hpp"
).read_text()
STATUS_MESSAGE = (
    PACKAGE_ROOT.parent
    / "sunhorizon_navigation_interfaces/msg/VelocitySupervisorStatus.msg"
).read_text()
ATTENDED_CONFIG = (
    PACKAGE_ROOT / "config" / "attended_five_cm_output.yaml"
).read_text()
ATTENDED_LAUNCH = (
    PACKAGE_ROOT / "launch" / "attended_five_cm_output.launch.py"
).read_text()
YAW_CONFIG = (
    PACKAGE_ROOT / "config" / "attended_yaw_output.yaml"
).read_text()
YAW_LAUNCH = (
    PACKAGE_ROOT / "launch" / "attended_yaw_output.launch.py"
).read_text()
FORWARD_CALIBRATION_CONFIG = (
    PACKAGE_ROOT / "config" / "attended_forward_calibration_output.yaml"
).read_text()
FORWARD_CALIBRATION_LAUNCH = (
    PACKAGE_ROOT / "launch" / "attended_forward_calibration_output.launch.py"
).read_text()
FIRST_CLICK_CONFIG = (
    PACKAGE_ROOT / "config" / "first_click_navigation.yaml"
).read_text()
ATTENDED_FIRST_CLICK_CONFIG = (
    PACKAGE_ROOT / "config" / "attended_first_click_navigation.yaml"
).read_text()


def _uses_read_only_descriptor(parameter_name: str) -> bool:
    declaration = re.compile(
        rf'declare_parameter<double>\(\s*"{re.escape(parameter_name)}"\s*,[^;]*'
        r"read_only_parameter\(",
        re.DOTALL,
    )
    return declaration.search(NODE_SOURCE) is not None


class MotionParameterContractTest(unittest.TestCase):
    def test_candidate_timeout_is_immutable_after_gate_construction(self):
        self.assertTrue(_uses_read_only_descriptor("candidate_timeout_sec"))

    def test_all_copied_safety_limits_are_immutable_after_gate_construction(self):
        for parameter_name in (
            "candidate_timeout_sec",
            "command_generation_timeout_sec",
            "platform_timeout_sec",
            "obstacle_coverage_timeout_sec",
            "localization_timeout_sec",
            "tf_timeout_sec",
            "navigation_lease_timeout_sec",
            "max_vx",
            "max_vy",
            "max_wz",
            "max_ax",
            "max_ay",
            "max_aw",
            "max_waist_yaw_rad",
            "attended_cross_track_stop_m",
            "attended_cross_track_observation_m",
        ):
            with self.subTest(parameter_name=parameter_name):
                self.assertTrue(_uses_read_only_descriptor(parameter_name))

    def test_control_rate_is_immutable_after_timer_construction(self):
        self.assertTrue(_uses_read_only_descriptor("control_rate_hz"))

    def test_release_sha_is_compiled_and_cannot_be_overridden(self):
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text()
        self.assertIn("SUNHORIZON_RELEASE_SHA", cmake)
        self.assertIn("target_compile_definitions", cmake)
        self.assertIn("SUNHORIZON_RELEASE_SHA_LENGTH EQUAL 40", cmake)
        self.assertIn('MATCHES "^[0-9a-f]+$"', cmake)
        self.assertNotIn("[0-9a-f]{40}", cmake)
        self.assertIn('"release_sha", SUNHORIZON_RELEASE_SHA', NODE_SOURCE)
        self.assertIn(
            "release_sha does not match compiled binary identity", NODE_SOURCE
        )

    def test_process_id_is_read_only_and_exposes_exact_linux_process(self):
        self.assertIn('"process_id", static_cast<int64_t>(::getpid())', NODE_SOURCE)
        self.assertIn("read_only_parameter(", NODE_SOURCE)

    def test_deployed_base_frame_is_base_footprint(self):
        config = (PACKAGE_ROOT / "config" / "velocity_supervisor.yaml").read_text()
        self.assertRegex(config, r"(?m)^\s*base_frame:\s*base_footprint\s*$")
        self.assertNotRegex(config, r"(?m)^\s*base_frame:\s*base_link\s*$")
        self.assertIn('declare_parameter("base_frame", "base_footprint")', NODE_SOURCE)

    def test_steady_clock_candidate_watchdog_latches_and_zeroes(self):
        self.assertIn("candidate_stale_steady_clock", NODE_SOURCE)
        self.assertIn("std::chrono::steady_clock::now()", NODE_SOURCE)
        self.assertRegex(
            NODE_SOURCE,
            r"if \(have_candidate_ && candidate_stale_by_steady_clock\(\)\)"
            r"[\s\S]*?enter_fault\(reason\);"
            r"[\s\S]*?publish_zero\(false, reason\);",
        )

    def test_state_contract_is_explicit_and_starts_disarmed(self):
        for constant in (
            "STATE_DISARMED=0",
            "STATE_ARMED_IDLE=1",
            "STATE_RUNNING=2",
            "STATE_FAULT_LATCHED=3",
            "uint8 state",
        ):
            self.assertIn(constant, STATUS_MESSAGE)
        self.assertIn(
            "SupervisorState state_{SupervisorState::kDisarmed}", NODE_SOURCE
        )
        self.assertIn('gate_.force_latch("startup_disarmed")', NODE_SOURCE)
        self.assertIn("status.state = static_cast<uint8_t>(state_)", NODE_SOURCE)

    def test_arm_without_candidate_clears_history_and_enters_idle(self):
        self.assertIn("reset_inputs.candidate_seen = false", NODE_SOURCE)
        self.assertIn("response->success = gate_.reset(reset_inputs, {}", NODE_SOURCE)
        self.assertRegex(
            NODE_SOURCE,
            r"candidate_ = \{\};[\s\S]*?have_candidate_ = false;"
            r"[\s\S]*?state_ = SupervisorState::kArmedIdle;"
            r"[\s\S]*?response->message = \"armed_idle\";",
        )
        self.assertIn("candidate_stale_by_steady_clock()", NODE_SOURCE)

    def test_reset_stages_fresh_tf_rebaseline_before_committing_it(self):
        update_tf = NODE_SOURCE[
            NODE_SOURCE.index("  void update_tf("):
            NODE_SOURCE.index("  void try_connect_output(")
        ]
        self.assertIn("tf_continuity_.observe(", update_tf)
        self.assertNotIn(".rebaseline(", update_tf)

        on_reset = NODE_SOURCE[
            NODE_SOURCE.index("  void on_reset("):
            NODE_SOURCE.index("  void on_stop(")
        ]
        stage = on_reset.index("TfContinuityTracker proposed_tf_continuity")
        rebaseline = on_reset.index("proposed_tf_continuity.rebaseline(")
        safety_reset = on_reset.index("gate_.reset(reset_inputs")
        commit = on_reset.index("tf_continuity_ = proposed_tf_continuity")
        self.assertLess(stage, rebaseline)
        self.assertLess(rebaseline, safety_reset)
        self.assertLess(safety_reset, commit)
        self.assertIn("reset_inputs.tf_stamp_sec", on_reset)
        self.assertIn("reset_inputs.tf_continuous = reset_tf_valid", on_reset)
        self.assertRegex(
            on_reset,
            r"gate_\.reset\(reset_inputs,[\s\S]*?"
            r"if \(!response->success\) return;[\s\S]*?"
            r"tf_continuity_ = proposed_tf_continuity",
        )

    def test_tf_lookup_rejects_non_finite_transform_before_rebaseline(self):
        lookup = NODE_SOURCE[
            NODE_SOURCE.index("latest_tf_observation()"):
            NODE_SOURCE.index("  void update_tf(")
        ]
        for component in (
            "stamp_sec",
            "x",
            "y",
            "z",
            "rotation.x",
            "rotation.y",
            "rotation.z",
            "rotation.w",
            "quaternion_norm_squared",
            "roll",
            "pitch",
            "yaw",
        ):
            with self.subTest(component=component):
                self.assertIn(f"std::isfinite({component})", lookup)
        self.assertIn("return std::nullopt", lookup)

    def test_live_output_has_host_lock_and_dds_publisher_guard(self):
        self.assertIn("if (motion_output_requested_) acquire_final_output_lock()", NODE_SOURCE)
        self.assertIn("LOCK_EX | LOCK_NB", NODE_SOURCE)
        self.assertIn("final_output_lock_already_held", NODE_SOURCE)
        self.assertIn("count_publishers(final_topic_)", NODE_SOURCE)
        self.assertIn("existing_final_velocity_publisher:", NODE_SOURCE)

    def test_collision_stop_and_source_failure_latch_on_first_audit(self):
        self.assertIn(
            '"collision_audit_topic", "/sunhorizon/navigation/collision_audit"',
            NODE_SOURCE,
        )
        self.assertIn("geometry_msgs::msg::TwistStamped", NODE_SOURCE)
        self.assertIn("parse_collision_audit_frame_id(input->header.frame_id)", NODE_SOURCE)
        self.assertIn("collision_audit_requires_immediate_latch(audit_class)", NODE_SOURCE)
        self.assertIn(
            "observe_candidate(input->twist, stamp_seconds(input->header.stamp))",
            NODE_SOURCE,
        )
        self.assertIn('"nav2_collision_stop"', NODE_SOURCE)
        self.assertIn('"nav2_collision_source_invalid"', NODE_SOURCE)
        self.assertNotIn("CollisionInterventionGate collision_intervention_gate_", NODE_SOURCE)
        self.assertRegex(
            NODE_SOURCE,
            r"void on_collision_audit[\s\S]*?"
            r"collision_audit_requires_immediate_latch\(audit_class\)"
            r"[\s\S]*?enter_fault\(reason\);"
            r"[\s\S]*?publish_zero\(false, reason\);",
        )

    def test_collision_audit_message_gid_and_endpoint_epoch_are_fail_closed(self):
        callback = NODE_SOURCE[
            NODE_SOURCE.index("void on_collision_audit"):
            NODE_SOURCE.index("void on_qualification_candidate")
        ]
        self.assertIn("const rclcpp::MessageInfo& message_info", callback)
        self.assertIn("message_info.get_rmw_message_info().publisher_gid", callback)
        self.assertIn("active_collision_endpoint_epoch_.audit_gid", callback)
        self.assertIn("collision_audit_message_gid_changed", callback)
        self.assertIn("invalidate_collision_endpoint_epoch", callback)
        self.assertIn("clear_candidate_history()", NODE_SOURCE)
        self.assertIn("publisher_endpoint_epoch_changed", NODE_SOURCE)
        self.assertIn("publisher_endpoint_epoch_invalid", NODE_SOURCE)
        self.assertIn("candidate_publisher_gid_invalid", NODE_SOURCE)
        self.assertIn("collision_audit_publisher_gid_invalid", NODE_SOURCE)
        self.assertIn("std::array<uint8_t, Size>", NODE_SOURCE)

    def test_attended_output_profile_is_fixed_forward_only_and_standalone(self):
        self.assertRegex(
            ATTENDED_CONFIG, r"(?m)^\s*motion_output_enabled:\s*true\s*$"
        )
        for parameter, value in (
            ("base_frame", "base_footprint"),
            ("motion_authority_mode", "attended_five_cm_qualification_v1"),
            (
                "qualification_profile_acknowledgement",
                "SUNHORIZON_ATTENDED_FIVE_CM_SUPERVISOR_V1",
            ),
            (
                "expected_candidate_publisher_fqn",
                "/sunhorizon_attended_five_cm_supervisor_output_probe",
            ),
            ("max_vx", "0.05"),
            ("max_vy", "0.0"),
            ("max_wz", "0.0"),
            ("autonomous_reverse_enabled", "false"),
        ):
            self.assertRegex(
                ATTENDED_CONFIG,
                rf"(?m)^\s*{parameter}:\s*{value}\s*$",
            )
        self.assertEqual(ATTENDED_LAUNCH.count("Node("), 1)
        self.assertIn('"attended_five_cm_output.yaml"', ATTENDED_LAUNCH)
        self.assertNotIn("nav2", ATTENDED_LAUNCH.lower())
        self.assertNotIn("cmd_vel_candidate", ATTENDED_LAUNCH)
        self.assertIn("velocity_calibration_accepted: false", ATTENDED_CONFIG)
        self.assertIn("vendor_watchdog_accepted: false", ATTENDED_CONFIG)

    def test_attended_profile_and_node_form_one_bootstrap_contract(self):
        for token in (
            "motion_authority_contract_failure(authority_contract)",
            "if (direct_qualification_candidate_mode_)",
            "on_qualification_candidate",
            "qualification_candidate_sub_",
            "safety_inputs_for_authority",
            "result.navigation_task_active = true",
            "qualification_collision_audit_publisher_count",
            "AttendedQualificationNonzeroBudget",
            "kAttendedQualificationBudgetExhaustedReason",
            "attended_five_cm_qualification_nonzero_budget_spent",
        ):
            self.assertIn(token, NODE_SOURCE)
        self.assertIn(
            "attended_five_cm_qualification_nonzero_budget_exhausted",
            AUTHORITY_SOURCE,
        )
        self.assertRegex(
            NODE_SOURCE,
            r"if \(!direct_qualification_candidate_mode_\) \{[\s\S]*?"
            r"navigation_status_missing_before_reset",
        )
        for parameter in (
            "motion_authority_mode",
            "qualification_profile_acknowledgement",
        ):
            self.assertRegex(
                NODE_SOURCE,
                rf'declare_parameter<std::string>\(\s*"{parameter}"[\s\S]*?'
                r"read_only_parameter\(",
            )

    def test_attended_yaw_profile_is_angular_only_and_standalone(self):
        for parameter, value in (
            ("motion_output_enabled", "true"),
            ("motion_authority_mode", "attended_yaw_qualification_v1"),
            (
                "qualification_profile_acknowledgement",
                "SUNHORIZON_ATTENDED_YAW_SUPERVISOR_V1",
            ),
            (
                "expected_candidate_publisher_fqn",
                "/sunhorizon_attended_yaw_supervisor_output_probe",
            ),
            ("max_vx", "0.0"),
            ("max_vy", "0.0"),
            ("max_wz", "0.03"),
            ("autonomous_reverse_enabled", "false"),
        ):
            self.assertRegex(YAW_CONFIG, rf"(?m)^\s*{parameter}:\s*{value}\s*$")
        self.assertEqual(YAW_LAUNCH.count("Node("), 1)
        self.assertIn('"attended_yaw_output.yaml"', YAW_LAUNCH)
        self.assertNotIn("nav2", YAW_LAUNCH.lower())
        self.assertIn("kAttendedYawQualificationMode", NODE_SOURCE)
        self.assertIn(
            "attended_yaw_qualification_nonzero_budget_exhausted", AUTHORITY_SOURCE
        )

    def test_attended_forward_calibration_profile_is_fixed_and_standalone(self):
        for parameter, value in (
            ("motion_output_enabled", "true"),
            ("motion_authority_mode", "attended_forward_calibration_v1"),
            (
                "qualification_profile_acknowledgement",
                "SUNHORIZON_ATTENDED_FORWARD_CALIBRATION_SUPERVISOR_V1",
            ),
            (
                "expected_candidate_publisher_fqn",
                "/sunhorizon_attended_forward_calibration_supervisor_output_probe",
            ),
            ("max_vx", "0.05"),
            ("max_vy", "0.0"),
            ("max_wz", "0.0"),
            ("autonomous_reverse_enabled", "false"),
        ):
            self.assertRegex(
                FORWARD_CALIBRATION_CONFIG,
                rf"(?m)^\s*{parameter}:\s*{value}\s*$",
            )
        self.assertEqual(FORWARD_CALIBRATION_LAUNCH.count("Node("), 1)
        self.assertIn(
            '"attended_forward_calibration_output.yaml"',
            FORWARD_CALIBRATION_LAUNCH,
        )
        self.assertNotIn("nav2", FORWARD_CALIBRATION_LAUNCH.lower())
        self.assertIn("kAttendedForwardCalibrationMode", NODE_SOURCE)
        self.assertIn(
            "AttendedQualificationNonzeroBudget::kForwardCalibrationHardLimit",
            NODE_SOURCE,
        )
        self.assertIn(
            "attended_forward_calibration_nonzero_budget_exhausted",
            AUTHORITY_SOURCE,
        )

    def test_forward_pulse_preserves_standard_gates_with_distinct_two_second_mode(self):
        config = (PACKAGE_ROOT / "config/attended_forward_pulse_output.yaml").read_text()
        old = dict(re.findall(r"^    (\w+): (.+)$", FORWARD_CALIBRATION_CONFIG, re.M))
        new = dict(re.findall(r"^    (\w+): (.+)$", config, re.M))
        expected = dict(old)
        expected.update({
            "motion_authority_mode": "attended_forward_pulse_v1",
            "qualification_profile_acknowledgement": "SUNHORIZON_ATTENDED_FORWARD_PULSE_SUPERVISOR_V1",
            "expected_candidate_publisher_fqn": "/sunhorizon_attended_forward_pulse",
            "tf_timeout_sec": "0.85",
        })
        self.assertEqual(new, expected)
        launch = (PACKAGE_ROOT / "launch/attended_forward_pulse_output.launch.py").read_text()
        self.assertEqual(launch.count("Node("), 1)
        self.assertIn('"attended_forward_pulse_output.yaml"', launch)
        self.assertIn("AttendedQualificationNonzeroBudget::kForwardPulseHardLimit", NODE_SOURCE)
        self.assertIn("kForwardPulseHardLimit = std::chrono::milliseconds(2000)", AUTHORITY_SOURCE)
        direct = NODE_SOURCE.split("direct_qualification_candidate_mode_ =", 1)[1].split(";", 1)[0]
        self.assertIn("kAttendedForwardPulseMode", direct)

    def test_attended_nonzero_budget_is_enforced_before_normal_output(self):
        observation = NODE_SOURCE[
            NODE_SOURCE.index("void observe_candidate"):
            NODE_SOURCE.index("void clear_candidate_history")
        ]
        control = NODE_SOURCE[
            NODE_SOURCE.index("void on_control()"):
            NODE_SOURCE.index("void on_reset(")
        ]
        reset = NODE_SOURCE[
            NODE_SOURCE.index("void on_reset("):
            NODE_SOURCE.index("void on_stop(")
        ]
        self.assertLess(
            observation.index("observe_first_nonzero"),
            observation.index("candidate_message_ = input"),
        )
        self.assertLess(
            control.index("qualification_nonzero_budget_.expired"),
            control.index("if (state_ == SupervisorState::kFaultLatched)"),
        )
        self.assertIn("enter_fault(reason)", control)
        self.assertIn("publish_zero(false, reason)", control)
        self.assertIn("qualification_nonzero_budget_.reset_allowed()", reset)

    def test_first_click_profile_starts_disabled_and_is_straight_only(self):
        for parameter, value in (
            ("motion_output_enabled", "false"),
            ("max_vx", "0.05"),
            ("max_vy", "0.0"),
            ("max_wz", "0.0"),
            ("autonomous_reverse_enabled", "false"),
        ):
            self.assertRegex(
                FIRST_CLICK_CONFIG,
                rf"(?m)^\s*{parameter}:\s*{value}\s*$",
            )
        self.assertIn(
            "collision_audit_topic: /sunhorizon/navigation/collision_audit",
            FIRST_CLICK_CONFIG,
        )
        self.assertIn(
            "motion_authority_mode: receipt_gated_navigation", FIRST_CLICK_CONFIG
        )

    def test_attended_first_click_uses_collision_audit_and_bounded_unaccepted_output(self):
        for parameter, value in (
            ("motion_output_enabled", "false"),
            ("motion_authority_mode", "attended_first_click_navigation_v1"),
            (
                "qualification_profile_acknowledgement",
                "SUNHORIZON_ATTENDED_FIRST_CLICK_NAVIGATION_V1",
            ),
            ("expected_candidate_publisher_fqn", "/collision_monitor"),
            ("velocity_calibration_accepted", "false"),
            ("vendor_watchdog_accepted", "false"),
            ("vendor_vx_gain", "1.0"),
            ("tf_timeout_sec", "0.85"),
            ("max_vx", "0.05"),
            ("max_vy", "0.0"),
            ("max_wz", "0.0"),
            ("autonomous_reverse_enabled", "false"),
        ):
            self.assertRegex(
                ATTENDED_FIRST_CLICK_CONFIG,
                rf"(?m)^\s*{parameter}:\s*{value}\s*$",
            )
        self.assertIn(
            "kFirstClickNavigationHardLimit", AUTHORITY_SOURCE
        )
        self.assertIn("std::chrono::milliseconds(45000)", AUTHORITY_SOURCE)
        self.assertIn(
            "motion_authority_mode_ == kAttendedFirstClickNavigationMode",
            NODE_SOURCE,
        )
        self.assertIn(
            "bounded_attended_output_mode_ = direct_qualification_candidate_mode_",
            NODE_SOURCE,
        )
        self.assertRegex(
            NODE_SOURCE,
            r"if \(direct_qualification_candidate_mode_\) \{[\s\S]*?"
            r"qualification_candidate_sub_[\s\S]*?else \{[\s\S]*?"
            r"collision_audit_sub_",
        )
        self.assertIn(
            "const bool navigation_lease_valid = direct_qualification_candidate_mode_",
            NODE_SOURCE,
        )

    def test_nonzero_motion_requires_navigation_server_lease(self):
        self.assertIn(
            "sunhorizon_navigation_interfaces/msg/navigation_status.hpp",
            NODE_SOURCE,
        )
        core = (
            PACKAGE_ROOT.parent / "sunhorizon_navigation_core/src/safety_gate.cpp"
        ).read_text()
        self.assertIn("navigation_execution_lease_missing", core)
        self.assertIn("input->state == NavigationStatus::STATE_NAVIGATING", NODE_SOURCE)
        self.assertIn("!input->task_id.empty()", NODE_SOURCE)
        self.assertIn("waiting_for_navigation_execution_lease", NODE_SOURCE)

    def test_idle_safety_and_endpoint_ownership_are_continuously_checked(self):
        self.assertIn(
            "SafetyInputs idle_inputs = safety_inputs_for_authority(now)", NODE_SOURCE
        )
        self.assertIn("gate_.evaluate(idle_inputs, {})", NODE_SOURCE)
        self.assertIn("get_publishers_info_by_topic(candidate_topic_)", NODE_SOURCE)
        self.assertIn("get_publishers_info_by_topic(collision_audit_topic_)", NODE_SOURCE)
        self.assertIn("get_publishers_info_by_topic(final_topic_)", NODE_SOURCE)
        self.assertIn("publisher_contract:", NODE_SOURCE)
        self.assertIn("expected_candidate_publisher_fqn", FIRST_CLICK_CONFIG)

    def test_live_output_requires_calibration_and_watchdog_receipts(self):
        self.assertIn(
            "live_output_requires_velocity_and_watchdog_receipts", AUTHORITY_SOURCE
        )
        self.assertIn("kReceiptGatedNavigationMode", NODE_SOURCE)
        self.assertIn("output.linear.x *= vendor_vx_gain_", NODE_SOURCE)
        self.assertIn("velocity_calibration_accepted: false", FIRST_CLICK_CONFIG)
        self.assertIn("vendor_watchdog_accepted: false", FIRST_CLICK_CONFIG)

    def test_nonfinite_or_unbounded_safety_parameters_are_rejected(self):
        self.assertIn("const bool timeouts_valid", NODE_SOURCE)
        self.assertIn("const bool motion_limits_valid", NODE_SOURCE)
        self.assertIn("axis_limits_valid", NODE_SOURCE)
        self.assertIn("velocity == 0.0 || acceleration > 0.0", NODE_SOURCE)
        self.assertIn("safety_limits_must_be_finite_and_bounded", NODE_SOURCE)


if __name__ == "__main__":
    unittest.main()
