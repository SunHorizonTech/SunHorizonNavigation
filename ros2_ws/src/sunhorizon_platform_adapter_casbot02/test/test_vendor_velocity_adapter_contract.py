from pathlib import Path
import re
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config" / "platform_adapter.yaml"
SOURCE = PACKAGE / "src" / "vendor_velocity_adapter_node.cpp"
LAUNCH = PACKAGE / "launch" / "platform_adapter.launch.py"


def yaml_scalar(text, key):
    match = re.search(rf"^\s*{re.escape(key)}:\s*([^#\n]*)", text, re.MULTILINE)
    if not match:
        raise AssertionError(f"missing YAML key: {key}")
    return match.group(1).strip().strip('"')


class VendorVelocityAdapterContractTest(unittest.TestCase):
    def setUp(self):
        self.config = CONFIG.read_text(encoding="utf-8")
        self.source = SOURCE.read_text(encoding="utf-8")
        self.launch = LAUNCH.read_text(encoding="utf-8")

    def test_default_contract_is_disconnected_and_unclaimed(self):
        self.assertEqual(yaml_scalar(self.config, "vendor_contract_accepted"), "false")
        self.assertEqual(yaml_scalar(self.config, "vendor_velocity_topic"), "")
        self.assertEqual(yaml_scalar(self.config, "vendor_velocity_message_type"), "")
        self.assertEqual(yaml_scalar(self.config, "vendor_velocity_semantics"), "")
        self.assertEqual(yaml_scalar(self.config, "vendor_qos_reliability"), "")
        self.assertEqual(yaml_scalar(self.config, "vendor_qos_durability"), "")
        self.assertEqual(yaml_scalar(self.config, "vendor_qos_depth"), "0")
        self.assertEqual(yaml_scalar(self.config, "vendor_watchdog_timeout_sec"), "-1.0")
        self.assertEqual(yaml_scalar(self.config, "vendor_contract_receipt_sha256"), "")

    def test_bridge_is_distinct_from_supervisor_and_requires_evidence(self):
        self.assertEqual(
            yaml_scalar(self.config, "supervised_input_topic"), "/navigation/cmd_vel"
        )
        self.assertIn("is_lower_hex_sha256(receipt_sha256)", self.source)
        self.assertIn('message_type != kSupportedMessageType', self.source)
        self.assertIn('semantics != kSupportedSemantics', self.source)
        self.assertIn('vendor_watchdog_timeout_sec_ <= 0.0', self.source)
        self.assertIn(
            "gate_.input_timeout_sec() > vendor_watchdog_timeout_sec_", self.source
        )
        self.assertIn(
            "gate_.platform_timeout_sec() > vendor_watchdog_timeout_sec_", self.source
        )
        output_guard = self.source.index(
            "if (!vendor_output_topic_allowed(resolved_output_topic))"
        )
        disabled_return = self.source.index("if (!contract_accepted_)")
        self.assertLess(output_guard, disabled_return)
        self.assertIn(
            "get_node_topics_interface()->resolve_topic_name(output_topic_)",
            self.source,
        )
        self.assertIn(
            "get_node_topics_interface()->resolve_topic_name(input_topic_)",
            self.source,
        )
        self.assertIn(
            "resolved_vendor_topics_distinct("
            "resolved_input_topic, resolved_output_topic)",
            self.source,
        )
        self.assertNotIn("input_topic_ == output_topic_", self.source)
        self.assertIn(
            "must never publish the project final velocity topic", self.source
        )

    def test_disabled_path_creates_no_command_endpoints(self):
        disabled = self.source.index("if (!contract_accepted_)")
        first_subscription = self.source.index("create_subscription<geometry_msgs::msg::Twist>")
        first_publisher = self.source.index("create_publisher<geometry_msgs::msg::Twist>")
        self.assertLess(disabled, first_subscription)
        self.assertLess(disabled, first_publisher)
        self.assertIn("return;", self.source[disabled:first_subscription])

    def test_adapter_never_changes_mode_or_publishes_joints(self):
        self.assertNotIn("SetRobotMode", self.source)
        self.assertNotIn("set_robot_mode", self.source)
        self.assertNotIn("JointState", self.source)
        self.assertNotIn("joint_command", self.source)

    def test_fail_closed_checks_and_zero_output_are_present(self):
        for token in (
            "input_publishers != 1U",
            "existing_output_publishers != 0U",
            "navigation_mode_discovered",
            "gate_.evaluate",
            "geometry_msgs::msg::Twist{}",
            "reset_vendor_velocity_latch",
        ):
            self.assertIn(token, self.source)

    def test_default_launch_includes_only_disconnected_contract(self):
        self.assertIn('executable="vendor_velocity_adapter_node"', self.launch)
        self.assertIn('name="sunhorizon_vendor_velocity_adapter"', self.launch)


if __name__ == "__main__":
    unittest.main()
