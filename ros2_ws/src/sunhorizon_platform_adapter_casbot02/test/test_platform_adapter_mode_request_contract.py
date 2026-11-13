#!/usr/bin/env python3
from pathlib import Path
import unittest


PACKAGE = Path(__file__).resolve().parents[1]


class PlatformAdapterModeRequestContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (PACKAGE / "src/platform_adapter_node.cpp").read_text(
            encoding="utf-8"
        )
        cls.config = (PACKAGE / "config/platform_adapter.yaml").read_text(
            encoding="utf-8"
        )

    def test_lost_response_is_expired_removed_then_retried(self):
        request_mode = self.source.split("  void request_mode() {", 1)[1].split(
            "\n  void on_timer() {", 1
        )[0]
        expire = request_mode.index("mode_request_lifecycle_.expire(")
        remove = request_mode.index("mode_client_->remove_pending_request(")
        ready_guard = request_mode.index("mode_request_lifecycle_.ready(now)")
        retry = request_mode.index("mode_client_->async_send_request(")
        self.assertLess(expire, remove)
        self.assertLess(remove, ready_guard)
        self.assertLess(ready_guard, retry)
        self.assertIn("pending.request_id", request_mode)
        self.assertNotIn("mode_request_pending_", self.source)

    def test_timeout_and_late_response_paths_are_fail_closed(self):
        self.assertIn(
            'declare_parameter("mode_request_timeout_sec", 0.50)', self.source
        )
        self.assertIn("!std::isfinite(mode_request_timeout_sec_)", self.source)
        self.assertIn("mode_request_timeout_sec_ > 5.0", self.source)
        self.assertIn("mode_request_timeout_sec: 0.50", self.config)
        self.assertIn(
            'declare_parameter("mode_request_period_sec", 0.20)', self.source
        )
        self.assertIn("!std::isfinite(mode_request_period_sec_)", self.source)
        self.assertIn(
            "mode_request_period_sec_ >= mode_timeout_sec_", self.source
        )
        self.assertIn("mode_request_period_sec: 0.20", self.config)
        self.assertIn('mode_name_ = "UNKNOWN";', self.source)
        self.assertIn("mode_allowed_ = false;", self.source)
        self.assertIn(
            "if (!mode_request_lifecycle_.complete(request_token)) return;",
            self.source,
        )

    def test_ethercat_timeout_is_separate_bounded_and_narrowly_applied(self):
        self.assertIn(
            'declare_parameter("ethercat_evidence_timeout_sec", 1.5)',
            self.source,
        )
        self.assertIn("valid_ethercat_evidence_timeout(", self.source)
        self.assertIn("ethercat_evidence_timeout_sec: 1.50", self.config)

        publish = self.source.split("  void on_timer() {", 1)[1].split(
            "\n  std::vector<int> leg_joint_ids_;", 1
        )[0]
        self.assertIn(
            "fresh(ethercat_stamp_, ethercat_evidence_timeout_sec_)", publish
        )
        self.assertIn("fresh(wkc_stamp_, ethercat_evidence_timeout_sec_)", publish)
        self.assertNotIn("fresh(ethercat_stamp_, evidence_timeout_sec_)", publish)
        self.assertNotIn("fresh(wkc_stamp_, evidence_timeout_sec_)", publish)
        self.assertNotIn("pmu_", self.source)
        self.assertNotIn("output.estop_clear", publish)
        self.assertIn("fresh(waist_stamp_, joint_state_timeout_sec_)", publish)
        self.assertIn("joint_fault_timeout_sec_", publish)


if __name__ == "__main__":
    unittest.main()
