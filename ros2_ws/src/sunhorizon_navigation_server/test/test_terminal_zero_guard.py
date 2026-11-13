"""Compile the terminal guard and real navigation state machine without ROS.

The goal17 case reproduces its observed stop-before-finish ordering. Negative
cases exercise independent safety faults and ACK failure, not a live motion test.
"""
import os
from pathlib import Path
import platform
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class TerminalZeroGuardTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.directory.cleanup)
        cls.executable = Path(cls.directory.name) / "terminal_zero_guard"
        command = [os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra", "-Werror"]
        if (platform.system() == "Darwin" and not Path(
                "/Library/Developer/CommandLineTools/usr/include/c++/v1/string").is_file()):
            sdk = subprocess.check_output(["xcrun", "--show-sdk-path"], text=True).strip()
            command += ["-isystem", f"{sdk}/usr/include/c++/v1"]
        core = ROOT.parent / "sunhorizon_navigation_core"
        command += [f"-I{ROOT / 'include'}", f"-I{core / 'include'}",
                    str(ROOT / "test/terminal_zero_guard_harness.cpp"),
                    str(core / "src/navigation_state_machine.cpp"),
                    "-o", str(cls.executable)]
        subprocess.run(command, check=True, capture_output=True, text=True)

    def run_case(self, case):
        subprocess.run([str(self.executable), case], check=True, capture_output=True, text=True)

    def test_goal17_mppi_abort_retains_original_terminal_reason(self):
        self.run_case("failure")

    def test_success_also_finishes_after_expected_stop_and_ack(self):
        self.run_case("success")

    def test_stop_without_terminal_context_remains_external(self):
        self.run_case("normal_stop")

    def test_other_safety_fault_wins_over_own_stop(self):
        self.run_case("external_fault")

    def test_explicit_cancel_and_localization_loss_win(self):
        self.run_case("cancel_and_localization")

    def test_status_alone_or_wrong_ack_cannot_finish(self):
        self.run_case("ack_failure")

    def test_task_timestamps_and_ack_deadline_bound_deferral(self):
        self.run_case("task_and_clock_scope")

    def test_old_task_cannot_clear_or_reuse_new_context(self):
        self.run_case("clear_and_reuse")


if __name__ == "__main__":
    unittest.main()
