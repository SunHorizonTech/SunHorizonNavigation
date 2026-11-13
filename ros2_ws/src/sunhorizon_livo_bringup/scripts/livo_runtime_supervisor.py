#!/usr/bin/env python3
"""Supervise live FAST-LIVO and fail the service when vendor odometry stalls.

The vendor process can enter an internal fail-stop without exiting.  This
process is therefore the systemd main process: it owns the ROS launch child,
observes progress of the volatile vendor odometry stream, and returns a
non-zero status after a bounded post-first-sample stall.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import os
import signal
import subprocess
import sys
import time
from typing import Optional, Sequence


EX_RUNTIME_FAIL_STOP = 70
EXPECTED_VENDOR_ODOMETRY_TOPIC = "/sunhorizon/livo/vendor/odom"


@dataclass(frozen=True)
class WatchdogDecision:
    armed: bool
    failed: bool
    reason: str


@dataclass(frozen=True)
class RuntimeExitDecision:
    exit_code: int
    reason: str


def runtime_exit_decision(
    signal_received: Optional[int],
    child_return_code: Optional[int],
    watchdog: WatchdogDecision,
) -> Optional[RuntimeExitDecision]:
    """Map every terminal runtime condition to an explicit process status."""
    if signal_received is not None:
        return RuntimeExitDecision(
            128 + signal_received,
            f"supervisor_signal:{signal_received}",
        )
    if child_return_code is not None:
        return RuntimeExitDecision(
            EX_RUNTIME_FAIL_STOP,
            f"localization_launch_exited:{child_return_code}",
        )
    if watchdog.failed:
        return RuntimeExitDecision(EX_RUNTIME_FAIL_STOP, watchdog.reason)
    return None


class VendorOdometryProgressGate:
    """A monotonic, latched gate for vendor odometry source-stamp progress."""

    def __init__(self, stale_timeout_sec: float) -> None:
        if not math.isfinite(stale_timeout_sec) or stale_timeout_sec <= 0.0:
            raise ValueError("stale_timeout_sec_must_be_positive_and_finite")
        self._stale_timeout_ns = int(stale_timeout_sec * 1_000_000_000)
        if self._stale_timeout_ns <= 0:
            raise ValueError("stale_timeout_sec_too_small")
        self._last_progress_receipt_ns: Optional[int] = None
        self._last_source_stamp_ns: Optional[int] = None
        self._failure_reason = ""

    @property
    def armed(self) -> bool:
        return self._last_progress_receipt_ns is not None

    @property
    def failed(self) -> bool:
        return bool(self._failure_reason)

    def _latch_stale(self, elapsed_ns: int) -> None:
        if self.failed:
            return
        elapsed_sec = max(0, elapsed_ns) / 1_000_000_000.0
        self._failure_reason = (
            f"vendor_odometry_progress_stale:{elapsed_sec:.3f}s"
        )

    def observe(self, receipt_ns: int, source_stamp_ns: int) -> WatchdogDecision:
        if receipt_ns < 0:
            raise ValueError("receipt_ns_must_be_nonnegative")
        if self.failed:
            return self.decision()

        if self._last_progress_receipt_ns is not None:
            elapsed_ns = receipt_ns - self._last_progress_receipt_ns
            if elapsed_ns < 0:
                self._failure_reason = "steady_clock_regressed"
                return self.decision()
            if elapsed_ns >= self._stale_timeout_ns:
                # A late sample cannot heal a gap which already crossed the
                # fail-stop boundary.
                self._latch_stale(elapsed_ns)
                return self.decision()

        # A zero stamp is not evidence that the vendor estimator has produced
        # a usable odometry sample, so it cannot arm or refresh the watchdog.
        if source_stamp_ns <= 0:
            return self.decision()

        if self._last_source_stamp_ns is None or (
            source_stamp_ns > self._last_source_stamp_ns
        ):
            self._last_source_stamp_ns = source_stamp_ns
            self._last_progress_receipt_ns = receipt_ns
        return self.decision()

    def evaluate(self, now_ns: int) -> WatchdogDecision:
        if now_ns < 0:
            raise ValueError("now_ns_must_be_nonnegative")
        if self.failed or self._last_progress_receipt_ns is None:
            return self.decision()
        elapsed_ns = now_ns - self._last_progress_receipt_ns
        if elapsed_ns < 0:
            self._failure_reason = "steady_clock_regressed"
        elif elapsed_ns >= self._stale_timeout_ns:
            self._latch_stale(elapsed_ns)
        return self.decision()

    def decision(self) -> WatchdogDecision:
        if self.failed:
            return WatchdogDecision(True, True, self._failure_reason)
        if self.armed:
            return WatchdogDecision(True, False, "monitoring_vendor_odometry")
        return WatchdogDecision(False, False, "waiting_for_first_vendor_odometry")


def _terminate_process_group(
    process: subprocess.Popen[bytes], interrupt_timeout_sec: float = 8.0
) -> None:
    """Stop ros2 launch and all of its children within a bounded interval."""
    stages = (
        (signal.SIGINT, interrupt_timeout_sec),
        (signal.SIGTERM, 2.0),
        (signal.SIGKILL, 1.0),
    )
    for signum, timeout_sec in stages:
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=timeout_sec)
            # The launch leader may exit before a child. Send no further
            # escalating signal when the process group is already gone.
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                break
        except subprocess.TimeoutExpired:
            continue
    try:
        process.wait(timeout=0.1)
    except subprocess.TimeoutExpired:
        pass


def _parse_arguments(arguments: Optional[Sequence[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--vendor-odometry-topic",
        default=EXPECTED_VENDOR_ODOMETRY_TOPIC,
    )
    parser.add_argument("--stale-timeout-sec", type=float, default=1.0)
    parser.add_argument("--check-period-sec", type=float, default=0.05)
    parser.add_argument("launch_command", nargs=argparse.REMAINDER)
    parsed = parser.parse_args(arguments)
    if parsed.launch_command[:1] == ["--"]:
        parsed.launch_command = parsed.launch_command[1:]
    if parsed.vendor_odometry_topic != EXPECTED_VENDOR_ODOMETRY_TOPIC:
        parser.error("vendor odometry topic is immutable for Robot A")
    if (
        not math.isfinite(parsed.stale_timeout_sec)
        or parsed.stale_timeout_sec <= 0.0
    ):
        parser.error("stale timeout must be positive and finite")
    if (
        not math.isfinite(parsed.check_period_sec)
        or parsed.check_period_sec <= 0.0
        or parsed.check_period_sec > parsed.stale_timeout_sec / 4.0
    ):
        parser.error("check period must be positive and no more than timeout / 4")
    if not parsed.launch_command:
        parser.error("a ros2 launch command is required after --")
    if parsed.launch_command[:3] != [
        "ros2",
        "launch",
        "sunhorizon_navigation_bringup",
    ]:
        parser.error("only the SunHorizon navigation launch command is allowed")
    return parsed


def run(arguments: argparse.Namespace) -> int:
    # ROS imports stay out of the pure gate so its boundary behavior is
    # deterministic and testable without a ROS installation.
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init(args=[])
    node = rclpy.create_node("sunhorizon_livo_runtime_supervisor")
    gate = VendorOdometryProgressGate(arguments.stale_timeout_sec)
    signal_received: Optional[int] = None
    child: Optional[subprocess.Popen[bytes]] = None

    def request_shutdown(signum: int, _frame: object) -> None:
        nonlocal signal_received
        signal_received = signum

    def on_odometry(message: Odometry) -> None:
        source_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        was_armed = gate.armed
        decision = gate.observe(time.monotonic_ns(), source_stamp_ns)
        if decision.armed and not was_armed:
            node.get_logger().info("vendor odometry progress watchdog armed")

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    subscription = node.create_subscription(
        Odometry,
        arguments.vendor_odometry_topic,
        on_odometry,
        qos,
    )
    previous_handlers = {
        signum: signal.getsignal(signum)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    for signum in previous_handlers:
        signal.signal(signum, request_shutdown)

    exit_code = EX_RUNTIME_FAIL_STOP
    try:
        child = subprocess.Popen(
            arguments.launch_command,
            start_new_session=True,
        )
        node.get_logger().info(
            "started supervised localization launch "
            f"pid={child.pid} stale_timeout={arguments.stale_timeout_sec:.3f}s"
        )
        while True:
            rclpy.spin_once(node, timeout_sec=arguments.check_period_sec)
            child_return_code = child.poll()
            decision = gate.evaluate(time.monotonic_ns())
            runtime_exit = runtime_exit_decision(
                signal_received,
                child_return_code,
                decision,
            )
            if runtime_exit is not None:
                exit_code = runtime_exit.exit_code
                node.get_logger().fatal(
                    f"{runtime_exit.reason}; terminating localization launch "
                    "for systemd recovery"
                )
                break
    except (OSError, RuntimeError) as exc:
        node.get_logger().fatal(f"localization runtime supervision failed: {exc}")
        exit_code = EX_RUNTIME_FAIL_STOP
    finally:
        if child is not None:
            _terminate_process_group(child)
        # Keep the subscription alive through child teardown so an already
        # latched failure cannot be confused with a discovery disappearance.
        del subscription
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


def main(arguments: Optional[Sequence[str]] = None) -> int:
    parsed = _parse_arguments(arguments)
    return run(parsed)


if __name__ == "__main__":
    sys.exit(main())
