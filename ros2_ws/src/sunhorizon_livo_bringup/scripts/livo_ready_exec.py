#!/usr/bin/env python3
"""Wait for fresh LIVO inputs, then replace this process with FAST-LIVO2."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import os
import sys
import threading
import time
from typing import Dict, Sequence


NSEC_PER_SEC = 1_000_000_000


@dataclass
class StreamState:
    minimum_samples: int
    samples: int = 0
    last_stamp_ns: int | None = None
    last_receipt_ns: int | None = None
    valid_since_receipt_ns: int | None = None
    invalid_reason: str = ""


class ReadinessTracker:
    """Pure state machine used by the ROS gate and deterministic unit tests."""

    def __init__(
        self,
        requirements: Dict[str, int],
        maximum_age_sec: float,
        maximum_future_sec: float,
        stable_window_sec: float = 0.0,
    ) -> None:
        if not requirements or any(value <= 0 for value in requirements.values()):
            raise ValueError("requirements_must_be_positive")
        if not math.isfinite(maximum_age_sec) or maximum_age_sec <= 0.0:
            raise ValueError("maximum_age_sec_must_be_positive")
        if not math.isfinite(maximum_future_sec) or maximum_future_sec < 0.0:
            raise ValueError("maximum_future_sec_must_be_nonnegative")
        if not math.isfinite(stable_window_sec) or stable_window_sec < 0.0:
            raise ValueError("stable_window_sec_must_be_nonnegative")
        self._states = {
            name: StreamState(minimum_samples=count)
            for name, count in requirements.items()
        }
        self._maximum_age_ns = int(maximum_age_sec * NSEC_PER_SEC)
        self._maximum_future_ns = int(maximum_future_sec * NSEC_PER_SEC)
        self._stable_window_ns = int(stable_window_sec * NSEC_PER_SEC)

    @property
    def states(self) -> Dict[str, StreamState]:
        return self._states

    def record(self, stream: str, stamp_ns: int, receipt_ns: int) -> None:
        state = self._states[stream]
        if stamp_ns <= 0:
            state.invalid_reason = "nonpositive_stamp"
            state.samples = 0
            state.valid_since_receipt_ns = None
            return
        if state.last_stamp_ns is not None and stamp_ns <= state.last_stamp_ns:
            state.invalid_reason = "nonmonotonic_stamp"
            state.samples = 0
            state.valid_since_receipt_ns = None
            return
        age_ns = receipt_ns - stamp_ns
        if age_ns > self._maximum_age_ns:
            state.invalid_reason = "stale_stamp"
            state.samples = 0
            state.valid_since_receipt_ns = None
            return
        if age_ns < -self._maximum_future_ns:
            state.invalid_reason = "future_stamp"
            state.samples = 0
            state.valid_since_receipt_ns = None
            return
        receipt_gap = (
            state.last_receipt_ns is not None
            and receipt_ns - state.last_receipt_ns > self._maximum_age_ns
        )
        stamp_gap = (
            state.last_stamp_ns is not None
            and stamp_ns - state.last_stamp_ns > self._maximum_age_ns
        )
        if receipt_gap or stamp_gap:
            state.samples = 0
            state.valid_since_receipt_ns = None
        state.invalid_reason = ""
        if state.samples == 0:
            state.valid_since_receipt_ns = receipt_ns
        state.samples += 1
        state.last_stamp_ns = stamp_ns
        state.last_receipt_ns = receipt_ns

    def invalidate(self, stream: str, reason: str) -> None:
        state = self._states[stream]
        state.invalid_reason = reason
        state.samples = 0
        state.valid_since_receipt_ns = None

    def ready(self, now_ns: int) -> bool:
        for state in self._states.values():
            if state.invalid_reason or state.samples < state.minimum_samples:
                return False
            if state.last_receipt_ns is None or state.last_stamp_ns is None:
                return False
            if state.valid_since_receipt_ns is None:
                return False
            if now_ns - state.valid_since_receipt_ns < self._stable_window_ns:
                return False
            if now_ns - state.last_receipt_ns > self._maximum_age_ns:
                state.invalid_reason = "receipt_gap"
                state.samples = 0
                state.valid_since_receipt_ns = None
                return False
            stamp_age_ns = now_ns - state.last_stamp_ns
            if stamp_age_ns > self._maximum_age_ns:
                return False
            if stamp_age_ns < -self._maximum_future_ns:
                return False
        return True

    def summary(self) -> str:
        parts = []
        for name, state in self._states.items():
            detail = f"{name}={state.samples}/{state.minimum_samples}"
            if state.invalid_reason:
                detail += f"({state.invalid_reason})"
            parts.append(detail)
        return ", ".join(parts)


def split_gate_and_command(argv: Sequence[str]) -> tuple[list[str], list[str]]:
    try:
        separator = argv.index("--")
    except ValueError as error:
        raise ValueError("vendor_command_separator_missing") from error
    gate_args = list(argv[:separator])
    command = list(argv[separator + 1 :])
    if not command:
        raise ValueError("vendor_command_missing")
    return gate_args, command


def parse_args(argv: Sequence[str]) -> tuple[argparse.Namespace, list[str]]:
    gate_args, command = split_gate_and_command(argv)
    parser = argparse.ArgumentParser(
        description="Fail-closed LIVO input readiness gate")
    parser.add_argument("--points-topic", required=True)
    parser.add_argument("--imu-topic", required=True)
    parser.add_argument("--image-topic", required=True)
    parser.add_argument("--airy-diagnostics-topic", default="")
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    parser.add_argument("--maximum-age-sec", type=float, default=0.5)
    parser.add_argument("--maximum-future-sec", type=float, default=0.1)
    parser.add_argument("--stable-window-sec", type=float, default=2.0)
    parser.add_argument("--point-samples", type=int, default=3)
    parser.add_argument("--imu-samples", type=int, default=20)
    parser.add_argument("--image-samples", type=int, default=2)
    parser.add_argument("--use-sim-time", choices=("true", "false"), default="false")
    parser.add_argument(
        "--reliable-sensor-qos", choices=("true", "false"), default="false")
    parser.add_argument(
        "--reliable-image-qos", choices=("true", "false"), default="true")
    args = parser.parse_args(gate_args)
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0.0:
        parser.error("--timeout-sec must be finite and positive")
    return args, command


def stamp_to_ns(message: object) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * NSEC_PER_SEC + int(stamp.nanosec)


def run_gate(args: argparse.Namespace, command: Sequence[str]) -> int:
    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from sensor_msgs.msg import Image, Imu, PointCloud2

    # The remainder of this process command line belongs to FAST-LIVO2 and may
    # contain its own --ros-args block. Never let the readiness node consume it.
    rclpy.init(args=[])
    node = Node(
        "sunhorizon_livo_input_gate",
        parameter_overrides=[
            Parameter(
                "use_sim_time",
                Parameter.Type.BOOL,
                args.use_sim_time == "true",
            )
        ],
    )
    requirements = {
        "points": args.point_samples,
        "imu": args.imu_samples,
        "image": args.image_samples,
    }
    if args.airy_diagnostics_topic:
        requirements["airy_transport"] = 2
    tracker = ReadinessTracker(
        requirements,
        args.maximum_age_sec,
        args.maximum_future_sec,
        args.stable_window_sec,
    )
    tracker_lock = threading.Lock()

    def qos(depth: int, reliable: bool) -> QoSProfile:
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=(
                ReliabilityPolicy.RELIABLE if reliable
                else ReliabilityPolicy.BEST_EFFORT),
            durability=DurabilityPolicy.VOLATILE,
        )

    def callback(stream: str):
        def receive(message: object) -> None:
            with tracker_lock:
                tracker.record(
                    stream,
                    stamp_to_ns(message),
                    node.get_clock().now().nanoseconds,
                )
        return receive

    def receive_airy_diagnostics(message: DiagnosticArray) -> None:
        matching = [
            status for status in message.status
            if status.name == "sunhorizon_airy_adapter"
        ]
        reason = "airy_diagnostics_status_missing"
        valid = False
        if len(matching) == 1:
            status = matching[0]
            values = {entry.key: entry.value for entry in status.values}
            required = {
                "header_time_offset_initialized": "true",
                "reliable_input_sensor_qos": "true",
                "reliable_output_sensor_qos": "true",
                "live_input_qos_promotion_pending": "false",
                "reliable_live_ready": "true",
            }
            valid = (
                status.level == DiagnosticStatus.OK
                and status.message == "ok"
                and all(values.get(key) == value for key, value in required.items())
            )
            reason = "airy_reliable_live_transport_not_ready"
        with tracker_lock:
            if valid:
                tracker.record(
                    "airy_transport",
                    stamp_to_ns(message),
                    node.get_clock().now().nanoseconds,
                )
            else:
                tracker.invalidate("airy_transport", reason)

    # Large PointCloud2 and Image samples must not serialize all callbacks behind
    # the high-rate IMU stream. Separate callback groups let the multithreaded
    # executor deserialize and dispatch the three readiness inputs independently.
    point_callback_group = MutuallyExclusiveCallbackGroup()
    imu_callback_group = MutuallyExclusiveCallbackGroup()
    image_callback_group = MutuallyExclusiveCallbackGroup()
    diagnostics_callback_group = MutuallyExclusiveCallbackGroup()
    subscriptions: list[object] = [
        node.create_subscription(
            PointCloud2, args.points_topic, callback("points"),
            qos(32 if args.reliable_sensor_qos == "true" else 4,
                args.reliable_sensor_qos == "true"),
            callback_group=point_callback_group),
        node.create_subscription(
            Imu, args.imu_topic, callback("imu"),
            qos(2000 if args.reliable_sensor_qos == "true" else 100,
                args.reliable_sensor_qos == "true"),
            callback_group=imu_callback_group),
        node.create_subscription(
            Image, args.image_topic, callback("image"),
            qos(1 if args.reliable_image_qos == "true" else 4,
                args.reliable_image_qos == "true"),
            callback_group=image_callback_group),
    ]
    if args.airy_diagnostics_topic:
        subscriptions.append(node.create_subscription(
            DiagnosticArray, args.airy_diagnostics_topic,
            receive_airy_diagnostics, qos(10, True),
            callback_group=diagnostics_callback_group))
    # Keep subscriptions alive until the process is replaced by the backend.
    assert subscriptions

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    cleaned_up = False

    def cleanup() -> None:
        nonlocal cleaned_up
        if cleaned_up:
            return
        executor.remove_node(node)
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cleaned_up = True

    deadline = time.monotonic() + args.timeout_sec
    last_log = 0.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            now_ns = node.get_clock().now().nanoseconds
            with tracker_lock:
                inputs_ready = tracker.ready(now_ns)
                tracker_summary = tracker.summary()
            if inputs_ready:
                node.get_logger().info(
                    f"LIVO inputs ready: {tracker_summary}; exec {command[0]}")
                cleanup()
                os.execv(command[0], list(command))
            now = time.monotonic()
            if now - last_log >= 2.0:
                node.get_logger().info(
                    f"Waiting for LIVO inputs: {tracker_summary}")
                last_log = now
    except KeyboardInterrupt:
        return 130
    finally:
        cleanup()

    with tracker_lock:
        tracker_summary = tracker.summary()
    print(
        f"LIVO input readiness timed out after {args.timeout_sec:.1f}s: "
        f"{tracker_summary}",
        file=sys.stderr,
    )
    return 2


def main(argv: Sequence[str] | None = None) -> int:
    try:
        args, command = parse_args(sys.argv[1:] if argv is None else argv)
        return run_gate(args, command)
    except ValueError as error:
        print(str(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
