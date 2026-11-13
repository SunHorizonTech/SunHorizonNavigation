"""Bounded, read-only readiness probe for the robot body IMU."""

from __future__ import annotations

from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import socket
import sys
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Imu

from .probe_core import RateTracker, VectorAccumulator


def _stamp_seconds(message: Imu) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def _enum_text(value: object) -> str:
    name = getattr(value, "name", None)
    return str(name if name is not None else value)


class BodyImuReadinessProbe(Node):
    """Observe body IMU health without sending serial commands or changing config."""

    def __init__(self) -> None:
        super().__init__("body_imu_readiness_probe")
        self.declare_parameter("topic", "/imu")
        self.declare_parameter("duration_sec", 20.0)
        self.declare_parameter("expected_frame", "")
        self.declare_parameter("expected_gravity_axis", "")
        self.declare_parameter("minimum_rate_hz", 100.0)
        self.declare_parameter("stationary_gravity_min", 8.0)
        self.declare_parameter("stationary_gravity_max", 11.5)
        self.declare_parameter("require_orientation", False)

        self.topic = str(self.get_parameter("topic").value)
        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.rate = RateTracker()
        self.acceleration = VectorAccumulator()
        self.angular_velocity = VectorAccumulator()
        self.frames: set[str] = set()
        self.orientation_samples = 0
        self.orientation_unavailable_samples = 0
        self.quaternion_norm_sum = 0.0
        self.done = False
        self._started_monotonic = time.monotonic()

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Imu, self.topic, self._on_imu, qos)
        self.create_timer(0.1, self._check_duration)
        self.get_logger().info(
            f"read-only body IMU probe started for {self.duration_sec:.1f}s: "
            f"topic={self.topic}"
        )

    def _on_imu(self, message: Imu) -> None:
        self.rate.observe(time.monotonic(), _stamp_seconds(message))
        self.frames.add(message.header.frame_id)
        self.acceleration.observe(
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        )
        self.angular_velocity.observe(
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )
        if message.orientation_covariance[0] == -1.0:
            self.orientation_unavailable_samples += 1
            return

        quaternion = message.orientation
        norm = math.sqrt(
            quaternion.x**2
            + quaternion.y**2
            + quaternion.z**2
            + quaternion.w**2
        )
        if math.isfinite(norm):
            self.orientation_samples += 1
            self.quaternion_norm_sum += norm

    def _check_duration(self) -> None:
        if time.monotonic() - self._started_monotonic >= self.duration_sec:
            self.done = True

    def report(self) -> dict[str, Any]:
        rate = self.rate.snapshot()
        acceleration = self.acceleration.snapshot()
        angular_velocity = self.angular_velocity.snapshot()
        publishers = self._publisher_report()
        checks: list[dict[str, Any]] = []

        def check(name: str, passed: bool, evidence: Any) -> None:
            checks.append(
                {
                    "name": name,
                    "status": "PASS" if passed else "FAIL",
                    "evidence": evidence,
                }
            )

        check(
            "single_publisher",
            len(publishers) == 1,
            {"count": len(publishers), "publishers": publishers},
        )
        check("messages_received", rate.samples > 0, asdict(rate))
        minimum_rate = float(self.get_parameter("minimum_rate_hz").value)
        check(
            "message_rate",
            rate.receive_rate_hz is not None and rate.receive_rate_hz >= minimum_rate,
            {"observed_hz": rate.receive_rate_hz, "minimum_hz": minimum_rate},
        )
        check(
            "header_monotonicity",
            rate.header_regressions == 0,
            {
                "regressions": rate.header_regressions,
                "duplicates": rate.header_duplicates,
            },
        )

        expected_frame = str(self.get_parameter("expected_frame").value)
        check(
            "frame",
            not expected_frame or self.frames == {expected_frame},
            {"observed": sorted(self.frames), "expected": expected_frame},
        )

        gravity_min = float(self.get_parameter("stationary_gravity_min").value)
        gravity_max = float(self.get_parameter("stationary_gravity_max").value)
        check(
            "stationary_gravity_magnitude",
            acceleration.mean_norm is not None
            and gravity_min <= acceleration.mean_norm <= gravity_max,
            {
                "acceleration": asdict(acceleration),
                "accepted_range": [gravity_min, gravity_max],
            },
        )
        expected_axis = str(self.get_parameter("expected_gravity_axis").value)
        check(
            "gravity_axis",
            not expected_axis or acceleration.dominant_axis == expected_axis,
            {"observed": acceleration.dominant_axis, "expected": expected_axis},
        )

        mean_quaternion_norm = (
            self.quaternion_norm_sum / self.orientation_samples
            if self.orientation_samples
            else None
        )
        require_orientation = bool(self.get_parameter("require_orientation").value)
        orientation_valid = (
            mean_quaternion_norm is not None
            and abs(mean_quaternion_norm - 1.0) <= 0.02
        )
        check(
            "orientation",
            orientation_valid or not require_orientation,
            {
                "required": require_orientation,
                "available_samples": self.orientation_samples,
                "unavailable_samples": self.orientation_unavailable_samples,
                "mean_quaternion_norm": mean_quaternion_norm,
            },
        )

        status = "FAIL" if any(item["status"] == "FAIL" for item in checks) else "PASS"
        return {
            "schema_version": 1,
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "host": socket.gethostname(),
            "status": status,
            "duration_sec": time.monotonic() - self._started_monotonic,
            "topic": self.topic,
            "angular_velocity": asdict(angular_velocity),
            "checks": checks,
        }

    def _publisher_report(self) -> list[dict[str, Any]]:
        publishers = []
        for endpoint in self.get_publishers_info_by_topic(self.topic):
            gid = getattr(endpoint, "endpoint_gid", b"")
            try:
                gid_bytes = bytes(gid)
                gid_text = gid_bytes.hex(".")
            except (TypeError, ValueError):
                gid_bytes = b""
                gid_text = str(gid)
            qos = endpoint.qos_profile
            publishers.append(
                {
                    "node_name": endpoint.node_name,
                    "node_namespace": endpoint.node_namespace,
                    "gid": gid_text,
                    "qos": {
                        "reliability": _enum_text(qos.reliability),
                        "durability": _enum_text(qos.durability),
                        "history": _enum_text(qos.history),
                        "depth": qos.depth,
                    },
                }
            )
        return publishers


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = BodyImuReadinessProbe()
    report: dict[str, Any]
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        report = node.report()
    except KeyboardInterrupt:
        report = node.report()
        report["status"] = "INTERRUPTED"
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2))
    sys.exit(0 if report["status"] == "PASS" else 2)


if __name__ == "__main__":
    main()
