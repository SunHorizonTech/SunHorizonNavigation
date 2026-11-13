"""Bounded, read-only Airy PointCloud2 and IMU readiness probe."""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import json
import socket
import struct
import sys
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2

from .probe_core import (
    RateTracker,
    TimestampAnalysis,
    VectorAccumulator,
    analyze_point_timestamps,
)


def _stamp_seconds(message: PointCloud2 | Imu) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def _enum_text(value: object) -> str:
    name = getattr(value, "name", None)
    return str(name if name is not None else value)


class AiryReadinessProbe(Node):
    """Collect a finite readiness report without republishing or changing state."""

    def __init__(self) -> None:
        super().__init__("airy_readiness_probe")
        self.declare_parameter("points_topic", "/sunhorizon/raw/airy/points")
        self.declare_parameter("imu_topic", "/sunhorizon/raw/airy/imu")
        self.declare_parameter("duration_sec", 15.0)
        self.declare_parameter("point_analysis_stride", 100)
        self.declare_parameter("timestamp_field", "timestamp")
        self.declare_parameter("expected_timestamp_mode", "")
        self.declare_parameter("expected_point_frame", "rslidar")
        self.declare_parameter("expected_imu_frame", "imu_link")
        self.declare_parameter("expected_gravity_axis", "x")
        self.declare_parameter("minimum_cloud_rate_hz", 9.0)
        self.declare_parameter("minimum_imu_rate_hz", 190.0)
        self.declare_parameter("stationary_gravity_min", 8.0)
        self.declare_parameter("stationary_gravity_max", 11.5)
        self.declare_parameter("expected_cloud_width", 96)
        self.declare_parameter("expected_point_step", 0)
        self.declare_parameter("minimum_cloud_height", 850)
        self.declare_parameter("maximum_cloud_height", 950)
        self.declare_parameter("cloud_warmup_messages", 5)
        self.declare_parameter("minimum_accepted_shape_ratio", 0.99)

        self.points_topic = str(self.get_parameter("points_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.point_analysis_stride = max(
            1, int(self.get_parameter("point_analysis_stride").value)
        )
        self.timestamp_field = str(self.get_parameter("timestamp_field").value)

        self.cloud_rate = RateTracker()
        self.imu_rate = RateTracker()
        self.acceleration = VectorAccumulator()
        self.angular_velocity = VectorAccumulator()
        self.cloud_frames: set[str] = set()
        self.imu_frames: set[str] = set()
        self.cloud_shapes: set[tuple[int, int, int, int]] = set()
        self.cloud_shape_counts: Counter[tuple[int, int, int, int]] = Counter()
        self.orientation_unavailable_samples = 0
        self.timestamp_analysis: Optional[TimestampAnalysis] = None
        self.timestamp_errors: list[str] = []
        self._cloud_messages = 0
        self._started_monotonic = time.monotonic()
        self.done = False

        self.create_subscription(
            PointCloud2,
            self.points_topic,
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            self.imu_topic,
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self._check_duration)

        self.get_logger().info(
            f"read-only probe started for {self.duration_sec:.1f}s: "
            f"points={self.points_topic}, imu={self.imu_topic}"
        )

    def _on_cloud(self, message: PointCloud2) -> None:
        received_at = time.monotonic()
        header_time = _stamp_seconds(message)
        self.cloud_rate.observe(received_at, header_time)
        self._cloud_messages += 1
        self.cloud_frames.add(message.header.frame_id)
        self.cloud_shapes.add(
            (message.height, message.width, message.point_step, message.row_step)
        )
        cloud_warmup_messages = int(
            self.get_parameter("cloud_warmup_messages").value
        )
        if self._cloud_messages > cloud_warmup_messages:
            self.cloud_shape_counts[
                (message.height, message.width, message.point_step, message.row_step)
            ] += 1

        if self._cloud_messages % self.point_analysis_stride != 0:
            return

        field = next(
            (item for item in message.fields if item.name == self.timestamp_field),
            None,
        )
        if field is None:
            self.timestamp_errors.append(
                f"PointCloud2 field {self.timestamp_field!r} is missing"
            )
            return
        if field.count != 1:
            self.timestamp_errors.append(
                f"timestamp field count is {field.count}, expected 1"
            )
            return

        try:
            self.timestamp_analysis = analyze_point_timestamps(
                message.data,
                width=message.width,
                height=message.height,
                point_step=message.point_step,
                row_step=message.row_step,
                field_offset=field.offset,
                field_datatype=field.datatype,
                is_bigendian=message.is_bigendian,
                header_time=header_time,
            )
        except (ValueError, struct.error) as exc:
            self.timestamp_errors.append(str(exc))

    def _on_imu(self, message: Imu) -> None:
        received_at = time.monotonic()
        self.imu_rate.observe(received_at, _stamp_seconds(message))
        self.imu_frames.add(message.header.frame_id)
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

    def _check_duration(self) -> None:
        if time.monotonic() - self._started_monotonic >= self.duration_sec:
            self.done = True

    def report(self) -> dict[str, Any]:
        cloud_snapshot = self.cloud_rate.snapshot()
        imu_snapshot = self.imu_rate.snapshot()
        acceleration = self.acceleration.snapshot()
        angular_velocity = self.angular_velocity.snapshot()
        checks: list[dict[str, Any]] = []

        def check(
            name: str, passed: bool, evidence: Any, severity: str = "FAIL"
        ) -> None:
            checks.append(
                {
                    "name": name,
                    "status": "PASS" if passed else severity,
                    "evidence": evidence,
                }
            )

        point_publishers = self._publisher_report(self.points_topic)
        imu_publishers = self._publisher_report(self.imu_topic)
        check(
            "single_pointcloud_publisher",
            len(point_publishers) == 1,
            {"count": len(point_publishers), "publishers": point_publishers},
        )
        check(
            "single_imu_publisher",
            len(imu_publishers) == 1,
            {"count": len(imu_publishers), "publishers": imu_publishers},
        )
        check(
            "pointcloud_received",
            cloud_snapshot.samples > 0,
            asdict(cloud_snapshot),
        )
        check("imu_received", imu_snapshot.samples > 0, asdict(imu_snapshot))

        minimum_cloud_rate = float(
            self.get_parameter("minimum_cloud_rate_hz").value
        )
        minimum_imu_rate = float(self.get_parameter("minimum_imu_rate_hz").value)
        check(
            "pointcloud_rate",
            cloud_snapshot.receive_rate_hz is not None
            and cloud_snapshot.receive_rate_hz >= minimum_cloud_rate,
            {
                "observed_hz": cloud_snapshot.receive_rate_hz,
                "minimum_hz": minimum_cloud_rate,
            },
        )
        check(
            "imu_rate",
            imu_snapshot.receive_rate_hz is not None
            and imu_snapshot.receive_rate_hz >= minimum_imu_rate,
            {
                "observed_hz": imu_snapshot.receive_rate_hz,
                "minimum_hz": minimum_imu_rate,
            },
        )
        check(
            "message_header_monotonicity",
            cloud_snapshot.header_regressions == 0
            and imu_snapshot.header_regressions == 0
            and cloud_snapshot.header_duplicates == 0
            and imu_snapshot.header_duplicates == 0,
            {
                "pointcloud_regressions": cloud_snapshot.header_regressions,
                "imu_regressions": imu_snapshot.header_regressions,
                "pointcloud_duplicates": cloud_snapshot.header_duplicates,
                "imu_duplicates": imu_snapshot.header_duplicates,
            },
        )

        expected_width = int(self.get_parameter("expected_cloud_width").value)
        expected_point_step = int(self.get_parameter("expected_point_step").value)
        minimum_height = int(self.get_parameter("minimum_cloud_height").value)
        maximum_height = int(self.get_parameter("maximum_cloud_height").value)
        shape_samples = sum(self.cloud_shape_counts.values())
        accepted_shape_samples = sum(
            count
            for shape, count in self.cloud_shape_counts.items()
            if minimum_height <= shape[0] <= maximum_height
            and shape[1] == expected_width
            and (expected_point_step <= 0 or shape[2] == expected_point_step)
        )
        accepted_shape_ratio = (
            accepted_shape_samples / shape_samples if shape_samples else None
        )
        minimum_accepted_shape_ratio = float(
            self.get_parameter("minimum_accepted_shape_ratio").value
        )
        check(
            "cloud_shape_bounds",
            accepted_shape_ratio is not None
            and accepted_shape_ratio >= minimum_accepted_shape_ratio,
            {
                "expected_width": expected_width,
                "expected_point_step": expected_point_step,
                "minimum_height": minimum_height,
                "maximum_height": maximum_height,
                "samples_after_warmup": shape_samples,
                "accepted_shape_samples": accepted_shape_samples,
                "accepted_shape_ratio": accepted_shape_ratio,
                "minimum_accepted_shape_ratio": minimum_accepted_shape_ratio,
                "shape_counts": [
                    {"shape": list(shape), "count": count}
                    for shape, count in sorted(self.cloud_shape_counts.items())
                ],
            },
        )

        timestamp_evidence: dict[str, Any] = {
            "field": self.timestamp_field,
            "expected_mode": str(
                self.get_parameter("expected_timestamp_mode").value
            ),
            "errors": sorted(set(self.timestamp_errors)),
            "analysis": (
                self.timestamp_analysis.to_dict()
                if self.timestamp_analysis is not None
                else None
            ),
        }
        timestamp_valid = (
            self.timestamp_analysis is not None
            and self.timestamp_analysis.finite_points
            == self.timestamp_analysis.total_points
            and len(self.timestamp_analysis.interpretations) > 0
        )
        expected_timestamp_mode = timestamp_evidence["expected_mode"]
        if timestamp_valid and expected_timestamp_mode:
            timestamp_valid = any(
                interpretation.mode == expected_timestamp_mode
                for interpretation in self.timestamp_analysis.interpretations
            )
        check("per_point_timestamp", timestamp_valid, timestamp_evidence)

        expected_point_frame = str(
            self.get_parameter("expected_point_frame").value
        )
        expected_imu_frame = str(self.get_parameter("expected_imu_frame").value)
        check(
            "pointcloud_frame",
            not expected_point_frame or self.cloud_frames == {expected_point_frame},
            {
                "observed": sorted(self.cloud_frames),
                "expected": expected_point_frame,
            },
        )
        check(
            "imu_frame",
            not expected_imu_frame or self.imu_frames == {expected_imu_frame},
            {"observed": sorted(self.imu_frames), "expected": expected_imu_frame},
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
            {
                "observed": acceleration.dominant_axis,
                "expected": expected_axis,
                "mean": [
                    acceleration.mean_x,
                    acceleration.mean_y,
                    acceleration.mean_z,
                ],
            },
        )

        failed = any(item["status"] == "FAIL" for item in checks)
        warned = any(item["status"] == "WARN" for item in checks)
        status = "FAIL" if failed else "WARN" if warned else "PASS"
        return {
            "schema_version": 1,
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "host": socket.gethostname(),
            "status": status,
            "duration_sec": time.monotonic() - self._started_monotonic,
            "topics": {"points": self.points_topic, "imu": self.imu_topic},
            "cloud_shapes": sorted(self.cloud_shapes),
            "orientation_unavailable_samples": self.orientation_unavailable_samples,
            "angular_velocity": asdict(angular_velocity),
            "checks": checks,
        }

    def _publisher_report(self, topic: str) -> list[dict[str, Any]]:
        publishers = []
        for endpoint in self.get_publishers_info_by_topic(topic):
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
    node = AiryReadinessProbe()
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
