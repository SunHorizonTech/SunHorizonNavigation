"""Read-only known-distance depth calibration probe for chest or belly RGB-D."""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import socket
import sys
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .depth_calibration_core import extract_depth_roi, summarize_depth_frames


CAMERA_TOPICS = {
    "chest": "/sunhorizon/raw/rgbd/chest/aligned_depth/image",
    "belly": "/sunhorizon/raw/rgbd/belly/depth/image",
}


def _stamp_seconds(message: Image) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


class RgbdDepthCalibrationProbe(Node):
    """Measure a flat board centered and perpendicular to one camera optical axis."""

    def __init__(self) -> None:
        super().__init__("rgbd_depth_calibration_probe")
        self.declare_parameter("camera", "chest")
        self.declare_parameter("depth_topic", "")
        self.declare_parameter("expected_distance_m", 0.0)
        self.declare_parameter("duration_sec", 8.0)
        self.declare_parameter("sample_rate_hz", 5.0)
        self.declare_parameter("minimum_frames", 30)
        self.declare_parameter("roi_fraction", 0.20)
        self.declare_parameter("pixel_stride", 4)
        self.declare_parameter("depth_scale_m", 0.001)
        self.declare_parameter("minimum_depth_m", 0.20)
        self.declare_parameter("maximum_depth_m", 8.0)
        self.declare_parameter("minimum_valid_ratio", 0.80)
        self.declare_parameter("output_path", "")

        self.camera = str(self.get_parameter("camera").value)
        if self.camera not in CAMERA_TOPICS:
            raise ValueError("camera_must_be_chest_or_belly")
        configured_topic = str(self.get_parameter("depth_topic").value)
        self.depth_topic = configured_topic or CAMERA_TOPICS[self.camera]
        self.expected_distance_m = float(
            self.get_parameter("expected_distance_m").value)
        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.sample_rate_hz = float(self.get_parameter("sample_rate_hz").value)
        self.minimum_frames = int(self.get_parameter("minimum_frames").value)
        self.roi_fraction = float(self.get_parameter("roi_fraction").value)
        self.pixel_stride = int(self.get_parameter("pixel_stride").value)
        self.depth_scale_m = float(self.get_parameter("depth_scale_m").value)
        self.minimum_depth_m = float(self.get_parameter("minimum_depth_m").value)
        self.maximum_depth_m = float(self.get_parameter("maximum_depth_m").value)
        self.minimum_valid_ratio = float(
            self.get_parameter("minimum_valid_ratio").value)
        self.output_path = str(self.get_parameter("output_path").value)
        self._validate_parameters()

        self.started_monotonic = time.monotonic()
        self.last_sample_monotonic = float("-inf")
        self.last_header_sec = 0.0
        self.header_regressions = 0
        self.received_frames = 0
        self.frames = []
        self.metadata: set[tuple[str, str, int, int, bool]] = set()
        self.done = False
        self._depth_subscription = self.create_subscription(
            Image, self.depth_topic, self._observe, qos_profile_sensor_data)
        self.create_timer(0.05, self._check_duration)

    def _validate_parameters(self) -> None:
        if self.expected_distance_m <= 0.0:
            raise ValueError("expected_distance_m_must_be_positive")
        if self.duration_sec <= 0.0 or self.sample_rate_hz <= 0.0:
            raise ValueError("duration_and_sample_rate_must_be_positive")
        if self.minimum_frames < 1:
            raise ValueError("minimum_frames_must_be_positive")
        if self.duration_sec * self.sample_rate_hz + 1e-12 < self.minimum_frames:
            raise ValueError("duration_cannot_collect_minimum_frames")
        if self.output_path and not Path(self.output_path).is_absolute():
            raise ValueError("output_path_must_be_absolute")

    def _observe(self, message: Image) -> None:
        self.received_frames += 1
        header_sec = _stamp_seconds(message)
        if self.last_header_sec > 0.0 and header_sec <= self.last_header_sec:
            self.header_regressions += 1
        self.last_header_sec = header_sec
        self.metadata.add((
            message.header.frame_id,
            message.encoding,
            int(message.width),
            int(message.height),
            bool(message.is_bigendian),
        ))
        now = time.monotonic()
        if now - self.last_sample_monotonic < 1.0 / self.sample_rate_hz:
            return
        self.last_sample_monotonic = now
        try:
            frame = extract_depth_roi(
                message.data,
                width=int(message.width),
                height=int(message.height),
                row_step=int(message.step),
                encoding=message.encoding,
                is_bigendian=bool(message.is_bigendian),
                depth_scale_m=self.depth_scale_m,
                minimum_depth_m=self.minimum_depth_m,
                maximum_depth_m=self.maximum_depth_m,
                roi_fraction=self.roi_fraction,
                pixel_stride=self.pixel_stride,
            )
            self.frames.append(frame)
        except ValueError as error:
            self.get_logger().warning(str(error))

    def _check_duration(self) -> None:
        if time.monotonic() - self.started_monotonic >= self.duration_sec:
            self.done = True

    def report(self) -> dict[str, Any]:
        summary = summarize_depth_frames(
            self.frames,
            expected_distance_m=self.expected_distance_m,
            minimum_frames=self.minimum_frames,
            minimum_valid_ratio=self.minimum_valid_ratio,
        )
        publishers = [
            {"node_name": endpoint.node_name, "node_namespace": endpoint.node_namespace}
            for endpoint in self.get_publishers_info_by_topic(self.depth_topic)
        ]
        metadata = [
            {
                "frame_id": value[0], "encoding": value[1],
                "width": value[2], "height": value[3], "is_bigendian": value[4],
            }
            for value in sorted(self.metadata)
        ]
        checks = {
            "single_publisher": len(publishers) == 1,
            "single_stream_format": len(metadata) == 1,
            "header_monotonic": self.header_regressions == 0,
            "depth_measurement": bool(summary["accepted"]),
        }
        status = "PASS" if all(checks.values()) else "FAIL"
        return {
            "schema_version": 1,
            "kind": "rgbd_depth_scale_measurement",
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "host": socket.gethostname(),
            "status": status,
            "camera": self.camera,
            "topic": self.depth_topic,
            "duration_sec": time.monotonic() - self.started_monotonic,
            "received_frames": self.received_frames,
            "header_regressions": self.header_regressions,
            "depth_scale_m": self.depth_scale_m,
            "roi_fraction": self.roi_fraction,
            "pixel_stride": self.pixel_stride,
            "publishers": publishers,
            "metadata": metadata,
            "checks": checks,
            "measurement": {
                "camera": self.camera,
                "expected_m": summary["expected_m"],
                "measured_m": summary["measured_m"],
                "absolute_error_m": summary["absolute_error_m"],
                "tolerance_m": summary["distance_tolerance_m"],
                "accepted": summary["accepted"],
            },
            "statistics": summary,
        }

    def write_report(self, report: dict[str, Any]) -> None:
        if not self.output_path:
            return
        path = Path(self.output_path)
        if path.exists():
            raise FileExistsError(f"output_exists:{path}")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RgbdDepthCalibrationProbe()
    report: dict[str, Any]
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = node.report()
        node.write_report(report)
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
