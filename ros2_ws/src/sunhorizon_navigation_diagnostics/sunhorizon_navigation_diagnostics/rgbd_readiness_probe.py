"""Bounded read-only timing probe for chest RGB-D and belly depth."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import socket
import sys
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .rgbd_probe_core import HeaderStreamTracker, match_pairs, timing_dict


def _stamp_seconds(message: Image) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


class RgbdReadinessProbe(Node):
    """Observe headers and metadata without saving or inspecting image payloads."""

    def __init__(self) -> None:
        super().__init__("rgbd_readiness_probe")
        self.declare_parameter(
            "chest_color_topic", "/sunhorizon/raw/rgbd/chest/color/image"
        )
        self.declare_parameter(
            "chest_depth_topic", "/sunhorizon/raw/rgbd/chest/aligned_depth/image"
        )
        self.declare_parameter(
            "belly_depth_topic", "/sunhorizon/raw/rgbd/belly/depth/image"
        )
        self.declare_parameter("duration_sec", 15.0)
        self.declare_parameter("minimum_pair_rate_hz", 12.0)
        self.declare_parameter("maximum_pair_delta_ms", 5.0)
        self.declare_parameter("maximum_arrival_gap_sec", 0.30)

        self.duration_sec = float(self.get_parameter("duration_sec").value)
        if self.duration_sec <= 0.0:
            raise ValueError("duration_sec must be positive")
        self.started = time.monotonic()
        self.done = False
        self.trackers = {
            "chest_color": HeaderStreamTracker(),
            "chest_depth": HeaderStreamTracker(),
            "belly_depth": HeaderStreamTracker(),
        }
        self.metadata: dict[str, set[tuple[str, str, int, int]]] = {
            name: set() for name in self.trackers
        }
        self.topics = {
            "chest_color": str(self.get_parameter("chest_color_topic").value),
            "chest_depth": str(self.get_parameter("chest_depth_topic").value),
            "belly_depth": str(self.get_parameter("belly_depth_topic").value),
        }
        # Do not shadow rclpy.Node.subscriptions, which is a read-only property.
        self._image_subscriptions = []
        for name, topic in self.topics.items():
            self._image_subscriptions.append(self.create_subscription(
                Image,
                topic,
                lambda message, stream=name: self._observe(stream, message),
                qos_profile_sensor_data,
            ))
        self.create_timer(0.05, self._check_duration)

    def _observe(self, stream: str, message: Image) -> None:
        self.trackers[stream].observe(time.monotonic(), _stamp_seconds(message))
        self.metadata[stream].add(
            (message.header.frame_id, message.encoding, message.width, message.height)
        )

    def _check_duration(self) -> None:
        if time.monotonic() - self.started >= self.duration_sec:
            self.done = True

    def _publisher_endpoints_for_topic(self, topic: str) -> list[dict[str, str]]:
        return [
            {"node_name": endpoint.node_name, "node_namespace": endpoint.node_namespace}
            for endpoint in self.get_publishers_info_by_topic(topic)
        ]

    def report(self) -> dict[str, Any]:
        actual_duration = time.monotonic() - self.started
        maximum_delta_ms = float(self.get_parameter("maximum_pair_delta_ms").value)
        pair_timing = match_pairs(
            self.trackers["chest_color"].headers,
            self.trackers["chest_depth"].headers,
            actual_duration,
            maximum_delta_ms / 1000.0,
        )
        streams: dict[str, Any] = {}
        checks: list[dict[str, Any]] = []

        def check(name: str, passed: bool, evidence: Any) -> None:
            checks.append({
                "name": name,
                "status": "PASS" if passed else "FAIL",
                "evidence": evidence,
            })

        maximum_gap = float(self.get_parameter("maximum_arrival_gap_sec").value)
        for name, tracker in self.trackers.items():
            timing = tracker.snapshot()
            publishers = self._publisher_endpoints_for_topic(self.topics[name])
            stream = {
                "topic": self.topics[name],
                "timing": timing_dict(timing),
                "metadata": [
                    {"frame_id": value[0], "encoding": value[1],
                     "width": value[2], "height": value[3]}
                    for value in sorted(self.metadata[name])
                ],
                "publishers": publishers,
            }
            streams[name] = stream
            check(f"{name}_single_publisher", len(publishers) == 1, publishers)
            check(f"{name}_messages_received", timing.samples > 0, timing.samples)
            check(
                f"{name}_header_monotonic",
                timing.header_regressions == 0 and timing.header_duplicates == 0,
                {"regressions": timing.header_regressions,
                 "duplicates": timing.header_duplicates},
            )
            check(
                f"{name}_arrival_gap",
                timing.maximum_arrival_gap_sec is not None
                and timing.maximum_arrival_gap_sec <= maximum_gap,
                {"observed_sec": timing.maximum_arrival_gap_sec,
                 "maximum_sec": maximum_gap},
            )

        minimum_rate = float(self.get_parameter("minimum_pair_rate_hz").value)
        pair_dict = timing_dict(pair_timing)
        check(
            "chest_matched_pair_rate",
            pair_timing.matched_pair_rate_hz >= minimum_rate,
            {"observed_hz": pair_timing.matched_pair_rate_hz,
             "minimum_hz": minimum_rate},
        )
        check(
            "chest_pair_delta_p95",
            pair_timing.pair_delta_p95_ms is not None
            and pair_timing.pair_delta_p95_ms <= maximum_delta_ms,
            {"observed_ms": pair_timing.pair_delta_p95_ms,
             "maximum_ms": maximum_delta_ms},
        )
        status = "FAIL" if any(item["status"] == "FAIL" for item in checks) else "PASS"
        return {
            "schema_version": 1,
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "host": socket.gethostname(),
            "status": status,
            "duration_sec": actual_duration,
            "streams": streams,
            "chest_pair_timing": pair_dict,
            "checks": checks,
        }


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RgbdReadinessProbe()
    report: dict[str, Any]
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
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
