"""Capture one strict chest RGB/aligned-depth pair for manual projection review."""

from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import socket
import sys
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .alignment_capture_core import (
    color_to_rgb,
    depth_to_raw,
    overlay_depth_edges,
    pgm16_bytes,
    ppm_bytes,
)


def _stamp_seconds(message: Image) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _camera_info_dict(message: CameraInfo) -> dict[str, Any]:
    return {
        "frame_id": message.header.frame_id,
        "width": int(message.width),
        "height": int(message.height),
        "distortion_model": message.distortion_model,
        "d": list(message.d),
        "k": list(message.k),
        "r": list(message.r),
        "p": list(message.p),
    }


class RgbdAlignmentCapture(Node):
    """Write review assets but never assert that projection is accepted."""

    def __init__(self) -> None:
        super().__init__("rgbd_alignment_capture")
        self.declare_parameter(
            "color_topic", "/sunhorizon/raw/rgbd/chest/color/image"
        )
        self.declare_parameter(
            "depth_topic", "/sunhorizon/raw/rgbd/chest/aligned_depth/image"
        )
        self.declare_parameter(
            "color_info_topic", "/sunhorizon/raw/rgbd/chest/color/camera_info"
        )
        self.declare_parameter(
            "depth_info_topic",
            "/sunhorizon/raw/rgbd/chest/aligned_depth/camera_info",
        )
        self.declare_parameter("maximum_pair_delta_ms", 5.0)
        self.declare_parameter("timeout_sec", 15.0)
        self.declare_parameter("queue_size", 30)
        self.declare_parameter("depth_scale_m", 0.001)
        self.declare_parameter("edge_threshold_m", 0.05)
        self.declare_parameter("output_directory", "")

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.color_info_topic = str(self.get_parameter("color_info_topic").value)
        self.depth_info_topic = str(self.get_parameter("depth_info_topic").value)
        self.maximum_pair_delta_sec = float(
            self.get_parameter("maximum_pair_delta_ms").value) / 1000.0
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.queue_size = int(self.get_parameter("queue_size").value)
        self.depth_scale_m = float(self.get_parameter("depth_scale_m").value)
        self.edge_threshold_m = float(self.get_parameter("edge_threshold_m").value)
        self.output_directory = Path(
            str(self.get_parameter("output_directory").value))
        self._validate_parameters()

        self.started = time.monotonic()
        self.done = False
        self.result: dict[str, Any] | None = None
        self.color_queue: deque[Image] = deque()
        self.depth_queue: deque[Image] = deque()
        self.color_info: CameraInfo | None = None
        self.depth_info: CameraInfo | None = None
        self._subscriptions = [
            self.create_subscription(
                Image, self.color_topic, self._on_color, qos_profile_sensor_data),
            self.create_subscription(
                Image, self.depth_topic, self._on_depth, qos_profile_sensor_data),
            self.create_subscription(
                CameraInfo, self.color_info_topic, self._on_color_info,
                qos_profile_sensor_data),
            self.create_subscription(
                CameraInfo, self.depth_info_topic, self._on_depth_info,
                qos_profile_sensor_data),
        ]
        self.create_timer(0.05, self._check_timeout)

    def _validate_parameters(self) -> None:
        if not self.output_directory.is_absolute():
            raise ValueError("output_directory_must_be_absolute")
        if self.output_directory.exists():
            raise FileExistsError(f"output_directory_exists:{self.output_directory}")
        if self.maximum_pair_delta_sec <= 0.0 or self.timeout_sec <= 0.0:
            raise ValueError("pair_delta_and_timeout_must_be_positive")
        if self.queue_size < 2 or self.depth_scale_m <= 0.0 or self.edge_threshold_m <= 0.0:
            raise ValueError("invalid_alignment_capture_parameters")

    def _on_color_info(self, message: CameraInfo) -> None:
        self.color_info = message
        self._try_capture()

    def _on_depth_info(self, message: CameraInfo) -> None:
        self.depth_info = message
        self._try_capture()

    def _on_color(self, message: Image) -> None:
        self.color_queue.append(message)
        while len(self.color_queue) > self.queue_size:
            self.color_queue.popleft()
        self._try_capture()

    def _on_depth(self, message: Image) -> None:
        self.depth_queue.append(message)
        while len(self.depth_queue) > self.queue_size:
            self.depth_queue.popleft()
        self._try_capture()

    def _try_capture(self) -> None:
        if self.done or not self.color_info or not self.depth_info:
            return
        while self.color_queue and self.depth_queue:
            color = self.color_queue[0]
            color_stamp = _stamp_seconds(color)
            closest_index = min(
                range(len(self.depth_queue)),
                key=lambda index: abs(
                    color_stamp - _stamp_seconds(self.depth_queue[index])),
            )
            depth = self.depth_queue[closest_index]
            delta = abs(color_stamp - _stamp_seconds(depth))
            if delta <= self.maximum_pair_delta_sec:
                self.color_queue.popleft()
                del self.depth_queue[closest_index]
                self.result = self._write_capture(color, depth, delta)
                self.done = True
                return
            if color_stamp < _stamp_seconds(self.depth_queue[-1]):
                self.color_queue.popleft()
            else:
                self.depth_queue.popleft()

    def _write_capture(
        self, color: Image, depth: Image, pair_delta_sec: float,
    ) -> dict[str, Any]:
        if color.width != depth.width or color.height != depth.height:
            raise ValueError("color_depth_dimensions_differ")
        rgb = color_to_rgb(
            color.data,
            width=int(color.width), height=int(color.height),
            row_step=int(color.step), encoding=color.encoding,
        )
        raw_depth = depth_to_raw(
            depth.data,
            width=int(depth.width), height=int(depth.height),
            row_step=int(depth.step), encoding=depth.encoding,
            is_bigendian=bool(depth.is_bigendian),
        )
        threshold_raw = max(1, round(self.edge_threshold_m / self.depth_scale_m))
        overlay, edge_count = overlay_depth_edges(
            rgb, raw_depth,
            width=int(color.width), height=int(color.height),
            edge_threshold_raw=threshold_raw,
        )
        assets = {
            "color.ppm": ppm_bytes(rgb, int(color.width), int(color.height)),
            "aligned_depth_u16.pgm": pgm16_bytes(
                raw_depth, int(depth.width), int(depth.height)),
            "depth_edges_on_color.ppm": ppm_bytes(
                overlay, int(color.width), int(color.height)),
        }
        self.output_directory.mkdir(parents=True, exist_ok=False)
        manifest_assets = []
        for name, payload in assets.items():
            (self.output_directory / name).write_bytes(payload)
            manifest_assets.append({
                "name": name, "bytes": len(payload), "sha256": _sha256(payload),
            })
        manifest = {
            "schema_version": 1,
            "kind": "rgbd_alignment_review_capture",
            "status": "CAPTURED_NOT_ACCEPTED",
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "host": socket.gethostname(),
            "pair_delta_ms": pair_delta_sec * 1000.0,
            "maximum_pair_delta_ms": self.maximum_pair_delta_sec * 1000.0,
            "color": {
                "topic": self.color_topic, "frame_id": color.header.frame_id,
                "encoding": color.encoding, "width": int(color.width),
                "height": int(color.height),
            },
            "depth": {
                "topic": self.depth_topic, "frame_id": depth.header.frame_id,
                "encoding": depth.encoding, "width": int(depth.width),
                "height": int(depth.height), "scale_m": self.depth_scale_m,
            },
            "color_camera_info": _camera_info_dict(self.color_info),
            "depth_camera_info": _camera_info_dict(self.depth_info),
            "edge_threshold_m": self.edge_threshold_m,
            "depth_edge_pixels": edge_count,
            "assets": sorted(manifest_assets, key=lambda value: value["name"]),
            "manual_review": {
                "accepted": False,
                "reviewer": "",
                "reviewed_at": "",
                "note": "Depth edges must visually coincide with target color edges.",
            },
        }
        manifest_payload = (
            json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
        ).encode("utf-8")
        (self.output_directory / "manifest.json").write_bytes(manifest_payload)
        return manifest

    def _check_timeout(self) -> None:
        if time.monotonic() - self.started >= self.timeout_sec:
            self.done = True


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RgbdAlignmentCapture()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.result is None:
            result = {
                "schema_version": 1,
                "kind": "rgbd_alignment_review_capture",
                "status": "FAIL",
                "reason": "strict_pair_or_camera_info_not_received_before_timeout",
            }
        else:
            result = node.result
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    sys.exit(0 if result["status"] == "CAPTURED_NOT_ACCEPTED" else 2)


if __name__ == "__main__":
    main()
