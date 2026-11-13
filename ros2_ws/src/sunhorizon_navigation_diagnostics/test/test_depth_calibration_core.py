import math
import unittest

from sunhorizon_navigation_diagnostics.depth_calibration_core import (
    DepthRoiFrame,
    extract_depth_roi,
    summarize_depth_frames,
)


def depth_image(width, height, raw, *, bigendian=False):
    payload = bytearray()
    for _ in range(width * height):
        payload.extend(raw.to_bytes(2, "big" if bigendian else "little"))
    return payload


class DepthCalibrationCoreTest(unittest.TestCase):
    def test_extracts_little_endian_center_roi(self):
        frame = extract_depth_roi(
            depth_image(8, 6, 1000),
            width=8,
            height=6,
            row_step=16,
            encoding="16UC1",
            is_bigendian=False,
            depth_scale_m=0.001,
            minimum_depth_m=0.2,
            maximum_depth_m=8.0,
            roi_fraction=0.5,
            pixel_stride=1,
        )
        self.assertEqual(frame.sampled_pixels, 12)
        self.assertEqual(frame.valid_pixels, 12)
        self.assertTrue(math.isclose(frame.median_m, 1.0))

    def test_extracts_big_endian_and_rejects_zero_depth(self):
        payload = depth_image(4, 4, 500, bigendian=True)
        payload[0:2] = b"\x00\x00"
        frame = extract_depth_roi(
            payload,
            width=4,
            height=4,
            row_step=8,
            encoding="mono16",
            is_bigendian=True,
            depth_scale_m=0.001,
            minimum_depth_m=0.2,
            maximum_depth_m=8.0,
            roi_fraction=1.0,
            pixel_stride=1,
        )
        self.assertEqual(frame.valid_pixels, 15)
        self.assertTrue(math.isclose(frame.valid_ratio, 15 / 16))
        self.assertTrue(math.isclose(frame.median_m, 0.5))

    def test_accepts_flat_target_inside_distance_tolerance(self):
        frames = [
            DepthRoiFrame(100, 100, 1.0, 1.01, 1.0, 1.02)
            for _ in range(30)
        ]
        summary = summarize_depth_frames(
            frames,
            expected_distance_m=1.0,
            minimum_frames=30,
            minimum_valid_ratio=0.8,
        )
        self.assertTrue(summary["accepted"])
        self.assertTrue(math.isclose(summary["measured_m"], 1.01))

    def test_rejects_wrong_distance_or_non_flat_target(self):
        frames = [
            DepthRoiFrame(100, 100, 1.0, 1.04, 0.95, 1.10)
            for _ in range(30)
        ]
        summary = summarize_depth_frames(
            frames,
            expected_distance_m=1.0,
            minimum_frames=30,
            minimum_valid_ratio=0.8,
        )
        self.assertFalse(summary["accepted"])
        self.assertFalse(summary["checks"]["distance_error"])
        self.assertFalse(summary["checks"]["flat_target_span"])


if __name__ == "__main__":
    unittest.main()
