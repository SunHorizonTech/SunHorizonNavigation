import unittest

from sunhorizon_navigation_diagnostics.alignment_capture_core import (
    color_to_rgb,
    depth_to_raw,
    overlay_depth_edges,
    pgm16_bytes,
    ppm_bytes,
)


class AlignmentCaptureCoreTest(unittest.TestCase):
    def test_converts_bgr_with_row_padding(self):
        payload = bytes([
            30, 20, 10, 60, 50, 40, 0, 0,
            90, 80, 70, 120, 110, 100, 0, 0,
        ])
        rgb = color_to_rgb(
            payload, width=2, height=2, row_step=8, encoding="bgr8")
        self.assertEqual(rgb, bytes([
            10, 20, 30, 40, 50, 60,
            70, 80, 90, 100, 110, 120,
        ]))

    def test_converts_little_endian_depth_with_padding(self):
        payload = bytes([
            1, 0, 2, 0, 99, 99,
            3, 0, 4, 0, 99, 99,
        ])
        values = depth_to_raw(
            payload, width=2, height=2, row_step=6,
            encoding="16UC1", is_bigendian=False,
        )
        self.assertEqual(values, [1, 2, 3, 4])

    def test_marks_valid_depth_discontinuity_red(self):
        rgb = bytes([10, 20, 30] * 4)
        overlay, count = overlay_depth_edges(
            rgb, [1000, 1200, 1000, 1000],
            width=2, height=2, edge_threshold_raw=50,
        )
        self.assertEqual(count, 1)
        self.assertEqual(overlay[:3], b"\xff\x00\x00")
        self.assertEqual(overlay[3:], rgb[3:])

    def test_emits_binary_netpbm_headers(self):
        self.assertTrue(ppm_bytes(bytes([1, 2, 3]), 1, 1).startswith(b"P6\n1 1\n255\n"))
        depth = pgm16_bytes([0x1234], 1, 1)
        self.assertTrue(depth.startswith(b"P5\n1 1\n65535\n"))
        self.assertTrue(depth.endswith(b"\x12\x34"))


if __name__ == "__main__":
    unittest.main()
