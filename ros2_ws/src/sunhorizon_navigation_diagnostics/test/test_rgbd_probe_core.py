import math
import unittest

from sunhorizon_navigation_diagnostics.rgbd_probe_core import (
    HeaderStreamTracker,
    match_pairs,
)


class RgbdProbeCoreTest(unittest.TestCase):
    def test_tracks_rate_gap_and_header_regression(self):
        tracker = HeaderStreamTracker()
        tracker.observe(10.0, 100.0)
        tracker.observe(10.1, 100.1)
        tracker.observe(10.3, 100.05)
        snapshot = tracker.snapshot()
        self.assertEqual(snapshot.samples, 3)
        self.assertTrue(math.isclose(snapshot.receive_rate_hz, 2.0 / 0.3))
        self.assertTrue(math.isclose(snapshot.maximum_arrival_gap_sec, 0.2))
        self.assertEqual(snapshot.header_regressions, 1)


    def test_matches_pairs_one_to_one_with_five_ms_gate(self):
        timing = match_pairs(
            [1.000, 1.033, 1.066, 1.099],
            [1.001, 1.035, 1.080, 1.100],
            duration_sec=0.1,
            maximum_delta_sec=0.005,
        )
        self.assertEqual(timing.matched_pairs, 3)
        self.assertEqual(timing.unmatched_color, 1)
        self.assertEqual(timing.unmatched_depth, 1)
        self.assertTrue(math.isclose(timing.matched_pair_rate_hz, 30.0))
        self.assertTrue(math.isclose(timing.pair_delta_p95_ms, 2.0))


if __name__ == "__main__":
    unittest.main()
