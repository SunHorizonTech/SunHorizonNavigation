import math
import struct
import unittest

from sunhorizon_navigation_diagnostics.probe_core import (
    RateTracker,
    VectorAccumulator,
    analyze_point_timestamps,
    infer_timestamp_interpretations,
)


class RateTrackerTest(unittest.TestCase):
    def test_tracks_rates_and_header_regressions(self) -> None:
        tracker = RateTracker()
        tracker.observe(10.0, 100.0)
        tracker.observe(10.1, 100.1)
        tracker.observe(10.2, 100.05)

        snapshot = tracker.snapshot()

        self.assertEqual(snapshot.samples, 3)
        self.assertEqual(snapshot.receive_interval_count, 2)
        self.assertAlmostEqual(snapshot.receive_span_sec or 0.0, 0.2)
        self.assertAlmostEqual(snapshot.receive_rate_hz or 0.0, 10.0)
        self.assertEqual(snapshot.header_samples, 3)
        self.assertEqual(snapshot.header_interval_count, 2)
        self.assertAlmostEqual(snapshot.header_span_sec or 0.0, 0.05)
        self.assertAlmostEqual(snapshot.header_rate_hz or 0.0, 40.0)
        self.assertEqual(snapshot.header_regressions, 1)

    def test_header_rate_uses_only_messages_with_headers(self) -> None:
        tracker = RateTracker()
        tracker.observe(1.0, None)
        tracker.observe(2.0, 10.0)
        tracker.observe(3.0, 10.5)

        snapshot = tracker.snapshot()

        self.assertEqual(snapshot.samples, 3)
        self.assertEqual(snapshot.header_samples, 2)
        self.assertEqual(snapshot.header_interval_count, 1)
        self.assertAlmostEqual(snapshot.header_span_sec or 0.0, 0.5)
        self.assertAlmostEqual(snapshot.header_rate_hz or 0.0, 2.0)


class TimestampAnalysisTest(unittest.TestCase):
    def test_reads_float64_from_organized_cloud_with_row_padding(self) -> None:
        width = 3
        height = 2
        point_step = 16
        row_step = width * point_step + 8
        data = bytearray(row_step * height)
        values = [100.00, 100.01, 100.02, 100.03, 100.04, 100.05]
        index = 0
        for row in range(height):
            for column in range(width):
                offset = row * row_step + column * point_step + 8
                struct.pack_into("<d", data, offset, values[index])
                index += 1

        analysis = analyze_point_timestamps(
            data,
            width=width,
            height=height,
            point_step=point_step,
            row_step=row_step,
            field_offset=8,
            field_datatype=8,
            is_bigendian=False,
            header_time=100.05,
        )

        self.assertEqual(analysis.total_points, 6)
        self.assertEqual(analysis.finite_points, 6)
        self.assertAlmostEqual(analysis.raw_span or 0.0, 0.05)
        self.assertEqual(analysis.storage_order_regressions, 0)
        self.assertEqual(len(analysis.interpretations), 1)
        self.assertEqual(analysis.interpretations[0].unit, "seconds")
        self.assertEqual(analysis.interpretations[0].mode, "absolute")

    def test_infers_relative_nanoseconds(self) -> None:
        candidates = infer_timestamp_interpretations(
            0.0, 100_000_000.0, header_time=1_700_000_000.0
        )

        self.assertEqual(len(candidates), 1)
        self.assertEqual(candidates[0].unit, "nanoseconds")
        self.assertEqual(candidates[0].mode, "scan_relative")
        self.assertAlmostEqual(candidates[0].span_seconds, 0.1)


class VectorAccumulatorTest(unittest.TestCase):
    def test_reports_dominant_gravity_axis(self) -> None:
        accumulator = VectorAccumulator()
        accumulator.observe(9.8, 0.2, 0.1)
        accumulator.observe(9.7, 0.3, 0.1)
        accumulator.observe(math.nan, 0.0, 0.0)

        snapshot = accumulator.snapshot()

        self.assertEqual(snapshot.samples, 2)
        self.assertEqual(snapshot.dominant_axis, "x")
        self.assertGreater(snapshot.mean_norm or 0.0, 9.0)


if __name__ == "__main__":
    unittest.main()
