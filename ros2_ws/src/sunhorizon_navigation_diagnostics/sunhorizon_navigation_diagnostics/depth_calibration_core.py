"""ROS-independent depth-scale measurement for a centered flat target."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Sequence


@dataclass(frozen=True)
class DepthRoiFrame:
    sampled_pixels: int
    valid_pixels: int
    valid_ratio: float
    median_m: float | None
    p05_m: float | None
    p95_m: float | None


def _percentile(values: Sequence[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def extract_depth_roi(
    data: bytes | bytearray | memoryview | Sequence[int],
    *,
    width: int,
    height: int,
    row_step: int,
    encoding: str,
    is_bigendian: bool,
    depth_scale_m: float,
    minimum_depth_m: float,
    maximum_depth_m: float,
    roi_fraction: float,
    pixel_stride: int,
) -> DepthRoiFrame:
    """Measure a centered ROI without retaining the source image."""
    if encoding.lower() not in {"16uc1", "mono16"}:
        raise ValueError(f"unsupported_depth_encoding:{encoding}")
    if width <= 0 or height <= 0 or row_step < width * 2:
        raise ValueError("invalid_depth_dimensions")
    if depth_scale_m <= 0.0 or minimum_depth_m < 0.0 or maximum_depth_m <= minimum_depth_m:
        raise ValueError("invalid_depth_limits")
    if not 0.05 <= roi_fraction <= 1.0:
        raise ValueError("roi_fraction_out_of_range")
    if pixel_stride < 1:
        raise ValueError("pixel_stride_must_be_positive")
    view = memoryview(bytes(data))
    required_size = row_step * height
    if len(view) < required_size:
        raise ValueError("depth_payload_too_small")

    roi_width = max(1, int(round(width * roi_fraction)))
    roi_height = max(1, int(round(height * roi_fraction)))
    column_begin = max(0, (width - roi_width) // 2)
    row_begin = max(0, (height - roi_height) // 2)
    column_end = min(width, column_begin + roi_width)
    row_end = min(height, row_begin + roi_height)
    measurements: list[float] = []
    sampled = 0
    for row in range(row_begin, row_end, pixel_stride):
        for column in range(column_begin, column_end, pixel_stride):
            offset = row * row_step + column * 2
            if is_bigendian:
                raw = (int(view[offset]) << 8) | int(view[offset + 1])
            else:
                raw = int(view[offset]) | (int(view[offset + 1]) << 8)
            sampled += 1
            meters = raw * depth_scale_m
            if raw != 0 and minimum_depth_m <= meters <= maximum_depth_m:
                measurements.append(meters)
    valid = len(measurements)
    return DepthRoiFrame(
        sampled_pixels=sampled,
        valid_pixels=valid,
        valid_ratio=valid / sampled if sampled else 0.0,
        median_m=_percentile(measurements, 0.50),
        p05_m=_percentile(measurements, 0.05),
        p95_m=_percentile(measurements, 0.95),
    )


def summarize_depth_frames(
    frames: Sequence[DepthRoiFrame],
    *,
    expected_distance_m: float,
    minimum_frames: int,
    minimum_valid_ratio: float,
) -> dict[str, Any]:
    """Apply the P1 distance tolerance and flat-target quality gates."""
    if expected_distance_m <= 0.0 or not math.isfinite(expected_distance_m):
        raise ValueError("expected_distance_m_must_be_positive")
    if minimum_frames < 1:
        raise ValueError("minimum_frames_must_be_positive")
    if not 0.0 < minimum_valid_ratio <= 1.0:
        raise ValueError("minimum_valid_ratio_out_of_range")

    medians = [frame.median_m for frame in frames if frame.median_m is not None]
    valid_ratios = [frame.valid_ratio for frame in frames]
    spans = [
        frame.p95_m - frame.p05_m
        for frame in frames
        if frame.p05_m is not None and frame.p95_m is not None
    ]
    measured = _percentile(medians, 0.50)
    absolute_error = (
        abs(measured - expected_distance_m) if measured is not None else None
    )
    distance_tolerance = max(0.02, expected_distance_m * 0.02)
    flat_target_span_limit = max(0.03, expected_distance_m * 0.03)
    valid_ratio_p05 = _percentile(valid_ratios, 0.05)
    span_p95 = _percentile(spans, 0.95)
    checks = {
        "minimum_frames": len(medians) >= minimum_frames,
        "roi_valid_ratio": (
            valid_ratio_p05 is not None and valid_ratio_p05 >= minimum_valid_ratio
        ),
        "flat_target_span": (
            span_p95 is not None and span_p95 <= flat_target_span_limit
        ),
        "distance_error": (
            absolute_error is not None and absolute_error <= distance_tolerance + 1e-12
        ),
    }
    return {
        "accepted": all(checks.values()),
        "checks": checks,
        "frames_observed": len(frames),
        "frames_with_valid_depth": len(medians),
        "minimum_frames": minimum_frames,
        "expected_m": expected_distance_m,
        "measured_m": measured,
        "absolute_error_m": absolute_error,
        "distance_tolerance_m": distance_tolerance,
        "roi_valid_ratio_p05": valid_ratio_p05,
        "minimum_valid_ratio": minimum_valid_ratio,
        "flat_target_span_p95_m": span_p95,
        "flat_target_span_limit_m": flat_target_span_limit,
        "per_frame_median_p05_m": _percentile(medians, 0.05),
        "per_frame_median_p95_m": _percentile(medians, 0.95),
        "frame_summaries": [asdict(frame) for frame in frames],
    }
