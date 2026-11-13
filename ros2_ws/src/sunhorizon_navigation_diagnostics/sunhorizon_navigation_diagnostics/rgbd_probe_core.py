"""ROS-independent timing statistics for the read-only RGB-D probe."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any


@dataclass(frozen=True)
class StreamTiming:
    samples: int
    receive_rate_hz: float | None
    header_rate_hz: float | None
    maximum_arrival_gap_sec: float | None
    header_regressions: int
    header_duplicates: int


@dataclass(frozen=True)
class PairTiming:
    matched_pairs: int
    unmatched_color: int
    unmatched_depth: int
    matched_pair_rate_hz: float
    pair_delta_p50_ms: float | None
    pair_delta_p95_ms: float | None
    pair_delta_max_ms: float | None


class HeaderStreamTracker:
    """Keep bounded scalar timing evidence without retaining image payloads."""

    def __init__(self) -> None:
        self.arrivals: list[float] = []
        self.headers: list[float] = []
        self.header_regressions = 0
        self.header_duplicates = 0

    def observe(self, arrival_sec: float, header_sec: float) -> None:
        if not math.isfinite(arrival_sec) or not math.isfinite(header_sec):
            return
        if self.headers:
            if header_sec < self.headers[-1]:
                self.header_regressions += 1
            elif header_sec == self.headers[-1]:
                self.header_duplicates += 1
        self.arrivals.append(arrival_sec)
        self.headers.append(header_sec)

    def snapshot(self) -> StreamTiming:
        receive_rate = None
        header_rate = None
        maximum_gap = None
        if len(self.arrivals) >= 2:
            arrival_span = self.arrivals[-1] - self.arrivals[0]
            if arrival_span > 0.0:
                receive_rate = (len(self.arrivals) - 1) / arrival_span
            maximum_gap = max(
                current - previous
                for previous, current in zip(self.arrivals, self.arrivals[1:])
            )
        if len(self.headers) >= 2:
            ordered = sorted(self.headers)
            header_span = ordered[-1] - ordered[0]
            if header_span > 0.0:
                header_rate = (len(ordered) - 1) / header_span
        return StreamTiming(
            samples=len(self.headers),
            receive_rate_hz=receive_rate,
            header_rate_hz=header_rate,
            maximum_arrival_gap_sec=maximum_gap,
            header_regressions=self.header_regressions,
            header_duplicates=self.header_duplicates,
        )


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def match_pairs(
    color_headers: list[float],
    depth_headers: list[float],
    duration_sec: float,
    maximum_delta_sec: float,
) -> PairTiming:
    """Greedily form chronological one-to-one RGB/depth pairs."""
    colors = sorted(value for value in color_headers if math.isfinite(value))
    depths = sorted(value for value in depth_headers if math.isfinite(value))
    color_index = 0
    depth_index = 0
    deltas: list[float] = []
    unmatched_color = 0
    unmatched_depth = 0
    while color_index < len(colors) and depth_index < len(depths):
        difference = colors[color_index] - depths[depth_index]
        if abs(difference) <= maximum_delta_sec:
            deltas.append(abs(difference))
            color_index += 1
            depth_index += 1
        elif difference < 0.0:
            unmatched_color += 1
            color_index += 1
        else:
            unmatched_depth += 1
            depth_index += 1
    unmatched_color += len(colors) - color_index
    unmatched_depth += len(depths) - depth_index
    deltas_ms = [value * 1000.0 for value in deltas]
    return PairTiming(
        matched_pairs=len(deltas),
        unmatched_color=unmatched_color,
        unmatched_depth=unmatched_depth,
        matched_pair_rate_hz=len(deltas) / duration_sec if duration_sec > 0.0 else 0.0,
        pair_delta_p50_ms=_percentile(deltas_ms, 0.50),
        pair_delta_p95_ms=_percentile(deltas_ms, 0.95),
        pair_delta_max_ms=max(deltas_ms) if deltas_ms else None,
    )


def timing_dict(value: StreamTiming | PairTiming) -> dict[str, Any]:
    return asdict(value)
