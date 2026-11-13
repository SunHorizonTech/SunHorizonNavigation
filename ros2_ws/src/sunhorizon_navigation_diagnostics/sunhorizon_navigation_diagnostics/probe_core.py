"""ROS-independent sensor quality calculations used by readiness probes."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import struct
from typing import Iterable, Optional


POINT_FIELD_FORMATS = {
    1: "b",   # INT8
    2: "B",   # UINT8
    3: "h",   # INT16
    4: "H",   # UINT16
    5: "i",   # INT32
    6: "I",   # UINT32
    7: "f",   # FLOAT32
    8: "d",   # FLOAT64
}

TIME_SCALES = {
    "seconds": 1.0,
    "milliseconds": 1e-3,
    "microseconds": 1e-6,
    "nanoseconds": 1e-9,
}


@dataclass(frozen=True)
class RateSnapshot:
    samples: int
    receive_interval_count: int
    receive_span_sec: Optional[float]
    receive_rate_hz: Optional[float]
    header_samples: int
    header_interval_count: int
    header_span_sec: Optional[float]
    header_rate_hz: Optional[float]
    header_regressions: int
    header_duplicates: int


class RateTracker:
    """Track receive and message-header rates without depending on ROS."""

    def __init__(self) -> None:
        self._samples = 0
        self._first_receive: Optional[float] = None
        self._last_receive: Optional[float] = None
        self._first_header: Optional[float] = None
        self._last_header: Optional[float] = None
        self._header_samples = 0
        self._header_regressions = 0
        self._header_duplicates = 0

    def observe(self, receive_time: float, header_time: Optional[float]) -> None:
        if self._first_receive is None:
            self._first_receive = receive_time
        self._last_receive = receive_time
        self._samples += 1

        if header_time is None:
            return
        self._header_samples += 1
        if self._first_header is None:
            self._first_header = header_time
        if self._last_header is not None:
            if header_time < self._last_header:
                self._header_regressions += 1
            elif header_time == self._last_header:
                self._header_duplicates += 1
        self._last_header = header_time

    def snapshot(self) -> RateSnapshot:
        receive_span = _span(self._first_receive, self._last_receive)
        header_span = _span(self._first_header, self._last_header)
        return RateSnapshot(
            samples=self._samples,
            receive_interval_count=max(0, self._samples - 1),
            receive_span_sec=receive_span,
            receive_rate_hz=_rate(self._samples, receive_span),
            header_samples=self._header_samples,
            header_interval_count=max(0, self._header_samples - 1),
            header_span_sec=header_span,
            header_rate_hz=_rate(self._header_samples, header_span),
            header_regressions=self._header_regressions,
            header_duplicates=self._header_duplicates,
        )


def _span(first_time: Optional[float], last_time: Optional[float]) -> Optional[float]:
    if first_time is None or last_time is None:
        return None
    span = last_time - first_time
    return span if span > 0.0 else None


def _rate(samples: int, span: Optional[float]) -> Optional[float]:
    if samples < 2 or span is None:
        return None
    return (samples - 1) / span


@dataclass(frozen=True)
class TimestampInterpretation:
    unit: str
    mode: str
    span_seconds: float
    header_delta_seconds: Optional[float]


@dataclass(frozen=True)
class TimestampAnalysis:
    total_points: int
    finite_points: int
    raw_min: Optional[float]
    raw_max: Optional[float]
    raw_span: Optional[float]
    storage_order_regressions: int
    interpretations: tuple[TimestampInterpretation, ...]

    def to_dict(self) -> dict:
        return asdict(self)


def analyze_point_timestamps(
    data: bytes | bytearray | memoryview,
    *,
    width: int,
    height: int,
    point_step: int,
    row_step: int,
    field_offset: int,
    field_datatype: int,
    is_bigendian: bool,
    header_time: Optional[float],
) -> TimestampAnalysis:
    """Read one numeric timestamp per point, respecting organized-row padding."""

    if width < 0 or height < 0 or point_step <= 0 or row_step < width * point_step:
        raise ValueError("invalid PointCloud2 dimensions")
    if field_datatype not in POINT_FIELD_FORMATS:
        raise ValueError(f"unsupported timestamp datatype: {field_datatype}")

    byte_order = ">" if is_bigendian else "<"
    unpacker = struct.Struct(byte_order + POINT_FIELD_FORMATS[field_datatype])
    if field_offset < 0 or field_offset + unpacker.size > point_step:
        raise ValueError("timestamp field exceeds point_step")

    view = memoryview(data)
    required_size = row_step * height
    if len(view) < required_size:
        raise ValueError(
            f"point data is truncated: {len(view)} bytes, expected {required_size}"
        )

    finite_points = 0
    raw_min: Optional[float] = None
    raw_max: Optional[float] = None
    previous: Optional[float] = None
    regressions = 0

    for row in range(height):
        row_base = row * row_step
        for column in range(width):
            offset = row_base + column * point_step + field_offset
            value = float(unpacker.unpack_from(view, offset)[0])
            if not math.isfinite(value):
                continue
            finite_points += 1
            raw_min = value if raw_min is None else min(raw_min, value)
            raw_max = value if raw_max is None else max(raw_max, value)
            if previous is not None and value < previous:
                regressions += 1
            previous = value

    raw_span = None
    interpretations: tuple[TimestampInterpretation, ...] = ()
    if raw_min is not None and raw_max is not None:
        raw_span = raw_max - raw_min
        interpretations = infer_timestamp_interpretations(
            raw_min, raw_max, header_time=header_time
        )

    return TimestampAnalysis(
        total_points=width * height,
        finite_points=finite_points,
        raw_min=raw_min,
        raw_max=raw_max,
        raw_span=raw_span,
        storage_order_regressions=regressions,
        interpretations=interpretations,
    )


def infer_timestamp_interpretations(
    raw_min: float,
    raw_max: float,
    *,
    header_time: Optional[float],
) -> tuple[TimestampInterpretation, ...]:
    """Return plausible absolute or scan-relative timestamp units.

    This deliberately reports ambiguity instead of silently changing timestamps.
    A candidate scan span must be between one microsecond and one second.
    Absolute candidates must also agree with the message header within 30 seconds.
    """

    if raw_max < raw_min:
        raise ValueError("raw_max must not be less than raw_min")

    absolute: list[TimestampInterpretation] = []
    relative: list[TimestampInterpretation] = []
    raw_mid = (raw_min + raw_max) * 0.5
    raw_span = raw_max - raw_min

    for unit, scale in TIME_SCALES.items():
        span_seconds = raw_span * scale
        if not 1e-6 <= span_seconds <= 1.0:
            continue

        header_delta: Optional[float] = None
        if header_time is not None:
            header_delta = abs(raw_mid * scale - header_time)
            if header_delta <= 30.0:
                absolute.append(
                    TimestampInterpretation(
                        unit=unit,
                        mode="absolute",
                        span_seconds=span_seconds,
                        header_delta_seconds=header_delta,
                    )
                )
                continue

        relative.append(
            TimestampInterpretation(
                unit=unit,
                mode="scan_relative",
                span_seconds=span_seconds,
                header_delta_seconds=header_delta,
            )
        )

    absolute.sort(key=lambda item: item.header_delta_seconds or 0.0)
    relative.sort(key=lambda item: abs(item.span_seconds - 0.1))
    return tuple(absolute if absolute else relative)


@dataclass(frozen=True)
class VectorSnapshot:
    samples: int
    mean_x: Optional[float]
    mean_y: Optional[float]
    mean_z: Optional[float]
    mean_norm: Optional[float]
    dominant_axis: Optional[str]


class VectorAccumulator:
    """Accumulate the mean of a three-dimensional sensor vector."""

    def __init__(self) -> None:
        self._samples = 0
        self._sum_x = 0.0
        self._sum_y = 0.0
        self._sum_z = 0.0

    def observe(self, x: float, y: float, z: float) -> None:
        if not all(math.isfinite(value) for value in (x, y, z)):
            return
        self._samples += 1
        self._sum_x += x
        self._sum_y += y
        self._sum_z += z

    def snapshot(self) -> VectorSnapshot:
        if self._samples == 0:
            return VectorSnapshot(0, None, None, None, None, None)
        mean_x = self._sum_x / self._samples
        mean_y = self._sum_y / self._samples
        mean_z = self._sum_z / self._samples
        mean_norm = math.sqrt(mean_x**2 + mean_y**2 + mean_z**2)
        components = {"x": abs(mean_x), "y": abs(mean_y), "z": abs(mean_z)}
        dominant_axis = max(components, key=components.get)
        return VectorSnapshot(
            self._samples,
            mean_x,
            mean_y,
            mean_z,
            mean_norm,
            dominant_axis,
        )


def dataclass_dicts(items: Iterable[object]) -> list[dict]:
    """Convert dataclass instances to JSON-friendly dictionaries."""

    return [asdict(item) for item in items]
