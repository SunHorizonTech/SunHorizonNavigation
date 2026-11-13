"""Dependency-free image conversion for RGB/aligned-depth evidence captures."""

from __future__ import annotations

from typing import Sequence


def _payload_view(data: bytes | bytearray | memoryview | Sequence[int]) -> memoryview:
    try:
        return memoryview(data)  # type: ignore[arg-type]
    except TypeError:
        return memoryview(bytes(data))


def color_to_rgb(
    data: bytes | bytearray | memoryview | Sequence[int],
    *,
    width: int,
    height: int,
    row_step: int,
    encoding: str,
) -> bytes:
    """Convert common ROS color encodings to tightly packed RGB8."""
    channels_by_encoding = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
    }
    normalized = encoding.lower()
    if normalized not in channels_by_encoding:
        raise ValueError(f"unsupported_color_encoding:{encoding}")
    channels = channels_by_encoding[normalized]
    if width <= 0 or height <= 0 or row_step < width * channels:
        raise ValueError("invalid_color_dimensions")
    view = _payload_view(data)
    if len(view) < row_step * height:
        raise ValueError("color_payload_too_small")
    output = bytearray(width * height * 3)
    target = 0
    for row in range(height):
        source = row * row_step
        for column in range(width):
            offset = source + column * channels
            if normalized in {"rgb8", "rgba8"}:
                red, green, blue = view[offset], view[offset + 1], view[offset + 2]
            elif normalized in {"bgr8", "bgra8"}:
                blue, green, red = view[offset], view[offset + 1], view[offset + 2]
            else:
                red = green = blue = view[offset]
            output[target:target + 3] = bytes((red, green, blue))
            target += 3
    return bytes(output)


def depth_to_raw(
    data: bytes | bytearray | memoryview | Sequence[int],
    *,
    width: int,
    height: int,
    row_step: int,
    encoding: str,
    is_bigendian: bool,
) -> list[int]:
    """Convert ROS 16-bit depth with possible row padding to host integers."""
    if encoding.lower() not in {"16uc1", "mono16"}:
        raise ValueError(f"unsupported_depth_encoding:{encoding}")
    if width <= 0 or height <= 0 or row_step < width * 2:
        raise ValueError("invalid_depth_dimensions")
    view = _payload_view(data)
    if len(view) < row_step * height:
        raise ValueError("depth_payload_too_small")
    output: list[int] = []
    for row in range(height):
        source = row * row_step
        for column in range(width):
            offset = source + column * 2
            if is_bigendian:
                output.append((int(view[offset]) << 8) | int(view[offset + 1]))
            else:
                output.append(int(view[offset]) | (int(view[offset + 1]) << 8))
    return output


def ppm_bytes(rgb: bytes, width: int, height: int) -> bytes:
    if len(rgb) != width * height * 3:
        raise ValueError("rgb_size_mismatch")
    return f"P6\n{width} {height}\n255\n".encode("ascii") + rgb


def pgm16_bytes(raw_depth: Sequence[int], width: int, height: int) -> bytes:
    if len(raw_depth) != width * height:
        raise ValueError("depth_size_mismatch")
    payload = bytearray()
    for value in raw_depth:
        if value < 0 or value > 65535:
            raise ValueError("depth_value_out_of_uint16_range")
        payload.extend(int(value).to_bytes(2, "big"))
    return f"P5\n{width} {height}\n65535\n".encode("ascii") + bytes(payload)


def overlay_depth_edges(
    rgb: bytes,
    raw_depth: Sequence[int],
    *,
    width: int,
    height: int,
    edge_threshold_raw: int,
) -> tuple[bytes, int]:
    """Paint red pixels where aligned depth has a valid right/down discontinuity."""
    if len(rgb) != width * height * 3 or len(raw_depth) != width * height:
        raise ValueError("overlay_size_mismatch")
    if edge_threshold_raw <= 0:
        raise ValueError("edge_threshold_raw_must_be_positive")
    output = bytearray(rgb)
    edge_count = 0
    for row in range(height - 1):
        for column in range(width - 1):
            index = row * width + column
            center = raw_depth[index]
            right = raw_depth[index + 1]
            below = raw_depth[index + width]
            if center == 0:
                continue
            right_edge = right != 0 and abs(center - right) >= edge_threshold_raw
            down_edge = below != 0 and abs(center - below) >= edge_threshold_raw
            if not right_edge and not down_edge:
                continue
            target = index * 3
            output[target:target + 3] = b"\xff\x00\x00"
            edge_count += 1
    return bytes(output), edge_count
