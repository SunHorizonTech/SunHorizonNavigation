#!/usr/bin/env python3
"""Project a canonical FAST-LIVO2 binary PCD into provisional ROS map assets."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import struct
import tempfile
from typing import Iterator


MAX_GRID_CELLS = 20_000_000


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_binary_pcd(path: Path) -> tuple[dict[str, object], Iterator[tuple[float, float, float]]]:
    stream = path.open("rb")
    header: dict[str, list[str]] = {}
    while True:
        line = stream.readline()
        if not line:
            stream.close()
            raise ValueError("pcd_header_incomplete")
        try:
            text = line.decode("ascii").strip()
        except UnicodeDecodeError as error:
            stream.close()
            raise ValueError("pcd_header_not_ascii") from error
        if not text or text.startswith("#"):
            continue
        parts = text.split()
        header[parts[0].upper()] = parts[1:]
        if parts[0].upper() == "DATA":
            break

    fields = header.get("FIELDS", [])
    sizes = [int(item) for item in header.get("SIZE", [])]
    types = header.get("TYPE", [])
    counts = [int(item) for item in header.get("COUNT", ["1"] * len(fields))]
    if header.get("DATA") != ["binary"]:
        stream.close()
        raise ValueError("pcd_data_must_be_binary")
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        stream.close()
        raise ValueError("pcd_field_metadata_mismatch")
    for required in ("x", "y", "z"):
        if required not in fields:
            stream.close()
            raise ValueError(f"pcd_field_missing:{required}")
        index = fields.index(required)
        if sizes[index] != 4 or types[index] != "F" or counts[index] != 1:
            stream.close()
            raise ValueError(f"pcd_field_not_float32:{required}")

    point_count = int(header.get("POINTS", ["0"])[0])
    if point_count <= 0:
        stream.close()
        raise ValueError("pcd_point_count_invalid")
    point_step = sum(size * count for size, count in zip(sizes, counts))
    offsets: list[int] = []
    offset = 0
    for size, count in zip(sizes, counts):
        offsets.append(offset)
        offset += size * count
    xyz_offsets = tuple(offsets[fields.index(axis)] for axis in ("x", "y", "z"))

    def points() -> Iterator[tuple[float, float, float]]:
        try:
            for _ in range(point_count):
                record = stream.read(point_step)
                if len(record) != point_step:
                    raise ValueError("pcd_binary_payload_truncated")
                xyz = tuple(struct.unpack_from("<f", record, item)[0] for item in xyz_offsets)
                if all(math.isfinite(value) for value in xyz):
                    yield xyz  # type: ignore[misc]
            if stream.read(1):
                raise ValueError("pcd_binary_payload_has_trailing_bytes")
        finally:
            stream.close()

    return {
        "fields": fields,
        "point_count": point_count,
        "point_step": point_step,
    }, points()


def read_trajectory(path: Path) -> list[tuple[float, float, float]]:
    output: list[tuple[float, float, float]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 8:
            raise ValueError(f"trajectory_field_count:{line_number}")
        values = [float(item) for item in fields]
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"trajectory_non_finite:{line_number}")
        output.append((values[1], values[2], values[3]))
    if len(output) < 2:
        raise ValueError("trajectory_has_fewer_than_two_samples")
    return output


def disk_offsets(radius: float, resolution: float) -> list[tuple[int, int]]:
    cells = math.ceil(radius / resolution)
    return [
        (dx, dy)
        for dy in range(-cells, cells + 1)
        for dx in range(-cells, cells + 1)
        if (dx * resolution) ** 2 + (dy * resolution) ** 2 <= radius ** 2 + 1e-12
    ]


def project(
    pcd: Path,
    trajectory: Path,
    output: Path,
    *,
    resolution: float,
    height_min: float,
    height_max: float,
    trajectory_margin: float,
    free_radius: float,
    obstacle_dilation: float,
) -> dict[str, object]:
    if resolution <= 0.0 or height_max <= height_min or trajectory_margin <= 0.0:
        raise ValueError("projection_parameters_invalid")
    if free_radius < 0.0 or obstacle_dilation < 0.0:
        raise ValueError("projection_radii_invalid")
    pcd = pcd.resolve(strict=True)
    trajectory = trajectory.resolve(strict=True)
    output = output.absolute()
    output.parent.resolve(strict=True)
    if output.exists() or output.is_symlink():
        raise FileExistsError(f"output_exists:{output}")

    trajectory_points = read_trajectory(trajectory)
    x_min = min(point[0] for point in trajectory_points) - trajectory_margin
    x_max = max(point[0] for point in trajectory_points) + trajectory_margin
    y_min = min(point[1] for point in trajectory_points) - trajectory_margin
    y_max = max(point[1] for point in trajectory_points) + trajectory_margin
    width = max(1, math.ceil((x_max - x_min) / resolution))
    height = max(1, math.ceil((y_max - y_min) / resolution))
    if width * height > MAX_GRID_CELLS:
        raise ValueError("projected_grid_too_large")

    def cell(x: float, y: float) -> tuple[int, int] | None:
        if not (x_min <= x <= x_max and y_min <= y <= y_max):
            return None
        return (
            min(width - 1, max(0, math.floor((x - x_min) / resolution))),
            min(height - 1, max(0, math.floor((y - y_min) / resolution))),
        )

    pcd_metadata, points = parse_binary_pcd(pcd)
    occupied: set[int] = set()
    selected_points = 0
    for x, y, z in points:
        if height_min <= z <= height_max:
            grid_cell = cell(x, y)
            if grid_cell is not None:
                occupied.add(grid_cell[1] * width + grid_cell[0])
                selected_points += 1

    occupied_dilated: set[int] = set()
    for index in occupied:
        center_x, center_y = index % width, index // width
        for dx, dy in disk_offsets(obstacle_dilation, resolution):
            x, y = center_x + dx, center_y + dy
            if 0 <= x < width and 0 <= y < height:
                occupied_dilated.add(y * width + x)

    free: set[int] = set()
    for x, y, _z in trajectory_points:
        center = cell(x, y)
        if center is None:
            continue
        for dx, dy in disk_offsets(free_radius, resolution):
            column, row = center[0] + dx, center[1] + dy
            if 0 <= column < width and 0 <= row < height:
                free.add(row * width + column)
    free.difference_update(occupied_dilated)

    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}.", dir=output.parent))
    try:
        pixels = bytearray([205]) * (width * height)
        for index in free:
            column, ros_row = index % width, index // width
            pixels[(height - 1 - ros_row) * width + column] = 254
        for index in occupied_dilated:
            column, ros_row = index % width, index // width
            pixels[(height - 1 - ros_row) * width + column] = 0
        pgm = staging / "map.pgm"
        pgm.write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + pixels)
        yaml = staging / "map.yaml"
        yaml.write_text(
            "image: map.pgm\n"
            f"resolution: {resolution:.12g}\n"
            f"origin: [{x_min:.12g}, {y_min:.12g}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n",
            encoding="utf-8",
        )
        manifest: dict[str, object] = {
            "schema_version": 1,
            "status": "provisional_livo_projection",
            "motion_output_connected": False,
            "source": {
                "pcd": str(pcd),
                "pcd_sha256": sha256_file(pcd),
                "trajectory": str(trajectory),
                "trajectory_sha256": sha256_file(trajectory),
                **pcd_metadata,
            },
            "parameters": {
                "resolution_m": resolution,
                "height_min_m": height_min,
                "height_max_m": height_max,
                "trajectory_margin_m": trajectory_margin,
                "known_free_radius_m": free_radius,
                "obstacle_dilation_m": obstacle_dilation,
            },
            "grid": {
                "width": width,
                "height": height,
                "origin_x_m": x_min,
                "origin_y_m": y_min,
                "occupied_cells": len(occupied_dilated),
                "free_cells": len(free),
                "selected_obstacle_points": selected_points,
            },
            "assets": {
                "map.pgm": {"sha256": sha256_file(pgm), "size_bytes": pgm.stat().st_size},
                "map.yaml": {"sha256": sha256_file(yaml), "size_bytes": yaml.stat().st_size},
            },
        }
        (staging / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(staging, output)
        return manifest
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", type=Path, required=True)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resolution", type=float, required=True)
    parser.add_argument("--height-min", type=float, required=True)
    parser.add_argument("--height-max", type=float, required=True)
    parser.add_argument("--trajectory-margin", type=float, required=True)
    parser.add_argument("--known-free-radius", type=float, required=True)
    parser.add_argument("--obstacle-dilation", type=float, required=True)
    arguments = parser.parse_args()
    result = project(
        arguments.pcd,
        arguments.trajectory,
        arguments.output,
        resolution=arguments.resolution,
        height_min=arguments.height_min,
        height_max=arguments.height_max,
        trajectory_margin=arguments.trajectory_margin,
        free_radius=arguments.known_free_radius,
        obstacle_dilation=arguments.obstacle_dilation,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
