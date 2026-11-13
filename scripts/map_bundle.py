#!/usr/bin/env python3
"""Create, convert, and validate immutable CASBOT02 navigation map bundles."""

from __future__ import annotations

import argparse
import ast
import datetime as dt
import hashlib
import json
import math
import os
import re
import shutil
import tempfile
from pathlib import Path, PurePosixPath
from typing import Any


SCHEMA_VERSION = 3
SUPPORTED_PROFILES = ("visual", "lidar")
LIVO_M3_KIND = "sunhorizon_livo_m3_navigation_map"
M2_FIXED_MAP_KIND = "sunhorizon_robot_a_livo_fixed_map"
M2_FIXED_MAP_SCOPE = "m2_localization_only"
KEEPOUT_RECEIPT_KIND = "sunhorizon_keepout_mask_receipt"
M2_FIXED_MAP_ASSETS = (
    "map.pcd",
    "backend.yaml",
    "mapping_run_receipt.json",
    "robot_acceptance.json",
    "trajectory_report.json",
)
COMMON_ASSETS = (
    "map.pgm",
    "map.yaml",
    "keepout.pgm",
    "keepout.yaml",
    "robot.urdf",
    "calibration.yaml",
    "trajectory_report.json",
    "metadata.json",
)
VISUAL_ASSETS = ("rtabmap.db", "map.pcd", "backend.yaml")
LIDAR_ASSETS = ("map.pcd", "backend.yaml")
PROFILE_ASSETS = {
    "visual": VISUAL_ASSETS,
    "lidar": LIDAR_ASSETS,
}
# Compatibility for callers that prepare the original visual bundle inputs.
REQUIRED_ASSETS = VISUAL_ASSETS + tuple(
    name for name in COMMON_ASSETS if name not in VISUAL_ASSETS)
V2_REQUIRED_ASSETS = REQUIRED_ASSETS
MAP_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
GIT_SHA_PATTERN = re.compile(r"^[0-9a-f]{40}$")
ASSET_RECORD_KEYS = {"path", "sha256", "size_bytes"}
MAXIMUM_JSON_BYTES = 1024 * 1024


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"invalid_json_constant:{value}")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def manifest_key(filename: str) -> str:
    """Return the legacy schema-v2 flat hash key for *filename*."""
    return filename.replace(".", "_") + "_sha256"


def _load_json_object(path: Path) -> dict[str, Any]:
    if path.stat().st_size > MAXIMUM_JSON_BYTES:
        raise ValueError(f"json_file_too_large:{path.name}")
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream, parse_constant=_reject_json_constant)
    if not isinstance(value, dict):
        raise ValueError(f"JSON must contain an object: {path}")
    return value


def _regular_nonempty_file(path: Path) -> bool:
    return path.is_file() and not path.is_symlink() and path.stat().st_size > 0


def _require_sha256(value: Any, label: str) -> str:
    if not isinstance(value, str) or not SHA256_PATTERN.fullmatch(value):
        raise ValueError(f"invalid_sha256:{label}")
    return value


def _require_git_sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or not GIT_SHA_PATTERN.fullmatch(value):
        raise ValueError(f"invalid_git_sha:{label}")
    return value


def _require_nonempty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"invalid_string:{label}")
    return value


def _require_positive_number(value: Any, label: str) -> float:
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or
            not math.isfinite(float(value)) or value <= 0):
        raise ValueError(f"invalid_positive_number:{label}")
    return float(value)


def _read_pgm_size(path: Path) -> tuple[int, int]:
    data = path.read_bytes()
    tokens: list[bytes] = []
    index = 0
    while len(tokens) < 4:
        while index < len(data) and chr(data[index]).isspace():
            index += 1
        if index < len(data) and data[index] == ord("#"):
            while index < len(data) and data[index] not in (10, 13):
                index += 1
            continue
        start = index
        while index < len(data) and not chr(data[index]).isspace():
            index += 1
        if start == index:
            raise ValueError("invalid_pgm_header")
        tokens.append(data[start:index])
    magic, width_raw, height_raw, maximum_raw = tokens
    try:
        width, height, maximum = int(width_raw), int(height_raw), int(maximum_raw)
    except ValueError as error:
        raise ValueError("invalid_pgm_header") from error
    if magic not in (b"P2", b"P5") or width <= 0 or height <= 0 or not (0 < maximum <= 255):
        raise ValueError("unsupported_pgm")
    if magic == b"P5":
        if data[index:index + 2] == b"\r\n":
            index += 2
        elif index < len(data) and chr(data[index]).isspace():
            index += 1
        else:
            raise ValueError("invalid_pgm_header_separator")
        if len(data) - index < width * height:
            raise ValueError("truncated_pgm")
    elif len(data[index:].split()) < width * height:
        raise ValueError("truncated_pgm")
    return width, height


def _map_yaml_geometry(path: Path, expected_image: str) -> tuple[float, list[float]]:
    fields: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        fields[key.strip()] = value.strip().strip("'\"")
    if not {"image", "resolution", "origin"}.issubset(fields):
        raise ValueError("map_yaml_geometry_missing")
    if fields["image"] != expected_image:
        raise ValueError("map_yaml_image_name_mismatch")
    try:
        resolution = float(fields["resolution"])
        origin_raw = ast.literal_eval(fields["origin"])
    except (SyntaxError, ValueError) as error:
        raise ValueError("map_yaml_geometry_invalid") from error
    if not isinstance(origin_raw, (list, tuple)) or len(origin_raw) != 3:
        raise ValueError("map_yaml_origin_invalid")
    origin = [float(value) for value in origin_raw]
    if (not math.isfinite(resolution) or resolution <= 0.0 or
            not all(math.isfinite(value) for value in origin) or abs(origin[2]) > 1e-9):
        raise ValueError("map_yaml_geometry_invalid")
    return resolution, origin


def _validate_profile(profile: str) -> None:
    if profile not in SUPPORTED_PROFILES:
        raise ValueError(f"unsupported_profile:{profile}")


def _asset_path(root: Path, value: Any, logical_name: str) -> Path:
    if not isinstance(value, str) or not value or "\\" in value:
        raise ValueError(f"invalid_asset_path:{logical_name}")
    relative = PurePosixPath(value)
    if relative.is_absolute() or any(part in {"", ".", ".."} for part in relative.parts):
        raise ValueError(f"invalid_asset_path:{logical_name}")
    path = root.joinpath(*relative.parts)
    if not _regular_nonempty_file(path):
        raise ValueError(f"missing_or_empty:{value}")
    resolved = path.resolve(strict=True)
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise ValueError(f"asset_path_escapes_bundle:{logical_name}") from error
    return path


def _validate_asset_section(
    root: Path,
    section_name: str,
    section: Any,
    expected_names: tuple[str, ...],
    seen_paths: set[str],
) -> dict[str, dict[str, Any]]:
    if not isinstance(section, dict) or set(section) != set(expected_names):
        raise ValueError(f"manifest_{section_name}_asset_set_mismatch")
    validated: dict[str, dict[str, Any]] = {}
    for logical_name in expected_names:
        record = section.get(logical_name)
        if not isinstance(record, dict) or set(record) != ASSET_RECORD_KEYS:
            raise ValueError(f"invalid_asset_record:{section_name}:{logical_name}")
        relative_path = record.get("path")
        path = _asset_path(root, relative_path, logical_name)
        if relative_path in seen_paths:
            raise ValueError(f"duplicate_asset_path:{relative_path}")
        seen_paths.add(relative_path)
        expected_hash = record.get("sha256")
        expected_size = record.get("size_bytes")
        if not isinstance(expected_hash, str) or not SHA256_PATTERN.fullmatch(expected_hash):
            raise ValueError(f"invalid_asset_sha256:{logical_name}")
        if type(expected_size) is not int or expected_size <= 0:
            raise ValueError(f"invalid_asset_size:{logical_name}")
        if sha256_file(path) != expected_hash or path.stat().st_size != expected_size:
            raise ValueError(f"asset_integrity_mismatch:{path.name}")
        validated[logical_name] = record
    return validated


def validate_bundle(
    root: Path,
    expected_map_id: str | None = None,
    expected_profile: str | None = None,
) -> dict[str, Any]:
    root = root.resolve()
    manifest_path = root / "manifest.json"
    if not _regular_nonempty_file(manifest_path):
        raise ValueError("missing_or_empty:manifest.json")
    manifest = _load_json_object(manifest_path)
    if type(manifest.get("schema_version")) is not int or manifest["schema_version"] != SCHEMA_VERSION:
        raise ValueError("unsupported_manifest_schema")
    map_id = manifest.get("map_id")
    if not isinstance(map_id, str) or not MAP_ID_PATTERN.fullmatch(map_id):
        raise ValueError("invalid_manifest_map_id")
    if expected_map_id is not None and map_id != expected_map_id:
        raise ValueError("manifest_map_id_mismatch")
    profile = manifest.get("profile")
    if not isinstance(profile, str) or profile not in SUPPORTED_PROFILES:
        raise ValueError("invalid_manifest_profile")
    if expected_profile is not None:
        _validate_profile(expected_profile)
        if profile != expected_profile:
            raise ValueError("bundle_profile_mismatch")
    created_at = manifest.get("created_at")
    if not isinstance(created_at, str) or not created_at:
        raise ValueError("invalid_manifest_created_at")
    assets = manifest.get("assets")
    if not isinstance(assets, dict) or set(assets) != {"common", "visual", "lidar"}:
        raise ValueError("manifest_asset_sections_mismatch")

    seen_paths: set[str] = set()
    common = _validate_asset_section(
        root, "common", assets["common"], COMMON_ASSETS, seen_paths)
    for backend_profile in SUPPORTED_PROFILES:
        expected_assets = PROFILE_ASSETS[backend_profile] if backend_profile == profile else ()
        _validate_asset_section(
            root, backend_profile, assets[backend_profile], expected_assets, seen_paths)

    metadata_path = _asset_path(root, common["metadata.json"]["path"], "metadata.json")
    metadata = _load_json_object(metadata_path)
    if metadata.get("map_id") != map_id:
        raise ValueError("metadata_map_id_mismatch")
    if (type(metadata.get("schema_version")) is not int or
            metadata["schema_version"] != SCHEMA_VERSION):
        raise ValueError("metadata_schema_mismatch")
    if metadata.get("profile") != profile:
        raise ValueError("metadata_profile_mismatch")
    if metadata.get("motion_output_connected") is not False:
        raise ValueError("mapping_motion_output_must_be_disconnected")
    if metadata.get("kind") == LIVO_M3_KIND:
        _validate_livo_m3_documents(manifest, metadata)
    return manifest


def _asset_record(path: Path, relative_path: str) -> dict[str, Any]:
    return {
        "path": relative_path,
        "sha256": sha256_file(path),
        "size_bytes": path.stat().st_size,
    }


def _write_bundle(
    source: Path,
    output: Path,
    map_id: str,
    profile: str,
    metadata: dict[str, Any],
    created_at: str,
    provenance: dict[str, Any] | None = None,
) -> dict[str, Any]:
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{map_id}.", dir=output.parent))
    input_names = tuple(dict.fromkeys(COMMON_ASSETS[:-1] + PROFILE_ASSETS[profile]))
    try:
        for filename in input_names:
            shutil.copy2(source / filename, staging / filename)
        with (staging / "metadata.json").open("w", encoding="utf-8") as stream:
            json.dump(metadata, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")

        sections: dict[str, dict[str, dict[str, Any]]] = {
            "common": {},
            "visual": {},
            "lidar": {},
        }
        for filename in COMMON_ASSETS:
            sections["common"][filename] = _asset_record(staging / filename, filename)
        for filename in PROFILE_ASSETS[profile]:
            sections[profile][filename] = _asset_record(staging / filename, filename)
        manifest: dict[str, Any] = {
            "schema_version": SCHEMA_VERSION,
            "map_id": map_id,
            "profile": profile,
            "created_at": created_at,
            "assets": sections,
        }
        if provenance is not None:
            manifest["provenance"] = provenance
        with (staging / "manifest.json").open("w", encoding="utf-8") as stream:
            json.dump(manifest, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
        validate_bundle(staging, map_id, profile)
        os.rename(staging, output)
        return manifest
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def create_bundle(
    source: Path,
    output: Path,
    map_id: str,
    metadata_input: Path,
    profile: str = "visual",
) -> dict[str, Any]:
    if not MAP_ID_PATTERN.fullmatch(map_id):
        raise ValueError("invalid_map_id")
    _validate_profile(profile)
    source = source.resolve()
    output = output.resolve()
    if output.exists():
        raise FileExistsError(f"output_exists:{output}")
    input_names = tuple(dict.fromkeys(COMMON_ASSETS[:-1] + PROFILE_ASSETS[profile]))
    for filename in input_names:
        if not _regular_nonempty_file(source / filename):
            raise ValueError(f"missing_or_empty_source:{filename}")
    metadata = _load_json_object(metadata_input.resolve())
    if metadata.get("motion_output_connected") is not False:
        raise ValueError("metadata_must_prove_motion_output_disconnected")
    if metadata.get("profile") not in (None, profile):
        raise ValueError("metadata_profile_mismatch")
    created_at = dt.datetime.now(dt.timezone.utc).isoformat()
    metadata["map_id"] = map_id
    metadata["created_at"] = created_at
    metadata["schema_version"] = SCHEMA_VERSION
    metadata["profile"] = profile
    return _write_bundle(source, output, map_id, profile, metadata, created_at)


def _validate_m2_fixed_map(root: Path) -> dict[str, Any]:
    root = root.resolve()
    manifest_path = root / "manifest.json"
    if not _regular_nonempty_file(manifest_path):
        raise ValueError("missing_or_empty:m2_manifest.json")
    manifest = _load_json_object(manifest_path)
    expected_manifest_keys = {
        "schema_version", "kind", "scope", "map_id", "profile", "robot_serial",
        "created_at", "assembly_release_sha", "mapping_release_sha",
        "navigation_bundle", "motion_output_connected", "canonical_tf_published",
        "limitations", "assets",
    }
    if set(manifest) != expected_manifest_keys:
        raise ValueError("m2_manifest_key_set_mismatch")
    if (manifest.get("schema_version") != 1 or
            manifest.get("kind") != M2_FIXED_MAP_KIND or
            manifest.get("scope") != M2_FIXED_MAP_SCOPE or
            manifest.get("profile") != "livo"):
        raise ValueError("m2_fixed_map_contract_mismatch")
    if (manifest.get("navigation_bundle") is not False or
            manifest.get("motion_output_connected") is not False or
            manifest.get("canonical_tf_published") is not False):
        raise ValueError("m2_fixed_map_must_be_no_motion_localization_only")
    map_id = manifest.get("map_id")
    if not isinstance(map_id, str) or not MAP_ID_PATTERN.fullmatch(map_id):
        raise ValueError("invalid_m2_map_id")
    serial = _require_nonempty_string(manifest.get("robot_serial"), "m2_robot_serial")
    assembly_sha = _require_git_sha(
        manifest.get("assembly_release_sha"), "m2_assembly_release_sha")
    mapping_sha = _require_git_sha(
        manifest.get("mapping_release_sha"), "m2_mapping_release_sha")
    assets = manifest.get("assets")
    if not isinstance(assets, dict) or set(assets) != set(M2_FIXED_MAP_ASSETS):
        raise ValueError("m2_fixed_map_asset_set_mismatch")
    asset_hashes: dict[str, str] = {}
    for name in M2_FIXED_MAP_ASSETS:
        record = assets.get(name)
        if not isinstance(record, dict) or set(record) != ASSET_RECORD_KEYS:
            raise ValueError(f"invalid_m2_asset_record:{name}")
        if record.get("path") != name:
            raise ValueError(f"invalid_m2_asset_path:{name}")
        path = _asset_path(root, name, f"m2:{name}")
        digest = _require_sha256(record.get("sha256"), f"m2:{name}")
        if record.get("size_bytes") != path.stat().st_size or digest != sha256_file(path):
            raise ValueError(f"m2_asset_integrity_mismatch:{name}")
        asset_hashes[name] = digest

    acceptance = _load_json_object(root / "robot_acceptance.json")
    if (acceptance.get("schema_version") != 1 or
            acceptance.get("kind") !=
            "user_confirmed_robot_a_livo_prototype_sensor_acceptance" or
            acceptance.get("robot_serial") != serial):
        raise ValueError("m2_robot_acceptance_mismatch")
    transform = acceptance.get("ndt_lidar_source_to_base")
    if not isinstance(transform, dict):
        raise ValueError("m2_robot_acceptance_transform_missing")
    translation = transform.get("translation_m")
    quaternion = transform.get("quaternion_xyzw")
    if (not isinstance(translation, list) or len(translation) != 3 or
            not isinstance(quaternion, list) or len(quaternion) != 4 or
            not all(isinstance(value, (int, float)) and not isinstance(value, bool) and
                    math.isfinite(float(value)) for value in translation + quaternion)):
        raise ValueError("m2_robot_acceptance_transform_invalid")
    quaternion_norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
    if not 0.999 <= quaternion_norm <= 1.001:
        raise ValueError("m2_robot_acceptance_quaternion_not_normalized")

    mapping_receipt = _load_json_object(root / "mapping_run_receipt.json")
    if (mapping_receipt.get("robot_serial") != serial or
            mapping_receipt.get("release_sha") != mapping_sha or
            mapping_receipt.get("motion_output_connected") is not False or
            mapping_receipt.get("canonical_tf_published") is not False):
        raise ValueError("m2_mapping_receipt_mismatch")
    trajectory = _load_json_object(root / "trajectory_report.json")
    if (trajectory.get("schema_version") != 1 or
            trajectory.get("kind") != "sunhorizon_livo_m1_trajectory_evidence"):
        raise ValueError("m2_trajectory_report_contract_mismatch")
    return {
        "root": root,
        "manifest": manifest,
        "manifest_sha256": sha256_file(manifest_path),
        "map_id": map_id,
        "robot_serial": serial,
        "assembly_release_sha": assembly_sha,
        "mapping_release_sha": mapping_sha,
        "asset_sha256": asset_hashes,
    }


def validate_livo_projection(root: Path, source_map_sha256: str) -> dict[str, Any]:
    root = root.resolve()
    manifest_path = root / "manifest.json"
    if not _regular_nonempty_file(manifest_path):
        raise ValueError("missing_or_empty:projection_manifest.json")
    manifest = _load_json_object(manifest_path)
    schema_version = manifest.get("schema_version")
    legacy_keys = {
        "schema_version", "map_id", "map_pcd_sha256", "map_pgm_sha256",
        "map_yaml_sha256", "resolution", "ground_z", "obstacle_z", "width", "height",
    }
    provenance_keys = {
        "padding_m", "minimum_ground_points_per_cell",
        "minimum_obstacle_points_per_cell", "free_space_dilation_radius_cells",
        "wall_closing_radius_cells", "unknown_border_cells",
    }
    expected_keys = legacy_keys if schema_version == 1 else legacy_keys | provenance_keys
    if schema_version not in {1, 2} or set(manifest) != expected_keys:
        raise ValueError("projection_manifest_contract_mismatch")
    projection_id = manifest.get("map_id")
    if not isinstance(projection_id, str) or not MAP_ID_PATTERN.fullmatch(projection_id):
        raise ValueError("invalid_projection_map_id")
    paths = {name: _asset_path(root, name, f"projection:{name}")
             for name in ("map.pcd", "map.pgm", "map.yaml")}
    expected_hashes = {
        "map.pcd": _require_sha256(manifest.get("map_pcd_sha256"), "projection:map.pcd"),
        "map.pgm": _require_sha256(manifest.get("map_pgm_sha256"), "projection:map.pgm"),
        "map.yaml": _require_sha256(manifest.get("map_yaml_sha256"), "projection:map.yaml"),
    }
    for name, path in paths.items():
        if sha256_file(path) != expected_hashes[name]:
            raise ValueError(f"projection_asset_integrity_mismatch:{name}")
    if expected_hashes["map.pcd"] != source_map_sha256:
        raise ValueError("projection_source_map_does_not_match_m2_map")
    width, height = _read_pgm_size(paths["map.pgm"])
    if (type(manifest.get("width")) is not int or manifest["width"] != width or
            type(manifest.get("height")) is not int or manifest["height"] != height):
        raise ValueError("projection_dimensions_mismatch")
    resolution = _require_positive_number(manifest.get("resolution"), "projection_resolution")
    yaml_resolution, origin = _map_yaml_geometry(paths["map.yaml"], "map.pgm")
    if abs(yaml_resolution - resolution) > 1e-12:
        raise ValueError("projection_yaml_resolution_mismatch")
    for label in ("ground_z", "obstacle_z"):
        limits = manifest.get(label)
        if (not isinstance(limits, list) or len(limits) != 2 or
                not all(isinstance(value, (int, float)) and not isinstance(value, bool) and
                        math.isfinite(float(value)) for value in limits) or
                float(limits[0]) >= float(limits[1])):
            raise ValueError(f"projection_{label}_invalid")
    parameters: dict[str, Any] | None = None
    if schema_version == 2:
        padding_m = manifest.get("padding_m")
        if (isinstance(padding_m, bool) or not isinstance(padding_m, (int, float)) or
                not math.isfinite(float(padding_m)) or float(padding_m) < 0.0):
            raise ValueError("projection_padding_m_invalid")
        for label in (
            "minimum_ground_points_per_cell",
            "minimum_obstacle_points_per_cell",
        ):
            value = manifest.get(label)
            if type(value) is not int or value <= 0:
                raise ValueError(f"projection_{label}_invalid")
        for label in (
            "free_space_dilation_radius_cells",
            "wall_closing_radius_cells",
            "unknown_border_cells",
        ):
            value = manifest.get(label)
            if type(value) is not int or value < 0:
                raise ValueError(f"projection_{label}_invalid")
        parameters = {
            "resolution_m": resolution,
            "ground_z": manifest["ground_z"],
            "obstacle_z": manifest["obstacle_z"],
            "padding_m": float(padding_m),
            "minimum_ground_points_per_cell":
                manifest["minimum_ground_points_per_cell"],
            "minimum_obstacle_points_per_cell":
                manifest["minimum_obstacle_points_per_cell"],
            "free_space_dilation_radius_cells":
                manifest["free_space_dilation_radius_cells"],
            "wall_closing_radius_cells": manifest["wall_closing_radius_cells"],
            "unknown_border_cells": manifest["unknown_border_cells"],
        }
    return {
        "root": root,
        "manifest": manifest,
        "manifest_sha256": sha256_file(manifest_path),
        "schema_version": schema_version,
        "map_id": projection_id,
        "asset_sha256": expected_hashes,
        "width": width,
        "height": height,
        "resolution_m": resolution,
        "origin_xyyaw": origin,
        "parameters": parameters,
    }


def _validate_keepout(root: Path, projection: dict[str, Any]) -> dict[str, Any]:
    root = root.resolve()
    receipt_path = root / "keepout_receipt.json"
    if not _regular_nonempty_file(receipt_path):
        raise ValueError("missing_or_empty:keepout_receipt.json")
    receipt = _load_json_object(receipt_path)
    required = {
        "schema_version", "kind", "frame_id", "map_id", "accepted_by", "measured_at",
        "map_binding_verified", "map_yaml_sha256", "map_image_sha256", "zones_sha256",
        "keepout_pgm_sha256", "keepout_yaml_sha256", "width", "height", "resolution_m",
        "origin_xyyaw", "unknown_cells_preserved", "keepout_cells", "zones",
    }
    if (set(receipt) != required or receipt.get("schema_version") != 1 or
            receipt.get("kind") != KEEPOUT_RECEIPT_KIND or
            receipt.get("frame_id") != "map"):
        raise ValueError("keepout_receipt_contract_mismatch")
    if receipt.get("map_binding_verified") is not True:
        raise ValueError("keepout_receipt_not_bound_to_projection")
    if receipt.get("map_id") != projection["map_id"]:
        raise ValueError("keepout_projection_map_id_mismatch")
    _require_nonempty_string(receipt.get("accepted_by"), "keepout_accepted_by")
    _require_nonempty_string(receipt.get("measured_at"), "keepout_measured_at")
    zones_sha256 = _require_sha256(receipt.get("zones_sha256"), "keepout_zones")
    zones_path = _asset_path(root, "zones.json", "keepout:zones.json")
    if sha256_file(zones_path) != zones_sha256:
        raise ValueError("keepout_zones_integrity_mismatch")
    zones_document = _load_json_object(zones_path)
    if (zones_document.get("schema_version") != 1 or
            zones_document.get("accepted") is not True or
            zones_document.get("frame_id") != "map" or
            zones_document.get("map_id") != receipt.get("map_id") or
            zones_document.get("accepted_by") != receipt.get("accepted_by") or
            zones_document.get("measured_at") != receipt.get("measured_at") or
            zones_document.get("map_yaml_sha256") != receipt.get("map_yaml_sha256") or
            zones_document.get("map_image_sha256") != receipt.get("map_image_sha256") or
            not isinstance(zones_document.get("zones"), list) or
            len(zones_document["zones"]) != len(receipt.get("zones", []))):
        raise ValueError("keepout_zones_receipt_mismatch")
    if (receipt.get("map_yaml_sha256") != projection["asset_sha256"]["map.yaml"] or
            receipt.get("map_image_sha256") != projection["asset_sha256"]["map.pgm"]):
        raise ValueError("keepout_projection_hash_mismatch")
    if (receipt.get("width") != projection["width"] or
            receipt.get("height") != projection["height"] or
            abs(_require_positive_number(
                receipt.get("resolution_m"), "keepout_resolution") -
                projection["resolution_m"]) > 1e-12):
        raise ValueError("keepout_projection_geometry_mismatch")
    if receipt.get("origin_xyyaw") != projection["origin_xyyaw"]:
        raise ValueError("keepout_projection_origin_mismatch")
    paths = {name: _asset_path(root, name, f"keepout:{name}")
             for name in ("keepout.pgm", "keepout.yaml")}
    expected_hashes = {
        "keepout.pgm": _require_sha256(
            receipt.get("keepout_pgm_sha256"), "keepout:keepout.pgm"),
        "keepout.yaml": _require_sha256(
            receipt.get("keepout_yaml_sha256"), "keepout:keepout.yaml"),
    }
    for name, path in paths.items():
        if sha256_file(path) != expected_hashes[name]:
            raise ValueError(f"keepout_asset_integrity_mismatch:{name}")
    width, height = _read_pgm_size(paths["keepout.pgm"])
    if width != projection["width"] or height != projection["height"]:
        raise ValueError("keepout_mask_dimensions_mismatch")
    keepout_resolution, keepout_origin = _map_yaml_geometry(
        paths["keepout.yaml"], "keepout.pgm")
    origin_matches = all(
        abs(left - right) <= 1e-12
        for left, right in zip(keepout_origin, projection["origin_xyyaw"])
    )
    if (abs(keepout_resolution - projection["resolution_m"]) > 1e-12 or
            not origin_matches):
        raise ValueError("keepout_yaml_geometry_mismatch")
    if type(receipt.get("keepout_cells")) is not int or receipt["keepout_cells"] <= 0:
        raise ValueError("keepout_mask_must_not_be_empty")
    if (type(receipt.get("unknown_cells_preserved")) is not int or
            receipt["unknown_cells_preserved"] < 0):
        raise ValueError("keepout_unknown_audit_invalid")
    if not isinstance(receipt.get("zones"), list) or not receipt["zones"]:
        raise ValueError("keepout_zones_audit_missing")
    return {
        "root": root,
        "receipt": receipt,
        "receipt_sha256": sha256_file(receipt_path),
        "zones_document": zones_document,
        "asset_sha256": expected_hashes,
    }


def _validate_livo_m3_documents(
    manifest: dict[str, Any], metadata: dict[str, Any],
) -> None:
    expected_metadata_keys = {
        "schema_version", "kind", "stage", "map_id", "profile", "created_at",
        "navigation_bundle", "motion_output_connected", "canonical_tf_published",
        "robot_serial", "assembly_release_sha", "source", "projection", "keepout",
        "calibration",
    }
    if set(metadata) != expected_metadata_keys:
        raise ValueError("livo_m3_metadata_key_set_mismatch")
    if (metadata.get("schema_version") != SCHEMA_VERSION or
            metadata.get("kind") != LIVO_M3_KIND or
            metadata.get("stage") != "m3_shadow_navigation" or
            metadata.get("profile") != "lidar" or
            metadata.get("navigation_bundle") is not True or
            metadata.get("motion_output_connected") is not False or
            metadata.get("canonical_tf_published") is not False):
        raise ValueError("livo_m3_metadata_contract_mismatch")
    if (metadata.get("map_id") != manifest.get("map_id") or
            metadata.get("created_at") != manifest.get("created_at")):
        raise ValueError("livo_m3_manifest_metadata_identity_mismatch")
    serial = _require_nonempty_string(metadata.get("robot_serial"), "robot_serial")
    release_sha = _require_git_sha(
        metadata.get("assembly_release_sha"), "assembly_release_sha")

    source = metadata.get("source")
    source_keys = {
        "kind", "scope", "map_id", "manifest_sha256", "map_pcd_sha256",
        "backend_sha256", "trajectory_report_sha256", "mapping_run_receipt_sha256",
        "robot_acceptance_sha256", "assembly_release_sha", "mapping_release_sha",
    }
    if not isinstance(source, dict) or set(source) != source_keys:
        raise ValueError("livo_m3_source_audit_mismatch")
    if source.get("kind") != M2_FIXED_MAP_KIND or source.get("scope") != M2_FIXED_MAP_SCOPE:
        raise ValueError("livo_m3_source_contract_mismatch")
    if not isinstance(source.get("map_id"), str) or not MAP_ID_PATTERN.fullmatch(
            source["map_id"]):
        raise ValueError("livo_m3_source_map_id_invalid")
    for name in (
        "manifest_sha256", "map_pcd_sha256", "backend_sha256",
        "trajectory_report_sha256", "mapping_run_receipt_sha256",
        "robot_acceptance_sha256",
    ):
        _require_sha256(source.get(name), f"source:{name}")
    _require_git_sha(source.get("assembly_release_sha"), "source:assembly_release_sha")
    _require_git_sha(source.get("mapping_release_sha"), "source:mapping_release_sha")

    projection = metadata.get("projection")
    projection_schema_version = projection.get("schema_version") \
        if isinstance(projection, dict) else None
    projection_keys = {
        "schema_version", "map_id", "manifest_sha256", "source_map_pcd_sha256",
        "map_pgm_sha256", "map_yaml_sha256", "width", "height", "resolution_m",
        "origin_xyyaw",
    }
    if projection_schema_version == 2:
        projection_keys.add("parameters")
    if (not isinstance(projection, dict) or
            projection_schema_version not in {1, 2} or
            set(projection) != projection_keys):
        raise ValueError("livo_m3_projection_audit_mismatch")
    if not isinstance(projection.get("map_id"), str) or not MAP_ID_PATTERN.fullmatch(
            projection["map_id"]):
        raise ValueError("livo_m3_projection_map_id_invalid")
    for name in (
        "manifest_sha256", "source_map_pcd_sha256", "map_pgm_sha256", "map_yaml_sha256",
    ):
        _require_sha256(projection.get(name), f"projection:{name}")
    if type(projection.get("width")) is not int or projection["width"] <= 0 or \
            type(projection.get("height")) is not int or projection["height"] <= 0:
        raise ValueError("livo_m3_projection_dimensions_invalid")
    _require_positive_number(projection.get("resolution_m"), "projection:resolution_m")
    origin = projection.get("origin_xyyaw")
    if (not isinstance(origin, list) or len(origin) != 3 or
            not all(isinstance(value, (int, float)) and not isinstance(value, bool) and
                    math.isfinite(float(value)) for value in origin) or abs(origin[2]) > 1e-9):
        raise ValueError("livo_m3_projection_origin_invalid")
    if projection_schema_version == 2:
        parameters = projection.get("parameters")
        expected_parameter_keys = {
            "resolution_m", "ground_z", "obstacle_z", "padding_m",
            "minimum_ground_points_per_cell", "minimum_obstacle_points_per_cell",
            "free_space_dilation_radius_cells", "wall_closing_radius_cells",
            "unknown_border_cells",
        }
        if (not isinstance(parameters, dict) or
                set(parameters) != expected_parameter_keys or
                parameters.get("resolution_m") != projection.get("resolution_m")):
            raise ValueError("livo_m3_projection_parameters_invalid")
        padding_m = parameters.get("padding_m")
        if (isinstance(padding_m, bool) or not isinstance(padding_m, (int, float)) or
                not math.isfinite(float(padding_m)) or float(padding_m) < 0.0):
            raise ValueError("livo_m3_projection_parameters_invalid")
        for label in ("ground_z", "obstacle_z"):
            limits = parameters.get(label)
            if (not isinstance(limits, list) or len(limits) != 2 or
                    not all(isinstance(value, (int, float)) and
                            not isinstance(value, bool) and math.isfinite(float(value))
                            for value in limits) or float(limits[0]) >= float(limits[1])):
                raise ValueError("livo_m3_projection_parameters_invalid")
        for label in (
            "minimum_ground_points_per_cell", "minimum_obstacle_points_per_cell",
        ):
            if type(parameters.get(label)) is not int or parameters[label] <= 0:
                raise ValueError("livo_m3_projection_parameters_invalid")
        for label in (
            "free_space_dilation_radius_cells", "wall_closing_radius_cells",
            "unknown_border_cells",
        ):
            if type(parameters.get(label)) is not int or parameters[label] < 0:
                raise ValueError("livo_m3_projection_parameters_invalid")

    keepout = metadata.get("keepout")
    if not isinstance(keepout, dict) or set(keepout) != {
            "receipt_sha256", "receipt", "zones_document"}:
        raise ValueError("livo_m3_keepout_audit_mismatch")
    _require_sha256(keepout.get("receipt_sha256"), "keepout:receipt_sha256")
    receipt = keepout.get("receipt")
    receipt_keys = {
        "schema_version", "kind", "frame_id", "map_id", "accepted_by", "measured_at",
        "map_binding_verified", "map_yaml_sha256", "map_image_sha256", "zones_sha256",
        "keepout_pgm_sha256", "keepout_yaml_sha256", "width", "height", "resolution_m",
        "origin_xyyaw", "unknown_cells_preserved", "keepout_cells", "zones",
    }
    if (not isinstance(receipt, dict) or set(receipt) != receipt_keys or
            receipt.get("schema_version") != 1 or
            receipt.get("kind") != KEEPOUT_RECEIPT_KIND or
            receipt.get("frame_id") != "map" or
            receipt.get("map_binding_verified") is not True or
            not isinstance(receipt.get("map_id"), str) or
            not MAP_ID_PATTERN.fullmatch(receipt["map_id"]) or
            type(receipt.get("width")) is not int or receipt["width"] <= 0 or
            type(receipt.get("height")) is not int or receipt["height"] <= 0 or
            type(receipt.get("unknown_cells_preserved")) is not int or
            receipt["unknown_cells_preserved"] < 0 or
            type(receipt.get("keepout_cells")) is not int or receipt["keepout_cells"] <= 0 or
            not isinstance(receipt.get("zones"), list) or not receipt["zones"]):
        raise ValueError("livo_m3_keepout_receipt_invalid")
    _require_nonempty_string(receipt.get("accepted_by"), "keepout:accepted_by")
    _require_nonempty_string(receipt.get("measured_at"), "keepout:measured_at")
    _require_positive_number(receipt.get("resolution_m"), "keepout:resolution_m")
    for name in (
        "map_yaml_sha256", "map_image_sha256", "zones_sha256",
        "keepout_pgm_sha256", "keepout_yaml_sha256",
    ):
        _require_sha256(receipt.get(name), f"keepout:{name}")
    receipt_origin = receipt.get("origin_xyyaw")
    if (not isinstance(receipt_origin, list) or len(receipt_origin) != 3 or
            not all(isinstance(value, (int, float)) and not isinstance(value, bool) and
                    math.isfinite(float(value)) for value in receipt_origin) or
            abs(receipt_origin[2]) > 1e-9):
        raise ValueError("livo_m3_keepout_origin_invalid")
    zones_document = keepout.get("zones_document")
    if (not isinstance(zones_document, dict) or
            zones_document.get("schema_version") != 1 or
            zones_document.get("accepted") is not True or
            zones_document.get("frame_id") != "map" or
            zones_document.get("map_id") != receipt.get("map_id") or
            zones_document.get("accepted_by") != receipt.get("accepted_by") or
            zones_document.get("measured_at") != receipt.get("measured_at") or
            zones_document.get("map_yaml_sha256") != receipt.get("map_yaml_sha256") or
            zones_document.get("map_image_sha256") != receipt.get("map_image_sha256") or
            not isinstance(zones_document.get("zones"), list) or
            len(zones_document["zones"]) != len(receipt.get("zones", []))):
        raise ValueError("livo_m3_keepout_zones_audit_invalid")

    calibration = metadata.get("calibration")
    if not isinstance(calibration, dict) or set(calibration) != {
            "robot_urdf_sha256", "calibration_yaml_sha256"}:
        raise ValueError("livo_m3_calibration_audit_mismatch")
    for name in calibration:
        _require_sha256(calibration[name], f"calibration:{name}")

    assets = manifest["assets"]
    common = assets["common"]
    lidar = assets["lidar"]
    checks = (
        (source["map_pcd_sha256"], lidar["map.pcd"]["sha256"], "source_map"),
        (source["backend_sha256"], lidar["backend.yaml"]["sha256"], "source_backend"),
        (source["trajectory_report_sha256"],
         common["trajectory_report.json"]["sha256"], "source_trajectory"),
        (projection["source_map_pcd_sha256"], source["map_pcd_sha256"],
         "projection_source_map"),
        (projection["map_pgm_sha256"], common["map.pgm"]["sha256"], "projection_pgm"),
        (projection["map_yaml_sha256"], common["map.yaml"]["sha256"], "projection_yaml"),
        (receipt.get("keepout_pgm_sha256"), common["keepout.pgm"]["sha256"],
         "keepout_pgm"),
        (receipt.get("keepout_yaml_sha256"), common["keepout.yaml"]["sha256"],
         "keepout_yaml"),
        (calibration["robot_urdf_sha256"], common["robot.urdf"]["sha256"], "robot_urdf"),
        (calibration["calibration_yaml_sha256"],
         common["calibration.yaml"]["sha256"], "calibration_yaml"),
    )
    for left, right, label in checks:
        if left != right:
            raise ValueError(f"livo_m3_audit_asset_mismatch:{label}")
    if (receipt.get("map_id") != projection.get("map_id") or
            receipt.get("map_yaml_sha256") != projection.get("map_yaml_sha256") or
            receipt.get("map_image_sha256") != projection.get("map_pgm_sha256") or
            receipt.get("width") != projection.get("width") or
            receipt.get("height") != projection.get("height") or
            receipt.get("origin_xyyaw") != projection.get("origin_xyyaw") or
            abs(float(receipt.get("resolution_m", -1.0)) -
                float(projection.get("resolution_m", -2.0))) > 1e-12):
        raise ValueError("livo_m3_keepout_projection_mismatch")

    provenance = manifest.get("provenance")
    expected_provenance = {
        "kind": LIVO_M3_KIND,
        "robot_serial": serial,
        "assembly_release_sha": release_sha,
        "source_m2_manifest_sha256": source["manifest_sha256"],
        "source_map_pcd_sha256": source["map_pcd_sha256"],
        "projection_manifest_sha256": projection["manifest_sha256"],
        "keepout_receipt_sha256": keepout["receipt_sha256"],
        "keepout_zones_sha256": receipt.get("zones_sha256"),
    }
    if provenance != expected_provenance:
        raise ValueError("livo_m3_manifest_provenance_mismatch")


def create_livo_m3_bundle(
    m2_fixed_map: Path,
    projection_directory: Path,
    keepout_directory: Path,
    robot_urdf: Path,
    calibration_yaml: Path,
    output: Path,
    map_id: str,
    robot_serial: str,
    release_sha: str,
) -> dict[str, Any]:
    """Promote audited M1/M2 LIVO assets into an immutable M3 shadow map bundle."""
    if not MAP_ID_PATTERN.fullmatch(map_id):
        raise ValueError("invalid_map_id")
    _require_nonempty_string(robot_serial, "robot_serial")
    _require_git_sha(release_sha, "release_sha")
    output = output.resolve()
    if output.exists():
        raise FileExistsError(f"output_exists:{output}")
    m2 = _validate_m2_fixed_map(m2_fixed_map)
    if m2["robot_serial"] != robot_serial:
        raise ValueError("m2_robot_serial_mismatch")
    projection = validate_livo_projection(
        projection_directory, m2["asset_sha256"]["map.pcd"])
    if projection["schema_version"] != 2:
        raise ValueError("projection_schema_v2_required_for_new_m3_bundle")
    keepout = _validate_keepout(keepout_directory, projection)
    robot_urdf = robot_urdf.resolve()
    calibration_yaml = calibration_yaml.resolve()
    for path, label in ((robot_urdf, "robot_urdf"),
                        (calibration_yaml, "calibration_yaml")):
        if not _regular_nonempty_file(path):
            raise ValueError(f"missing_or_empty:{label}")

    output.parent.mkdir(parents=True, exist_ok=True)
    source_staging = Path(tempfile.mkdtemp(prefix=".livo-m3-source.", dir=output.parent))
    try:
        copies = {
            "map.pgm": projection["root"] / "map.pgm",
            "map.yaml": projection["root"] / "map.yaml",
            "keepout.pgm": keepout["root"] / "keepout.pgm",
            "keepout.yaml": keepout["root"] / "keepout.yaml",
            "robot.urdf": robot_urdf,
            "calibration.yaml": calibration_yaml,
            "trajectory_report.json": m2["root"] / "trajectory_report.json",
            "map.pcd": m2["root"] / "map.pcd",
            "backend.yaml": m2["root"] / "backend.yaml",
        }
        for name, source in copies.items():
            shutil.copy2(source, source_staging / name)
        created_at = dt.datetime.now(dt.timezone.utc).isoformat()
        projection_metadata = {
            "schema_version": projection["schema_version"],
            "map_id": projection["map_id"],
            "manifest_sha256": projection["manifest_sha256"],
            "source_map_pcd_sha256": projection["asset_sha256"]["map.pcd"],
            "map_pgm_sha256": projection["asset_sha256"]["map.pgm"],
            "map_yaml_sha256": projection["asset_sha256"]["map.yaml"],
            "width": projection["width"],
            "height": projection["height"],
            "resolution_m": projection["resolution_m"],
            "origin_xyyaw": projection["origin_xyyaw"],
        }
        if projection["schema_version"] == 2:
            projection_metadata["parameters"] = projection["parameters"]
        metadata = {
            "schema_version": SCHEMA_VERSION,
            "kind": LIVO_M3_KIND,
            "stage": "m3_shadow_navigation",
            "map_id": map_id,
            "profile": "lidar",
            "created_at": created_at,
            "navigation_bundle": True,
            "motion_output_connected": False,
            "canonical_tf_published": False,
            "robot_serial": robot_serial,
            "assembly_release_sha": release_sha,
            "source": {
                "kind": M2_FIXED_MAP_KIND,
                "scope": M2_FIXED_MAP_SCOPE,
                "map_id": m2["map_id"],
                "manifest_sha256": m2["manifest_sha256"],
                "map_pcd_sha256": m2["asset_sha256"]["map.pcd"],
                "backend_sha256": m2["asset_sha256"]["backend.yaml"],
                "trajectory_report_sha256":
                    m2["asset_sha256"]["trajectory_report.json"],
                "mapping_run_receipt_sha256":
                    m2["asset_sha256"]["mapping_run_receipt.json"],
                "robot_acceptance_sha256":
                    m2["asset_sha256"]["robot_acceptance.json"],
                "assembly_release_sha": m2["assembly_release_sha"],
                "mapping_release_sha": m2["mapping_release_sha"],
            },
            "projection": projection_metadata,
            "keepout": {
                "receipt_sha256": keepout["receipt_sha256"],
                "receipt": keepout["receipt"],
                "zones_document": keepout["zones_document"],
            },
            "calibration": {
                "robot_urdf_sha256": sha256_file(robot_urdf),
                "calibration_yaml_sha256": sha256_file(calibration_yaml),
            },
        }
        provenance = {
            "kind": LIVO_M3_KIND,
            "robot_serial": robot_serial,
            "assembly_release_sha": release_sha,
            "source_m2_manifest_sha256": m2["manifest_sha256"],
            "source_map_pcd_sha256": m2["asset_sha256"]["map.pcd"],
            "projection_manifest_sha256": projection["manifest_sha256"],
            "keepout_receipt_sha256": keepout["receipt_sha256"],
            "keepout_zones_sha256": keepout["receipt"]["zones_sha256"],
        }
        return _write_bundle(
            source_staging, output, map_id, "lidar", metadata, created_at, provenance)
    finally:
        shutil.rmtree(source_staging, ignore_errors=True)


def _validate_v2_bundle(root: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    root = root.resolve()
    manifest_path = root / "manifest.json"
    if not _regular_nonempty_file(manifest_path):
        raise ValueError("missing_or_empty:manifest.json")
    manifest = _load_json_object(manifest_path)
    if manifest.get("schema_version") != 2:
        raise ValueError("source_is_not_schema_v2")
    map_id = manifest.get("map_id")
    if not isinstance(map_id, str) or not MAP_ID_PATTERN.fullmatch(map_id):
        raise ValueError("invalid_manifest_map_id")
    assets = manifest.get("assets")
    if not isinstance(assets, dict) or set(assets) != set(V2_REQUIRED_ASSETS):
        raise ValueError("manifest_asset_set_mismatch")
    for filename in V2_REQUIRED_ASSETS:
        path = root / filename
        if not _regular_nonempty_file(path):
            raise ValueError(f"missing_or_empty:{filename}")
        record = assets.get(filename)
        if not isinstance(record, dict):
            raise ValueError(f"invalid_asset_record:{filename}")
        actual_hash = sha256_file(path)
        if record.get("sha256") != actual_hash or record.get("size_bytes") != path.stat().st_size:
            raise ValueError(f"asset_integrity_mismatch:{filename}")
        if manifest.get(manifest_key(filename)) != actual_hash:
            raise ValueError(f"flat_asset_hash_mismatch:{filename}")
    metadata = _load_json_object(root / "metadata.json")
    if metadata.get("map_id") != map_id:
        raise ValueError("metadata_map_id_mismatch")
    if metadata.get("schema_version") != 2:
        raise ValueError("metadata_schema_mismatch")
    if metadata.get("profile") not in (None, "visual"):
        raise ValueError("v2_profile_must_be_visual")
    if metadata.get("motion_output_connected") is not False:
        raise ValueError("mapping_motion_output_must_be_disconnected")
    return manifest, metadata


def convert_v2_bundle(source: Path, output: Path) -> dict[str, Any]:
    """Copy a valid immutable v2 visual bundle into a new schema-v3 bundle."""
    source = source.resolve()
    output = output.resolve()
    if output.exists():
        raise FileExistsError(f"output_exists:{output}")
    try:
        output.relative_to(source)
    except ValueError:
        pass
    else:
        raise ValueError("output_must_not_be_inside_source_bundle")

    manifest, metadata = _validate_v2_bundle(source)
    source_manifest_hash = sha256_file(source / "manifest.json")
    source_asset_hashes = {
        filename: manifest["assets"][filename]["sha256"]
        for filename in V2_REQUIRED_ASSETS
    }
    converted_at = dt.datetime.now(dt.timezone.utc).isoformat()
    conversion = {
        "source_schema_version": 2,
        "source_manifest_sha256": source_manifest_hash,
        "source_created_at": manifest.get("created_at", ""),
        "source_asset_sha256": source_asset_hashes,
        "converted_at": converted_at,
    }
    converted_metadata = dict(metadata)
    converted_metadata["schema_version"] = SCHEMA_VERSION
    converted_metadata["profile"] = "visual"
    converted_metadata["conversion_provenance"] = conversion
    created_at = manifest.get("created_at")
    if not isinstance(created_at, str) or not created_at:
        created_at = converted_at
    return _write_bundle(
        source,
        output,
        manifest["map_id"],
        "visual",
        converted_metadata,
        created_at,
        provenance={"v2_conversion": conversion},
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    create = subparsers.add_parser("create")
    create.add_argument("--source", type=Path, required=True)
    create.add_argument("--output", type=Path, required=True)
    create.add_argument("--map-id", required=True)
    create.add_argument("--metadata", type=Path, required=True)
    create.add_argument("--profile", choices=SUPPORTED_PROFILES, default="visual")
    validate = subparsers.add_parser("validate")
    validate.add_argument("--bundle", type=Path, required=True)
    validate.add_argument("--map-id")
    validate.add_argument("--profile", choices=SUPPORTED_PROFILES)
    convert = subparsers.add_parser("convert-v2")
    convert.add_argument("--source", type=Path, required=True)
    convert.add_argument("--output", type=Path, required=True)
    livo_m3 = subparsers.add_parser("create-livo-m3")
    livo_m3.add_argument("--m2-fixed-map", type=Path, required=True)
    livo_m3.add_argument("--projection", type=Path, required=True)
    livo_m3.add_argument("--keepout", type=Path, required=True)
    livo_m3.add_argument("--robot-urdf", type=Path, required=True)
    livo_m3.add_argument("--calibration", type=Path, required=True)
    livo_m3.add_argument("--output", type=Path, required=True)
    livo_m3.add_argument("--map-id", required=True)
    livo_m3.add_argument("--robot-serial", required=True)
    livo_m3.add_argument("--release-sha", required=True)
    arguments = parser.parse_args()
    if arguments.command == "create":
        result = create_bundle(
            arguments.source,
            arguments.output,
            arguments.map_id,
            arguments.metadata,
            arguments.profile,
        )
        bundle_path = arguments.output
    elif arguments.command == "convert-v2":
        result = convert_v2_bundle(arguments.source, arguments.output)
        bundle_path = arguments.output
    elif arguments.command == "create-livo-m3":
        result = create_livo_m3_bundle(
            arguments.m2_fixed_map,
            arguments.projection,
            arguments.keepout,
            arguments.robot_urdf,
            arguments.calibration,
            arguments.output,
            arguments.map_id,
            arguments.robot_serial,
            arguments.release_sha,
        )
        bundle_path = arguments.output
    else:
        result = validate_bundle(arguments.bundle, arguments.map_id, arguments.profile)
        bundle_path = arguments.bundle
    print(json.dumps({
        "ok": True,
        "map_id": result["map_id"],
        "profile": result["profile"],
        "manifest_sha256": sha256_file(bundle_path / "manifest.json"),
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
