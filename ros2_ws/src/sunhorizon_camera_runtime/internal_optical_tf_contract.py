#!/usr/bin/env python3
"""Validate production camera link-to-optical TF parameters and receipt.

No camera transform numbers live in this module.  A separately measured and
reviewed Robot A parameter file must be hash-bound by a motion-only receipt.
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import sys


EXPECTED_ROBOT_SERIAL = "1421026005616"
EXPECTED_KIND = "robot_a_camera_internal_optical_tf_acceptance"
EXPECTED_USAGE = ["motion_acceptance"]
EXPECTED_OWNERSHIP = {
    "publisher_node": "/sunhorizon_camera_runtime",
    "vendor_orbbec_tf_publishers_allowed": False,
    "duplicate_child_frames_allowed": False,
}
ROLE_FRAMES = {
    "chest": ("camera_chest_link", "camera_chest_color_optical_frame"),
    "belly": ("camera_belly_link", "camera_belly_depth_optical_frame"),
}
EXPECTED_CAMERA_SERIALS = {"chest": "CPCAC53000NF", "belly": "CP9W463000ZW"}
CAPTURE_BINDING_SCHEMA = "robot_a_camera_rosbag_source_binding_v1"
CAPTURE_SHA_SEMANTICS = "canonical_json(robot_camera_and_rosbag_source_binding_v1)"
SEGMENT_SUFFIXES = (".db3", ".mcap", ".mcap.zstd")
TF_STATIC_TOPIC_CONTRACT = {
    "name": "/tf_static",
    "type": "tf2_msgs/msg/TFMessage",
    "serialization_format": "cdr",
}
EXPECTED_ASSETS = {
    "camera_params",
    "measurement_record",
    "chest_vendor_tf_static",
    "chest_source_capture",
    "belly_vendor_tf_static",
    "belly_source_capture",
}
SHA256_RE = re.compile(r"[0-9a-f]{64}")
IDENTITY_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._@:-]{2,127}")
TRANSFORM_TOLERANCE = 1e-8
REPEATABILITY_EVIDENCE_KIND = "robot_a_tf_repeatability_evidence"
REVIEW_EDGE_BINDINGS = {
    f"camera_internal.{role}": {
        "parent": frames[0], "child": frames[1],
        "sensor_role": f"camera_{role}",
        "sensor_serial": EXPECTED_CAMERA_SERIALS[role],
    }
    for role, frames in ROLE_FRAMES.items()
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _nonempty(value: object) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _identity(value: object, label: str) -> str:
    if (
        not isinstance(value, str)
        or value != value.strip()
        or IDENTITY_RE.fullmatch(value) is None
    ):
        raise ValueError(f"camera_internal_tf_identity_invalid:{label}")
    return value.casefold()


def _iso_date(value: object) -> bool:
    if not _nonempty(value):
        return False
    try:
        datetime.date.fromisoformat(value)
    except ValueError:
        return False
    return True


def _accepted_date(value: object) -> datetime.date:
    if not _iso_date(value):
        raise ValueError("camera_internal_tf_date_invalid")
    result = datetime.date.fromisoformat(str(value))
    if result > datetime.date.today():
        raise ValueError("camera_internal_tf_date_in_future")
    return result


def _regular_absolute(path: Path, label: str) -> Path:
    if (
        not path.is_absolute()
        or path.is_symlink()
        or not path.is_file()
        or path.stat().st_size == 0
    ):
        raise ValueError(f"{label}_must_be_absolute_nonempty_regular_file")
    return path


def _canonical_sha256(value: object) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _absolute_directory(path: Path, label: str) -> Path:
    if not path.is_absolute() or path.is_symlink() or not path.is_dir():
        raise ValueError(f"{label}_must_be_absolute_non_symlink_directory")
    return path


def _safe_relative_path(value: object, label: str) -> str:
    if not _nonempty(value):
        raise ValueError(f"{label}_relative_path_invalid")
    path = Path(str(value))
    if path.is_absolute() or ".." in path.parts or path.as_posix() in {"", "."}:
        raise ValueError(f"{label}_relative_path_invalid")
    return path.as_posix()


def _bound_bag_file(
    bag: Path, asset: object, label: str, expected_relative: str | None = None
) -> tuple[dict, Path]:
    if not isinstance(asset, dict) or set(asset) != {
        "relative_path", "bytes", "sha256", "stable_regular_file_checked"
    }:
        raise ValueError(f"{label}_asset_invalid")
    relative = _safe_relative_path(asset.get("relative_path"), label)
    if expected_relative is not None and relative != expected_relative:
        raise ValueError(f"{label}_relative_path_invalid")
    path = bag / relative
    try:
        path.resolve(strict=True).relative_to(bag.resolve(strict=True))
    except (OSError, ValueError) as error:
        raise ValueError(f"{label}_outside_bag") from error
    path = _regular_absolute(path, label)
    if (
        not isinstance(asset.get("bytes"), int)
        or isinstance(asset.get("bytes"), bool)
        or asset["bytes"] != path.stat().st_size
        or asset.get("stable_regular_file_checked") is not True
        or not isinstance(asset.get("sha256"), str)
        or SHA256_RE.fullmatch(asset["sha256"]) is None
        or asset.get("sha256") != sha256(path)
    ):
        raise ValueError(f"{label}_asset_hash_mismatch")
    return dict(asset), path


def _validate_capture_binding(
    document: dict, role: str
) -> tuple[dict[str, object], set[tuple[int, int, str]]]:
    capture = document.get("source_capture")
    if (
        not isinstance(capture, dict)
        or capture.get("binding_complete") is not True
        or capture.get("binding_blockers") != []
        or capture.get("sha256_semantics") != CAPTURE_SHA_SEMANTICS
    ):
        raise ValueError(f"camera_internal_tf_capture_unbound:{role}")
    bag = _absolute_directory(
        Path(str(capture.get("path", ""))), f"camera_internal_tf_capture:{role}"
    )
    metadata_asset, _ = _bound_bag_file(
        bag, capture.get("metadata"), f"camera_internal_tf_metadata:{role}",
        "metadata.yaml")
    segments = capture.get("actual_read_segments")
    if not isinstance(segments, list) or not segments:
        raise ValueError(f"camera_internal_tf_segments_missing:{role}")
    segment_assets = []
    segment_hashes = {}
    for index, asset in enumerate(segments):
        validated, path = _bound_bag_file(
            bag, asset, f"camera_internal_tf_segment:{role}:{index}")
        relative = validated["relative_path"]
        if relative in segment_hashes or not any(
            path.name.endswith(suffix) for suffix in SEGMENT_SUFFIXES
        ):
            raise ValueError(f"camera_internal_tf_segment_invalid:{role}:{relative}")
        segment_hashes[relative] = validated["sha256"]
        segment_assets.append(validated)
    if segment_assets != sorted(segment_assets, key=lambda item: item["relative_path"]):
        raise ValueError(f"camera_internal_tf_segments_not_sorted:{role}")

    read_records = capture.get("read_records")
    if not isinstance(read_records, list) or not read_records:
        raise ValueError(f"camera_internal_tf_read_records_missing:{role}")
    references = set()
    for index, record in enumerate(read_records):
        if not isinstance(record, dict) or set(record) != {
            "filtered_record_index", "storage_timestamp_ns", "segment_relative_path",
            "segment_resolution_method", "segment_sha256"
        }:
            raise ValueError(f"camera_internal_tf_read_record_invalid:{role}:{index}")
        filtered_index = record.get("filtered_record_index")
        timestamp = record.get("storage_timestamp_ns")
        relative = _safe_relative_path(
            record.get("segment_relative_path"),
            f"camera_internal_tf_read_record:{role}:{index}")
        if (
            not isinstance(filtered_index, int)
            or isinstance(filtered_index, bool)
            or filtered_index <= 0
            or not isinstance(timestamp, int)
            or isinstance(timestamp, bool)
            or timestamp < 0
            or not _nonempty(record.get("segment_resolution_method"))
            or record.get("segment_sha256") != segment_hashes.get(relative)
        ):
            raise ValueError(f"camera_internal_tf_read_record_invalid:{role}:{index}")
        reference = (filtered_index, timestamp, relative)
        if reference in references:
            raise ValueError(f"camera_internal_tf_read_record_duplicate:{role}")
        references.add(reference)
    if set(segment_hashes) != {reference[2] for reference in references}:
        raise ValueError(f"camera_internal_tf_segment_reference_invalid:{role}")

    reader_metadata = capture.get("reader_metadata")
    if not isinstance(reader_metadata, dict) or reader_metadata.get("api_available") is not True:
        raise ValueError(f"camera_internal_tf_reader_metadata_invalid:{role}")
    storage_identifier = reader_metadata.get("storage_identifier")
    relative_paths = reader_metadata.get("relative_file_paths")
    if (
        not _nonempty(storage_identifier)
        or not isinstance(relative_paths, list)
        or len(relative_paths) != len(set(relative_paths))
    ):
        raise ValueError(f"camera_internal_tf_reader_metadata_invalid:{role}")
    normalized_paths = {
        _safe_relative_path(value, f"camera_internal_tf_reader_metadata:{role}")
        for value in relative_paths
    }
    if not set(segment_hashes).issubset(normalized_paths):
        raise ValueError(f"camera_internal_tf_reader_metadata_invalid:{role}")

    binding = capture.get("binding")
    expected_binding = {
        "schema": CAPTURE_BINDING_SCHEMA,
        "robot_serial": EXPECTED_ROBOT_SERIAL,
        "camera_role": role,
        "camera_serial": EXPECTED_CAMERA_SERIALS[role],
        "observed_by": document.get("observed_by"),
        "chain_parent": ROLE_FRAMES[role][0],
        "chain_child": ROLE_FRAMES[role][1],
        "bag_path": str(bag),
        "metadata": metadata_asset,
        "storage_identifier": storage_identifier,
        "tf_static_topic": TF_STATIC_TOPIC_CONTRACT,
        "read_records": read_records,
        "segments": segment_assets,
    }
    if binding != expected_binding:
        raise ValueError(f"camera_internal_tf_binding_document_invalid:{role}")
    binding_sha = capture.get("sha256")
    if (
        not isinstance(binding_sha, str)
        or SHA256_RE.fullmatch(binding_sha) is None
        or binding_sha != _canonical_sha256(binding)
    ):
        raise ValueError(f"camera_internal_tf_binding_hash_mismatch:{role}")
    return {
        "path": str(bag), "filename": bag.name, "sha256": binding_sha,
    }, references


def _unique_yaml_scalar(text: str, key: str) -> str:
    matches = re.findall(
        rf"^\s*{re.escape(key)}:\s*([^#\n]+?)\s*(?:#.*)?$", text, re.MULTILINE)
    if len(matches) != 1:
        raise ValueError(f"camera_internal_tf_parameter_invalid:{key}")
    return matches[0].strip().strip("\"'")


def _vector(value: object, length: int, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"camera_internal_tf_vector_invalid:{label}")
    parsed = [float(component) for component in value]
    if not all(math.isfinite(component) for component in parsed):
        raise ValueError(f"camera_internal_tf_vector_invalid:{label}")
    return parsed


def _unit_quaternion(value: object, label: str) -> list[float]:
    quaternion = _vector(value, 4, label)
    norm = math.sqrt(sum(component * component for component in quaternion))
    if not math.isfinite(norm) or norm <= 0.0 or abs(norm - 1.0) > 1e-6:
        raise ValueError(f"camera_internal_tf_quaternion_invalid:{label}")
    return [component / norm for component in quaternion]


def _same_vector(left: list[float], right: list[float]) -> bool:
    return len(left) == len(right) and all(
        abs(actual - expected) <= TRANSFORM_TOLERANCE
        for actual, expected in zip(left, right)
    )


def _same_quaternion(left: list[float], right: list[float]) -> bool:
    try:
        left_q = _unit_quaternion(left, "comparison.left")
        right_q = _unit_quaternion(right, "comparison.right")
    except ValueError:
        return False
    return 1.0 - abs(sum(a * b for a, b in zip(left_q, right_q))) <= TRANSFORM_TOLERANCE


def _quaternion_geodesic_deg(left: list[float], right: list[float]) -> float:
    left_q = _unit_quaternion(left, "repeatability.left")
    right_q = _unit_quaternion(right, "repeatability.right")
    dot = min(1.0, abs(sum(a * b for a, b in zip(left_q, right_q))))
    return math.degrees(2.0 * math.acos(dot))


def _asset_file(item: object, key: str) -> Path:
    if not isinstance(item, dict):
        raise ValueError(f"camera_internal_tf_asset_invalid:{key}")
    path = _regular_absolute(Path(str(item.get("path", ""))), f"camera_internal_tf_{key}")
    if (
        item.get("filename") != path.name
        or not isinstance(item.get("sha256"), str)
        or SHA256_RE.fullmatch(item["sha256"]) is None
        or sha256(path) != item["sha256"]
    ):
        raise ValueError(f"camera_internal_tf_asset_hash_mismatch:{key}")
    return path


def _measurement_timestamp(value: object, label: str) -> str:
    if not isinstance(value, str) or value != value.strip() or not value:
        raise ValueError(f"camera_internal_tf_repeatability_timestamp_invalid:{label}")
    try:
        parsed = datetime.datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as error:
        raise ValueError(
            f"camera_internal_tf_repeatability_timestamp_invalid:{label}"
        ) from error
    if parsed.tzinfo is None:
        raise ValueError(f"camera_internal_tf_repeatability_timestamp_invalid:{label}")
    now = datetime.datetime.now(parsed.tzinfo)
    if parsed > now:
        raise ValueError(f"camera_internal_tf_repeatability_timestamp_in_future:{label}")
    return value


def _repeatability_from_evidence(evidence: Path, role: str) -> tuple[float, float]:
    label = f"camera_internal.{role}"
    try:
        document = json.loads(evidence.read_text(encoding="utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(f"camera_internal_tf_repeatability_evidence_invalid:{role}") from error
    if (
        not isinstance(document, dict)
        or document.get("schema_version") != 1
        or document.get("kind") != REPEATABILITY_EVIDENCE_KIND
        or document.get("robot_scope") != "Robot A"
        or document.get("robot_serial") != EXPECTED_ROBOT_SERIAL
    ):
        raise ValueError(f"camera_internal_tf_repeatability_evidence_invalid:{role}")
    edges = document.get("edges")
    record = edges.get(label) if isinstance(edges, dict) else None
    expected = REVIEW_EDGE_BINDINGS[label]
    if not isinstance(record, dict) or any(
        record.get(key) != value for key, value in expected.items()
    ):
        raise ValueError(f"camera_internal_tf_repeatability_edge_binding_invalid:{role}")
    samples = record.get("samples")
    if not isinstance(samples, list) or len(samples) < 3:
        raise ValueError(f"camera_internal_tf_repeatability_samples_insufficient:{role}")
    translations, rotations, timestamps, hashes = [], [], set(), set()
    for index, sample in enumerate(samples):
        sample_label = f"{role}:{index}"
        if not isinstance(sample, dict) or set(sample) != {
            "captured_at", "translation_xyz", "rotation_xyzw", "capture"
        }:
            raise ValueError(f"camera_internal_tf_repeatability_sample_invalid:{sample_label}")
        timestamp = _measurement_timestamp(sample.get("captured_at"), sample_label)
        if timestamp in timestamps:
            raise ValueError(f"camera_internal_tf_repeatability_timestamp_duplicate:{role}")
        timestamps.add(timestamp)
        translations.append(_vector(
            sample.get("translation_xyz"), 3, f"{sample_label}.translation"))
        rotations.append(_unit_quaternion(
            sample.get("rotation_xyzw"), f"{sample_label}.rotation"))
        capture = sample.get("capture")
        path = _asset_file(capture, f"repeatability:{sample_label}")
        if not isinstance(capture, dict) or capture.get("sha256") in hashes:
            raise ValueError(f"camera_internal_tf_repeatability_capture_invalid:{sample_label}")
        hashes.add(capture["sha256"])
    translation_spread = max(
        math.dist(translations[left], translations[right])
        for left in range(len(samples)) for right in range(left + 1, len(samples))
    )
    rotation_spread = max(
        _quaternion_geodesic_deg(rotations[left], rotations[right])
        for left in range(len(samples)) for right in range(left + 1, len(samples))
    )
    return translation_spread, rotation_spread


def _quaternion_multiply(left: list[float], right: list[float]) -> list[float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return [
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ]


def _rotate(q: list[float], vector: list[float]) -> list[float]:
    x, y, z, w = q
    matrix = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    return [sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3)]


def _compose(
    left_translation: list[float], left_rotation: list[float],
    right_translation: list[float], right_rotation: list[float],
) -> tuple[list[float], list[float]]:
    rotated = _rotate(left_rotation, right_translation)
    translation = [left_translation[index] + rotated[index] for index in range(3)]
    rotation = _quaternion_multiply(left_rotation, right_rotation)
    norm = math.sqrt(sum(value * value for value in rotation))
    return translation, [value / norm for value in rotation]


def _camera_evidence_transform(
    path: Path, role: str
) -> tuple[dict[str, object], dict[str, object], str, datetime.date]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(f"camera_internal_tf_evidence_unreadable:{role}") from error
    if (
        not isinstance(document, dict)
        or document.get("schema_version") != 1
        or document.get("kind") != "robot_a_camera_vendor_tf_static_evidence"
        or document.get("robot_scope") != "Robot A"
        or document.get("robot_serial") != EXPECTED_ROBOT_SERIAL
        or document.get("role") != role
        or document.get("camera_serial") != EXPECTED_CAMERA_SERIALS[role]
    ):
        raise ValueError(f"camera_internal_tf_evidence_identity_invalid:{role}")
    observed_by = _identity(document.get("observed_by"), f"{role}.observed_by")
    observed_on = _accepted_date(document.get("observed_on"))
    capture_asset, read_references = _validate_capture_binding(document, role)
    edges = {}
    children = set()
    for index, item in enumerate(document.get("transforms", [])):
        if not isinstance(item, dict):
            raise ValueError(f"camera_internal_tf_evidence_transform_invalid:{role}:{index}")
        parent, child = item.get("parent"), item.get("child")
        translation = _vector(item.get("translation_xyz"), 3, f"{role}:{index}:translation")
        rotation = _vector(item.get("rotation_xyzw"), 4, f"{role}:{index}:rotation")
        header_stamp_ns = item.get("header_stamp_ns")
        source = item.get("source_record")
        if not isinstance(source, dict) or set(source) != {
            "filtered_record_index", "storage_timestamp_ns", "segment_relative_path"
        }:
            raise ValueError(f"camera_internal_tf_transform_source_invalid:{role}:{index}")
        reference = (
            source.get("filtered_record_index"), source.get("storage_timestamp_ns"),
            source.get("segment_relative_path"))
        if (
            not _nonempty(parent) or not _nonempty(child) or parent == child
            or parent in edges or child in children
            or abs(math.sqrt(sum(value * value for value in rotation)) - 1.0) > 1e-6
            or not isinstance(header_stamp_ns, int) or isinstance(header_stamp_ns, bool)
            or header_stamp_ns < 0
            or not isinstance(reference[0], int) or isinstance(reference[0], bool)
            or not isinstance(reference[1], int) or isinstance(reference[1], bool)
            or not _nonempty(reference[2])
            or reference not in read_references
        ):
            raise ValueError(f"camera_internal_tf_evidence_transform_invalid:{role}:{index}")
        edges[parent] = (child, translation, rotation)
        children.add(child)
    current, target = ROLE_FRAMES[role]
    translation = [0.0, 0.0, 0.0]
    rotation = [0.0, 0.0, 0.0, 1.0]
    visited = set()
    while current != target:
        if current in visited or current not in edges:
            raise ValueError(f"camera_internal_tf_evidence_chain_incomplete:{role}")
        visited.add(current)
        child, edge_translation, edge_rotation = edges[current]
        translation, rotation = _compose(translation, rotation, edge_translation, edge_rotation)
        current = child
    if len(visited) != len(edges):
        raise ValueError(f"camera_internal_tf_evidence_chain_ambiguous:{role}")
    return {
        "parent": ROLE_FRAMES[role][0],
        "child": target,
        "translation_xyz": translation,
        "rotation_xyzw": rotation,
    }, capture_asset, observed_by, observed_on


def _review_identities(
    review: object, role: str
) -> tuple[set[str], list[datetime.date]]:
    if not isinstance(review, dict):
        raise ValueError(f"camera_internal_tf_review_evidence_missing:{role}")
    measured_by = _identity(review.get("measured_by"), f"{role}.measured_by")
    reviewed_by = _identity(review.get("reviewed_by"), f"{role}.reviewed_by")
    if measured_by == reviewed_by:
        raise ValueError(f"camera_internal_tf_review_independence_invalid:{role}")
    measured_on = _accepted_date(review.get("measured_on"))
    reviewed_on = _accepted_date(review.get("reviewed_on"))
    if reviewed_on < measured_on:
        raise ValueError(f"camera_internal_tf_review_precedes_measurement:{role}")
    try:
        translation = float(review.get("translation_repeatability_m"))
        rotation = float(review.get("rotation_repeatability_deg"))
    except (TypeError, ValueError) as error:
        raise ValueError(f"camera_internal_tf_repeatability_invalid:{role}") from error
    if (
        not math.isfinite(translation)
        or not math.isfinite(rotation)
        or translation < 0.0
        or translation > 0.01
        or rotation < 0.0
        or rotation > 0.5
    ):
        raise ValueError(f"camera_internal_tf_repeatability_invalid:{role}")
    evidence = _regular_absolute(
        Path(str(review.get("evidence_path", ""))), f"camera_internal_tf_review_evidence:{role}")
    if review.get("evidence_sha256") != sha256(evidence):
        raise ValueError(f"camera_internal_tf_review_evidence_hash_mismatch:{role}")
    measured_translation, measured_rotation = _repeatability_from_evidence(evidence, role)
    if (
        not math.isclose(translation, measured_translation, rel_tol=0.0, abs_tol=1e-9)
        or not math.isclose(rotation, measured_rotation, rel_tol=0.0, abs_tol=1e-9)
    ):
        raise ValueError(f"camera_internal_tf_repeatability_claim_mismatch:{role}")
    return {measured_by, reviewed_by}, [measured_on, reviewed_on]


def _yaml_vector(text: str, key: str, length: int) -> list[float]:
    scalar = _unique_yaml_scalar(text, key)
    if not scalar.startswith("[") or not scalar.endswith("]"):
        raise ValueError(f"camera_internal_tf_parameter_invalid:{key}")
    try:
        return _vector(
            [float(item.strip()) for item in scalar[1:-1].split(",")], length, key)
    except ValueError as error:
        raise ValueError(f"camera_internal_tf_parameter_invalid:{key}") from error


def _parameter_transforms(text: str) -> dict[str, dict[str, object]]:
    if _unique_yaml_scalar(text, "publish_internal_optical_tf").lower() != "true":
        raise ValueError("camera_internal_tf_publish_not_enabled")
    transforms = {}
    children = set()
    for role, expected_frames in ROLE_FRAMES.items():
        parent = _unique_yaml_scalar(text, f"{role}_internal_tf_parent_frame")
        child = _unique_yaml_scalar(text, f"{role}_internal_tf_child_frame")
        if (parent, child) != expected_frames or parent == child:
            raise ValueError(f"camera_internal_tf_frame_invalid:{role}")
        if child in children:
            raise ValueError("camera_internal_tf_duplicate_child_frame")
        children.add(child)
        translation = _yaml_vector(text, f"{role}_internal_tf_translation_xyz", 3)
        rotation = _yaml_vector(text, f"{role}_internal_tf_rotation_xyzw", 4)
        if abs(math.sqrt(sum(value * value for value in rotation)) - 1.0) > 1e-6:
            raise ValueError(f"camera_internal_tf_quaternion_invalid:{role}")
        transforms[role] = {
            "parent": parent,
            "child": child,
            "translation_xyz": translation,
            "rotation_xyzw": rotation,
        }
    return transforms


def validate(receipt_path: Path, params_path: Path, expected_serial: str) -> str:
    receipt_path = _regular_absolute(receipt_path, "camera_internal_tf_receipt")
    params_path = _regular_absolute(params_path, "camera_internal_tf_params")
    try:
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ValueError("camera_internal_tf_receipt_unreadable") from error
    if not isinstance(receipt, dict) or receipt.get("schema_version") != 1:
        raise ValueError("camera_internal_tf_receipt_schema_invalid")
    if receipt.get("kind") != EXPECTED_KIND:
        raise ValueError("camera_internal_tf_receipt_kind_invalid")
    if receipt.get("robot_scope") != "Robot A":
        raise ValueError("camera_internal_tf_robot_scope_invalid")
    if expected_serial != EXPECTED_ROBOT_SERIAL or receipt.get("robot_serial") != expected_serial:
        raise ValueError("camera_internal_tf_robot_serial_invalid")
    if receipt.get("allowed_usage") != EXPECTED_USAGE:
        raise ValueError("camera_internal_tf_usage_invalid")
    if receipt.get("tf_ownership") != EXPECTED_OWNERSHIP:
        raise ValueError("camera_internal_tf_ownership_invalid")
    assets = receipt.get("assets")
    if not isinstance(assets, dict) or set(assets) != EXPECTED_ASSETS:
        raise ValueError("camera_internal_tf_assets_invalid")
    params_asset = assets.get("camera_params")
    if (
        not isinstance(params_asset, dict)
        or params_asset.get("filename") != params_path.name
        or params_asset.get("sha256") != sha256(params_path)
    ):
        raise ValueError("camera_internal_tf_params_hash_mismatch")
    measurement_path = _asset_file(assets["measurement_record"], "measurement_record")
    try:
        measurement = json.loads(measurement_path.read_text(encoding="utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ValueError("camera_internal_tf_measurement_unreadable") from error
    if (
        not isinstance(measurement, dict)
        or measurement.get("schema_version") != 1
        or measurement.get("kind") != "robot_a_production_tf_measurements"
        or measurement.get("robot_scope") != "Robot A"
        or measurement.get("robot_serial") != EXPECTED_ROBOT_SERIAL
        or measurement.get("status") != "measured_and_reviewed"
        or measurement.get("motion_authorized") is not False
    ):
        raise ValueError("camera_internal_tf_measurement_invalid")
    identities = set()
    evidence_dates = []
    expected_from_evidence = {}
    measurement_sources = measurement.get("sources", {})
    measurement_camera = measurement.get("camera_internal", {})
    for role in ("chest", "belly"):
        evidence_path = _asset_file(
            assets[f"{role}_vendor_tf_static"], f"{role}_vendor_tf_static")
        source_record = measurement_sources.get(f"{role}_vendor_tf_static")
        if (
            not isinstance(source_record, dict)
            or source_record.get("path") != str(evidence_path)
            or source_record.get("sha256") != sha256(evidence_path)
        ):
            raise ValueError(f"camera_internal_tf_measurement_source_mismatch:{role}")
        transform, capture_asset, observed_by, observed_on = _camera_evidence_transform(
            evidence_path, role)
        if assets[f"{role}_source_capture"] != capture_asset:
            raise ValueError(f"camera_internal_tf_capture_asset_mismatch:{role}")
        expected_from_evidence[role] = transform
        identities.add(observed_by)
        evidence_dates.append(observed_on)
        camera_record = measurement_camera.get(role)
        if (
            not isinstance(camera_record, dict)
            or camera_record.get("source") != f"{role}_vendor_tf_static"
        ):
            raise ValueError(f"camera_internal_tf_measurement_review_missing:{role}")
        review_identities, review_dates = _review_identities(
            camera_record.get("physical_review"), role)
        identities.update(review_identities)
        evidence_dates.extend(review_dates)
    accepted_transforms = receipt.get("transforms")
    actual_transforms = _parameter_transforms(params_path.read_text(encoding="utf-8"))
    if (
        not isinstance(accepted_transforms, dict)
        or set(accepted_transforms) != set(actual_transforms)
    ):
        raise ValueError("camera_internal_tf_receipt_transform_mismatch")
    for role, actual in actual_transforms.items():
        accepted = accepted_transforms.get(role)
        if (
            not isinstance(accepted, dict)
            or accepted.get("parent") != actual["parent"]
            or accepted.get("child") != actual["child"]
            or not _same_vector(
                _vector(accepted.get("translation_xyz"), 3, f"receipt.{role}.translation"),
                actual["translation_xyz"],
            )
            or not _same_quaternion(
                _vector(accepted.get("rotation_xyzw"), 4, f"receipt.{role}.rotation"),
                actual["rotation_xyzw"],
            )
        ):
            raise ValueError("camera_internal_tf_receipt_transform_mismatch")
    for role in ("chest", "belly"):
        actual = actual_transforms[role]
        expected = expected_from_evidence[role]
        if not _same_vector(
            actual["translation_xyz"], expected["translation_xyz"]
        ) or not _same_quaternion(
            actual["rotation_xyzw"], expected["rotation_xyzw"]
        ):
            raise ValueError(f"camera_internal_tf_evidence_transform_mismatch:{role}")
    acceptance = receipt.get("acceptance")
    if (
        not isinstance(acceptance, dict)
        or acceptance.get("accepted") is not True
        or not _nonempty(acceptance.get("accepted_by"))
        or not _nonempty(acceptance.get("accepted_on"))
    ):
        raise ValueError("camera_internal_tf_review_missing")
    accepted_by = _identity(acceptance["accepted_by"], "final.accepted_by")
    if accepted_by in identities:
        raise ValueError("camera_internal_tf_final_review_independence_invalid")
    accepted_on = _accepted_date(acceptance["accepted_on"])
    if evidence_dates and accepted_on < max(evidence_dates):
        raise ValueError("camera_internal_tf_final_review_precedes_evidence")
    if "draft" in receipt:
        raise ValueError("camera_internal_tf_draft_receipt_rejected")
    return sha256(receipt_path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--params", type=Path, required=True)
    parser.add_argument("--expected-serial", default=EXPECTED_ROBOT_SERIAL)
    arguments = parser.parse_args()
    try:
        digest = validate(arguments.receipt, arguments.params, arguments.expected_serial)
    except (OSError, UnicodeError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 2
    print(digest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
