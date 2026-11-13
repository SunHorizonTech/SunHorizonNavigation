#!/usr/bin/env python3
"""Validate Robot A's evidence-bound production sensor-TF contract.

No Robot A transform numbers live here.  Motion use requires independent,
non-future reviews over the exact measurement, camera, Airy, URDF, and platform
assets consumed at runtime.
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
import xml.etree.ElementTree as ET


EXPECTED_ROBOT_SERIAL = "1421026005616"
EXPECTED_KIND = "robot_a_production_sensor_tf_acceptance"
EXPECTED_USAGE = ["motion_acceptance"]
EXPECTED_AIRY_KIND = "robot_a_airy_normalized_frame_acceptance"
EXPECTED_CAMERA_KIND = "robot_a_camera_internal_optical_tf_acceptance"
EXPECTED_CAMERA_OWNERSHIP = {
    "publisher_node": "/sunhorizon_camera_runtime",
    "vendor_orbbec_tf_publishers_allowed": False,
    "duplicate_child_frames_allowed": False,
}
EXPECTED_AIRY_SERIAL = "3009bee00063"
EXPECTED_OWNERSHIP = {
    "base_footprint_to_base_link_owner": "/sunhorizon_platform_adapter",
    "articulated_sensor_tree_owner": "/sunhorizon_robot_state_publisher",
    "camera_internal_optical_tf_owner": "/sunhorizon_camera_runtime",
    "robot_state_joint_states_topic": "/sunhorizon/platform/proof/live_joint_states",
    "duplicate_child_frames_allowed": False,
}
REQUIRED_LINKS = {
    "base_link",
    "waist_yaw_link",
    "camera_chest_link",
    "camera_belly_link",
    "lidar_mount",
    "rslidar",
    "airy_lidar",
}
REQUIRED_DIRECT_JOINTS = {
    ("base_link", "waist_yaw_link"): {"revolute", "continuous"},
    ("waist_yaw_link", "camera_chest_link"): {"fixed"},
    ("waist_yaw_link", "camera_belly_link"): {"fixed"},
    ("waist_yaw_link", "lidar_mount"): {"fixed"},
    ("lidar_mount", "rslidar"): {"fixed"},
    ("lidar_mount", "airy_lidar"): {"fixed"},
}
MEASUREMENT_SOURCE_KEYS = {
    "vendor_7dof_urdf",
    "airy_adapter_config",
    "airy_difop_pcap",
    "chest_vendor_tf_static",
    "belly_vendor_tf_static",
}
MOUNT_MEASUREMENT_EDGES = {
    "waist_yaw_to_camera_chest": ("waist_yaw_link", "camera_chest_link"),
    "waist_yaw_to_camera_belly": ("waist_yaw_link", "camera_belly_link"),
    "waist_yaw_to_lidar_mount": ("waist_yaw_link", "lidar_mount"),
    "lidar_mount_to_rslidar": ("lidar_mount", "rslidar"),
}
CAMERA_SERIALS = {"chest": "CPCAC53000NF", "belly": "CP9W463000ZW"}
CAMERA_FRAMES = {
    "chest": ("camera_chest_link", "camera_chest_color_optical_frame"),
    "belly": ("camera_belly_link", "camera_belly_depth_optical_frame"),
}
CAMERA_CAPTURE_BINDING_SCHEMA = "robot_a_camera_rosbag_source_binding_v1"
CAMERA_CAPTURE_SHA_SEMANTICS = (
    "canonical_json(robot_camera_and_rosbag_source_binding_v1)"
)
CAMERA_SEGMENT_SUFFIXES = (".db3", ".mcap", ".mcap.zstd")
TF_STATIC_TOPIC_CONTRACT = {
    "name": "/tf_static",
    "type": "tf2_msgs/msg/TFMessage",
    "serialization_format": "cdr",
}
FINAL_ASSET_KEYS = {
    "robot_urdf",
    "platform_base_tf_config",
    "measurement_record",
    "camera_params",
    "camera_internal_tf_receipt",
    "airy_adapter_config",
    "airy_frame_receipt",
}
SHA256_RE = re.compile(r"[0-9a-f]{64}")
IDENTITY_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._@:-]{2,127}")
TRANSFORM_TOLERANCE = 1e-8
FORBIDDEN_ROBOT_STATE_LINKS = {
    "base_footprint",
    "camera_chest_color_optical_frame",
    "camera_belly_depth_optical_frame",
}
PLATFORM_NUMERIC_KEYS = (
    "base_link_x_m",
    "base_link_y_m",
    "base_link_z_m",
    "base_link_roll_rad",
    "base_link_pitch_rad",
    "base_link_yaw_rad",
)
MAX_BASE_FOOTPRINT_Z_M = 0.01
MAX_BASE_FOOTPRINT_ROLL_PITCH_RAD = math.radians(1.0)
REPEATABILITY_EVIDENCE_KIND = "robot_a_tf_repeatability_evidence"
REVIEW_EDGE_BINDINGS = {
    "waist_yaw_to_camera_chest": {
        "parent": "waist_yaw_link", "child": "camera_chest_link",
        "sensor_role": "camera_chest", "sensor_serial": CAMERA_SERIALS["chest"],
    },
    "waist_yaw_to_camera_belly": {
        "parent": "waist_yaw_link", "child": "camera_belly_link",
        "sensor_role": "camera_belly", "sensor_serial": CAMERA_SERIALS["belly"],
    },
    "waist_yaw_to_lidar_mount": {
        "parent": "waist_yaw_link", "child": "lidar_mount",
        "sensor_role": "airy_mount", "sensor_serial": EXPECTED_AIRY_SERIAL,
    },
    "lidar_mount_to_rslidar": {
        "parent": "lidar_mount", "child": "rslidar",
        "sensor_role": "airy_lidar", "sensor_serial": EXPECTED_AIRY_SERIAL,
    },
    "camera_internal.chest": {
        "parent": CAMERA_FRAMES["chest"][0], "child": CAMERA_FRAMES["chest"][1],
        "sensor_role": "camera_chest", "sensor_serial": CAMERA_SERIALS["chest"],
    },
    "camera_internal.belly": {
        "parent": CAMERA_FRAMES["belly"][0], "child": CAMERA_FRAMES["belly"][1],
        "sensor_role": "camera_belly", "sensor_serial": CAMERA_SERIALS["belly"],
    },
    "platform_base_tf": {
        "parent": "base_footprint", "child": "base_link",
        "sensor_role": "robot_platform", "sensor_serial": EXPECTED_ROBOT_SERIAL,
    },
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _regular_absolute(path: Path, label: str) -> Path:
    if (
        not path.is_absolute()
        or path.is_symlink()
        or not path.is_file()
        or path.stat().st_size == 0
    ):
        raise ValueError(f"{label}_must_be_absolute_nonempty_regular_file")
    return path


def _nonempty(value: object) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _identity(value: object, label: str) -> str:
    """Return a canonical stable reviewer/operator ID, rejecting display names."""
    if (
        not isinstance(value, str)
        or value != value.strip()
        or IDENTITY_RE.fullmatch(value) is None
    ):
        raise ValueError(f"production_sensor_tf_identity_invalid:{label}")
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
        raise ValueError("acceptance_date_invalid")
    result = datetime.date.fromisoformat(str(value))
    if result > datetime.date.today():
        raise ValueError("acceptance_date_in_future")
    return result


def _valid_sha256(value: object) -> bool:
    return isinstance(value, str) and SHA256_RE.fullmatch(value) is not None


def _load_json(path: Path, label: str) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(f"{label}_unreadable") from error
    if not isinstance(value, dict):
        raise ValueError(f"{label}_schema_invalid")
    return value


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
        resolved = path.resolve(strict=True)
        resolved.relative_to(bag.resolve(strict=True))
    except (OSError, ValueError) as error:
        raise ValueError(f"{label}_outside_bag") from error
    path = _regular_absolute(path, label)
    if (
        not isinstance(asset.get("bytes"), int)
        or isinstance(asset.get("bytes"), bool)
        or asset["bytes"] != path.stat().st_size
        or asset.get("stable_regular_file_checked") is not True
        or not _valid_sha256(asset.get("sha256"))
        or asset.get("sha256") != sha256(path)
    ):
        raise ValueError(f"{label}_asset_hash_mismatch")
    return dict(asset), path


def _validate_camera_capture_binding(
    document: dict, role: str
) -> tuple[dict[str, object], set[tuple[int, int, str]]]:
    capture = document.get("source_capture")
    if (
        not isinstance(capture, dict)
        or capture.get("binding_complete") is not True
        or capture.get("binding_blockers") != []
        or capture.get("sha256_semantics") != CAMERA_CAPTURE_SHA_SEMANTICS
    ):
        raise ValueError(f"production_sensor_tf_camera_capture_unbound:{role}")
    bag = _absolute_directory(
        Path(str(capture.get("path", ""))),
        f"production_sensor_tf_camera_capture:{role}",
    )
    metadata_asset, _ = _bound_bag_file(
        bag,
        capture.get("metadata"),
        f"production_sensor_tf_camera_metadata:{role}",
        "metadata.yaml",
    )
    segments = capture.get("actual_read_segments")
    if not isinstance(segments, list) or not segments:
        raise ValueError(f"production_sensor_tf_camera_segments_missing:{role}")
    segment_assets = []
    segment_hashes = {}
    for index, asset in enumerate(segments):
        validated, segment_path = _bound_bag_file(
            bag, asset, f"production_sensor_tf_camera_segment:{role}:{index}"
        )
        relative = validated["relative_path"]
        if relative in segment_hashes or not any(
            segment_path.name.endswith(suffix) for suffix in CAMERA_SEGMENT_SUFFIXES
        ):
            raise ValueError(f"production_sensor_tf_camera_segment_invalid:{role}:{relative}")
        segment_hashes[relative] = validated["sha256"]
        segment_assets.append(validated)
    if segment_assets != sorted(segment_assets, key=lambda item: item["relative_path"]):
        raise ValueError(f"production_sensor_tf_camera_segments_not_sorted:{role}")

    read_records = capture.get("read_records")
    if not isinstance(read_records, list) or not read_records:
        raise ValueError(f"production_sensor_tf_camera_read_records_missing:{role}")
    record_references = set()
    for index, record in enumerate(read_records):
        if not isinstance(record, dict) or set(record) != {
            "filtered_record_index",
            "storage_timestamp_ns",
            "segment_relative_path",
            "segment_resolution_method",
            "segment_sha256",
        }:
            raise ValueError(f"production_sensor_tf_camera_read_record_invalid:{role}:{index}")
        filtered_index = record.get("filtered_record_index")
        timestamp = record.get("storage_timestamp_ns")
        relative = _safe_relative_path(
            record.get("segment_relative_path"),
            f"production_sensor_tf_camera_read_record:{role}:{index}",
        )
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
            raise ValueError(f"production_sensor_tf_camera_read_record_invalid:{role}:{index}")
        reference = (filtered_index, timestamp, relative)
        if reference in record_references:
            raise ValueError(f"production_sensor_tf_camera_read_record_duplicate:{role}")
        record_references.add(reference)
    if set(segment_hashes) != {reference[2] for reference in record_references}:
        raise ValueError(f"production_sensor_tf_camera_segment_reference_invalid:{role}")

    reader_metadata = capture.get("reader_metadata")
    if not isinstance(reader_metadata, dict) or reader_metadata.get("api_available") is not True:
        raise ValueError(f"production_sensor_tf_camera_reader_metadata_invalid:{role}")
    storage_identifier = reader_metadata.get("storage_identifier")
    relative_paths = reader_metadata.get("relative_file_paths")
    if (
        not _nonempty(storage_identifier)
        or not isinstance(relative_paths, list)
        or len(relative_paths) != len(set(relative_paths))
    ):
        raise ValueError(f"production_sensor_tf_camera_reader_metadata_invalid:{role}")
    normalized_relative_paths = {
        _safe_relative_path(
            value, f"production_sensor_tf_camera_reader_metadata:{role}"
        )
        for value in relative_paths
    }
    if not set(segment_hashes).issubset(normalized_relative_paths):
        raise ValueError(f"production_sensor_tf_camera_reader_metadata_invalid:{role}")

    observed_by = document.get("observed_by")
    binding = capture.get("binding")
    expected_binding = {
        "schema": CAMERA_CAPTURE_BINDING_SCHEMA,
        "robot_serial": EXPECTED_ROBOT_SERIAL,
        "camera_role": role,
        "camera_serial": CAMERA_SERIALS[role],
        "observed_by": observed_by,
        "chain_parent": CAMERA_FRAMES[role][0],
        "chain_child": CAMERA_FRAMES[role][1],
        "bag_path": str(bag),
        "metadata": metadata_asset,
        "storage_identifier": storage_identifier,
        "tf_static_topic": TF_STATIC_TOPIC_CONTRACT,
        "read_records": read_records,
        "segments": segment_assets,
    }
    if binding != expected_binding:
        raise ValueError(f"production_sensor_tf_camera_binding_document_invalid:{role}")
    binding_sha = capture.get("sha256")
    if not _valid_sha256(binding_sha) or binding_sha != _canonical_sha256(binding):
        raise ValueError(f"production_sensor_tf_camera_binding_hash_mismatch:{role}")
    return {
        "path": str(bag),
        "filename": bag.name,
        "sha256": binding_sha,
    }, record_references


def _finite_xml_vector(value: str | None, length: int) -> bool:
    if value is None:
        return False
    try:
        parts = [float(item) for item in value.split()]
    except ValueError:
        return False
    return len(parts) == length and all(math.isfinite(item) for item in parts)


def _validate_urdf(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    try:
        robot = ET.fromstring(text)
    except ET.ParseError as error:
        raise ValueError("production_sensor_tf_urdf_invalid") from error
    if robot.tag != "robot":
        raise ValueError("production_sensor_tf_urdf_invalid")
    links = [item.get("name", "") for item in robot.findall("link")]
    if len(links) != len(set(links)) or not REQUIRED_LINKS.issubset(links):
        raise ValueError("production_sensor_tf_urdf_links_invalid")
    if FORBIDDEN_ROBOT_STATE_LINKS.intersection(links):
        raise ValueError("production_sensor_tf_urdf_ownership_violation")

    child_owners: dict[str, tuple[str, str]] = {}
    joints: dict[tuple[str, str], str] = {}
    joint_elements: dict[tuple[str, str], ET.Element] = {}
    for joint in robot.findall("joint"):
        parent_element = joint.find("parent")
        child_element = joint.find("child")
        if parent_element is None or child_element is None:
            raise ValueError("production_sensor_tf_urdf_joint_invalid")
        parent = parent_element.get("link", "")
        child = child_element.get("link", "")
        joint_name = joint.get("name", "")
        joint_type = joint.get("type", "")
        if not parent or not child or not joint_name or not joint_type or parent == child:
            raise ValueError("production_sensor_tf_urdf_joint_invalid")
        if parent in FORBIDDEN_ROBOT_STATE_LINKS or child in FORBIDDEN_ROBOT_STATE_LINKS:
            raise ValueError("production_sensor_tf_urdf_ownership_violation")
        if child in child_owners:
            raise ValueError("production_sensor_tf_duplicate_child_frame")
        child_owners[child] = (parent, joint_name)
        joints[(parent, child)] = joint_type
        joint_elements[(parent, child)] = joint
    for edge, allowed_types in REQUIRED_DIRECT_JOINTS.items():
        if joints.get(edge) not in allowed_types:
            raise ValueError("production_sensor_tf_required_joint_invalid")
        origin = joint_elements[edge].find("origin")
        if (
            origin is None
            or not _finite_xml_vector(origin.get("xyz"), 3)
            or not _finite_xml_vector(origin.get("rpy"), 3)
        ):
            raise ValueError("production_sensor_tf_required_joint_origin_invalid")
    waist_axis = joint_elements[("base_link", "waist_yaw_link")].find("axis")
    if waist_axis is None or not _finite_xml_vector(waist_axis.get("xyz"), 3):
        raise ValueError("production_sensor_tf_waist_axis_invalid")
    waist_axis_values = [float(item) for item in waist_axis.get("xyz").split()]
    if any(abs(actual - expected) > 1e-9 for actual, expected in zip(
        waist_axis_values, (0.0, 0.0, 1.0)
    )):
        raise ValueError("production_sensor_tf_waist_axis_invalid")
    return text


def _unique_yaml_scalar(text: str, key: str) -> str:
    matches = re.findall(
        rf"^\s*{re.escape(key)}:\s*([^#\n]+?)\s*(?:#.*)?$", text, re.MULTILINE
    )
    if len(matches) != 1:
        raise ValueError(f"production_sensor_tf_platform_key_invalid:{key}")
    return matches[0].strip().strip("\"'")


def _validate_platform_config(path: Path) -> tuple[tuple[float, ...], tuple[float, ...]]:
    text = path.read_text(encoding="utf-8")
    if _unique_yaml_scalar(text, "publish_base_footprint_to_base_link").lower() != "true":
        raise ValueError("production_sensor_tf_platform_publish_not_enabled")
    if _unique_yaml_scalar(
        text, "base_footprint_to_base_link_calibration_accepted"
    ).lower() != "true":
        raise ValueError("production_sensor_tf_platform_calibration_not_accepted")
    if _unique_yaml_scalar(text, "base_footprint_frame") != "base_footprint":
        raise ValueError("production_sensor_tf_platform_parent_invalid")
    if _unique_yaml_scalar(text, "base_link_frame") != "base_link":
        raise ValueError("production_sensor_tf_platform_child_invalid")
    values = []
    for key in PLATFORM_NUMERIC_KEYS:
        try:
            value = float(_unique_yaml_scalar(text, key))
        except ValueError as error:
            raise ValueError(f"production_sensor_tf_platform_value_invalid:{key}") from error
        if not math.isfinite(value):
            raise ValueError(f"production_sensor_tf_platform_value_invalid:{key}")
        values.append(value)
    return tuple(values[:3]), tuple(values[3:])


def _vector(value: object, length: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"production_sensor_tf_vector_invalid:{label}")
    try:
        output = tuple(float(component) for component in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"production_sensor_tf_vector_invalid:{label}") from error
    if not all(math.isfinite(component) for component in output):
        raise ValueError(f"production_sensor_tf_vector_invalid:{label}")
    return output


def _same_vector(left: tuple[float, ...], right: tuple[float, ...]) -> bool:
    return len(left) == len(right) and all(
        abs(actual - expected) <= TRANSFORM_TOLERANCE
        for actual, expected in zip(left, right)
    )


def _unit_quaternion(value: object, label: str) -> tuple[float, ...]:
    quaternion = _vector(value, 4, label)
    norm = math.sqrt(sum(component * component for component in quaternion))
    if not math.isfinite(norm) or norm <= 0.0 or abs(norm - 1.0) > 1e-6:
        raise ValueError(f"production_sensor_tf_quaternion_invalid:{label}")
    return tuple(component / norm for component in quaternion)


def _same_quaternion(
    left: tuple[float, ...], right: tuple[float, ...], tolerance: float = TRANSFORM_TOLERANCE
) -> bool:
    try:
        left_q = _unit_quaternion(list(left), "comparison.left")
        right_q = _unit_quaternion(list(right), "comparison.right")
    except ValueError:
        return False
    return 1.0 - abs(sum(a * b for a, b in zip(left_q, right_q))) <= tolerance


def _angle_delta(left: float, right: float) -> float:
    return math.atan2(math.sin(left - right), math.cos(left - right))


def _same_rpy(
    left: tuple[float, ...], right: tuple[float, ...], tolerance: float = TRANSFORM_TOLERANCE
) -> bool:
    return len(left) == len(right) and all(
        abs(_angle_delta(actual, expected)) <= tolerance
        for actual, expected in zip(left, right)
    )


def _quaternion_geodesic_deg(
    left: tuple[float, ...], right: tuple[float, ...]
) -> float:
    left_q = _unit_quaternion(list(left), "repeatability.left")
    right_q = _unit_quaternion(list(right), "repeatability.right")
    dot = min(1.0, abs(sum(a * b for a, b in zip(left_q, right_q))))
    return math.degrees(2.0 * math.acos(dot))


def _measurement_timestamp(value: object, label: str) -> str:
    if not _nonempty(value) or not isinstance(value, str) or value != value.strip():
        raise ValueError(f"production_sensor_tf_repeatability_timestamp_invalid:{label}")
    try:
        parsed = datetime.datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as error:
        raise ValueError(
            f"production_sensor_tf_repeatability_timestamp_invalid:{label}"
        ) from error
    if parsed.tzinfo is None:
        raise ValueError(f"production_sensor_tf_repeatability_timestamp_invalid:{label}")
    now = datetime.datetime.now(parsed.tzinfo)
    if parsed > now:
        raise ValueError(f"production_sensor_tf_repeatability_timestamp_in_future:{label}")
    return value


def _repeatability_from_evidence(
    evidence_path: Path, label: str
) -> tuple[float, float]:
    document = _load_json(evidence_path, f"production_sensor_tf_review_evidence:{label}")
    if (
        document.get("schema_version") != 1
        or document.get("kind") != REPEATABILITY_EVIDENCE_KIND
        or document.get("robot_scope") != "Robot A"
        or document.get("robot_serial") != EXPECTED_ROBOT_SERIAL
    ):
        raise ValueError(f"production_sensor_tf_repeatability_evidence_invalid:{label}")
    edges = document.get("edges")
    record = edges.get(label) if isinstance(edges, dict) else None
    expected = REVIEW_EDGE_BINDINGS.get(label)
    if not isinstance(record, dict) or expected is None or any(
        record.get(key) != value for key, value in expected.items()
    ):
        raise ValueError(f"production_sensor_tf_repeatability_edge_binding_invalid:{label}")
    samples = record.get("samples")
    if not isinstance(samples, list) or len(samples) < 3:
        raise ValueError(f"production_sensor_tf_repeatability_samples_insufficient:{label}")
    translations = []
    rotations = []
    timestamps = set()
    capture_hashes = set()
    for index, sample in enumerate(samples):
        sample_label = f"{label}:{index}"
        if not isinstance(sample, dict) or set(sample) != {
            "captured_at", "translation_xyz", "rotation_xyzw", "capture"
        }:
            raise ValueError(f"production_sensor_tf_repeatability_sample_invalid:{sample_label}")
        timestamp = _measurement_timestamp(sample.get("captured_at"), sample_label)
        if timestamp in timestamps:
            raise ValueError(f"production_sensor_tf_repeatability_timestamp_duplicate:{label}")
        timestamps.add(timestamp)
        translations.append(_vector(
            sample.get("translation_xyz"), 3, f"{sample_label}.translation"
        ))
        rotations.append(_unit_quaternion(
            sample.get("rotation_xyzw"), f"{sample_label}.rotation"
        ))
        capture = sample.get("capture")
        capture_path = _source_file(capture, f"repeatability:{sample_label}")
        if (
            not isinstance(capture, dict)
            or capture.get("filename") != capture_path.name
            or capture.get("sha256") in capture_hashes
        ):
            raise ValueError(f"production_sensor_tf_repeatability_capture_invalid:{sample_label}")
        capture_hashes.add(capture["sha256"])
    translation_spread = max(
        math.dist(translations[left], translations[right])
        for left in range(len(translations))
        for right in range(left + 1, len(translations))
    )
    rotation_spread = max(
        _quaternion_geodesic_deg(rotations[left], rotations[right])
        for left in range(len(rotations))
        for right in range(left + 1, len(rotations))
    )
    return translation_spread, rotation_spread


def _review(review: object, label: str) -> tuple[set[str], list[datetime.date]]:
    if not isinstance(review, dict):
        raise ValueError(f"production_sensor_tf_physical_review_missing:{label}")
    measured_by = _identity(review.get("measured_by"), f"{label}.measured_by")
    reviewed_by = _identity(review.get("reviewed_by"), f"{label}.reviewed_by")
    if measured_by == reviewed_by:
        raise ValueError(f"production_sensor_tf_review_independence_invalid:{label}")
    try:
        measured_on = _accepted_date(review.get("measured_on"))
        reviewed_on = _accepted_date(review.get("reviewed_on"))
    except ValueError as error:
        raise ValueError(f"production_sensor_tf_review_date_invalid:{label}:{error}") from error
    if reviewed_on < measured_on:
        raise ValueError(f"production_sensor_tf_review_precedes_measurement:{label}")
    try:
        translation_repeatability = float(review.get("translation_repeatability_m"))
        rotation_repeatability = float(review.get("rotation_repeatability_deg"))
    except (TypeError, ValueError) as error:
        raise ValueError(f"production_sensor_tf_repeatability_invalid:{label}") from error
    if (
        not math.isfinite(translation_repeatability)
        or not math.isfinite(rotation_repeatability)
        or translation_repeatability < 0.0
        or rotation_repeatability < 0.0
        or translation_repeatability > 0.01
        or rotation_repeatability > 0.5
    ):
        raise ValueError(f"production_sensor_tf_repeatability_invalid:{label}")
    evidence_path = _regular_absolute(
        Path(str(review.get("evidence_path", ""))),
        f"production_sensor_tf_review_evidence:{label}",
    )
    if not _valid_sha256(review.get("evidence_sha256")) or sha256(evidence_path) != review.get(
        "evidence_sha256"
    ):
        raise ValueError(f"production_sensor_tf_review_evidence_hash_mismatch:{label}")
    measured_translation, measured_rotation = _repeatability_from_evidence(
        evidence_path, label
    )
    if (
        not math.isclose(
            translation_repeatability, measured_translation, rel_tol=0.0, abs_tol=1e-9
        )
        or not math.isclose(
            rotation_repeatability, measured_rotation, rel_tol=0.0, abs_tol=1e-9
        )
    ):
        raise ValueError(f"production_sensor_tf_repeatability_claim_mismatch:{label}")
    return {measured_by, reviewed_by}, [measured_on, reviewed_on]


def _source_file(record: object, label: str) -> Path:
    if not isinstance(record, dict):
        raise ValueError(f"production_sensor_tf_source_invalid:{label}")
    path = _regular_absolute(
        Path(str(record.get("path", ""))), f"production_sensor_tf_source:{label}"
    )
    if not _valid_sha256(record.get("sha256")) or sha256(path) != record.get("sha256"):
        raise ValueError(f"production_sensor_tf_source_hash_mismatch:{label}")
    return path


def _quaternion_multiply(
    left: tuple[float, ...], right: tuple[float, ...]
) -> tuple[float, ...]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _quaternion_matrix(q: tuple[float, ...]) -> tuple[tuple[float, ...], ...]:
    x, y, z, w = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def _rotate(q: tuple[float, ...], vector: tuple[float, ...]) -> tuple[float, ...]:
    matrix = _quaternion_matrix(q)
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def _compose(
    left_translation: tuple[float, ...],
    left_rotation: tuple[float, ...],
    right_translation: tuple[float, ...],
    right_rotation: tuple[float, ...],
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    rotated = _rotate(left_rotation, right_translation)
    translation = tuple(left_translation[index] + rotated[index] for index in range(3))
    rotation = _quaternion_multiply(left_rotation, right_rotation)
    norm = math.sqrt(sum(component * component for component in rotation))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("production_sensor_tf_quaternion_invalid")
    return translation, tuple(component / norm for component in rotation)


def _inverse(
    translation: tuple[float, ...], rotation: tuple[float, ...]
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    inverse_rotation = (-rotation[0], -rotation[1], -rotation[2], rotation[3])
    inverse_translation = tuple(-value for value in _rotate(inverse_rotation, translation))
    return inverse_translation, inverse_rotation


def _rpy_quaternion(rpy: tuple[float, ...]) -> tuple[float, ...]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quaternion_rpy(q: tuple[float, ...]) -> tuple[float, ...]:
    x, y, z, w = q
    return (
        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    )


def _yaml_vector(text: str, key: str, length: int) -> tuple[float, ...]:
    scalar = _unique_yaml_scalar(text, key)
    if not scalar.startswith("[") or not scalar.endswith("]"):
        raise ValueError(f"production_sensor_tf_yaml_vector_invalid:{key}")
    return _vector([part.strip() for part in scalar[1:-1].split(",")], length, key)


def _urdf_edges(path: Path) -> dict[tuple[str, str], tuple[str, tuple[float, ...], tuple[float, ...]]]:
    robot = ET.fromstring(path.read_text(encoding="utf-8"))
    output = {}
    for joint in robot.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        origin = joint.find("origin")
        if parent is None or child is None or origin is None:
            continue
        output[(parent.get("link", ""), child.get("link", ""))] = (
            joint.get("type", ""),
            tuple(float(value) for value in origin.get("xyz", "").split()),
            tuple(float(value) for value in origin.get("rpy", "").split()),
        )
    return output


def _camera_evidence(
    path: Path, role: str
) -> tuple[dict[str, object], dict[str, object], str, datetime.date]:
    document = _load_json(path, f"production_sensor_tf_{role}_camera_evidence")
    if (
        document.get("schema_version") != 1
        or document.get("kind") != "robot_a_camera_vendor_tf_static_evidence"
        or document.get("robot_scope") != "Robot A"
        or document.get("robot_serial") != EXPECTED_ROBOT_SERIAL
        or document.get("role") != role
        or document.get("camera_serial") != CAMERA_SERIALS[role]
    ):
        raise ValueError(f"production_sensor_tf_camera_evidence_identity_invalid:{role}")
    try:
        observed_on = _accepted_date(document.get("observed_on"))
    except ValueError as error:
        raise ValueError(f"production_sensor_tf_camera_evidence_date_invalid:{role}:{error}") from error
    observed_by = _identity(document.get("observed_by"), f"camera.{role}.observed_by")
    capture_asset, read_references = _validate_camera_capture_binding(document, role)
    transforms = document.get("transforms")
    if not isinstance(transforms, list) or not transforms:
        raise ValueError(f"production_sensor_tf_camera_chain_missing:{role}")
    edges = {}
    children = set()
    for index, transform in enumerate(transforms):
        if not isinstance(transform, dict):
            raise ValueError(f"production_sensor_tf_camera_transform_invalid:{role}:{index}")
        parent, child = transform.get("parent"), transform.get("child")
        translation = _vector(transform.get("translation_xyz"), 3, f"{role}:{index}:translation")
        rotation = _vector(transform.get("rotation_xyzw"), 4, f"{role}:{index}:rotation")
        header_stamp_ns = transform.get("header_stamp_ns")
        source = transform.get("source_record")
        if not isinstance(source, dict) or set(source) != {
            "filtered_record_index", "storage_timestamp_ns", "segment_relative_path"
        }:
            raise ValueError(f"production_sensor_tf_camera_transform_source_invalid:{role}:{index}")
        reference = (
            source.get("filtered_record_index"),
            source.get("storage_timestamp_ns"),
            source.get("segment_relative_path"),
        )
        if (
            not isinstance(reference[0], int)
            or isinstance(reference[0], bool)
            or not isinstance(reference[1], int)
            or isinstance(reference[1], bool)
            or not _nonempty(reference[2])
            or not isinstance(header_stamp_ns, int)
            or isinstance(header_stamp_ns, bool)
            or header_stamp_ns < 0
            or reference not in read_references
        ):
            raise ValueError(f"production_sensor_tf_camera_transform_source_invalid:{role}:{index}")
        if (
            not _nonempty(parent) or not _nonempty(child) or parent == child
            or parent in edges or child in children
        ):
            raise ValueError(f"production_sensor_tf_camera_transform_invalid:{role}:{index}")
        if abs(math.sqrt(sum(value * value for value in rotation)) - 1.0) > 1e-6:
            raise ValueError(f"production_sensor_tf_camera_quaternion_invalid:{role}:{index}")
        edges[parent] = (child, translation, rotation)
        children.add(child)
    current, target = CAMERA_FRAMES[role]
    translation = (0.0, 0.0, 0.0)
    rotation = (0.0, 0.0, 0.0, 1.0)
    visited = set()
    while current != target:
        if current in visited or current not in edges:
            raise ValueError(f"production_sensor_tf_camera_chain_incomplete:{role}")
        visited.add(current)
        child, edge_translation, edge_rotation = edges[current]
        translation, rotation = _compose(
            translation, rotation, edge_translation, edge_rotation
        )
        current = child
    if len(visited) != len(edges):
        raise ValueError(f"production_sensor_tf_camera_chain_ambiguous:{role}")
    return {
        "parent": CAMERA_FRAMES[role][0],
        "child": target,
        "translation_xyz": list(translation),
        "rotation_xyzw": list(rotation),
    }, capture_asset, observed_by, observed_on


def _validate_measurements(
    path: Path,
) -> tuple[
    dict,
    dict[str, Path],
    set[str],
    list[datetime.date],
    dict[str, dict],
    dict[str, dict[str, object]],
    list[datetime.date],
]:
    document = _load_json(path, "production_sensor_tf_measurement_record")
    if (
        document.get("schema_version") != 1
        or document.get("kind") != "robot_a_production_tf_measurements"
        or document.get("robot_scope") != "Robot A"
        or document.get("robot_serial") != EXPECTED_ROBOT_SERIAL
        or document.get("status") != "measured_and_reviewed"
        or document.get("motion_authorized") is not False
    ):
        raise ValueError("production_sensor_tf_measurement_identity_invalid")
    sources = document.get("sources")
    if not isinstance(sources, dict) or set(sources) != MEASUREMENT_SOURCE_KEYS:
        raise ValueError("production_sensor_tf_measurement_sources_invalid")
    source_paths = {
        name: _source_file(sources[name], name) for name in sorted(MEASUREMENT_SOURCE_KEYS)
    }
    difop = sources["airy_difop_pcap"]
    if (
        difop.get("lidar_serial") != EXPECTED_AIRY_SERIAL
        or not isinstance(difop.get("identical_samples"), int)
        or difop.get("identical_samples") < 5
        or not _nonempty(difop.get("observed_by"))
    ):
        raise ValueError("production_sensor_tf_difop_identity_invalid")
    try:
        difop_date = _accepted_date(difop.get("observed_on"))
    except ValueError as error:
        raise ValueError(f"production_sensor_tf_difop_date_invalid:{error}") from error

    identities = {_identity(difop["observed_by"], "airy_difop.observed_by")}
    dates = [difop_date]
    mount_transforms = document.get("mount_transforms")
    if not isinstance(mount_transforms, dict) or set(mount_transforms) != set(
        MOUNT_MEASUREMENT_EDGES
    ):
        raise ValueError("production_sensor_tf_mount_measurements_invalid")
    for name in sorted(MOUNT_MEASUREMENT_EDGES):
        record = mount_transforms[name]
        if not isinstance(record, dict):
            raise ValueError(f"production_sensor_tf_mount_measurement_invalid:{name}")
        _vector(record.get("translation_xyz"), 3, f"{name}:translation")
        _vector(record.get("rotation_rpy"), 3, f"{name}:rotation")
        reviewers, review_dates = _review(record.get("physical_review"), name)
        identities.update(reviewers)
        dates.extend(review_dates)

    camera_internal = document.get("camera_internal")
    if not isinstance(camera_internal, dict) or set(camera_internal) != {"chest", "belly"}:
        raise ValueError("production_sensor_tf_camera_measurements_invalid")
    camera_transforms = {}
    camera_capture_assets = {}
    camera_evidence_dates = []
    for role in ("chest", "belly"):
        record = camera_internal[role]
        if not isinstance(record, dict):
            raise ValueError(f"production_sensor_tf_camera_measurement_invalid:{role}")
        if record.get("source") != f"{role}_vendor_tf_static":
            raise ValueError(f"production_sensor_tf_camera_measurement_source_invalid:{role}")
        reviewers, review_dates = _review(
            record.get("physical_review"), f"camera_internal.{role}"
        )
        identities.update(reviewers)
        dates.extend(review_dates)
        camera_evidence_dates.extend(review_dates)
        transform, capture_asset, observed_by, observed_on = _camera_evidence(
            source_paths[f"{role}_vendor_tf_static"], role
        )
        camera_transforms[role] = transform
        camera_capture_assets[role] = capture_asset
        identities.add(observed_by)
        dates.append(observed_on)
        camera_evidence_dates.append(observed_on)

    platform = document.get("platform_base_tf")
    if not isinstance(platform, dict):
        raise ValueError("production_sensor_tf_platform_measurement_missing")
    _vector(platform.get("translation_xyz"), 3, "platform_base_tf.translation")
    _vector(platform.get("rotation_rpy"), 3, "platform_base_tf.rotation")
    reviewers, review_dates = _review(
        platform.get("physical_review"), "platform_base_tf"
    )
    identities.update(reviewers)
    dates.extend(review_dates)
    return (
        document,
        source_paths,
        identities,
        dates,
        camera_transforms,
        camera_capture_assets,
        camera_evidence_dates,
    )


def _validate_airy_receipt(
    receipt_path: Path,
    adapter_path: Path,
    difop_path: Path,
    difop_record: dict,
    forbidden_identities: set[str],
) -> tuple[set[str], list[datetime.date], tuple[float, ...], tuple[float, ...]]:
    receipt = _load_json(receipt_path, "production_sensor_tf_airy_receipt")
    if (
        receipt.get("schema_version") != 1
        or receipt.get("kind") != EXPECTED_AIRY_KIND
        or receipt.get("robot_scope") != "Robot A"
        or receipt.get("robot_serial") != EXPECTED_ROBOT_SERIAL
        or receipt.get("allowed_usage") != EXPECTED_USAGE
        or receipt.get("frames") != {"raw_points": "rslidar", "normalized_points": "airy_lidar"}
    ):
        raise ValueError("production_sensor_tf_airy_receipt_invalid")
    assets = receipt.get("assets")
    if not isinstance(assets, dict) or set(assets) != {
        "airy_adapter_config", "airy_difop_pcap"
    }:
        raise ValueError("production_sensor_tf_airy_assets_invalid")
    for key, path in (
        ("airy_adapter_config", adapter_path),
        ("airy_difop_pcap", difop_path),
    ):
        asset = assets[key]
        if (
            not isinstance(asset, dict)
            or asset.get("filename") != path.name
            or asset.get("sha256") != sha256(path)
        ):
            raise ValueError(f"production_sensor_tf_airy_asset_hash_mismatch:{key}")
    adapter_text = adapter_path.read_text(encoding="utf-8")
    if (
        _unique_yaml_scalar(adapter_text, "expected_input_points_frame") != "rslidar"
        or _unique_yaml_scalar(adapter_text, "output_points_frame") != "airy_lidar"
        or _unique_yaml_scalar(adapter_text, "point_calibration_valid").lower() != "true"
    ):
        raise ValueError("production_sensor_tf_airy_adapter_invalid")
    translation = _yaml_vector(adapter_text, "point_translation_xyz", 3)
    rotation = _yaml_vector(adapter_text, "point_rotation_xyzw", 4)
    if abs(math.sqrt(sum(value * value for value in rotation)) - 1.0) > 1e-6:
        raise ValueError("production_sensor_tf_airy_adapter_quaternion_invalid")
    normalization = receipt.get("point_normalization")
    if not isinstance(normalization, dict) or (
        not _same_vector(
            _vector(normalization.get("translation_xyz"), 3, "airy_receipt.translation"),
            translation,
        )
        or not _same_quaternion(
            _vector(normalization.get("rotation_xyzw"), 4, "airy_receipt.rotation"),
            rotation,
        )
    ):
        raise ValueError("production_sensor_tf_airy_normalization_mismatch")
    difop = receipt.get("factory_difop_extrinsic")
    if (
        not isinstance(difop, dict)
        or difop.get("source") != "per_unit_difop"
        or difop.get("accepted") is not True
        or difop.get("lidar_serial") != EXPECTED_AIRY_SERIAL
        or difop.get("evidence_sha256") != sha256(difop_path)
        or difop.get("observed_on") != difop_record.get("observed_on")
        or difop.get("observed_by") != difop_record.get("observed_by")
    ):
        raise ValueError("production_sensor_tf_airy_difop_binding_invalid")
    acceptance = receipt.get("acceptance")
    if not isinstance(acceptance, dict) or acceptance.get("accepted") is not True:
        raise ValueError("production_sensor_tf_airy_review_missing")
    observed_by = _identity(difop.get("observed_by"), "airy_receipt.observed_by")
    accepted_by = _identity(acceptance.get("accepted_by"), "airy_receipt.accepted_by")
    if (
        accepted_by == observed_by
        or accepted_by in forbidden_identities
    ):
        raise ValueError("production_sensor_tf_airy_review_independence_invalid")
    try:
        accepted_on = _accepted_date(acceptance.get("accepted_on"))
    except ValueError as error:
        raise ValueError(f"production_sensor_tf_airy_review_date_invalid:{error}") from error
    if accepted_on < _accepted_date(difop.get("observed_on")):
        raise ValueError("production_sensor_tf_airy_review_precedes_observation")
    if "draft" in receipt:
        raise ValueError("production_sensor_tf_airy_draft_receipt_rejected")
    return {observed_by, accepted_by}, [accepted_on], translation, rotation


def _validate_camera_receipt(
    receipt_path: Path,
    params_path: Path,
    measurement_path: Path,
    source_paths: dict[str, Path],
    capture_assets: dict[str, dict[str, object]],
    camera_transforms: dict[str, dict],
    forbidden_identities: set[str],
    evidence_dates: list[datetime.date],
) -> tuple[set[str], list[datetime.date]]:
    receipt = _load_json(receipt_path, "production_sensor_tf_camera_receipt")
    if (
        receipt.get("schema_version") != 1
        or receipt.get("kind") != EXPECTED_CAMERA_KIND
        or receipt.get("robot_scope") != "Robot A"
        or receipt.get("robot_serial") != EXPECTED_ROBOT_SERIAL
        or receipt.get("allowed_usage") != EXPECTED_USAGE
        or receipt.get("tf_ownership") != EXPECTED_CAMERA_OWNERSHIP
    ):
        raise ValueError("production_sensor_tf_camera_receipt_invalid")
    assets = receipt.get("assets")
    expected_assets = {
        "camera_params",
        "measurement_record",
        "chest_vendor_tf_static",
        "chest_source_capture",
        "belly_vendor_tf_static",
        "belly_source_capture",
    }
    if not isinstance(assets, dict) or set(assets) != expected_assets:
        raise ValueError("production_sensor_tf_camera_assets_invalid")
    for key, path in (("camera_params", params_path), ("measurement_record", measurement_path)):
        asset = assets[key]
        if (
            not isinstance(asset, dict)
            or asset.get("filename") != path.name
            or asset.get("sha256") != sha256(path)
            or (key == "measurement_record" and asset.get("path") != str(path))
        ):
            raise ValueError(f"production_sensor_tf_camera_asset_hash_mismatch:{key}")
    for role in ("chest", "belly"):
        for suffix, path in (
            ("vendor_tf_static", source_paths[f"{role}_vendor_tf_static"]),
            ("source_capture", capture_assets[role]),
        ):
            key = f"{role}_{suffix}"
            asset = assets[key]
            if suffix == "source_capture":
                matches = asset == path
            else:
                matches = (
                    isinstance(asset, dict)
                    and asset.get("path") == str(path)
                    and asset.get("filename") == path.name
                    and asset.get("sha256") == sha256(path)
                )
            if not matches:
                raise ValueError(f"production_sensor_tf_camera_asset_hash_mismatch:{key}")
    text = params_path.read_text(encoding="utf-8")
    if _unique_yaml_scalar(text, "publish_internal_optical_tf").lower() != "true":
        raise ValueError("production_sensor_tf_camera_publication_disabled")
    actual = {}
    for role, (parent, child) in CAMERA_FRAMES.items():
        if (
            _unique_yaml_scalar(text, f"{role}_internal_tf_parent_frame") != parent
            or _unique_yaml_scalar(text, f"{role}_internal_tf_child_frame") != child
        ):
            raise ValueError(f"production_sensor_tf_camera_frame_invalid:{role}")
        actual[role] = {
            "parent": parent,
            "child": child,
            "translation_xyz": list(_yaml_vector(text, f"{role}_internal_tf_translation_xyz", 3)),
            "rotation_xyzw": list(_yaml_vector(text, f"{role}_internal_tf_rotation_xyzw", 4)),
        }
        if not _same_vector(
            tuple(actual[role]["translation_xyz"]),
            tuple(camera_transforms[role]["translation_xyz"]),
        ) or not _same_quaternion(
            tuple(actual[role]["rotation_xyzw"]),
            tuple(camera_transforms[role]["rotation_xyzw"]),
        ):
            raise ValueError(f"production_sensor_tf_camera_measurement_mismatch:{role}")
    accepted_transforms = receipt.get("transforms")
    if not isinstance(accepted_transforms, dict) or set(accepted_transforms) != set(actual):
        raise ValueError("production_sensor_tf_camera_receipt_transform_mismatch")
    for role in actual:
        accepted = accepted_transforms.get(role)
        if (
            not isinstance(accepted, dict)
            or accepted.get("parent") != actual[role]["parent"]
            or accepted.get("child") != actual[role]["child"]
            or not _same_vector(
                _vector(accepted.get("translation_xyz"), 3, f"receipt.{role}.translation"),
                tuple(actual[role]["translation_xyz"]),
            )
            or not _same_quaternion(
                _vector(accepted.get("rotation_xyzw"), 4, f"receipt.{role}.rotation"),
                tuple(actual[role]["rotation_xyzw"]),
            )
        ):
            raise ValueError("production_sensor_tf_camera_receipt_transform_mismatch")
    acceptance = receipt.get("acceptance")
    if not isinstance(acceptance, dict) or acceptance.get("accepted") is not True:
        raise ValueError("production_sensor_tf_camera_review_missing")
    accepted_by = _identity(acceptance.get("accepted_by"), "camera_receipt.accepted_by")
    if accepted_by in forbidden_identities:
        raise ValueError("production_sensor_tf_camera_review_independence_invalid")
    try:
        accepted_on = _accepted_date(acceptance.get("accepted_on"))
    except ValueError as error:
        raise ValueError(f"production_sensor_tf_camera_review_date_invalid:{error}") from error
    if evidence_dates and accepted_on < max(evidence_dates):
        raise ValueError("production_sensor_tf_camera_review_precedes_evidence")
    if "draft" in receipt:
        raise ValueError("production_sensor_tf_camera_draft_receipt_rejected")
    return {accepted_by}, [accepted_on]


def _asset_matches(receipt: dict, key: str, path: Path) -> None:
    item = receipt["assets"].get(key)
    if (
        not isinstance(item, dict)
        or item.get("filename") != path.name
        or item.get("sha256") != sha256(path)
    ):
        raise ValueError(f"production_sensor_tf_{key}_hash_mismatch")


def validate(
    receipt_path: Path,
    robot_urdf_path: Path,
    platform_base_tf_config_path: Path,
    measurement_record_path: Path | None = None,
    camera_params_path: Path | None = None,
    camera_receipt_path: Path | None = None,
    airy_adapter_config_path: Path | None = None,
    airy_frame_receipt_path: Path | None = None,
    expected_serial: str = EXPECTED_ROBOT_SERIAL,
) -> str:
    receipt_path = _regular_absolute(receipt_path, "production_sensor_tf_receipt")
    robot_urdf_path = _regular_absolute(robot_urdf_path, "production_sensor_tf_robot_urdf")
    platform_base_tf_config_path = _regular_absolute(
        platform_base_tf_config_path, "production_sensor_tf_platform_base_tf_config"
    )
    if any(path is None for path in (
        measurement_record_path,
        camera_params_path,
        camera_receipt_path,
        airy_adapter_config_path,
        airy_frame_receipt_path,
    )):
        raise ValueError("production_sensor_tf_evidence_assets_required")
    measurement_record_path = _regular_absolute(
        measurement_record_path, "production_sensor_tf_measurement_record")
    camera_params_path = _regular_absolute(camera_params_path, "production_sensor_tf_camera_params")
    camera_receipt_path = _regular_absolute(camera_receipt_path, "production_sensor_tf_camera_receipt")
    airy_adapter_config_path = _regular_absolute(
        airy_adapter_config_path, "production_sensor_tf_airy_adapter_config")
    airy_frame_receipt_path = _regular_absolute(
        airy_frame_receipt_path, "production_sensor_tf_airy_frame_receipt")
    receipt = _load_json(receipt_path, "production_sensor_tf_receipt")
    if not isinstance(receipt, dict) or receipt.get("schema_version") != 1:
        raise ValueError("production_sensor_tf_receipt_schema_invalid")
    if receipt.get("kind") != EXPECTED_KIND:
        raise ValueError("production_sensor_tf_receipt_kind_invalid")
    if receipt.get("robot_scope") != "Robot A":
        raise ValueError("production_sensor_tf_robot_scope_invalid")
    if expected_serial != EXPECTED_ROBOT_SERIAL or receipt.get("robot_serial") != expected_serial:
        raise ValueError("production_sensor_tf_robot_serial_invalid")
    if receipt.get("allowed_usage") != EXPECTED_USAGE:
        raise ValueError("production_sensor_tf_usage_invalid")
    if receipt.get("tf_ownership") != EXPECTED_OWNERSHIP:
        raise ValueError("production_sensor_tf_ownership_invalid")
    if not isinstance(receipt.get("assets"), dict) or set(receipt["assets"]) != FINAL_ASSET_KEYS:
        raise ValueError("production_sensor_tf_assets_invalid")
    asset_paths = {
        "robot_urdf": robot_urdf_path,
        "platform_base_tf_config": platform_base_tf_config_path,
        "measurement_record": measurement_record_path,
        "camera_params": camera_params_path,
        "camera_internal_tf_receipt": camera_receipt_path,
        "airy_adapter_config": airy_adapter_config_path,
        "airy_frame_receipt": airy_frame_receipt_path,
    }
    for key, path in asset_paths.items():
        _asset_matches(receipt, key, path)

    (
        measurement,
        source_paths,
        identities,
        evidence_dates,
        camera_transforms,
        camera_capture_assets,
        camera_evidence_dates,
    ) = _validate_measurements(measurement_record_path)
    if source_paths["airy_adapter_config"] != airy_adapter_config_path:
        raise ValueError("production_sensor_tf_airy_adapter_path_mismatch")
    airy_source = measurement["sources"]["airy_adapter_config"]
    if (
        airy_source.get("motion_acceptance_receipt_path")
        != str(airy_frame_receipt_path)
        or airy_source.get("motion_acceptance_receipt_sha256")
        != sha256(airy_frame_receipt_path)
    ):
        raise ValueError("production_sensor_tf_airy_receipt_binding_invalid")
    airy_identities, airy_dates, normalization_translation, normalization_rotation = (
        _validate_airy_receipt(
            airy_frame_receipt_path,
            airy_adapter_config_path,
            source_paths["airy_difop_pcap"],
            measurement["sources"]["airy_difop_pcap"],
            identities,
        )
    )
    identities.update(airy_identities)
    evidence_dates.extend(airy_dates)
    camera_identities, camera_dates = _validate_camera_receipt(
        camera_receipt_path,
        camera_params_path,
        measurement_record_path,
        source_paths,
        camera_capture_assets,
        camera_transforms,
        identities,
        camera_evidence_dates,
    )
    identities.update(camera_identities)
    evidence_dates.extend(camera_dates)

    acceptance = receipt.get("acceptance")
    if (
        not isinstance(acceptance, dict)
        or acceptance.get("accepted") is not True
        or not _nonempty(acceptance.get("accepted_by"))
        or not _nonempty(acceptance.get("accepted_on"))
    ):
        raise ValueError("production_sensor_tf_review_missing")
    accepted_by = _identity(acceptance["accepted_by"], "final.accepted_by")
    if accepted_by in identities:
        raise ValueError("production_sensor_tf_final_review_independence_invalid")
    try:
        accepted_on = _accepted_date(acceptance["accepted_on"])
    except ValueError as error:
        raise ValueError(f"production_sensor_tf_final_review_date_invalid:{error}") from error
    if evidence_dates and accepted_on < max(evidence_dates):
        raise ValueError("production_sensor_tf_final_review_precedes_evidence")
    if "draft" in receipt:
        raise ValueError("production_sensor_tf_draft_receipt_rejected")

    robot_description = _validate_urdf(robot_urdf_path)
    urdf_edges = _urdf_edges(robot_urdf_path)
    vendor_edges = _urdf_edges(source_paths["vendor_7dof_urdf"])
    vendor_waist = vendor_edges.get(("base_link", "waist_yaw_link"))
    if vendor_waist is None or urdf_edges.get(("base_link", "waist_yaw_link")) != vendor_waist:
        raise ValueError("production_sensor_tf_vendor_waist_mismatch")
    for name, edge in MOUNT_MEASUREMENT_EDGES.items():
        record = measurement["mount_transforms"][name]
        actual = urdf_edges.get(edge)
        expected_translation = _vector(record["translation_xyz"], 3, f"{name}:translation")
        expected_rpy = _vector(record["rotation_rpy"], 3, f"{name}:rotation")
        if actual is None or actual[0] != "fixed" or not _same_vector(
            actual[1], expected_translation
        ) or not _same_rpy(actual[2], expected_rpy):
            raise ValueError(f"production_sensor_tf_mount_urdf_mismatch:{name}")
    raw_record = measurement["mount_transforms"]["lidar_mount_to_rslidar"]
    raw_translation = _vector(raw_record["translation_xyz"], 3, "raw_lidar.translation")
    raw_rotation = _rpy_quaternion(
        _vector(raw_record["rotation_rpy"], 3, "raw_lidar.rotation")
    )
    raw_from_airy = _inverse(normalization_translation, normalization_rotation)
    expected_airy_translation, expected_airy_rotation = _compose(
        raw_translation, raw_rotation, raw_from_airy[0], raw_from_airy[1]
    )
    airy_edge = urdf_edges.get(("lidar_mount", "airy_lidar"))
    if airy_edge is None or not _same_vector(
        airy_edge[1], expected_airy_translation
    ) or not _same_rpy(airy_edge[2], _quaternion_rpy(expected_airy_rotation)):
        raise ValueError("production_sensor_tf_canonical_airy_mismatch")
    platform_translation, platform_rpy = _validate_platform_config(platform_base_tf_config_path)
    if not _same_vector(
        platform_translation,
        _vector(measurement["platform_base_tf"]["translation_xyz"], 3, "platform.translation"),
    ) or not _same_rpy(
        platform_rpy,
        _vector(measurement["platform_base_tf"]["rotation_rpy"], 3, "platform.rotation"),
    ):
        raise ValueError("production_sensor_tf_platform_measurement_mismatch")
    return robot_description


def validated_platform_transform(
    platform_base_tf_config_path: Path,
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    path = _regular_absolute(
        platform_base_tf_config_path, "production_sensor_tf_platform_base_tf_config"
    )
    return _validate_platform_config(path)


def validated_camera_transforms(
    camera_receipt_path: Path, camera_params_path: Path
) -> dict[str, tuple[tuple[float, ...], tuple[float, ...]]]:
    receipt = _load_json(
        _regular_absolute(camera_receipt_path, "production_sensor_tf_camera_receipt"),
        "production_sensor_tf_camera_receipt",
    )
    text = _regular_absolute(
        camera_params_path, "production_sensor_tf_camera_params"
    ).read_text(encoding="utf-8")
    output = {}
    for role, (parent, child) in CAMERA_FRAMES.items():
        transform = receipt.get("transforms", {}).get(role)
        if not isinstance(transform, dict) or (
            transform.get("parent") != parent or transform.get("child") != child
        ):
            raise ValueError(f"production_sensor_tf_camera_receipt_transform_invalid:{role}")
        translation = _yaml_vector(text, f"{role}_internal_tf_translation_xyz", 3)
        rotation = _yaml_vector(text, f"{role}_internal_tf_rotation_xyzw", 4)
        if not _same_vector(
            translation, _vector(transform.get("translation_xyz"), 3, f"{role}.translation")
        ) or not _same_quaternion(
            rotation, _vector(transform.get("rotation_xyzw"), 4, f"{role}.rotation")
        ):
            raise ValueError(f"production_sensor_tf_camera_receipt_transform_mismatch:{role}")
        output[role] = (translation, _quaternion_rpy(rotation))
    return output


def validated_airy_transform(
    robot_urdf_path: Path,
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    path = _regular_absolute(robot_urdf_path, "production_sensor_tf_robot_urdf")
    _validate_urdf(path)
    edge = _urdf_edges(path).get(("lidar_mount", "airy_lidar"))
    if edge is None:
        raise ValueError("production_sensor_tf_canonical_airy_missing")
    return edge[1], edge[2]


def _tf_echo_samples(text: str) -> list[tuple[tuple[float, ...], tuple[float, ...]]]:
    translations = re.findall(
        r"Translation:\s*\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\]",
        text,
    )
    rpy_samples = re.findall(
        r"RPY \(radian\)\s*\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\]",
        text,
    )
    if not translations or len(translations) != len(rpy_samples):
        raise ValueError("tf_sample_missing")
    samples = []
    for translation, rpy in zip(translations, rpy_samples):
        parsed_translation = tuple(float(value) for value in translation)
        parsed_rpy = tuple(float(value) for value in rpy)
        if not all(math.isfinite(value) for value in (*parsed_translation, *parsed_rpy)):
            raise ValueError("tf_sample_nonfinite")
        samples.append((parsed_translation, parsed_rpy))
    return samples


def validate_tf_echo_available(text: str) -> None:
    _tf_echo_samples(text)


def validate_tf_echo_matches(
    text: str,
    expected_translation: tuple[float, ...],
    expected_rpy: tuple[float, ...],
    tolerance: float = 1e-6,
) -> None:
    for translation, rpy in _tf_echo_samples(text):
        if any(
            abs(actual - expected) > tolerance
            for actual, expected in zip(translation, expected_translation)
        ) or any(abs(_angle_delta(actual, expected)) > tolerance
                 for actual, expected in zip(rpy, expected_rpy)):
            raise ValueError("tf_sample_does_not_match_accepted_config")


def validate_planar_base_footprint_echo(text: str) -> None:
    try:
        samples = _tf_echo_samples(text)
    except ValueError as error:
        if str(error) == "tf_sample_missing":
            raise ValueError("base_footprint_planarity_sample_missing") from error
        if str(error) == "tf_sample_nonfinite":
            raise ValueError("base_footprint_planarity_nonfinite") from error
        raise
    if not samples:
        raise ValueError("base_footprint_planarity_sample_missing")
    for translation, rpy in samples:
        z = translation[2]
        roll, pitch = rpy[0], rpy[1]
        if abs(z) > MAX_BASE_FOOTPRINT_Z_M:
            raise ValueError("base_footprint_nonplanar_z")
        if max(abs(roll), abs(pitch)) > MAX_BASE_FOOTPRINT_ROLL_PITCH_RAD:
            raise ValueError("base_footprint_nonplanar_roll_pitch")


def validate_tf_static_publishers(text: str, expected_fqns: set[str]) -> None:
    """Validate every /tf_static publisher endpoint by FQN and unique DDS GID.

    ROS graph introspection cannot attribute individual child frames to an
    endpoint, so the production contract accepts no unlisted static publisher.
    This makes the managed-child ownership check fail closed.
    """
    count_match = re.search(r"^Publisher count:\s*(\d+)\s*$", text, re.MULTILINE)
    if count_match is None:
        raise ValueError("tf_static_publisher_count_missing")
    publishers = []
    current: dict[str, str] = {}
    for raw_line in [*text.splitlines(), "Node name:"]:
        line = raw_line.strip()
        if line.startswith("Node name:"):
            if current.get("endpoint") == "PUBLISHER":
                publishers.append(current)
            current = {"name": line.split(":", 1)[1].strip()}
        elif line.startswith("Node namespace:"):
            current["namespace"] = line.split(":", 1)[1].strip()
        elif line.startswith("Endpoint type:"):
            current["endpoint"] = line.split(":", 1)[1].strip()
        elif line.startswith("GID:"):
            current["gid"] = line.split(":", 1)[1].strip().casefold()
    if int(count_match.group(1)) != len(publishers):
        raise ValueError("tf_static_publisher_endpoint_count_mismatch")
    observed_fqns, gids = set(), set()
    for endpoint in publishers:
        name, namespace, gid = (
            endpoint.get("name", ""), endpoint.get("namespace", ""), endpoint.get("gid", "")
        )
        if not name or not namespace.startswith("/") or not gid:
            raise ValueError("tf_static_publisher_endpoint_invalid")
        compact_gid = re.sub(r"[^0-9a-f]", "", gid)
        if len(compact_gid) < 16 or set(compact_gid) <= {"0"} or gid in gids:
            raise ValueError("tf_static_publisher_gid_invalid")
        gids.add(gid)
        fqn = f"/{name}" if namespace == "/" else f"{namespace.rstrip('/')}/{name}"
        if fqn in observed_fqns:
            raise ValueError(f"tf_static_publisher_fqn_duplicate:{fqn}")
        observed_fqns.add(fqn)
    if observed_fqns != expected_fqns:
        raise ValueError(
            "tf_static_publisher_owners_mismatch:expected="
            + ",".join(sorted(expected_fqns))
            + ":observed=" + ",".join(sorted(observed_fqns))
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receipt", type=Path)
    parser.add_argument("--robot-urdf", type=Path)
    parser.add_argument("--platform-base-tf-config", type=Path)
    parser.add_argument("--measurement-record", type=Path)
    parser.add_argument("--camera-params", type=Path)
    parser.add_argument("--camera-receipt", type=Path)
    parser.add_argument("--airy-adapter-config", type=Path)
    parser.add_argument("--airy-frame-receipt", type=Path)
    parser.add_argument("--expected-serial", default=EXPECTED_ROBOT_SERIAL)
    parser.add_argument("--tf-echo-output", type=Path)
    parser.add_argument("--tf-role", choices=("platform", "chest", "belly", "airy"))
    parser.add_argument("--tf-static-publishers-output", type=Path)
    parser.add_argument("--expected-tf-static-publisher", action="append", default=[])
    arguments = parser.parse_args()
    try:
        if arguments.tf_static_publishers_output is not None:
            publisher_path = _regular_absolute(
                arguments.tf_static_publishers_output, "tf_static_publishers_output")
            if not arguments.expected_tf_static_publisher:
                raise ValueError("tf_static_expected_publishers_missing")
            validate_tf_static_publishers(
                publisher_path.read_text(encoding="utf-8"),
                set(arguments.expected_tf_static_publisher),
            )
        elif arguments.tf_echo_output is not None:
            path = _regular_absolute(arguments.tf_echo_output, "base_footprint_tf_echo")
            text = path.read_text(encoding="utf-8")
            if arguments.tf_role is None:
                validate_planar_base_footprint_echo(text)
            else:
                if any(
                    value is None
                    for value in (
                        arguments.receipt,
                        arguments.robot_urdf,
                        arguments.platform_base_tf_config,
                        arguments.measurement_record,
                        arguments.camera_params,
                        arguments.camera_receipt,
                        arguments.airy_adapter_config,
                        arguments.airy_frame_receipt,
                    )
                ):
                    raise ValueError("production_sensor_tf_assets_required")
                validate(
                    arguments.receipt,
                    arguments.robot_urdf,
                    arguments.platform_base_tf_config,
                    arguments.measurement_record,
                    arguments.camera_params,
                    arguments.camera_receipt,
                    arguments.airy_adapter_config,
                    arguments.airy_frame_receipt,
                    arguments.expected_serial,
                )
            if arguments.tf_role == "platform":
                expected = validated_platform_transform(arguments.platform_base_tf_config)
                validate_tf_echo_matches(text, *expected)
            elif arguments.tf_role in {"chest", "belly"}:
                expected = validated_camera_transforms(
                    arguments.camera_receipt, arguments.camera_params
                )[arguments.tf_role]
                validate_tf_echo_matches(text, *expected)
            elif arguments.tf_role == "airy":
                expected = validated_airy_transform(arguments.robot_urdf)
                validate_tf_echo_matches(text, *expected)
        else:
            if any(
                value is None
                for value in (
                    arguments.receipt,
                    arguments.robot_urdf,
                    arguments.platform_base_tf_config,
                    arguments.measurement_record,
                    arguments.camera_params,
                    arguments.camera_receipt,
                    arguments.airy_adapter_config,
                    arguments.airy_frame_receipt,
                )
            ):
                raise ValueError("production_sensor_tf_assets_required")
            validate(
                arguments.receipt,
                arguments.robot_urdf,
                arguments.platform_base_tf_config,
                arguments.measurement_record,
                arguments.camera_params,
                arguments.camera_receipt,
                arguments.airy_adapter_config,
                arguments.airy_frame_receipt,
                arguments.expected_serial,
            )
    except (OSError, UnicodeError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 2
    print("production_sensor_tf_contract_accepted")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
