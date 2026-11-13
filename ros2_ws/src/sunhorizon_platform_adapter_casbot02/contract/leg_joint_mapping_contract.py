#!/usr/bin/env python3
"""Fail-closed Robot A vendor-id to URDF leg-joint mapping contract."""

from __future__ import annotations

import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import tempfile
from typing import Any, Callable
import xml.etree.ElementTree as ET


ROBOT_SCOPE = "Robot A"
ROBOT_SERIAL = "1421026005616"
KIND = "robot_a_leg_joint_name_mapping_receipt"
EVIDENCE_KIND = "robot_a_leg_joint_mapping_evidence"
REVIEW_STATEMENT_KIND = "robot_a_leg_joint_mapping_independent_review"
REVIEW_SIGNATURE_NAMESPACE = "sunhorizon-robot-a-leg-joint-mapping-review-v2"
SSH_KEYGEN = "/usr/bin/ssh-keygen"
BASE_LINK = "base_link"
LEFT_FOOT_LINK = "left_leg_ankle_roll_link"
RIGHT_FOOT_LINK = "right_leg_ankle_roll_link"
IDENTITY_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._@:-]{2,127}")
SHA256_RE = re.compile(r"[0-9a-f]{64}")
RELEASE_RE = re.compile(r"[0-9a-f]{40}")
ABSOLUTE_PATH_RE = re.compile(r"/[A-Za-z0-9._/@:+-]+")
EXPECTED_IDS = list(range(12))
CANONICAL_SOURCE_NAMES = [
    "LJ0", "LJ1", "LJ2", "LJ3", "LJ4", "LJ5",
    "RJ6", "RJ7", "RJ8", "RJ9", "RJ10", "RJ11",
]
EXPECTED_TOP_LEVEL_KEYS = {
    "schema_version",
    "kind",
    "robot_scope",
    "robot_serial",
    "release_sha",
    "source_contract",
    "vendor_urdf",
    "mapping_evidence",
    "mappings",
    "review",
    "accepted_for_position_proof",
    "motion_authorized",
    "canonical_binding",
    "canonical_binding_sha256",
}
CANONICAL_KEYS = EXPECTED_TOP_LEVEL_KEYS - {
    "schema_version", "kind", "canonical_binding", "canonical_binding_sha256"
}
REVIEW_KEYS = {
    "recorded_by", "reviewed_by", "reviewed_at", "accepted",
    "statement", "signature", "allowed_signers",
}
REVIEW_STATEMENT_KEYS = {
    "schema_version", "kind", "robot_scope", "robot_serial", "release_sha",
    "reviewed_by", "reviewed_at", "decision", "draft_receipt",
    "mapping_evidence",
}
EVIDENCE_KEYS = {
    "schema_version", "kind", "robot_scope", "robot_serial", "release_sha",
    "source_log", "sealed_status_block", "status_timestamp",
    "vendor_urdf", "mappings", "observed_fault_status",
}
SOURCE_LOG_KEYS = {
    "path", "size_bytes", "mtime_ns", "tail_offset_bytes", "tail_size_bytes",
    "status_block_offset_bytes", "status_block_size_bytes",
}
OBSERVED_FAULT_STATUS_KEYS = {
    "raw_fault_joint_id", "side", "group_index", "statusword", "error_code",
}
STATUS_MARKER = "状态信息(ROH)".encode("utf-8")
COMMAND_MARKER = "控制命令(ROH)".encode("utf-8")
STATUS_TIMESTAMP_RE = re.compile(
    r"20[0-9]{2}-[0-9]{2}-[0-9]{2} [0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]{3}"
)
JOINT_LINE_RE = re.compile(
    r"^\s*([0-9]+):\s*状态字=0x([0-9A-Fa-f]+).*?"
    r"错误=0x([0-9A-Fa-f]+)",
    re.MULTILINE,
)


class MappingContractError(ValueError):
    """The mapping receipt or one of its bound assets is invalid."""


def verify_review_signature(
    *,
    statement_payload: bytes,
    signature_payload: bytes,
    allowed_signers_payload: bytes,
    reviewer_identity: str,
) -> None:
    """Verify one review against only the bytes pinned by receipt hashes."""

    try:
        if not statement_payload or not signature_payload or not allowed_signers_payload:
            raise MappingContractError("mapping_review_signature_invalid")
        with tempfile.TemporaryDirectory(
            prefix="sunhorizon-leg-mapping-review-"
        ) as temporary:
            directory = Path(temporary)
            signature_path = directory / "signature"
            allowed_signers_path = directory / "allowed_signers"
            for path, payload in (
                (signature_path, signature_payload),
                (allowed_signers_path, allowed_signers_payload),
            ):
                descriptor = os.open(
                    path,
                    os.O_WRONLY
                    | os.O_CREAT
                    | os.O_EXCL
                    | getattr(os, "O_CLOEXEC", 0)
                    | getattr(os, "O_NOFOLLOW", 0),
                    0o600,
                )
                try:
                    offset = 0
                    while offset < len(payload):
                        written = os.write(descriptor, payload[offset:])
                        if written <= 0:
                            raise OSError("pinned_review_write_made_no_progress")
                        offset += written
                finally:
                    os.close(descriptor)
            result = subprocess.run(
                [
                    SSH_KEYGEN, "-Y", "verify", "-f", str(allowed_signers_path),
                    "-I", reviewer_identity, "-n", REVIEW_SIGNATURE_NAMESPACE,
                    "-s", str(signature_path),
                ],
                input=statement_payload,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
                timeout=5.0,
            )
    except MappingContractError:
        raise
    except (OSError, subprocess.SubprocessError) as exc:
        raise MappingContractError("mapping_review_signature_unavailable") from exc
    if result.returncode != 0:
        raise MappingContractError("mapping_review_signature_invalid")


def _validate_ed25519_allowed_signers(payload: bytes, reviewer_identity: str) -> None:
    try:
        lines = payload.decode("utf-8").splitlines()
    except UnicodeDecodeError as exc:
        raise MappingContractError("mapping_review_authority_invalid") from exc
    entries = [
        line.split() for line in lines
        if line.strip() and not line.lstrip().startswith("#")
    ]
    if (
        not entries
        or any(len(entry) != 3 or entry[1] != "ssh-ed25519" for entry in entries)
        or not any(reviewer_identity in entry[0].split(",") for entry in entries)
    ):
        raise MappingContractError("mapping_review_authority_invalid")


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def _stable_bytes(path: Path, label: str) -> bytes:
    if not path.is_absolute():
        raise MappingContractError(f"{label}_path_not_absolute")
    try:
        canonical = path.resolve(strict=True)
    except OSError as exc:
        raise MappingContractError(f"{label}_unavailable") from exc
    if canonical != path:
        raise MappingContractError(f"{label}_path_not_canonical_or_has_symlink")
    try:
        before = path.lstat()
    except OSError as exc:
        raise MappingContractError(f"{label}_unavailable") from exc
    if not path.is_file() or path.is_symlink() or before.st_nlink != 1:
        raise MappingContractError(f"{label}_not_single_regular_file")
    try:
        payload = path.read_bytes()
        after = path.lstat()
    except OSError as exc:
        raise MappingContractError(f"{label}_unreadable") from exc
    identity = lambda stat: (
        stat.st_dev, stat.st_ino, stat.st_mode, stat.st_nlink,
        stat.st_size, stat.st_mtime_ns,
    )
    if identity(before) != identity(after):
        raise MappingContractError(f"{label}_changed_while_reading")
    if not payload:
        raise MappingContractError(f"{label}_empty")
    return payload


def _load_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(_stable_bytes(path, label))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise MappingContractError(f"{label}_invalid_json") from exc
    if not isinstance(value, dict):
        raise MappingContractError(f"{label}_not_object")
    return value


def _asset(value: Any, label: str) -> tuple[dict[str, Any], bytes]:
    if not isinstance(value, dict) or set(value) != {"path", "sha256"}:
        raise MappingContractError(f"{label}_descriptor_invalid")
    path_value = value.get("path")
    digest = value.get("sha256")
    if not isinstance(path_value, str) or not SHA256_RE.fullmatch(str(digest)):
        raise MappingContractError(f"{label}_descriptor_invalid")
    payload = _stable_bytes(Path(path_value), label)
    if hashlib.sha256(payload).hexdigest() != digest:
        raise MappingContractError(f"{label}_sha256_mismatch")
    return value, payload


def _json_object(payload: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise MappingContractError(f"{label}_invalid_json") from exc
    if not isinstance(value, dict):
        raise MappingContractError(f"{label}_not_object")
    return value


def parse_status_block(block_payload: bytes) -> dict[str, Any]:
    """Parse exactly one completed ROH status block, never a command block."""

    try:
        text = block_payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise MappingContractError("roh_status_block_invalid_utf8") from exc
    if text.count("状态信息(ROH)") != 1 or "控制命令(ROH)" in text:
        raise MappingContractError("roh_status_block_boundary_invalid")
    timestamp_match = STATUS_TIMESTAMP_RE.search(text.splitlines()[0] if text else "")
    if timestamp_match is None:
        raise MappingContractError("roh_status_timestamp_missing")
    try:
        datetime.datetime.strptime(
            timestamp_match.group(0), "%Y-%m-%d %H:%M:%S.%f"
        )
    except ValueError as exc:
        raise MappingContractError("roh_status_timestamp_invalid") from exc
    left_header = re.search(r"^\[左腿\s*\]\s*电机0-5\s*$", text, re.MULTILINE)
    right_header = re.search(r"^\[右腿\s*\]\s*电机6-11\s*$", text, re.MULTILINE)
    if (
        left_header is None
        or right_header is None
        or left_header.end() >= right_header.start()
    ):
        raise MappingContractError("roh_leg_group_headers_invalid")
    next_group = re.search(r"^\[", text[right_header.end():], re.MULTILINE)
    right_end = (
        right_header.end() + next_group.start()
        if next_group is not None else len(text)
    )

    def parse_group(
        section: str, expected_ids: list[int], side: str
    ) -> list[dict[str, Any]]:
        values: dict[int, tuple[int, int]] = {}
        for match in JOINT_LINE_RE.finditer(section):
            joint_id = int(match.group(1))
            if joint_id in values:
                raise MappingContractError(f"roh_{side}_joint_duplicate")
            try:
                statusword = int(match.group(2), 16)
                error_code = int(match.group(3), 16)
            except ValueError as exc:
                raise MappingContractError(f"roh_{side}_fault_field_invalid") from exc
            if statusword > 0xffff or error_code > 0xffffffff:
                raise MappingContractError(f"roh_{side}_fault_field_out_of_range")
            values[joint_id] = (statusword, error_code)
        if sorted(values) != expected_ids:
            raise MappingContractError(f"roh_{side}_joint_ids_invalid")
        return [
            {
                "raw_fault_joint_id": joint_id,
                "side": side,
                "group_index": index,
                "statusword": values[joint_id][0],
                "error_code": values[joint_id][1],
            }
            for index, joint_id in enumerate(expected_ids)
        ]

    left = parse_group(
        text[left_header.end():right_header.start()], list(range(6)), "left"
    )
    right = parse_group(
        text[right_header.end():right_end], list(range(6, 12)), "right"
    )
    return {
        "status_timestamp": timestamp_match.group(0),
        "observed_fault_status": left + right,
    }


def find_latest_complete_status_block(tail_payload: bytes) -> dict[str, Any]:
    """Find the newest terminated status block in a stable log tail."""

    starts: list[int] = []
    offset = tail_payload.find(STATUS_MARKER)
    while offset >= 0:
        starts.append(offset)
        offset = tail_payload.find(STATUS_MARKER, offset + len(STATUS_MARKER))
    if not starts:
        raise MappingContractError("roh_status_block_missing")
    for index in range(len(starts) - 1, -1, -1):
        marker_start = starts[index]
        start = tail_payload.rfind(b"\n", 0, marker_start) + 1
        command_marker = tail_payload.find(
            COMMAND_MARKER, marker_start + len(STATUS_MARKER)
        )
        next_status = starts[index + 1] if index + 1 < len(starts) else -1
        if command_marker < 0 or (
            next_status >= 0 and next_status < command_marker
        ):
            continue
        end = tail_payload.rfind(b"\n", 0, command_marker) + 1
        # Include the separator prefix before the command marker only as part of
        # the source boundary, not the command marker or any commanded values.
        block = tail_payload[start:end]
        parsed = parse_status_block(block)
        return {"offset": start, "payload": block, **parsed}
    raise MappingContractError("roh_complete_status_block_missing")


def _urdf_chain(
    urdf_payload: bytes, base_link: str, foot_link: str, label: str
) -> list[str]:
    try:
        root = ET.fromstring(urdf_payload)
    except ET.ParseError as exc:
        raise MappingContractError("vendor_urdf_invalid_xml") from exc
    if root.tag != "robot":
        raise MappingContractError("vendor_urdf_root_invalid")
    links = {item.get("name") for item in root.findall("link")}
    if base_link not in links or foot_link not in links or base_link == foot_link:
        raise MappingContractError(f"{label}_links_missing")
    parents: dict[str, tuple[str, str, str]] = {}
    names: set[str] = set()
    for joint in root.findall("joint"):
        name = joint.get("name")
        joint_type = joint.get("type")
        parent = joint.find("parent")
        child = joint.find("child")
        parent_link = parent.get("link") if parent is not None else None
        child_link = child.get("link") if child is not None else None
        if not all(isinstance(item, str) and item for item in (
                name, joint_type, parent_link, child_link)):
            raise MappingContractError("vendor_urdf_joint_invalid")
        if name in names or child_link in parents:
            raise MappingContractError("vendor_urdf_not_tree")
        names.add(name)
        parents[child_link] = (parent_link, name, joint_type)
    movable_reverse: list[str] = []
    current = foot_link
    visited: set[str] = set()
    while current != base_link:
        if current in visited or current not in parents:
            raise MappingContractError(f"{label}_chain_not_connected")
        visited.add(current)
        parent_link, joint_name, joint_type = parents[current]
        if joint_type in {"revolute", "continuous", "prismatic"}:
            movable_reverse.append(joint_name)
        elif joint_type != "fixed":
            raise MappingContractError(f"{label}_unsupported_joint_type")
        current = parent_link
    return list(reversed(movable_reverse))


def derive_robot_a_leg_mappings(
    urdf_payload: bytes,
    *,
    base_link: str = BASE_LINK,
    left_foot_link: str = LEFT_FOOT_LINK,
    right_foot_link: str = RIGHT_FOOT_LINK,
) -> list[dict[str, Any]]:
    """Derive the fixed Robot A ID ordering from both actual URDF chains."""

    left = _urdf_chain(urdf_payload, base_link, left_foot_link, "left_leg")
    right = _urdf_chain(urdf_payload, base_link, right_foot_link, "right_leg")
    if len(left) != 6 or len(right) != 6 or len(set(left + right)) != 12:
        raise MappingContractError("vendor_urdf_leg_chains_must_be_6_plus_6")
    return [
        {
            "source_joint_name": CANONICAL_SOURCE_NAMES[index],
            "source_index": index,
            "raw_fault_joint_id": index,
            "joint_name": name,
            "side": "left" if index < 6 else "right",
            "chain_index": index % 6,
        }
        for index, name in enumerate(left + right)
    ]


def _validate_mapping_evidence(
    evidence_descriptor: Any,
    *,
    expected_robot_serial: str,
    expected_release_sha: str,
    expected_vendor_urdf: dict[str, Any],
    expected_mappings: list[dict[str, Any]],
) -> dict[str, Any]:
    _, evidence_payload = _asset(evidence_descriptor, "mapping_evidence")
    evidence = _json_object(evidence_payload, "mapping_evidence")
    if set(evidence) != EVIDENCE_KEYS:
        raise MappingContractError("mapping_evidence_shape_invalid")
    if (
        evidence.get("schema_version") != 2
        or evidence.get("kind") != EVIDENCE_KIND
        or evidence.get("robot_scope") != ROBOT_SCOPE
        or evidence.get("robot_serial") != expected_robot_serial
        or evidence.get("release_sha") != expected_release_sha
        or evidence.get("vendor_urdf") != expected_vendor_urdf
        or evidence.get("mappings") != expected_mappings
    ):
        raise MappingContractError("mapping_evidence_identity_invalid")
    source = evidence.get("source_log")
    if not isinstance(source, dict) or set(source) != SOURCE_LOG_KEYS:
        raise MappingContractError("mapping_source_log_shape_invalid")
    path = source.get("path")
    integer_fields = (
        "size_bytes", "mtime_ns", "tail_offset_bytes", "tail_size_bytes",
        "status_block_offset_bytes", "status_block_size_bytes",
    )
    if (
        not isinstance(path, str)
        or not ABSOLUTE_PATH_RE.fullmatch(path)
        or os.path.normpath(path) != path
        or any(
            not isinstance(source.get(key), int)
            or isinstance(source.get(key), bool)
            or source[key] < (
                1 if key in {
                    "size_bytes", "mtime_ns", "tail_size_bytes",
                    "status_block_size_bytes",
                } else 0
            )
            for key in integer_fields
        )
        or source["tail_offset_bytes"] + source["tail_size_bytes"]
        != source["size_bytes"]
        or source["status_block_offset_bytes"] < source["tail_offset_bytes"]
        or source["status_block_offset_bytes"]
        + source["status_block_size_bytes"] > source["size_bytes"]
    ):
        raise MappingContractError("mapping_source_log_metadata_invalid")
    _, sealed_payload = _asset(
        evidence.get("sealed_status_block"), "sealed_status_block"
    )
    if len(sealed_payload) != source["status_block_size_bytes"]:
        raise MappingContractError("sealed_status_block_size_mismatch")
    parsed = parse_status_block(sealed_payload)
    if (
        evidence.get("status_timestamp") != parsed["status_timestamp"]
        or evidence.get("observed_fault_status") != parsed["observed_fault_status"]
    ):
        raise MappingContractError("sealed_status_block_evidence_mismatch")
    return evidence


def validate_mapping_receipt(
    receipt_path: Path,
    *,
    expected_robot_serial: str,
    expected_release_sha: str,
    expected_review_authority_sha256: str | None = None,
    signature_verifier: Callable[..., None] = verify_review_signature,
) -> dict[str, Any]:
    if expected_robot_serial != ROBOT_SERIAL:
        raise MappingContractError("robot_a_serial_mismatch")
    if not RELEASE_RE.fullmatch(expected_release_sha):
        raise MappingContractError("release_sha_invalid")
    authority_sha256 = (
        expected_review_authority_sha256
        if expected_review_authority_sha256 is not None
        else os.environ.get("SUNHORIZON_LEG_MAPPING_REVIEW_AUTHORITY_SHA256", "")
    )
    if not SHA256_RE.fullmatch(authority_sha256):
        raise MappingContractError("mapping_review_authority_sha256_invalid")
    receipt_payload = _stable_bytes(receipt_path, "mapping_receipt")
    try:
        receipt = json.loads(receipt_payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise MappingContractError("mapping_receipt_invalid_json") from exc
    if not isinstance(receipt, dict) or set(receipt) != EXPECTED_TOP_LEVEL_KEYS:
        raise MappingContractError("mapping_receipt_shape_invalid")
    if (
        receipt.get("schema_version") != 2
        or receipt.get("kind") != KIND
        or receipt.get("robot_scope") != ROBOT_SCOPE
        or receipt.get("robot_serial") != expected_robot_serial
        or receipt.get("release_sha") != expected_release_sha
        or receipt.get("accepted_for_position_proof") is not True
        or receipt.get("motion_authorized") is not False
    ):
        raise MappingContractError("mapping_receipt_identity_or_acceptance_invalid")
    canonical = receipt.get("canonical_binding")
    expected_canonical = {key: receipt[key] for key in CANONICAL_KEYS}
    if canonical != expected_canonical:
        raise MappingContractError("mapping_receipt_canonical_binding_mismatch")
    if (
        not SHA256_RE.fullmatch(str(receipt.get("canonical_binding_sha256")))
        or canonical_sha256(canonical) != receipt["canonical_binding_sha256"]
    ):
        raise MappingContractError("mapping_receipt_canonical_hash_mismatch")

    source = receipt.get("source_contract")
    if source != {
        "position_source": {
            "topic": "/joint_states",
            "type": "sensor_msgs/msg/JointState",
            "selected_joint_names": CANONICAL_SOURCE_NAMES,
            "position_unit": "rad",
            "value_transform": "identity",
            "source_stamp": "ros_message_header",
        },
        "fault_source": {
            "format": "roh_ethercat_station_log",
            "required_raw_joint_ids": EXPECTED_IDS,
            "used_fields": ["statusword", "error_code"],
            "ignored_fields": ["position"],
            "source_stamp": "filesystem_mtime_ns",
        },
    }:
        raise MappingContractError("position_source_contract_invalid")

    urdf_descriptor = receipt.get("vendor_urdf")
    if not isinstance(urdf_descriptor, dict) or set(urdf_descriptor) != {
        "path", "sha256", "base_link", "left_foot_link", "right_foot_link"
    }:
        raise MappingContractError("vendor_urdf_descriptor_invalid")
    urdf_asset, urdf_payload = _asset(
        {key: urdf_descriptor[key] for key in ("path", "sha256")}, "vendor_urdf"
    )
    del urdf_asset
    for field in ("base_link", "left_foot_link", "right_foot_link"):
        if not isinstance(urdf_descriptor[field], str) or not urdf_descriptor[field]:
            raise MappingContractError("vendor_urdf_descriptor_invalid")
    if urdf_descriptor["left_foot_link"] == urdf_descriptor["right_foot_link"]:
        raise MappingContractError("vendor_urdf_foot_links_not_unique")

    mappings = receipt.get("mappings")
    if not isinstance(mappings, list) or len(mappings) != 12:
        raise MappingContractError("leg_joint_mappings_must_have_12_entries")
    expected_sides = ["left"] * 6 + ["right"] * 6
    names: list[str] = []
    for index, value in enumerate(mappings):
        if not isinstance(value, dict) or set(value) != {
            "source_joint_name", "source_index", "raw_fault_joint_id",
            "joint_name", "side", "chain_index"
        }:
            raise MappingContractError("leg_joint_mapping_entry_invalid")
        name = value.get("joint_name")
        if (
            value.get("source_joint_name") != CANONICAL_SOURCE_NAMES[index]
            or value.get("source_index") != index
            or value.get("raw_fault_joint_id") != index
            or value.get("side") != expected_sides[index]
            or value.get("chain_index") != index % 6
            or not isinstance(name, str)
            or not IDENTITY_RE.fullmatch(name)
            or name == "waist_yaw"
        ):
            raise MappingContractError("leg_joint_mapping_order_or_name_invalid")
        names.append(name)
    if len(set(names)) != 12:
        raise MappingContractError("leg_joint_names_not_unique")
    derived_mappings = derive_robot_a_leg_mappings(
        urdf_payload,
        base_link=urdf_descriptor["base_link"],
        left_foot_link=urdf_descriptor["left_foot_link"],
        right_foot_link=urdf_descriptor["right_foot_link"],
    )
    if derived_mappings != mappings:
        raise MappingContractError("mapping_does_not_match_vendor_urdf_leg_chains")

    _validate_mapping_evidence(
        receipt.get("mapping_evidence"),
        expected_robot_serial=expected_robot_serial,
        expected_release_sha=expected_release_sha,
        expected_vendor_urdf=urdf_descriptor,
        expected_mappings=mappings,
    )

    review = receipt.get("review")
    if not isinstance(review, dict) or set(review) != REVIEW_KEYS:
        raise MappingContractError("mapping_review_invalid")
    recorded_by = review.get("recorded_by")
    reviewed_by = review.get("reviewed_by")
    if (
        not isinstance(recorded_by, str)
        or not isinstance(reviewed_by, str)
        or not IDENTITY_RE.fullmatch(recorded_by)
        or not IDENTITY_RE.fullmatch(reviewed_by)
        or recorded_by.casefold() == reviewed_by.casefold()
        or review.get("accepted") is not True
        or not isinstance(review.get("reviewed_at"), str)
        or not re.fullmatch(
            r"20[0-9]{2}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z",
            review["reviewed_at"],
        )
    ):
        raise MappingContractError("mapping_review_invalid")
    try:
        datetime.datetime.strptime(review["reviewed_at"], "%Y-%m-%dT%H:%M:%SZ")
    except ValueError as exc:
        raise MappingContractError("mapping_review_invalid") from exc

    statement_payload = _asset(review.get("statement"), "mapping_review")[1]
    signature_payload = _asset(
        review.get("signature"), "mapping_review_signature"
    )[1]
    allowed_signers_payload = _asset(
        review.get("allowed_signers"), "mapping_review_allowed_signers"
    )[1]
    if review["allowed_signers"]["sha256"] != authority_sha256:
        raise MappingContractError("mapping_review_authority_mismatch")
    _validate_ed25519_allowed_signers(allowed_signers_payload, reviewed_by)
    statement = _json_object(statement_payload, "mapping_review")
    if set(statement) != REVIEW_STATEMENT_KEYS:
        raise MappingContractError("mapping_review_statement_shape_invalid")
    if (
        statement.get("schema_version") != 2
        or statement.get("kind") != REVIEW_STATEMENT_KIND
        or statement.get("robot_scope") != ROBOT_SCOPE
        or statement.get("robot_serial") != expected_robot_serial
        or statement.get("release_sha") != expected_release_sha
        or statement.get("reviewed_by") != reviewed_by
        or statement.get("reviewed_at") != review["reviewed_at"]
        or statement.get("decision") != {
            "accepted_for_position_proof": True,
            "motion_authorized": False,
        }
        or statement.get("mapping_evidence") != receipt["mapping_evidence"]
    ):
        raise MappingContractError("mapping_review_statement_invalid")
    draft_descriptor = statement.get("draft_receipt")
    if (
        not isinstance(draft_descriptor, dict)
        or set(draft_descriptor) != {"path", "sha256"}
        or not isinstance(draft_descriptor.get("path"), str)
        or not SHA256_RE.fullmatch(str(draft_descriptor.get("sha256")))
        or Path(draft_descriptor["path"]).name
        != "leg-joint-mapping.receipt.draft.json"
    ):
        raise MappingContractError("mapping_review_draft_binding_invalid")
    _, draft_payload = _asset(draft_descriptor, "mapping_review_draft")
    draft = _json_object(draft_payload, "mapping_review_draft")
    draft_review = draft.get("review")
    if (
        set(draft) != EXPECTED_TOP_LEVEL_KEYS
        or draft.get("schema_version") != 2
        or draft.get("kind") != KIND
        or draft.get("robot_scope") != ROBOT_SCOPE
        or draft.get("robot_serial") != expected_robot_serial
        or draft.get("release_sha") != expected_release_sha
        or draft.get("source_contract") != source
        or draft.get("vendor_urdf") != urdf_descriptor
        or draft.get("mapping_evidence") != receipt["mapping_evidence"]
        or draft.get("mappings") != mappings
        or draft.get("accepted_for_position_proof") is not False
        or draft.get("motion_authorized") is not False
        or not isinstance(draft_review, dict)
        or set(draft_review) != REVIEW_KEYS
        or draft_review.get("recorded_by") != recorded_by
        or draft_review.get("reviewed_by") is not None
        or draft_review.get("reviewed_at") is not None
        or draft_review.get("accepted") is not False
        or draft_review.get("statement") is not None
        or draft_review.get("signature") is not None
        or draft_review.get("allowed_signers") is not None
        or draft.get("canonical_binding")
        != {key: draft[key] for key in CANONICAL_KEYS}
        or canonical_sha256(draft["canonical_binding"])
        != draft.get("canonical_binding_sha256")
    ):
        raise MappingContractError("mapping_review_draft_invalid")
    signature_verifier(
        statement_payload=statement_payload,
        signature_payload=signature_payload,
        allowed_signers_payload=allowed_signers_payload,
        reviewer_identity=reviewed_by,
    )

    return {
        "leg_joint_names": names,
        "source_joint_names": CANONICAL_SOURCE_NAMES,
        "source_joint_indices": EXPECTED_IDS,
        "position_unit": "rad",
        "value_transform": "identity",
        "receipt_path": str(receipt_path),
        "receipt_sha256": hashlib.sha256(receipt_payload).hexdigest(),
        "evidence_sha256": receipt["mapping_evidence"]["sha256"],
        "source_stamp_kind": source["position_source"]["source_stamp"],
    }


def disabled_mapping_parameters() -> dict[str, Any]:
    return {
        "leg_joint_name_mapping_verified": False,
        "leg_joint_names": [],
        "source_joint_names": [],
        "source_joint_indices": [],
        "position_unit": "",
        "value_transform": "",
        "leg_joint_name_mapping_receipt_path": "",
        "leg_joint_name_mapping_receipt_sha256": "",
        "leg_joint_name_mapping_evidence_sha256": "",
    }


def launch_mapping_parameters(receipt_path: str) -> dict[str, Any]:
    if not receipt_path.strip():
        return disabled_mapping_parameters()
    serial = os.environ.get("SUNHORIZON_ROBOT_SERIAL", "")
    release = os.environ.get("SUNHORIZON_PLATFORM_EVIDENCE_RELEASE_SHA", "")
    validated = validate_mapping_receipt(
        Path(receipt_path), expected_robot_serial=serial,
        expected_release_sha=release,
        expected_review_authority_sha256=os.environ.get(
            "SUNHORIZON_LEG_MAPPING_REVIEW_AUTHORITY_SHA256", ""
        ),
    )
    return {
        "leg_joint_name_mapping_verified": True,
        "leg_joint_names": validated["leg_joint_names"],
        "source_joint_names": validated["source_joint_names"],
        "source_joint_indices": validated["source_joint_indices"],
        "position_unit": validated["position_unit"],
        "value_transform": validated["value_transform"],
        "leg_joint_name_mapping_receipt_path": validated["receipt_path"],
        "leg_joint_name_mapping_receipt_sha256": validated["receipt_sha256"],
        "leg_joint_name_mapping_evidence_sha256": validated["evidence_sha256"],
    }


def launch_ros_mapping_parameters(receipt_path: str) -> dict[str, Any]:
    """Return mapping overrides representable by ROS 2 Humble launch."""
    parameters = launch_mapping_parameters(receipt_path)
    if parameters["leg_joint_name_mapping_verified"]:
        return parameters

    if parameters["leg_joint_names"] != []:
        raise MappingContractError("disabled_mapping_must_not_supply_joint_names")
    # Humble launch_ros cannot infer an element type for an empty Python list.
    # Leave the declared/YAML empty string-array default in force instead.
    del parameters["leg_joint_names"]
    del parameters["source_joint_names"]
    del parameters["source_joint_indices"]
    return parameters
