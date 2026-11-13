#!/usr/bin/env python3
"""Record and independently finalize Robot A live ``/joint_states`` evidence.

The ``record`` command is read-only with respect to ROS: it validates the
publisher graph and subscribes to the vendor JointState stream.  It atomically
publishes a rate-evidence artifact plus an explicitly unaccepted receipt draft.
The separate ``finalize`` command revalidates those assets and requires an
independent reviewer before it writes an accepted platform-freshness receipt.
Neither command publishes ROS data or authorizes robot motion.
"""

from __future__ import annotations

import argparse
import copy
import ctypes
from dataclasses import dataclass
import datetime
import errno
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Any, Callable, Protocol

import live_joint_state_source_contract as contract


DEFAULT_DURATION_SEC = 12.0
GRAPH_TIMEOUT_SEC = 12.0
CAPTURE_OVERRUN_SEC = 10.0
DRAFT_FILENAME = "live-joint-state-source.receipt.draft.json"
EVIDENCE_FILENAME = "live-joint-state-rate.evidence.json"
UTC_SECONDS_RE = re.compile(
    r"20[0-9]{2}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z"
)


class RecorderError(ValueError):
    """The graph, live stream, draft, or output target is invalid."""


@dataclass(frozen=True)
class PublisherIdentity:
    node_name: str
    node_namespace: str
    topic_type: str
    endpoint_gid: str


@dataclass(frozen=True)
class JointStateSample:
    names: tuple[str, ...]
    positions: tuple[float, ...]
    stamp_ns: int
    received_ros_time_ns: int
    received_monotonic_ns: int


class CaptureBackend(Protocol):
    def capture(
        self,
        *,
        duration_sec: float,
        graph_timeout_sec: float,
    ) -> tuple[PublisherIdentity, list[JointStateSample]]:
        """Capture a validated source identity and raw samples."""


def _json_bytes(document: dict[str, Any]) -> bytes:
    return (
        json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _validate_release_and_serial(robot_serial: str, release_sha: str) -> None:
    if robot_serial != contract.ROBOT_SERIAL:
        raise RecorderError("robot_a_serial_mismatch")
    if contract.RELEASE_RE.fullmatch(release_sha) is None:
        raise RecorderError("release_sha_invalid")


def _validate_identity(identity: PublisherIdentity) -> None:
    if (
        identity.node_name != contract.SOURCE_OWNER_NAME
        or identity.node_namespace != contract.SOURCE_OWNER_NAMESPACE
        or identity.topic_type != contract.SOURCE_TYPE
        or contract.ENDPOINT_GID_RE.fullmatch(identity.endpoint_gid) is None
    ):
        raise RecorderError("source_publisher_identity_invalid")


def _validate_recorded_by(recorded_by: str) -> str:
    value = recorded_by.strip()
    if value != recorded_by or contract.IDENTITY_RE.fullmatch(value) is None:
        raise RecorderError("recorded_by_invalid")
    return value


def _source(identity: PublisherIdentity, names: tuple[str, ...]) -> dict[str, Any]:
    _validate_identity(identity)
    if (
        len(names) != contract.EXPECTED_SOURCE_NAME_COUNT
        or len(set(names)) != contract.EXPECTED_SOURCE_NAME_COUNT
        or any(contract.IDENTITY_RE.fullmatch(name) is None for name in names)
        or any(name not in names for name in contract.SELECTED_SOURCE_NAMES)
    ):
        raise RecorderError("source_joint_names_invalid")
    return {
        "topic": contract.SOURCE_TOPIC,
        "type": identity.topic_type,
        "publisher_node_name": identity.node_name,
        "publisher_node_namespace": identity.node_namespace,
        # This is evidence about the capture epoch, not a cross-restart
        # runtime identity.  DDS endpoint GIDs are expected to rotate when the
        # reviewed hlorin executable restarts.
        "recorded_publisher_endpoint_gid": identity.endpoint_gid,
        "runtime_binding": contract.RUNTIME_BINDING,
        "ordered_joint_names": list(names),
        "position_semantics": {
            "unit": contract.POSITION_UNIT,
            "value_transform": contract.VALUE_TRANSFORM,
            "selected_joint_names": contract.SELECTED_SOURCE_NAMES,
            "selected_joint_indices": [
                names.index(name) for name in contract.SELECTED_SOURCE_NAMES
            ],
            "output_topic": contract.OUTPUT_TOPIC,
        },
    }


def calculate_metrics(
    samples: list[JointStateSample], *, duration_sec: float
) -> tuple[tuple[str, ...], dict[str, Any]]:
    """Strictly validate frames and calculate source-stamp rate metrics."""

    if not math.isfinite(duration_sec) or duration_sec < contract.MINIMUM_EVIDENCE_DURATION_SEC:
        raise RecorderError("capture_duration_below_contract_minimum")
    if len(samples) < 2:
        raise RecorderError("insufficient_joint_state_samples")
    expected_names = samples[0].names
    previous_source_stamp: int | None = None
    previous_receive_monotonic: int | None = None
    maximum_source_gap_sec = 0.0
    maximum_receive_gap_sec = 0.0
    source_ages_sec: list[float] = []
    for sample in samples:
        if sample.names != expected_names:
            raise RecorderError("joint_name_sequence_changed")
        if len(sample.positions) != contract.EXPECTED_SOURCE_NAME_COUNT:
            raise RecorderError("joint_position_count_invalid")
        if any(not math.isfinite(value) for value in sample.positions):
            raise RecorderError("joint_position_nonfinite")
        if sample.stamp_ns <= 0:
            raise RecorderError("source_stamp_invalid")
        if (
            sample.received_ros_time_ns <= 0
            or sample.received_monotonic_ns <= 0
            or sample.stamp_ns > sample.received_ros_time_ns
        ):
            raise RecorderError("source_stamp_in_future")
        source_age_sec = (
            sample.received_ros_time_ns - sample.stamp_ns
        ) / 1e9
        if source_age_sec > contract.MAXIMUM_ACCEPTED_SOURCE_AGE_SEC:
            raise RecorderError("source_stamp_stale")
        source_ages_sec.append(source_age_sec)
        if previous_source_stamp is not None:
            if sample.stamp_ns <= previous_source_stamp:
                raise RecorderError("source_stamp_not_strictly_monotonic")
            if sample.received_monotonic_ns <= previous_receive_monotonic:
                raise RecorderError("receive_time_not_strictly_monotonic")
            maximum_source_gap_sec = max(
                maximum_source_gap_sec,
                (sample.stamp_ns - previous_source_stamp) / 1e9,
            )
            maximum_receive_gap_sec = max(
                maximum_receive_gap_sec,
                (
                    sample.received_monotonic_ns - previous_receive_monotonic
                ) / 1e9,
            )
        previous_source_stamp = sample.stamp_ns
        previous_receive_monotonic = sample.received_monotonic_ns

    source_duration = (samples[-1].stamp_ns - samples[0].stamp_ns) / 1e9
    receive_duration = (
        samples[-1].received_monotonic_ns
        - samples[0].received_monotonic_ns
    ) / 1e9
    observed_rate = (
        (len(samples) - 1) / receive_duration if receive_duration > 0.0 else 0.0
    )
    metrics = {
        "sample_count": len(samples),
        "duration_sec": source_duration,
        "receive_duration_sec": receive_duration,
        "observed_rate_hz": observed_rate,
        "maximum_gap_sec": maximum_source_gap_sec,
        "maximum_receive_gap_sec": maximum_receive_gap_sec,
        "maximum_source_age_sec": max(source_ages_sec),
        "minimum_accepted_rate_hz": contract.MINIMUM_ACCEPTED_RATE_HZ,
        "maximum_accepted_receive_gap_sec": (
            contract.MAXIMUM_ACCEPTED_RECEIVE_GAP_SEC
        ),
        "maximum_accepted_source_age_sec": (
            contract.MAXIMUM_ACCEPTED_SOURCE_AGE_SEC
        ),
        "first_source_stamp_ns": samples[0].stamp_ns,
        "last_source_stamp_ns": samples[-1].stamp_ns,
        "first_receive_ros_time_ns": samples[0].received_ros_time_ns,
        "last_receive_ros_time_ns": samples[-1].received_ros_time_ns,
        "first_receive_monotonic_ns": samples[0].received_monotonic_ns,
        "last_receive_monotonic_ns": samples[-1].received_monotonic_ns,
        "first_source_age_sec": source_ages_sec[0],
        "last_source_age_sec": source_ages_sec[-1],
    }
    if source_duration < duration_sec:
        raise RecorderError("source_stamp_duration_below_requested_duration")
    if receive_duration < duration_sec:
        raise RecorderError("receive_duration_below_requested_duration")
    if (
        len(samples) < int(contract.MINIMUM_ACCEPTED_RATE_HZ * receive_duration)
        or observed_rate < contract.MINIMUM_ACCEPTED_RATE_HZ
        or maximum_source_gap_sec <= 0.0
        or maximum_source_gap_sec > contract.MAXIMUM_ACCEPTED_GAP_SEC
        or maximum_receive_gap_sec <= 0.0
        or maximum_receive_gap_sec > contract.MAXIMUM_ACCEPTED_RECEIVE_GAP_SEC
    ):
        raise RecorderError("live_joint_state_rate_contract_not_met")
    return expected_names, metrics


def build_documents(
    *,
    identity: PublisherIdentity,
    samples: list[JointStateSample],
    duration_sec: float,
    robot_serial: str,
    release_sha: str,
    recorded_by: str,
    final_output_directory: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Build a schema-exact evidence artifact and an unaccepted draft."""

    _validate_release_and_serial(robot_serial, release_sha)
    recorded_by = _validate_recorded_by(recorded_by)
    names, metrics = calculate_metrics(samples, duration_sec=duration_sec)
    source = _source(identity, names)
    evidence = {
        "schema_version": 2,
        "kind": contract.RATE_EVIDENCE_KIND,
        "robot_scope": contract.ROBOT_SCOPE,
        "robot_serial": robot_serial,
        "release_sha": release_sha,
        "source": source,
        **metrics,
    }
    if set(evidence) != contract.RATE_EVIDENCE_ARTIFACT_KEYS:
        raise RecorderError("rate_evidence_internal_schema_mismatch")
    evidence_payload = _json_bytes(evidence)
    receipt_metrics = {
        key: metrics[key] for key in contract.RATE_RECEIPT_METRIC_KEYS
    }
    draft: dict[str, Any] = {
        "schema_version": 2,
        "kind": contract.KIND,
        "robot_scope": contract.ROBOT_SCOPE,
        "robot_serial": robot_serial,
        "release_sha": release_sha,
        "source": source,
        "rate_evidence": {
            "artifact": {
                "path": str(final_output_directory / EVIDENCE_FILENAME),
                "sha256": _sha256_bytes(evidence_payload),
            },
            **receipt_metrics,
        },
        "review": {
            "recorded_by": recorded_by,
            "reviewed_by": None,
            "reviewed_at": None,
            "accepted": False,
            "statement": None,
            "signature": None,
            "allowed_signers": None,
        },
        "accepted_for_platform_freshness": False,
        "motion_authorized": False,
    }
    draft["canonical_binding"] = {
        key: copy.deepcopy(draft[key]) for key in contract.CANONICAL_KEYS
    }
    draft["canonical_binding_sha256"] = contract.canonical_sha256(
        draft["canonical_binding"]
    )
    if set(draft) != contract.EXPECTED_KEYS:
        raise RecorderError("draft_receipt_internal_schema_mismatch")
    return evidence, draft


def _canonical_existing_directory(path: Path, label: str) -> Path:
    if not path.is_absolute():
        raise RecorderError(f"{label}_not_absolute")
    try:
        before = path.lstat()
        resolved = path.resolve(strict=True)
    except OSError as exc:
        raise RecorderError(f"{label}_unavailable") from exc
    if path.is_symlink() or not path.is_dir() or resolved != path or before.st_nlink < 1:
        raise RecorderError(f"{label}_not_canonical_directory")
    return path


def _write_file_exclusive(path: Path, payload: bytes) -> None:
    descriptor = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    try:
        offset = 0
        while offset < len(payload):
            count = os.write(descriptor, payload[offset:])
            if count <= 0:
                raise OSError("output_write_made_no_progress")
            offset += count
        os.fsync(descriptor)
    except Exception:
        os.close(descriptor)
        path.unlink(missing_ok=True)
        raise
    else:
        os.close(descriptor)


def _rename_directory_no_replace(source: Path, target: Path) -> None:
    """Atomically publish a directory without replacing any existing inode."""

    libc = ctypes.CDLL(None, use_errno=True)
    if sys.platform.startswith("linux") and hasattr(libc, "renameat2"):
        renameat2 = libc.renameat2
        renameat2.argtypes = [
            ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p,
            ctypes.c_uint,
        ]
        renameat2.restype = ctypes.c_int
        result = renameat2(
            -100, os.fsencode(source), -100, os.fsencode(target), 1
        )
        if result == 0:
            return
        code = ctypes.get_errno()
        if code in (errno.EEXIST, errno.ENOTEMPTY):
            raise RecorderError("output_directory_already_exists")
        if code not in (errno.ENOSYS, errno.EINVAL):
            raise OSError(code, os.strerror(code), str(target))
    elif sys.platform == "darwin" and hasattr(libc, "renamex_np"):
        renamex_np = libc.renamex_np
        renamex_np.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint]
        renamex_np.restype = ctypes.c_int
        result = renamex_np(os.fsencode(source), os.fsencode(target), 0x00000004)
        if result == 0:
            return
        code = ctypes.get_errno()
        if code in (errno.EEXIST, errno.ENOTEMPTY):
            raise RecorderError("output_directory_already_exists")
        if code not in (errno.ENOTSUP, errno.EINVAL):
            raise OSError(code, os.strerror(code), str(target))

    raise RecorderError("atomic_no_replace_rename_unavailable")


def atomic_write_recording(
    output_directory: Path,
    evidence: dict[str, Any],
    draft: dict[str, Any],
) -> None:
    """Publish both files together by atomically renaming a sibling directory."""

    if not output_directory.is_absolute():
        raise RecorderError("output_directory_not_absolute")
    parent = _canonical_existing_directory(output_directory.parent, "output_parent")
    try:
        output_directory.lstat()
    except FileNotFoundError:
        pass
    else:
        raise RecorderError("output_directory_already_exists")
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{output_directory.name}.tmp.", dir=str(parent))
    )
    try:
        _write_file_exclusive(temporary / EVIDENCE_FILENAME, _json_bytes(evidence))
        _write_file_exclusive(temporary / DRAFT_FILENAME, _json_bytes(draft))
        descriptor = os.open(temporary, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        _rename_directory_no_replace(temporary, output_directory)
        temporary = Path()
        descriptor = os.open(parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        if temporary != Path() and temporary.exists():
            shutil.rmtree(temporary)


def record(
    *,
    output_directory: Path,
    release_sha: str,
    recorded_by: str,
    duration_sec: float = DEFAULT_DURATION_SEC,
    robot_serial: str = contract.ROBOT_SERIAL,
    graph_timeout_sec: float = GRAPH_TIMEOUT_SEC,
    backend: CaptureBackend,
) -> dict[str, Any]:
    """Capture live samples and atomically create evidence plus a draft."""

    if not math.isfinite(graph_timeout_sec) or graph_timeout_sec <= 0.0:
        raise RecorderError("graph_timeout_invalid")
    if not output_directory.is_absolute():
        raise RecorderError("output_directory_not_absolute")
    if output_directory.exists() or output_directory.is_symlink():
        raise RecorderError("output_directory_already_exists")
    identity, samples = backend.capture(
        duration_sec=duration_sec,
        graph_timeout_sec=graph_timeout_sec,
    )
    evidence, draft = build_documents(
        identity=identity,
        samples=samples,
        duration_sec=duration_sec,
        robot_serial=robot_serial,
        release_sha=release_sha,
        recorded_by=recorded_by,
        final_output_directory=output_directory,
    )
    atomic_write_recording(output_directory, evidence, draft)
    return {
        "accepted": False,
        "motion_authorized": False,
        "output_directory": str(output_directory),
        "evidence": str(output_directory / EVIDENCE_FILENAME),
        "evidence_sha256": _sha256_file(output_directory / EVIDENCE_FILENAME),
        "draft_receipt": str(output_directory / DRAFT_FILENAME),
        "draft_receipt_sha256": _sha256_file(output_directory / DRAFT_FILENAME),
        "sample_count": evidence["sample_count"],
        "duration_sec": evidence["duration_sec"],
        "receive_duration_sec": evidence["receive_duration_sec"],
        "observed_rate_hz": evidence["observed_rate_hz"],
        "maximum_gap_sec": evidence["maximum_gap_sec"],
        "maximum_receive_gap_sec": evidence["maximum_receive_gap_sec"],
        "maximum_source_age_sec": evidence["maximum_source_age_sec"],
    }


def _load_draft(path: Path) -> tuple[dict[str, Any], bytes, dict[str, Any]]:
    payload = contract._stable_bytes(path, "live_joint_state_draft")
    document = contract._json_object(payload, "live_joint_state_draft")
    if set(document) != contract.EXPECTED_KEYS:
        raise RecorderError("draft_receipt_shape_invalid")
    if (
        document.get("schema_version") != 2
        or document.get("kind") != contract.KIND
        or document.get("robot_scope") != contract.ROBOT_SCOPE
        or document.get("accepted_for_platform_freshness") is not False
        or document.get("motion_authorized") is not False
    ):
        raise RecorderError("draft_receipt_state_invalid")
    review = document.get("review")
    if not isinstance(review, dict) or set(review) != contract.REVIEW_KEYS:
        raise RecorderError("draft_review_shape_invalid")
    if (
        review.get("accepted") is not False
        or review.get("reviewed_by") is not None
        or review.get("reviewed_at") is not None
        or review.get("statement") is not None
        or review.get("signature") is not None
        or review.get("allowed_signers") is not None
        or not isinstance(review.get("recorded_by"), str)
        or contract.IDENTITY_RE.fullmatch(review["recorded_by"]) is None
    ):
        raise RecorderError("draft_review_state_invalid")
    canonical = document.get("canonical_binding")
    if canonical != {key: document[key] for key in contract.CANONICAL_KEYS}:
        raise RecorderError("draft_canonical_binding_mismatch")
    if contract.canonical_sha256(canonical) != document.get("canonical_binding_sha256"):
        raise RecorderError("draft_canonical_hash_mismatch")

    source = document.get("source")
    names = source.get("ordered_joint_names") if isinstance(source, dict) else None
    if not isinstance(names, list):
        raise RecorderError("draft_source_invalid")
    identity = PublisherIdentity(
        node_name=source.get("publisher_node_name"),
        node_namespace=source.get("publisher_node_namespace"),
        topic_type=source.get("type"),
        endpoint_gid=source.get("recorded_publisher_endpoint_gid"),
    )
    expected_source = _source(identity, tuple(names))
    if source != expected_source:
        raise RecorderError("draft_source_invalid")

    rate = document.get("rate_evidence")
    if not isinstance(rate, dict) or set(rate) != {
        "artifact", *contract.RATE_RECEIPT_METRIC_KEYS,
    }:
        raise RecorderError("draft_rate_evidence_shape_invalid")
    evidence_payload = contract._asset(
        rate.get("artifact"), "live_joint_state_rate_evidence"
    )
    evidence = contract._json_object(
        evidence_payload, "live_joint_state_rate_evidence"
    )
    if set(evidence) != contract.RATE_EVIDENCE_ARTIFACT_KEYS:
        raise RecorderError("draft_rate_evidence_artifact_shape_invalid")
    if (
        evidence.get("schema_version") != 2
        or evidence.get("kind") != contract.RATE_EVIDENCE_KIND
        or evidence.get("robot_scope") != contract.ROBOT_SCOPE
        or evidence.get("robot_serial") != document.get("robot_serial")
        or evidence.get("release_sha") != document.get("release_sha")
        or evidence.get("source") != source
    ):
        raise RecorderError("draft_rate_evidence_identity_invalid")
    metric_keys = contract.RATE_RECEIPT_METRIC_KEYS
    if contract.canonical_bytes({key: evidence[key] for key in metric_keys}) != (
        contract.canonical_bytes({key: rate[key] for key in metric_keys})
    ):
        raise RecorderError("draft_rate_evidence_mismatch")
    try:
        contract._validate_rate_metrics(evidence)
        contract._validate_rate_artifact_timestamps(evidence, rate)
    except contract.LiveJointStateContractError as exc:
        raise RecorderError("draft_rate_evidence_invalid") from exc
    _validate_release_and_serial(document.get("robot_serial"), document.get("release_sha"))
    return document, payload, evidence


def _validate_review_time(
    reviewed_at: str,
    *,
    evidence: dict[str, Any],
    now_utc: datetime.datetime | None,
) -> None:
    try:
        contract._validate_review_time(
            reviewed_at,
            last_receive_ros_time_ns=evidence["last_receive_ros_time_ns"],
            now_utc=now_utc,
        )
    except contract.LiveJointStateContractError as exc:
        raise RecorderError(str(exc)) from exc


def _write_json_exclusive(
    path: Path,
    document: dict[str, Any],
    *,
    validate_staged: Callable[[Path], Any] | None = None,
) -> None:
    if not path.is_absolute():
        raise RecorderError("output_not_absolute")
    parent = _canonical_existing_directory(path.parent, "output_parent")
    try:
        path.lstat()
    except FileNotFoundError:
        pass
    else:
        raise RecorderError("output_already_exists")
    staging = Path(tempfile.mkdtemp(prefix=f".{path.name}.tmp.", dir=str(parent)))
    try:
        staged_path = staging / path.name
        _write_file_exclusive(staged_path, _json_bytes(document))
        if validate_staged is not None:
            validate_staged(staged_path)
        _rename_directory_no_replace(staged_path, path)
        descriptor = os.open(parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        shutil.rmtree(staging)


def create_review_statement(
    *,
    draft_path: Path,
    output_path: Path,
    reviewed_by: str,
    reviewed_at: str,
    now_utc: datetime.datetime | None = None,
) -> dict[str, Any]:
    """Create the exact statement an independent reviewer must sign."""

    reviewed_by = reviewed_by.strip()
    if contract.IDENTITY_RE.fullmatch(reviewed_by) is None:
        raise RecorderError("reviewed_by_invalid")
    if UTC_SECONDS_RE.fullmatch(reviewed_at) is None:
        raise RecorderError("reviewed_at_invalid")
    try:
        datetime.datetime.strptime(reviewed_at, "%Y-%m-%dT%H:%M:%SZ")
    except ValueError as exc:
        raise RecorderError("reviewed_at_invalid") from exc
    draft, draft_payload, evidence = _load_draft(draft_path)
    _validate_review_time(
        reviewed_at, evidence=evidence, now_utc=now_utc
    )
    if reviewed_by.casefold() == draft["review"]["recorded_by"].casefold():
        raise RecorderError("reviewer_must_be_independent_of_recorder")
    statement = {
        "schema_version": 2,
        "kind": contract.REVIEW_STATEMENT_KIND,
        "robot_scope": contract.ROBOT_SCOPE,
        "robot_serial": draft["robot_serial"],
        "release_sha": draft["release_sha"],
        "reviewed_by": reviewed_by,
        "reviewed_at": reviewed_at,
        "decision": {
            "accepted_for_platform_freshness": True,
            "motion_authorized": False,
        },
        "draft_receipt": {
            "path": str(draft_path),
            "sha256": _sha256_bytes(draft_payload),
        },
        "rate_evidence": copy.deepcopy(draft["rate_evidence"]["artifact"]),
    }
    if set(statement) != contract.REVIEW_STATEMENT_KEYS:
        raise RecorderError("review_statement_internal_schema_mismatch")
    _write_json_exclusive(output_path, statement)
    return {
        "accepted": False,
        "motion_authorized": False,
        "statement": str(output_path),
        "statement_sha256": _sha256_file(output_path),
        "signature_namespace": contract.REVIEW_SIGNATURE_NAMESPACE,
    }


def _descriptor(path: Path, payload: bytes) -> dict[str, str]:
    return {"path": str(path), "sha256": _sha256_bytes(payload)}


def finalize(
    *,
    draft_path: Path,
    review_statement_path: Path,
    review_signature_path: Path,
    allowed_signers_path: Path,
    expected_review_authority_sha256: str,
    output_path: Path,
    now_utc: datetime.datetime | None = None,
) -> dict[str, Any]:
    """Create a receipt from a signed, authority-pinned independent review."""

    if contract.SHA256_RE.fullmatch(expected_review_authority_sha256) is None:
        raise RecorderError("review_authority_sha256_invalid")
    draft, draft_payload, evidence = _load_draft(draft_path)
    statement_payload = contract._stable_bytes(
        review_statement_path, "live_joint_state_review"
    )
    signature_payload = contract._stable_bytes(
        review_signature_path, "live_joint_state_review_signature"
    )
    allowed_signers_payload = contract._stable_bytes(
        allowed_signers_path, "live_joint_state_review_allowed_signers"
    )
    statement_descriptor = _descriptor(review_statement_path, statement_payload)
    signature_descriptor = _descriptor(review_signature_path, signature_payload)
    allowed_signers_descriptor = _descriptor(
        allowed_signers_path, allowed_signers_payload
    )
    if allowed_signers_descriptor["sha256"] != expected_review_authority_sha256:
        raise RecorderError("review_authority_sha256_mismatch")
    statement = contract._json_object(
        statement_payload, "live_joint_state_review"
    )
    if set(statement) != contract.REVIEW_STATEMENT_KEYS:
        raise RecorderError("review_statement_shape_invalid")
    reviewed_by = statement.get("reviewed_by")
    reviewed_at = statement.get("reviewed_at")
    if (
        not isinstance(reviewed_by, str)
        or contract.IDENTITY_RE.fullmatch(reviewed_by) is None
        or reviewed_by.casefold() == draft["review"]["recorded_by"].casefold()
        or not isinstance(reviewed_at, str)
        or UTC_SECONDS_RE.fullmatch(reviewed_at) is None
        or statement.get("schema_version") != 2
        or statement.get("kind") != contract.REVIEW_STATEMENT_KIND
        or statement.get("robot_scope") != contract.ROBOT_SCOPE
        or statement.get("robot_serial") != draft["robot_serial"]
        or statement.get("release_sha") != draft["release_sha"]
        or statement.get("decision") != {
            "accepted_for_platform_freshness": True,
            "motion_authorized": False,
        }
        or statement.get("draft_receipt") != {
            "path": str(draft_path),
            "sha256": _sha256_bytes(draft_payload),
        }
        or statement.get("rate_evidence") != draft["rate_evidence"]["artifact"]
    ):
        raise RecorderError("review_statement_binding_invalid")
    try:
        datetime.datetime.strptime(reviewed_at, "%Y-%m-%dT%H:%M:%SZ")
    except ValueError as exc:
        raise RecorderError("reviewed_at_invalid") from exc
    _validate_review_time(
        reviewed_at, evidence=evidence, now_utc=now_utc
    )
    contract._validate_ed25519_allowed_signers(
        allowed_signers_payload, reviewed_by,
    )
    contract.verify_review_signature(
        statement_payload=statement_payload,
        signature_payload=signature_payload,
        allowed_signers_payload=allowed_signers_payload,
        reviewer_identity=reviewed_by,
    )

    accepted = copy.deepcopy(draft)
    accepted["review"] = {
        "recorded_by": draft["review"]["recorded_by"],
        "reviewed_by": reviewed_by,
        "reviewed_at": reviewed_at,
        "accepted": True,
        "statement": statement_descriptor,
        "signature": signature_descriptor,
        "allowed_signers": allowed_signers_descriptor,
    }
    accepted["accepted_for_platform_freshness"] = True
    accepted["motion_authorized"] = False
    accepted["canonical_binding"] = {
        key: copy.deepcopy(accepted[key]) for key in contract.CANONICAL_KEYS
    }
    accepted["canonical_binding_sha256"] = contract.canonical_sha256(
        accepted["canonical_binding"]
    )
    _write_json_exclusive(
        output_path,
        accepted,
        validate_staged=lambda staged: contract.validate_live_joint_state_receipt(
            staged,
            expected_robot_serial=accepted["robot_serial"],
            expected_release_sha=accepted["release_sha"],
            expected_review_authority_sha256=(
                expected_review_authority_sha256
            ),
            signature_verifier=contract.verify_review_signature,
            now_utc=now_utc,
        ),
    )
    return {
        "accepted_for_platform_freshness": True,
        "motion_authorized": False,
        "draft_receipt_sha256": _sha256_bytes(draft_payload),
        "review_statement_sha256": statement_descriptor["sha256"],
        "review_signature_sha256": signature_descriptor["sha256"],
        "review_authority_sha256": allowed_signers_descriptor["sha256"],
        "output": str(output_path),
        "output_sha256": _sha256_file(output_path),
    }


CAPTURE_HELPER_EXECUTABLE = "live_joint_state_source_capture"
CAPTURE_HELPER_SCHEMA = "sunhorizon.robot_a.live_joint_state_capture.v1"
MAXIMUM_CAPTURE_ARTIFACT_BYTES = 64 * 1024 * 1024


class RclcppCaptureBackend:
    """Run the Humble-compatible rclcpp MessageInfo capture helper.

    Humble rclpy subscriptions only invoke ``callback(message)`` and the
    binding's take result does not expose the publisher GID.  The package-local
    rclcpp helper therefore performs the per-frame MessageInfo/graph endpoint
    check.  This Python side strictly parses the helper's private temporary
    artifact before the existing metric and receipt validation runs.
    """

    def __init__(
        self,
        *,
        helper_path: Path | None = None,
        runner: Callable[..., Any] = subprocess.run,
    ) -> None:
        self._helper_path = helper_path
        self._runner = runner

    @staticmethod
    def _installed_helper_path() -> Path:
        script_path = Path(__file__).resolve()
        try:
            package_share = script_path.parents[1]
            share_directory = script_path.parents[2]
            install_prefix = script_path.parents[3]
        except IndexError as exc:
            raise RecorderError("capture_helper_install_layout_invalid") from exc
        if (
            script_path.parent.name != "contract"
            or package_share.name != "sunhorizon_platform_adapter_casbot02"
            or share_directory.name != "share"
        ):
            raise RecorderError("capture_helper_not_run_from_install_space")
        return (
            install_prefix
            / "lib"
            / "sunhorizon_platform_adapter_casbot02"
            / CAPTURE_HELPER_EXECUTABLE
        )

    def _resolved_helper_path(self) -> Path:
        helper_path = (
            self._helper_path
            if self._helper_path is not None
            else self._installed_helper_path()
        )
        if (
            not helper_path.is_absolute()
            or not helper_path.is_file()
            or helper_path.is_symlink()
            or not os.access(helper_path, os.X_OK)
        ):
            raise RecorderError("capture_helper_unavailable")
        return helper_path.resolve(strict=True)

    @staticmethod
    def _strict_integer(value: Any, reason: str) -> int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise RecorderError(reason)
        return value

    @classmethod
    def _parse_capture_artifact(
        cls, artifact_path: Path
    ) -> tuple[PublisherIdentity, list[JointStateSample]]:
        try:
            artifact_size = artifact_path.stat().st_size
        except OSError as exc:
            raise RecorderError("capture_helper_artifact_missing") from exc
        if artifact_size <= 0 or artifact_size > MAXIMUM_CAPTURE_ARTIFACT_BYTES:
            raise RecorderError("capture_helper_artifact_size_invalid")
        document = json.loads(artifact_path.read_text(encoding="utf-8"))
        if not isinstance(document, dict) or set(document) != {
            "schema",
            "source",
            "samples",
        }:
            raise RecorderError("capture_helper_artifact_schema_invalid")
        if document["schema"] != CAPTURE_HELPER_SCHEMA:
            raise RecorderError("capture_helper_artifact_schema_invalid")
        source = document["source"]
        if not isinstance(source, dict) or set(source) != {
            "node_name",
            "node_namespace",
            "topic_type",
            "endpoint_gid",
        }:
            raise RecorderError("capture_helper_source_invalid")
        if any(not isinstance(value, str) for value in source.values()):
            raise RecorderError("capture_helper_source_invalid")
        identity = PublisherIdentity(
            node_name=source["node_name"],
            node_namespace=source["node_namespace"],
            topic_type=source["topic_type"],
            endpoint_gid=source["endpoint_gid"],
        )
        _validate_identity(identity)
        raw_samples = document["samples"]
        if not isinstance(raw_samples, list):
            raise RecorderError("capture_helper_samples_invalid")
        samples: list[JointStateSample] = []
        for raw_sample in raw_samples:
            if not isinstance(raw_sample, dict) or set(raw_sample) != {
                "names",
                "positions",
                "stamp_ns",
                "received_ros_time_ns",
                "received_monotonic_ns",
                "publisher_endpoint_gid",
            }:
                raise RecorderError("capture_helper_sample_schema_invalid")
            names = raw_sample["names"]
            positions = raw_sample["positions"]
            if (
                not isinstance(names, list)
                or any(not isinstance(name, str) for name in names)
                or not isinstance(positions, list)
                or any(
                    isinstance(value, bool) or not isinstance(value, (int, float))
                    for value in positions
                )
                or raw_sample["publisher_endpoint_gid"] != identity.endpoint_gid
            ):
                raise RecorderError("capture_helper_sample_invalid")
            samples.append(
                JointStateSample(
                    names=tuple(names),
                    positions=tuple(float(value) for value in positions),
                    stamp_ns=cls._strict_integer(
                        raw_sample["stamp_ns"], "capture_helper_sample_invalid"
                    ),
                    received_ros_time_ns=cls._strict_integer(
                        raw_sample["received_ros_time_ns"],
                        "capture_helper_sample_invalid",
                    ),
                    received_monotonic_ns=cls._strict_integer(
                        raw_sample["received_monotonic_ns"],
                        "capture_helper_sample_invalid",
                    ),
                )
            )
        return identity, samples

    def capture(
        self,
        *,
        duration_sec: float,
        graph_timeout_sec: float,
    ) -> tuple[PublisherIdentity, list[JointStateSample]]:
        if (
            not math.isfinite(duration_sec)
            or duration_sec < contract.MINIMUM_EVIDENCE_DURATION_SEC
        ):
            raise RecorderError("capture_duration_below_contract_minimum")
        if not math.isfinite(graph_timeout_sec) or graph_timeout_sec <= 0.0:
            raise RecorderError("graph_timeout_invalid")
        helper_path = self._resolved_helper_path()
        with tempfile.TemporaryDirectory(
            prefix="sunhorizon-live-joint-capture-"
        ) as temporary:
            artifact_path = Path(temporary).resolve() / "capture.json"
            command = [
                str(helper_path),
                "--output",
                str(artifact_path),
                "--duration-sec",
                format(duration_sec, ".17g"),
                "--graph-timeout-sec",
                format(graph_timeout_sec, ".17g"),
            ]
            try:
                completed = self._runner(
                    command,
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=(
                        duration_sec
                        + graph_timeout_sec
                        + CAPTURE_OVERRUN_SEC
                        + 5.0
                    ),
                )
            except (OSError, subprocess.SubprocessError) as exc:
                raise RecorderError("capture_helper_execution_failed") from exc
            if completed.returncode != 0:
                detail = completed.stderr.strip().splitlines()
                suffix = detail[-1][:256] if detail else "no_detail"
                raise RecorderError(f"capture_helper_failed:{suffix}")
            return self._parse_capture_artifact(artifact_path)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    record_parser = commands.add_parser("record", help="capture evidence and draft")
    record_parser.add_argument("--output-directory", type=Path, required=True)
    record_parser.add_argument("--release-sha", required=True)
    record_parser.add_argument("--recorded-by", required=True)
    record_parser.add_argument("--robot-serial", default=contract.ROBOT_SERIAL)
    record_parser.add_argument(
        "--duration-sec", type=float, default=DEFAULT_DURATION_SEC
    )
    record_parser.add_argument(
        "--graph-timeout-sec", type=float, default=GRAPH_TIMEOUT_SEC
    )
    review_parser = commands.add_parser(
        "review-statement", help="create the statement an independent key must sign"
    )
    review_parser.add_argument("--draft", type=Path, required=True)
    review_parser.add_argument("--output", type=Path, required=True)
    review_parser.add_argument("--reviewed-by", required=True)
    review_parser.add_argument("--reviewed-at", required=True)
    finalize_parser = commands.add_parser(
        "finalize", help="verify signed independent review and accept the draft"
    )
    finalize_parser.add_argument("--draft", type=Path, required=True)
    finalize_parser.add_argument("--output", type=Path, required=True)
    finalize_parser.add_argument("--review-statement", type=Path, required=True)
    finalize_parser.add_argument("--review-signature", type=Path, required=True)
    finalize_parser.add_argument("--allowed-signers", type=Path, required=True)
    return parser


def main() -> int:
    arguments = _parser().parse_args()
    try:
        if arguments.command == "record":
            result = record(
                output_directory=arguments.output_directory,
                release_sha=arguments.release_sha,
                recorded_by=arguments.recorded_by,
                duration_sec=arguments.duration_sec,
                robot_serial=arguments.robot_serial,
                graph_timeout_sec=arguments.graph_timeout_sec,
                backend=RclcppCaptureBackend(),
            )
        elif arguments.command == "review-statement":
            result = create_review_statement(
                draft_path=arguments.draft,
                output_path=arguments.output,
                reviewed_by=arguments.reviewed_by,
                reviewed_at=arguments.reviewed_at,
            )
        else:
            authority_sha256 = os.environ.get(
                "SUNHORIZON_LIVE_JOINT_REVIEW_AUTHORITY_SHA256", ""
            )
            result = finalize(
                draft_path=arguments.draft,
                output_path=arguments.output,
                review_statement_path=arguments.review_statement,
                review_signature_path=arguments.review_signature,
                allowed_signers_path=arguments.allowed_signers,
                expected_review_authority_sha256=authority_sha256,
            )
    except (
        OSError,
        RecorderError,
        contract.LiveJointStateContractError,
        json.JSONDecodeError,
    ) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, sort_keys=True))
        return 1
    print(json.dumps({"ok": True, **result}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
