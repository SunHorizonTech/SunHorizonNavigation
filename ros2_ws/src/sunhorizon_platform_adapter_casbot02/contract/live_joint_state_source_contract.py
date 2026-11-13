#!/usr/bin/env python3
"""Fail-closed Robot A vendor JointState source receipt contract."""

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


ROBOT_SCOPE = "Robot A"
ROBOT_SERIAL = "1421026005616"
KIND = "robot_a_live_joint_state_source_receipt"
RATE_EVIDENCE_KIND = "robot_a_live_joint_state_rate_evidence"
REVIEW_STATEMENT_KIND = "robot_a_live_joint_state_independent_review"
REVIEW_SIGNATURE_NAMESPACE = "sunhorizon-robot-a-live-joint-state-review-v2"
SSH_KEYGEN = "/usr/bin/ssh-keygen"
SOURCE_TOPIC = "/joint_states"
SOURCE_TYPE = "sensor_msgs/msg/JointState"
SOURCE_OWNER_NAME = "hl_orin_core_msg"
SOURCE_OWNER_NAMESPACE = "/"
OUTPUT_TOPIC = "/sunhorizon/platform/proof/live_joint_states"
POSITION_UNIT = "rad"
VALUE_TRANSFORM = "identity"
RUNTIME_BINDING = "unique_current_endpoint_name_type_and_message_gid"
SELECTED_SOURCE_NAMES = [
    "LJ0", "LJ1", "LJ2", "LJ3", "LJ4", "LJ5",
    "RJ6", "RJ7", "RJ8", "RJ9", "RJ10", "RJ11",
    "waist_yaw",
]
EXPECTED_SOURCE_NAME_COUNT = 49
MINIMUM_ACCEPTED_RATE_HZ = 100.0
MINIMUM_EVIDENCE_DURATION_SEC = 10.0
MAXIMUM_ACCEPTED_GAP_SEC = 0.05
MAXIMUM_ACCEPTED_RECEIVE_GAP_SEC = 0.05
MAXIMUM_ACCEPTED_SOURCE_AGE_SEC = 0.25
MAXIMUM_REVIEW_CLOCK_SKEW_SEC = 300.0
SHA256_RE = re.compile(r"[0-9a-f]{64}")
RELEASE_RE = re.compile(r"[0-9a-f]{40}")
IDENTITY_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._@:/-]{1,127}")
ENDPOINT_GID_RE = re.compile(r"(?:[0-9a-f]{2}){8,64}")
EXPECTED_KEYS = {
    "schema_version", "kind", "robot_scope", "robot_serial", "release_sha",
    "source", "rate_evidence", "review", "accepted_for_platform_freshness",
    "motion_authorized", "canonical_binding", "canonical_binding_sha256",
}
CANONICAL_KEYS = EXPECTED_KEYS - {
    "schema_version", "kind", "canonical_binding", "canonical_binding_sha256",
}
RATE_EVIDENCE_ARTIFACT_KEYS = {
    "schema_version", "kind", "robot_scope", "robot_serial", "release_sha",
    "source", "sample_count", "duration_sec", "observed_rate_hz",
    "maximum_gap_sec", "receive_duration_sec", "maximum_receive_gap_sec",
    "maximum_source_age_sec", "minimum_accepted_rate_hz",
    "maximum_accepted_receive_gap_sec", "maximum_accepted_source_age_sec",
    "first_source_stamp_ns", "last_source_stamp_ns",
    "first_receive_ros_time_ns", "last_receive_ros_time_ns",
    "first_receive_monotonic_ns", "last_receive_monotonic_ns",
    "first_source_age_sec", "last_source_age_sec",
}
RATE_RECEIPT_METRIC_KEYS = {
    "sample_count", "duration_sec", "observed_rate_hz", "maximum_gap_sec",
    "receive_duration_sec", "maximum_receive_gap_sec", "maximum_source_age_sec",
    "minimum_accepted_rate_hz", "maximum_accepted_receive_gap_sec",
    "maximum_accepted_source_age_sec",
}
SOURCE_KEYS = {
    "topic", "type", "publisher_node_name", "publisher_node_namespace",
    "recorded_publisher_endpoint_gid", "runtime_binding",
    "ordered_joint_names", "position_semantics",
}
POSITION_SEMANTICS_KEYS = {
    "unit", "value_transform", "selected_joint_names",
    "selected_joint_indices", "output_topic",
}
REVIEW_KEYS = {
    "recorded_by", "reviewed_by", "reviewed_at", "accepted",
    "statement", "signature", "allowed_signers",
}
REVIEW_STATEMENT_KEYS = {
    "schema_version", "kind", "robot_scope", "robot_serial", "release_sha",
    "reviewed_by", "reviewed_at", "decision", "draft_receipt",
    "rate_evidence",
}


class LiveJointStateContractError(ValueError):
    """The accepted runtime source receipt or a bound asset is invalid."""


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def _stable_bytes(path: Path, label: str) -> bytes:
    if not path.is_absolute():
        raise LiveJointStateContractError(f"{label}_path_not_absolute")
    try:
        canonical = path.resolve(strict=True)
        before = path.lstat()
    except OSError as exc:
        raise LiveJointStateContractError(f"{label}_unavailable") from exc
    if canonical != path or path.is_symlink() or not path.is_file() or before.st_nlink != 1:
        raise LiveJointStateContractError(f"{label}_not_single_canonical_regular_file")
    try:
        payload = path.read_bytes()
        after = path.lstat()
    except OSError as exc:
        raise LiveJointStateContractError(f"{label}_unreadable") from exc
    identity = lambda stat: (
        stat.st_dev, stat.st_ino, stat.st_mode, stat.st_nlink,
        stat.st_size, stat.st_mtime_ns,
    )
    if identity(before) != identity(after):
        raise LiveJointStateContractError(f"{label}_changed_while_reading")
    if not payload:
        raise LiveJointStateContractError(f"{label}_empty")
    return payload


def _asset(value: Any, label: str) -> bytes:
    if not isinstance(value, dict) or set(value) != {"path", "sha256"}:
        raise LiveJointStateContractError(f"{label}_descriptor_invalid")
    path_value = value.get("path")
    digest = value.get("sha256")
    if not isinstance(path_value, str) or not SHA256_RE.fullmatch(str(digest)):
        raise LiveJointStateContractError(f"{label}_descriptor_invalid")
    payload = _stable_bytes(Path(path_value), label)
    if hashlib.sha256(payload).hexdigest() != digest:
        raise LiveJointStateContractError(f"{label}_sha256_mismatch")
    return payload


def _json_object(payload: bytes, label: str) -> dict[str, Any]:
    def reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        output: dict[str, Any] = {}
        for key, value in pairs:
            if key in output:
                raise LiveJointStateContractError(f"{label}_duplicate_key")
            output[key] = value
        return output

    try:
        value = json.loads(payload, object_pairs_hook=reject_duplicate_keys)
    except LiveJointStateContractError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise LiveJointStateContractError(f"{label}_invalid_json") from exc
    if not isinstance(value, dict):
        raise LiveJointStateContractError(f"{label}_not_object")
    return value


def verify_review_signature(
    *,
    statement_payload: bytes,
    signature_payload: bytes,
    allowed_signers_payload: bytes,
    reviewer_identity: str,
) -> None:
    """Verify the exact review bytes already accepted by the caller.

    ``ssh-keygen`` requires paths for the detached signature and allowed-signers
    inputs.  Never give it the receipt paths: those could be replaced after the
    stable read/hash check.  Private temporary copies make the verifier consume
    the same bytes that were checked, while the statement is supplied directly
    on stdin.
    """

    try:
        with tempfile.TemporaryDirectory(
            prefix="sunhorizon-live-joint-review-"
        ) as temporary:
            private_directory = Path(temporary)
            os.chmod(private_directory, 0o700)
            signature_path = private_directory / "review.sig"
            allowed_signers_path = private_directory / "allowed_signers"
            signature_path.write_bytes(signature_payload)
            allowed_signers_path.write_bytes(allowed_signers_payload)
            os.chmod(signature_path, 0o600)
            os.chmod(allowed_signers_path, 0o600)
            result = subprocess.run(
                [
                    SSH_KEYGEN,
                    "-Y", "verify",
                    "-f", str(allowed_signers_path),
                    "-I", reviewer_identity,
                    "-n", REVIEW_SIGNATURE_NAMESPACE,
                    "-s", str(signature_path),
                ],
                input=statement_payload,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
                timeout=5.0,
            )
    except (OSError, TypeError, subprocess.SubprocessError) as exc:
        raise LiveJointStateContractError("live_joint_state_review_signature_unavailable") from exc
    if result.returncode != 0:
        raise LiveJointStateContractError("live_joint_state_review_signature_invalid")


def _finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _validate_rate_metrics(rate: dict[str, Any]) -> None:
    sample_count = rate.get("sample_count")
    source_duration = rate.get("duration_sec")
    receive_duration = rate.get("receive_duration_sec")
    observed_rate = rate.get("observed_rate_hz")
    source_gap = rate.get("maximum_gap_sec")
    receive_gap = rate.get("maximum_receive_gap_sec")
    source_age = rate.get("maximum_source_age_sec")
    if (
        not isinstance(sample_count, int) or isinstance(sample_count, bool)
        or not all(_finite_number(value) for value in (
            source_duration, receive_duration, observed_rate, source_gap,
            receive_gap, source_age,
        ))
        or rate.get("minimum_accepted_rate_hz") != MINIMUM_ACCEPTED_RATE_HZ
        or rate.get("maximum_accepted_receive_gap_sec")
        != MAXIMUM_ACCEPTED_RECEIVE_GAP_SEC
        or rate.get("maximum_accepted_source_age_sec")
        != MAXIMUM_ACCEPTED_SOURCE_AGE_SEC
        or source_duration < MINIMUM_EVIDENCE_DURATION_SEC
        or receive_duration < MINIMUM_EVIDENCE_DURATION_SEC
        or sample_count < int(MINIMUM_ACCEPTED_RATE_HZ * receive_duration)
        or observed_rate < MINIMUM_ACCEPTED_RATE_HZ
        or source_gap <= 0.0 or source_gap > MAXIMUM_ACCEPTED_GAP_SEC
        or receive_gap <= 0.0 or receive_gap > MAXIMUM_ACCEPTED_RECEIVE_GAP_SEC
        or source_age < 0.0 or source_age > MAXIMUM_ACCEPTED_SOURCE_AGE_SEC
        or abs((sample_count - 1) / receive_duration - observed_rate)
        > observed_rate * 0.05
    ):
        raise LiveJointStateContractError("live_joint_state_rate_evidence_invalid")


def _validate_ed25519_allowed_signers(payload: bytes, reviewer_identity: str) -> None:
    try:
        lines = payload.decode("utf-8").splitlines()
    except UnicodeDecodeError as exc:
        raise LiveJointStateContractError(
            "live_joint_state_review_authority_invalid"
        ) from exc
    entries = [
        line.split()
        for line in lines
        if line.strip() and not line.lstrip().startswith("#")
    ]
    if (
        not entries
        or any(len(entry) != 3 or entry[1] != "ssh-ed25519" for entry in entries)
        or not any(
            reviewer_identity in entry[0].split(",") for entry in entries
        )
    ):
        raise LiveJointStateContractError("live_joint_state_review_authority_invalid")


def _validate_rate_artifact_timestamps(
    rate_artifact: dict[str, Any], rate: dict[str, Any]
) -> None:
    integer_stamp_keys = (
        "first_source_stamp_ns", "last_source_stamp_ns",
        "first_receive_ros_time_ns", "last_receive_ros_time_ns",
        "first_receive_monotonic_ns", "last_receive_monotonic_ns",
    )
    if any(
        not isinstance(rate_artifact.get(key), int)
        or isinstance(rate_artifact.get(key), bool)
        or rate_artifact[key] <= 0
        for key in integer_stamp_keys
    ):
        raise LiveJointStateContractError(
            "live_joint_state_rate_evidence_timestamps_invalid")
    first_source = rate_artifact["first_source_stamp_ns"]
    last_source = rate_artifact["last_source_stamp_ns"]
    first_receive_ros = rate_artifact["first_receive_ros_time_ns"]
    last_receive_ros = rate_artifact["last_receive_ros_time_ns"]
    first_receive_mono = rate_artifact["first_receive_monotonic_ns"]
    last_receive_mono = rate_artifact["last_receive_monotonic_ns"]
    first_age = rate_artifact.get("first_source_age_sec")
    last_age = rate_artifact.get("last_source_age_sec")
    if (
        not _finite_number(first_age) or not _finite_number(last_age)
        or first_source >= last_source
        or first_receive_mono >= last_receive_mono
        or first_receive_ros < first_source or last_receive_ros < last_source
        or first_age < 0.0 or last_age < 0.0
        or first_age > MAXIMUM_ACCEPTED_SOURCE_AGE_SEC
        or last_age > MAXIMUM_ACCEPTED_SOURCE_AGE_SEC
        or abs((last_source - first_source) / 1e9 - rate["duration_sec"])
        > 1e-9
        or abs(
            (last_receive_mono - first_receive_mono) / 1e9
            - rate["receive_duration_sec"]
        ) > 1e-9
        or abs((first_receive_ros - first_source) / 1e9 - first_age) > 1e-9
        or abs((last_receive_ros - last_source) / 1e9 - last_age) > 1e-9
        or rate["maximum_source_age_sec"] < max(first_age, last_age)
    ):
        raise LiveJointStateContractError(
            "live_joint_state_rate_evidence_timestamps_invalid")


def _validate_review_time(
    reviewed_at: str,
    *,
    last_receive_ros_time_ns: int,
    now_utc: datetime.datetime | None = None,
) -> None:
    """Require a causally ordered review with a tightly bounded future skew."""

    try:
        reviewed = datetime.datetime.strptime(
            reviewed_at, "%Y-%m-%dT%H:%M:%SZ"
        ).replace(tzinfo=datetime.timezone.utc)
    except (TypeError, ValueError) as exc:
        raise LiveJointStateContractError(
            "live_joint_state_review_invalid"
        ) from exc
    if (
        not isinstance(last_receive_ros_time_ns, int)
        or isinstance(last_receive_ros_time_ns, bool)
        or last_receive_ros_time_ns <= 0
    ):
        raise LiveJointStateContractError(
            "live_joint_state_review_evidence_time_invalid"
        )
    reviewed_ns = int(reviewed.timestamp()) * 1_000_000_000
    if reviewed_ns < last_receive_ros_time_ns:
        raise LiveJointStateContractError(
            "live_joint_state_review_before_evidence"
        )
    current = now_utc or datetime.datetime.now(datetime.timezone.utc)
    if current.tzinfo is None or current.utcoffset() is None:
        raise LiveJointStateContractError("live_joint_state_review_clock_invalid")
    current = current.astimezone(datetime.timezone.utc)
    if reviewed > current + datetime.timedelta(
        seconds=MAXIMUM_REVIEW_CLOCK_SKEW_SEC
    ):
        raise LiveJointStateContractError("live_joint_state_review_in_future")


def validate_live_joint_state_receipt(
    receipt_path: Path,
    *,
    expected_robot_serial: str,
    expected_release_sha: str,
    expected_review_authority_sha256: str,
    signature_verifier: Callable[..., None] = verify_review_signature,
    now_utc: datetime.datetime | None = None,
) -> dict[str, Any]:
    if expected_robot_serial != ROBOT_SERIAL:
        raise LiveJointStateContractError("robot_a_serial_mismatch")
    if not RELEASE_RE.fullmatch(expected_release_sha):
        raise LiveJointStateContractError("release_sha_invalid")
    if not SHA256_RE.fullmatch(expected_review_authority_sha256):
        raise LiveJointStateContractError("review_authority_sha256_invalid")
    payload = _stable_bytes(receipt_path, "live_joint_state_receipt")
    try:
        receipt = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise LiveJointStateContractError("live_joint_state_receipt_invalid_json") from exc
    if not isinstance(receipt, dict) or set(receipt) != EXPECTED_KEYS:
        raise LiveJointStateContractError("live_joint_state_receipt_shape_invalid")
    if (
        receipt.get("schema_version") != 2
        or receipt.get("kind") != KIND
        or receipt.get("robot_scope") != ROBOT_SCOPE
        or receipt.get("robot_serial") != expected_robot_serial
        or receipt.get("release_sha") != expected_release_sha
        or receipt.get("accepted_for_platform_freshness") is not True
        or receipt.get("motion_authorized") is not False
    ):
        raise LiveJointStateContractError("live_joint_state_identity_or_acceptance_invalid")
    canonical = receipt.get("canonical_binding")
    if canonical != {key: receipt[key] for key in CANONICAL_KEYS}:
        raise LiveJointStateContractError("live_joint_state_canonical_binding_mismatch")
    if (
        not SHA256_RE.fullmatch(str(receipt.get("canonical_binding_sha256")))
        or canonical_sha256(canonical) != receipt["canonical_binding_sha256"]
    ):
        raise LiveJointStateContractError("live_joint_state_canonical_hash_mismatch")

    source = receipt.get("source")
    if not isinstance(source, dict) or set(source) != SOURCE_KEYS:
        raise LiveJointStateContractError("live_joint_state_source_shape_invalid")
    names = source.get("ordered_joint_names")
    semantics = source.get("position_semantics")
    if (
        source.get("topic") != SOURCE_TOPIC
        or source.get("type") != SOURCE_TYPE
        or source.get("publisher_node_name") != SOURCE_OWNER_NAME
        or source.get("publisher_node_namespace") != SOURCE_OWNER_NAMESPACE
        or not isinstance(source.get("recorded_publisher_endpoint_gid"), str)
        or ENDPOINT_GID_RE.fullmatch(
            source["recorded_publisher_endpoint_gid"]
        ) is None
        or source.get("runtime_binding") != RUNTIME_BINDING
        or not isinstance(names, list)
        or len(names) != EXPECTED_SOURCE_NAME_COUNT
        or len(set(names)) != EXPECTED_SOURCE_NAME_COUNT
        or any(not isinstance(name, str) or not IDENTITY_RE.fullmatch(name) for name in names)
        or any(name not in names for name in SELECTED_SOURCE_NAMES)
        or not isinstance(semantics, dict)
        or set(semantics) != POSITION_SEMANTICS_KEYS
        or semantics.get("unit") != POSITION_UNIT
        or semantics.get("value_transform") != VALUE_TRANSFORM
        or semantics.get("selected_joint_names") != SELECTED_SOURCE_NAMES
        or semantics.get("selected_joint_indices")
        != [names.index(name) for name in SELECTED_SOURCE_NAMES]
        or semantics.get("output_topic") != OUTPUT_TOPIC
    ):
        raise LiveJointStateContractError("live_joint_state_source_contract_invalid")

    rate = receipt.get("rate_evidence")
    if not isinstance(rate, dict) or set(rate) != {
        "artifact", *RATE_RECEIPT_METRIC_KEYS,
    }:
        raise LiveJointStateContractError("live_joint_state_rate_evidence_shape_invalid")
    rate_artifact_payload = _asset(
        rate["artifact"], "live_joint_state_rate_evidence")
    _validate_rate_metrics(rate)

    rate_artifact = _json_object(
        rate_artifact_payload, "live_joint_state_rate_evidence")
    if set(rate_artifact) != RATE_EVIDENCE_ARTIFACT_KEYS:
        raise LiveJointStateContractError(
            "live_joint_state_rate_evidence_artifact_shape_invalid")
    if (
        not isinstance(rate_artifact.get("schema_version"), int)
        or isinstance(rate_artifact.get("schema_version"), bool)
        or rate_artifact.get("schema_version") != 2
        or rate_artifact.get("kind") != RATE_EVIDENCE_KIND
        or rate_artifact.get("robot_scope") != ROBOT_SCOPE
        or rate_artifact.get("robot_serial") != expected_robot_serial
        or rate_artifact.get("release_sha") != expected_release_sha
        or rate_artifact.get("source") != source
    ):
        raise LiveJointStateContractError(
            "live_joint_state_rate_evidence_artifact_identity_invalid")
    artifact_metrics = {
        key: rate_artifact[key] for key in RATE_RECEIPT_METRIC_KEYS
    }
    receipt_metrics = {key: rate[key] for key in artifact_metrics}
    if canonical_bytes(artifact_metrics) != canonical_bytes(receipt_metrics):
        raise LiveJointStateContractError(
            "live_joint_state_rate_evidence_artifact_mismatch")
    _validate_rate_artifact_timestamps(rate_artifact, rate)

    review = receipt.get("review")
    if not isinstance(review, dict) or set(review) != REVIEW_KEYS:
        raise LiveJointStateContractError("live_joint_state_review_invalid")
    if (
        not isinstance(review.get("recorded_by"), str)
        or not isinstance(review.get("reviewed_by"), str)
        or not IDENTITY_RE.fullmatch(review["recorded_by"])
        or not IDENTITY_RE.fullmatch(review["reviewed_by"])
        or review["recorded_by"].casefold() == review["reviewed_by"].casefold()
        or review.get("accepted") is not True
        or not isinstance(review.get("reviewed_at"), str)
        or re.fullmatch(
            r"20[0-9]{2}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z",
            review["reviewed_at"],
        ) is None
    ):
        raise LiveJointStateContractError("live_joint_state_review_invalid")
    _validate_review_time(
        review["reviewed_at"],
        last_receive_ros_time_ns=rate_artifact["last_receive_ros_time_ns"],
        now_utc=now_utc,
    )

    statement_payload = _asset(review.get("statement"), "live_joint_state_review")
    signature_payload = _asset(
        review.get("signature"), "live_joint_state_review_signature")
    allowed_signers_payload = _asset(
        review.get("allowed_signers"), "live_joint_state_review_allowed_signers")
    if review["allowed_signers"]["sha256"] != expected_review_authority_sha256:
        raise LiveJointStateContractError("live_joint_state_review_authority_mismatch")
    _validate_ed25519_allowed_signers(
        allowed_signers_payload, review["reviewed_by"]
    )
    statement = _json_object(statement_payload, "live_joint_state_review")
    if set(statement) != REVIEW_STATEMENT_KEYS:
        raise LiveJointStateContractError("live_joint_state_review_statement_shape_invalid")
    if (
        statement.get("schema_version") != 2
        or statement.get("kind") != REVIEW_STATEMENT_KIND
        or statement.get("robot_scope") != ROBOT_SCOPE
        or statement.get("robot_serial") != expected_robot_serial
        or statement.get("release_sha") != expected_release_sha
        or statement.get("reviewed_by") != review["reviewed_by"]
        or statement.get("reviewed_at") != review["reviewed_at"]
        or statement.get("decision") != {
            "accepted_for_platform_freshness": True,
            "motion_authorized": False,
        }
        or statement.get("rate_evidence") != rate["artifact"]
    ):
        raise LiveJointStateContractError("live_joint_state_review_statement_invalid")
    draft_descriptor = statement.get("draft_receipt")
    if (
        not isinstance(draft_descriptor, dict)
        or set(draft_descriptor) != {"path", "sha256"}
        or not isinstance(draft_descriptor.get("path"), str)
        or not SHA256_RE.fullmatch(str(draft_descriptor.get("sha256")))
    ):
        raise LiveJointStateContractError("live_joint_state_review_draft_binding_invalid")
    if Path(draft_descriptor["path"]).name != (
        "live-joint-state-source.receipt.draft.json"
    ):
        raise LiveJointStateContractError("live_joint_state_review_draft_binding_invalid")
    draft_payload = _asset(draft_descriptor, "live_joint_state_review_draft")
    draft = _json_object(draft_payload, "live_joint_state_review_draft")
    draft_review = draft.get("review")
    if (
        set(draft) != EXPECTED_KEYS
        or draft.get("schema_version") != 2
        or draft.get("kind") != KIND
        or draft.get("robot_scope") != ROBOT_SCOPE
        or draft.get("robot_serial") != expected_robot_serial
        or draft.get("release_sha") != expected_release_sha
        or draft.get("source") != source
        or draft.get("rate_evidence") != rate
        or draft.get("accepted_for_platform_freshness") is not False
        or draft.get("motion_authorized") is not False
        or not isinstance(draft_review, dict)
        or set(draft_review) != REVIEW_KEYS
        or draft_review.get("recorded_by") != review["recorded_by"]
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
        raise LiveJointStateContractError("live_joint_state_review_draft_invalid")
    signature_verifier(
        statement_payload=statement_payload,
        signature_payload=signature_payload,
        allowed_signers_payload=allowed_signers_payload,
        reviewer_identity=review["reviewed_by"],
    )

    return {
        "source_topic": SOURCE_TOPIC,
        "source_type": SOURCE_TYPE,
        "source_owner_name": SOURCE_OWNER_NAME,
        "source_owner_namespace": SOURCE_OWNER_NAMESPACE,
        "runtime_binding": RUNTIME_BINDING,
        "expected_source_joint_names": names,
        "selected_source_joint_names": SELECTED_SOURCE_NAMES,
        "selected_source_joint_indices": semantics["selected_joint_indices"],
        "position_unit": POSITION_UNIT,
        "value_transform": VALUE_TRANSFORM,
        "output_topic": OUTPUT_TOPIC,
        "receipt_path": str(receipt_path),
        "receipt_sha256": hashlib.sha256(payload).hexdigest(),
        "evidence_sha256": rate["artifact"]["sha256"],
    }
