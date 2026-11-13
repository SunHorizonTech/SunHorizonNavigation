#!/usr/bin/env python3
"""Record and independently finalize Robot A leg-joint mapping evidence."""

from __future__ import annotations

import argparse
import copy
import ctypes
import datetime
import errno
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import stat
import sys
import tempfile
from typing import Any, Callable


CONTRACT_PATH = Path(__file__).with_name("leg_joint_mapping_contract.py")
SPEC = importlib.util.spec_from_file_location(
    "sunhorizon_leg_joint_mapping_contract", CONTRACT_PATH
)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("leg_joint_mapping_contract_unavailable")
contract = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(contract)

EVIDENCE_FILENAME = "leg-joint-mapping.evidence.json"
DRAFT_FILENAME = "leg-joint-mapping.receipt.draft.json"
SEALED_BLOCK_FILENAME = "latest-complete-roh-status-block.txt"
DEFAULT_MAXIMUM_LOG_TAIL_BYTES = 4 * 1024 * 1024
UTC_SECONDS_RE = re.compile(
    r"20[0-9]{2}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z"
)


class RecorderError(ValueError):
    """The requested evidence operation failed closed."""


def _json_bytes(value: Any) -> bytes:
    return contract.canonical_bytes(value) + b"\n"


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    return _sha256_bytes(contract._stable_bytes(path, "output"))


def _identity(metadata: os.stat_result) -> tuple[int, ...]:
    return (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_mode,
        metadata.st_nlink,
        metadata.st_size,
        metadata.st_mtime_ns,
    )


def _canonical_regular_file(path: Path, label: str) -> os.stat_result:
    if not path.is_absolute():
        raise RecorderError(f"{label}_path_not_absolute")
    try:
        resolved = path.resolve(strict=True)
        metadata = path.lstat()
    except OSError as exc:
        raise RecorderError(f"{label}_unavailable") from exc
    if (
        resolved != path
        or path.is_symlink()
        or not stat.S_ISREG(metadata.st_mode)
        or metadata.st_nlink != 1
        or metadata.st_size <= 0
        or metadata.st_mtime_ns <= 0
    ):
        raise RecorderError(f"{label}_not_single_canonical_regular_file")
    return metadata


def _read_stable_log_tail(
    path: Path, maximum_bytes: int
) -> tuple[bytes, dict[str, int | str], os.stat_result]:
    if not isinstance(maximum_bytes, int) or isinstance(maximum_bytes, bool):
        raise RecorderError("maximum_log_tail_bytes_invalid")
    if maximum_bytes < 65536 or maximum_bytes > 64 * 1024 * 1024:
        raise RecorderError("maximum_log_tail_bytes_invalid")
    before = _canonical_regular_file(path, "vendor_joint_log")
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise RecorderError("vendor_joint_log_unreadable") from exc
    try:
        opened = os.fstat(descriptor)
        if _identity(opened) != _identity(before):
            raise RecorderError("vendor_joint_log_changed_while_opening")
        tail_size = min(opened.st_size, maximum_bytes)
        tail_offset = opened.st_size - tail_size
        os.lseek(descriptor, tail_offset, os.SEEK_SET)
        chunks: list[bytes] = []
        remaining = tail_size
        while remaining:
            chunk = os.read(descriptor, min(remaining, 1024 * 1024))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        after = os.fstat(descriptor)
    except OSError as exc:
        raise RecorderError("vendor_joint_log_unreadable") from exc
    finally:
        os.close(descriptor)
    try:
        path_after = path.lstat()
    except OSError as exc:
        raise RecorderError("vendor_joint_log_changed_while_reading") from exc
    payload = b"".join(chunks)
    if (
        len(payload) != tail_size
        or _identity(before) != _identity(after)
        or _identity(before) != _identity(path_after)
    ):
        raise RecorderError("vendor_joint_log_changed_while_reading")
    return (
        payload,
        {
            "path": str(path),
            "size_bytes": before.st_size,
            "mtime_ns": before.st_mtime_ns,
            "tail_offset_bytes": tail_offset,
            "tail_size_bytes": tail_size,
        },
        before,
    )


def _canonical_existing_directory(path: Path, label: str) -> Path:
    if not path.is_absolute():
        raise RecorderError(f"{label}_not_absolute")
    try:
        resolved = path.resolve(strict=True)
        metadata = path.lstat()
    except OSError as exc:
        raise RecorderError(f"{label}_unavailable") from exc
    if path.is_symlink() or not path.is_dir() or resolved != path or metadata.st_nlink < 1:
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


def _rename_no_replace(source: Path, target: Path) -> None:
    """Atomically publish a file or directory without replacing an inode."""

    libc = ctypes.CDLL(None, use_errno=True)
    if sys.platform.startswith("linux") and hasattr(libc, "renameat2"):
        renameat2 = libc.renameat2
        renameat2.argtypes = [
            ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p,
            ctypes.c_uint,
        ]
        renameat2.restype = ctypes.c_int
        result = renameat2(-100, os.fsencode(source), -100, os.fsencode(target), 1)
        if result == 0:
            return
        code = ctypes.get_errno()
        if code in (errno.EEXIST, errno.ENOTEMPTY):
            raise RecorderError("output_already_exists")
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
            raise RecorderError("output_already_exists")
        if code not in (errno.ENOTSUP, errno.EINVAL):
            raise OSError(code, os.strerror(code), str(target))
    raise RecorderError("atomic_no_replace_rename_unavailable")


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _atomic_write_recording(
    output_directory: Path,
    *,
    sealed_block: bytes,
    evidence: dict[str, Any],
    draft: dict[str, Any],
) -> None:
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
        _write_file_exclusive(temporary / SEALED_BLOCK_FILENAME, sealed_block)
        _write_file_exclusive(temporary / EVIDENCE_FILENAME, _json_bytes(evidence))
        _write_file_exclusive(temporary / DRAFT_FILENAME, _json_bytes(draft))
        _fsync_directory(temporary)
        _rename_no_replace(temporary, output_directory)
        temporary = Path()
        _fsync_directory(parent)
    finally:
        if temporary != Path() and temporary.exists():
            shutil.rmtree(temporary)


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
        staged = staging / path.name
        _write_file_exclusive(staged, _json_bytes(document))
        if validate_staged is not None:
            validate_staged(staged)
        _rename_no_replace(staged, path)
        _fsync_directory(parent)
    finally:
        shutil.rmtree(staging)


def _descriptor(
    path: Path, label: str
) -> tuple[dict[str, str], bytes]:
    payload = contract._stable_bytes(path, label)
    return {"path": str(path), "sha256": _sha256_bytes(payload)}, payload


def _validate_identity(robot_serial: str, release_sha: str, identity: str) -> str:
    if robot_serial != contract.ROBOT_SERIAL:
        raise RecorderError("robot_a_serial_mismatch")
    if contract.RELEASE_RE.fullmatch(release_sha) is None:
        raise RecorderError("release_sha_invalid")
    value = identity.strip()
    if contract.IDENTITY_RE.fullmatch(value) is None:
        raise RecorderError("recorded_by_invalid")
    return value


def _source_contract() -> dict[str, Any]:
    return {
        "position_source": {
            "topic": "/joint_states",
            "type": "sensor_msgs/msg/JointState",
            "selected_joint_names": contract.CANONICAL_SOURCE_NAMES,
            "position_unit": "rad",
            "value_transform": "identity",
            "source_stamp": "ros_message_header",
        },
        "fault_source": {
            "format": "roh_ethercat_station_log",
            "required_raw_joint_ids": contract.EXPECTED_IDS,
            "used_fields": ["statusword", "error_code"],
            "ignored_fields": ["position"],
            "source_stamp": "filesystem_mtime_ns",
        },
    }


def record(
    *,
    vendor_joint_log: Path,
    vendor_urdf: Path,
    output_directory: Path,
    release_sha: str,
    recorded_by: str,
    robot_serial: str = contract.ROBOT_SERIAL,
    maximum_log_tail_bytes: int = DEFAULT_MAXIMUM_LOG_TAIL_BYTES,
) -> dict[str, Any]:
    """Atomically publish a sealed real-log/URDF mapping draft."""

    recorded_by = _validate_identity(robot_serial, release_sha, recorded_by)
    if not output_directory.is_absolute():
        raise RecorderError("output_directory_not_absolute")
    if output_directory.exists() or output_directory.is_symlink():
        raise RecorderError("output_directory_already_exists")
    urdf_payload = contract._stable_bytes(vendor_urdf, "vendor_urdf")
    mappings = contract.derive_robot_a_leg_mappings(urdf_payload)
    tail, source_log, source_stat = _read_stable_log_tail(
        vendor_joint_log, maximum_log_tail_bytes
    )
    selected = contract.find_latest_complete_status_block(tail)
    sealed_block = selected["payload"]
    source_log.update({
        "status_block_offset_bytes": (
            source_log["tail_offset_bytes"] + selected["offset"]
        ),
        "status_block_size_bytes": len(sealed_block),
    })
    final_sealed_path = output_directory / SEALED_BLOCK_FILENAME
    final_evidence_path = output_directory / EVIDENCE_FILENAME
    urdf_descriptor = {
        "path": str(vendor_urdf),
        "sha256": _sha256_bytes(urdf_payload),
        "base_link": contract.BASE_LINK,
        "left_foot_link": contract.LEFT_FOOT_LINK,
        "right_foot_link": contract.RIGHT_FOOT_LINK,
    }
    evidence = {
        "schema_version": 2,
        "kind": contract.EVIDENCE_KIND,
        "robot_scope": contract.ROBOT_SCOPE,
        "robot_serial": robot_serial,
        "release_sha": release_sha,
        "source_log": source_log,
        "sealed_status_block": {
            "path": str(final_sealed_path),
            "sha256": _sha256_bytes(sealed_block),
        },
        "status_timestamp": selected["status_timestamp"],
        "vendor_urdf": urdf_descriptor,
        "mappings": mappings,
        "observed_fault_status": selected["observed_fault_status"],
    }
    evidence_payload = _json_bytes(evidence)
    draft: dict[str, Any] = {
        "schema_version": 2,
        "kind": contract.KIND,
        "robot_scope": contract.ROBOT_SCOPE,
        "robot_serial": robot_serial,
        "release_sha": release_sha,
        "source_contract": _source_contract(),
        "vendor_urdf": urdf_descriptor,
        "mapping_evidence": {
            "path": str(final_evidence_path),
            "sha256": _sha256_bytes(evidence_payload),
        },
        "mappings": mappings,
        "review": {
            "recorded_by": recorded_by,
            "reviewed_by": None,
            "reviewed_at": None,
            "accepted": False,
            "statement": None,
            "signature": None,
            "allowed_signers": None,
        },
        "accepted_for_position_proof": False,
        "motion_authorized": False,
    }
    draft["canonical_binding"] = {
        key: copy.deepcopy(draft[key]) for key in contract.CANONICAL_KEYS
    }
    draft["canonical_binding_sha256"] = contract.canonical_sha256(
        draft["canonical_binding"]
    )
    if set(evidence) != contract.EVIDENCE_KEYS:
        raise RecorderError("mapping_evidence_internal_schema_mismatch")
    if set(draft) != contract.EXPECTED_TOP_LEVEL_KEYS:
        raise RecorderError("mapping_draft_internal_schema_mismatch")
    try:
        current = vendor_joint_log.lstat()
    except OSError as exc:
        raise RecorderError("vendor_joint_log_changed_before_publish") from exc
    if _identity(current) != _identity(source_stat):
        raise RecorderError("vendor_joint_log_changed_before_publish")
    _atomic_write_recording(
        output_directory, sealed_block=sealed_block, evidence=evidence, draft=draft
    )
    return {
        "accepted_for_position_proof": False,
        "motion_authorized": False,
        "output_directory": str(output_directory),
        "sealed_status_block": str(output_directory / SEALED_BLOCK_FILENAME),
        "mapping_evidence": str(output_directory / EVIDENCE_FILENAME),
        "mapping_evidence_sha256": _sha256_file(output_directory / EVIDENCE_FILENAME),
        "draft_receipt": str(output_directory / DRAFT_FILENAME),
        "draft_receipt_sha256": _sha256_file(output_directory / DRAFT_FILENAME),
        "status_timestamp": selected["status_timestamp"],
        "leg_joint_names": [item["joint_name"] for item in mappings],
    }


def _load_draft(path: Path) -> tuple[dict[str, Any], bytes]:
    payload = contract._stable_bytes(path, "mapping_draft")
    draft = contract._json_object(payload, "mapping_draft")
    if set(draft) != contract.EXPECTED_TOP_LEVEL_KEYS:
        raise RecorderError("mapping_draft_shape_invalid")
    if (
        draft.get("schema_version") != 2
        or draft.get("kind") != contract.KIND
        or draft.get("robot_scope") != contract.ROBOT_SCOPE
        or draft.get("robot_serial") != contract.ROBOT_SERIAL
        or contract.RELEASE_RE.fullmatch(str(draft.get("release_sha"))) is None
        or draft.get("source_contract") != _source_contract()
        or draft.get("accepted_for_position_proof") is not False
        or draft.get("motion_authorized") is not False
    ):
        raise RecorderError("mapping_draft_state_invalid")
    review = draft.get("review")
    if not isinstance(review, dict) or set(review) != contract.REVIEW_KEYS:
        raise RecorderError("mapping_draft_review_shape_invalid")
    if (
        not isinstance(review.get("recorded_by"), str)
        or contract.IDENTITY_RE.fullmatch(review["recorded_by"]) is None
        or review.get("reviewed_by") is not None
        or review.get("reviewed_at") is not None
        or review.get("accepted") is not False
        or review.get("statement") is not None
        or review.get("signature") is not None
        or review.get("allowed_signers") is not None
    ):
        raise RecorderError("mapping_draft_review_state_invalid")
    canonical = draft.get("canonical_binding")
    if canonical != {key: draft[key] for key in contract.CANONICAL_KEYS}:
        raise RecorderError("mapping_draft_canonical_binding_mismatch")
    if contract.canonical_sha256(canonical) != draft.get("canonical_binding_sha256"):
        raise RecorderError("mapping_draft_canonical_hash_mismatch")
    urdf = draft.get("vendor_urdf")
    if not isinstance(urdf, dict) or set(urdf) != {
        "path", "sha256", "base_link", "left_foot_link", "right_foot_link"
    }:
        raise RecorderError("mapping_draft_urdf_invalid")
    try:
        _, urdf_payload = contract._asset(
            {key: urdf[key] for key in ("path", "sha256")}, "vendor_urdf"
        )
        expected_mappings = contract.derive_robot_a_leg_mappings(urdf_payload)
        if draft.get("mappings") != expected_mappings:
            raise RecorderError("mapping_draft_mappings_invalid")
        contract._validate_mapping_evidence(
            draft.get("mapping_evidence"),
            expected_robot_serial=draft["robot_serial"],
            expected_release_sha=draft["release_sha"],
            expected_vendor_urdf=urdf,
            expected_mappings=expected_mappings,
        )
    except contract.MappingContractError as exc:
        raise RecorderError("mapping_draft_bound_evidence_invalid") from exc
    return draft, payload


def create_review_statement(
    *, draft_path: Path, output_path: Path, reviewed_by: str, reviewed_at: str
) -> dict[str, Any]:
    draft, draft_payload = _load_draft(draft_path)
    reviewer = reviewed_by.strip()
    if (
        contract.IDENTITY_RE.fullmatch(reviewer) is None
        or reviewer.casefold() == draft["review"]["recorded_by"].casefold()
    ):
        raise RecorderError("reviewed_by_invalid_or_not_independent")
    if UTC_SECONDS_RE.fullmatch(reviewed_at) is None:
        raise RecorderError("reviewed_at_invalid")
    try:
        datetime.datetime.strptime(reviewed_at, "%Y-%m-%dT%H:%M:%SZ")
    except ValueError as exc:
        raise RecorderError("reviewed_at_invalid") from exc
    statement = {
        "schema_version": 2,
        "kind": contract.REVIEW_STATEMENT_KIND,
        "robot_scope": contract.ROBOT_SCOPE,
        "robot_serial": draft["robot_serial"],
        "release_sha": draft["release_sha"],
        "reviewed_by": reviewer,
        "reviewed_at": reviewed_at,
        "decision": {
            "accepted_for_position_proof": True,
            "motion_authorized": False,
        },
        "draft_receipt": {
            "path": str(draft_path),
            "sha256": _sha256_bytes(draft_payload),
        },
        "mapping_evidence": copy.deepcopy(draft["mapping_evidence"]),
    }
    if set(statement) != contract.REVIEW_STATEMENT_KEYS:
        raise RecorderError("mapping_review_statement_internal_schema_mismatch")
    _write_json_exclusive(output_path, statement)
    return {
        "accepted_for_position_proof": False,
        "motion_authorized": False,
        "statement": str(output_path),
        "statement_sha256": _sha256_file(output_path),
        "signature_namespace": contract.REVIEW_SIGNATURE_NAMESPACE,
    }


def finalize(
    *,
    draft_path: Path,
    review_statement_path: Path,
    review_signature_path: Path,
    allowed_signers_path: Path,
    expected_review_authority_sha256: str,
    output_path: Path,
) -> dict[str, Any]:
    if contract.SHA256_RE.fullmatch(expected_review_authority_sha256) is None:
        raise RecorderError("mapping_review_authority_sha256_invalid")
    draft, draft_payload = _load_draft(draft_path)
    statement_descriptor, statement_payload = _descriptor(
        review_statement_path, "mapping_review"
    )
    signature_descriptor, signature_payload = _descriptor(
        review_signature_path, "mapping_review_signature"
    )
    allowed_descriptor, allowed_payload = _descriptor(
        allowed_signers_path, "mapping_review_allowed_signers"
    )
    if allowed_descriptor["sha256"] != expected_review_authority_sha256:
        raise RecorderError("mapping_review_authority_sha256_mismatch")
    statement = contract._json_object(statement_payload, "mapping_review")
    reviewer = statement.get("reviewed_by")
    reviewed_at = statement.get("reviewed_at")
    if (
        set(statement) != contract.REVIEW_STATEMENT_KEYS
        or statement.get("schema_version") != 2
        or statement.get("kind") != contract.REVIEW_STATEMENT_KIND
        or statement.get("robot_scope") != contract.ROBOT_SCOPE
        or statement.get("robot_serial") != draft["robot_serial"]
        or statement.get("release_sha") != draft["release_sha"]
        or not isinstance(reviewer, str)
        or contract.IDENTITY_RE.fullmatch(reviewer) is None
        or reviewer.casefold() == draft["review"]["recorded_by"].casefold()
        or not isinstance(reviewed_at, str)
        or UTC_SECONDS_RE.fullmatch(reviewed_at) is None
        or statement.get("decision") != {
            "accepted_for_position_proof": True,
            "motion_authorized": False,
        }
        or statement.get("draft_receipt") != {
            "path": str(draft_path),
            "sha256": _sha256_bytes(draft_payload),
        }
        or statement.get("mapping_evidence") != draft["mapping_evidence"]
    ):
        raise RecorderError("mapping_review_statement_binding_invalid")
    try:
        datetime.datetime.strptime(reviewed_at, "%Y-%m-%dT%H:%M:%SZ")
        contract._validate_ed25519_allowed_signers(allowed_payload, reviewer)
        contract.verify_review_signature(
            statement_payload=statement_payload,
            signature_payload=signature_payload,
            allowed_signers_payload=allowed_payload,
            reviewer_identity=reviewer,
        )
    except (ValueError, contract.MappingContractError) as exc:
        raise RecorderError("mapping_review_verification_failed") from exc

    accepted = copy.deepcopy(draft)
    accepted["review"] = {
        "recorded_by": draft["review"]["recorded_by"],
        "reviewed_by": reviewer,
        "reviewed_at": reviewed_at,
        "accepted": True,
        "statement": statement_descriptor,
        "signature": signature_descriptor,
        "allowed_signers": allowed_descriptor,
    }
    accepted["accepted_for_position_proof"] = True
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
        validate_staged=lambda staged: contract.validate_mapping_receipt(
            staged,
            expected_robot_serial=accepted["robot_serial"],
            expected_release_sha=accepted["release_sha"],
            expected_review_authority_sha256=expected_review_authority_sha256,
        ),
    )
    return {
        "accepted_for_position_proof": True,
        "motion_authorized": False,
        "output": str(output_path),
        "output_sha256": _sha256_file(output_path),
        "draft_receipt_sha256": _sha256_bytes(draft_payload),
        "review_statement_sha256": statement_descriptor["sha256"],
        "review_signature_sha256": signature_descriptor["sha256"],
        "review_authority_sha256": allowed_descriptor["sha256"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    record_parser = subparsers.add_parser("record")
    record_parser.add_argument("--vendor-joint-log", type=Path, required=True)
    record_parser.add_argument("--vendor-urdf", type=Path, required=True)
    record_parser.add_argument("--output-directory", type=Path, required=True)
    record_parser.add_argument("--release-sha", required=True)
    record_parser.add_argument("--robot-serial", default=contract.ROBOT_SERIAL)
    record_parser.add_argument("--recorded-by", required=True)
    record_parser.add_argument(
        "--maximum-log-tail-bytes",
        type=int,
        default=DEFAULT_MAXIMUM_LOG_TAIL_BYTES,
    )
    review_parser = subparsers.add_parser("review-statement")
    review_parser.add_argument("--draft", type=Path, required=True)
    review_parser.add_argument("--output", type=Path, required=True)
    review_parser.add_argument("--reviewed-by", required=True)
    review_parser.add_argument("--reviewed-at", required=True)
    finalize_parser = subparsers.add_parser("finalize")
    finalize_parser.add_argument("--draft", type=Path, required=True)
    finalize_parser.add_argument("--review-statement", type=Path, required=True)
    finalize_parser.add_argument("--review-signature", type=Path, required=True)
    finalize_parser.add_argument("--allowed-signers", type=Path, required=True)
    finalize_parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    try:
        if arguments.command == "record":
            result = record(
                vendor_joint_log=arguments.vendor_joint_log,
                vendor_urdf=arguments.vendor_urdf,
                output_directory=arguments.output_directory,
                release_sha=arguments.release_sha,
                robot_serial=arguments.robot_serial,
                recorded_by=arguments.recorded_by,
                maximum_log_tail_bytes=arguments.maximum_log_tail_bytes,
            )
        elif arguments.command == "review-statement":
            result = create_review_statement(
                draft_path=arguments.draft,
                output_path=arguments.output,
                reviewed_by=arguments.reviewed_by,
                reviewed_at=arguments.reviewed_at,
            )
        else:
            authority = os.environ.get(
                "SUNHORIZON_LEG_MAPPING_REVIEW_AUTHORITY_SHA256", ""
            )
            result = finalize(
                draft_path=arguments.draft,
                review_statement_path=arguments.review_statement,
                review_signature_path=arguments.review_signature,
                allowed_signers_path=arguments.allowed_signers,
                expected_review_authority_sha256=authority,
                output_path=arguments.output,
            )
    except (OSError, RecorderError, contract.MappingContractError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, sort_keys=True))
        return 1
    print(json.dumps({"ok": True, **result}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
