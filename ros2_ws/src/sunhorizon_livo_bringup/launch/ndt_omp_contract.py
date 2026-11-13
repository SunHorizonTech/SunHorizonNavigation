"""Offline-only validation for the pinned ndt_omp candidate receipt."""

import hashlib
import json
from pathlib import Path, PurePosixPath
import re


PCL_BACKEND = "pcl"
NDT_OMP_BACKEND = "ndt_omp"
REGISTRATION_BACKENDS = {PCL_BACKEND, NDT_OMP_BACKEND}
NDT_OMP_NEIGHBORHOODS = {"KDTREE", "DIRECT1", "DIRECT7"}
NDT_OMP_UPSTREAM_URL = "https://github.com/tier4/ndt_omp.git"
NDT_OMP_UPSTREAM_COMMIT = "cd8838d4280f79d09629b0a4f54fe5f93849a099"
NDT_OMP_UPSTREAM_TREE = "0269c37051e02e216f571c7e6057a153a043de47"
NDT_OMP_REQUIRED_ARTIFACTS = {
    "install/lib/libndt_omp.so",
    "install/lib/libmultigrid_ndt_omp.so",
}
_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _absolute_regular_file(
    path_value: str,
    label: str,
    maximum_size: int | None = None,
    allow_empty: bool = False,
) -> Path:
    path = Path(path_value)
    try:
        path.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise RuntimeError(f"ndt_omp {label} must be an existing absolute regular file") from error
    if (
        not path.is_absolute()
        or path.is_symlink()
        or not path.is_file()
        or (not allow_empty and path.stat().st_size <= 0)
        or (maximum_size is not None and path.stat().st_size > maximum_size)
    ):
        raise RuntimeError(f"ndt_omp {label} must be an existing absolute regular file")
    return path


def _load_receipt(receipt_path_value: str) -> tuple[Path, dict]:
    receipt_path = _absolute_regular_file(
        receipt_path_value, "receipt", maximum_size=4 * 1024 * 1024)
    try:
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise RuntimeError("ndt_omp receipt must contain valid JSON") from error
    if not isinstance(receipt, dict):
        raise RuntimeError("ndt_omp receipt must contain a JSON object")
    return receipt_path, receipt


def _require_hash_record(root: Path, record, label: str) -> str:
    if not isinstance(record, dict) or set(record) != {"path", "sha256"}:
        raise RuntimeError(f"ndt_omp {label} hash record is invalid")
    relative_value = record.get("path")
    expected_hash = record.get("sha256")
    if not isinstance(relative_value, str) or not isinstance(expected_hash, str):
        raise RuntimeError(f"ndt_omp {label} hash record is invalid")
    relative = PurePosixPath(relative_value)
    if relative.is_absolute() or not relative.parts or any(
        part in {"", ".", ".."} for part in relative.parts
    ):
        raise RuntimeError(f"ndt_omp {label} path is unsafe")
    if _SHA256_PATTERN.fullmatch(expected_hash) is None:
        raise RuntimeError(f"ndt_omp {label} SHA-256 is invalid")
    path = root
    for part in relative.parts:
        path = path / part
        if path.is_symlink():
            raise RuntimeError(f"ndt_omp {label} path contains a symbolic link")
    path = _absolute_regular_file(str(path), label)
    try:
        path.resolve(strict=True).relative_to(root.resolve(strict=True))
    except (OSError, RuntimeError, ValueError) as error:
        raise RuntimeError(f"ndt_omp {label} path escapes the receipt root") from error
    if _sha256_file(path) != expected_hash:
        raise RuntimeError(f"ndt_omp {label} SHA-256 mismatch")
    return relative.as_posix()


def _validate_artifacts(root: Path, artifacts) -> None:
    if not isinstance(artifacts, list) or not artifacts:
        raise RuntimeError("ndt_omp receipt must list hashed artifacts")
    seen = set()
    for index, record in enumerate(artifacts):
        if not isinstance(record, dict) or set(record) != {
            "path", "sha256", "size_bytes"
        }:
            raise RuntimeError(f"ndt_omp artifact record {index} is invalid")
        path_value = record.get("path")
        expected_hash = record.get("sha256")
        expected_size = record.get("size_bytes")
        if (
            not isinstance(path_value, str)
            or not isinstance(expected_hash, str)
            or _SHA256_PATTERN.fullmatch(expected_hash) is None
            or not isinstance(expected_size, int)
            or isinstance(expected_size, bool)
            or expected_size < 0
        ):
            raise RuntimeError(f"ndt_omp artifact record {index} is invalid")
        relative = PurePosixPath(path_value)
        if relative.is_absolute() or not relative.parts or any(
            part in {"", ".", ".."} for part in relative.parts
        ):
            raise RuntimeError(f"ndt_omp artifact path is unsafe: {path_value}")
        normalized = relative.as_posix()
        if normalized in seen:
            raise RuntimeError(f"ndt_omp artifact path is duplicated: {normalized}")
        seen.add(normalized)
        artifact = root
        for part in relative.parts:
            artifact = artifact / part
            if artifact.is_symlink():
                raise RuntimeError(
                    f"ndt_omp artifact path contains a symbolic link: {normalized}")
        artifact = _absolute_regular_file(
            str(artifact), f"artifact {normalized}", allow_empty=True)
        try:
            artifact.resolve(strict=True).relative_to(root.resolve(strict=True))
        except (OSError, RuntimeError, ValueError) as error:
            raise RuntimeError(
                f"ndt_omp artifact path escapes the receipt root: {normalized}") from error
        if artifact.stat().st_size != expected_size:
            raise RuntimeError(f"ndt_omp artifact size mismatch: {normalized}")
        if _sha256_file(artifact) != expected_hash:
            raise RuntimeError(f"ndt_omp artifact SHA-256 mismatch: {normalized}")
    missing = NDT_OMP_REQUIRED_ARTIFACTS - seen
    if missing:
        raise RuntimeError(
            "ndt_omp receipt is missing required artifacts: " + ",".join(sorted(missing)))


def validate_ndt_omp_receipt(receipt_path_value: str) -> str:
    receipt_path, receipt = _load_receipt(receipt_path_value)
    if receipt.get("schema_version") != 1:
        raise RuntimeError("ndt_omp receipt schema_version must be 1")
    if receipt.get("candidate") != NDT_OMP_BACKEND:
        raise RuntimeError("ndt_omp receipt candidate mismatch")
    if receipt.get("implementation") != "tier4/ndt_omp":
        raise RuntimeError("ndt_omp receipt implementation mismatch")

    upstream = receipt.get("upstream")
    if not isinstance(upstream, dict):
        raise RuntimeError("ndt_omp receipt upstream metadata is missing")
    if upstream.get("url") != NDT_OMP_UPSTREAM_URL:
        raise RuntimeError("ndt_omp receipt upstream URL mismatch")
    if upstream.get("commit") != NDT_OMP_UPSTREAM_COMMIT:
        raise RuntimeError("ndt_omp receipt upstream commit mismatch")
    if upstream.get("tree") != NDT_OMP_UPSTREAM_TREE:
        raise RuntimeError("ndt_omp receipt upstream tree mismatch")

    build = receipt.get("build")
    if not isinstance(build, dict):
        raise RuntimeError("ndt_omp receipt build metadata is missing")
    if str(build.get("architecture", "")).lower() not in {"arm64", "aarch64"}:
        raise RuntimeError("ndt_omp receipt architecture must be ARM64")
    if build.get("elf_machine") != "AArch64":
        raise RuntimeError("ndt_omp receipt ELF machine must be AArch64")
    if not str(build.get("pcl_version", "")).startswith("1.12."):
        raise RuntimeError("ndt_omp receipt PCL version must be 1.12.x")
    if build.get("openmp_required") is not True or build.get(
        "openmp_runtime_linked") is not True:
        raise RuntimeError("ndt_omp receipt must prove an OpenMP runtime link")

    verification = receipt.get("verification")
    if not isinstance(verification, dict):
        raise RuntimeError("ndt_omp receipt verification metadata is missing")
    required_verification = {
        "downstream_find_package_smoke": True,
        "arm64_build_verified": True,
        "runtime_started": False,
        "ros_or_dds_started": False,
        "deployed": False,
    }
    for name, expected in required_verification.items():
        if verification.get(name) is not expected:
            raise RuntimeError(f"ndt_omp receipt verification mismatch: {name}")

    root = receipt_path.parent
    _require_hash_record(root, upstream.get("source_archive"), "source archive")
    license_record = upstream.get("license")
    if not isinstance(license_record, dict) or license_record.get("spdx") != "BSD-2-Clause":
        raise RuntimeError("ndt_omp receipt license must be BSD-2-Clause")
    _require_hash_record(
        root,
        {name: license_record.get(name) for name in ("path", "sha256")},
        "license",
    )
    _validate_artifacts(root, receipt.get("artifacts"))
    return str(receipt_path)


def validate_registration_selection(
    registration_backend: str,
    ndt_omp_threads: str,
    ndt_omp_neighborhood: str,
    ndt_omp_receipt_path: str,
) -> dict[str, str]:
    backend = registration_backend.strip().lower()
    if backend not in REGISTRATION_BACKENDS:
        raise RuntimeError("registration_backend must be pcl or ndt_omp")
    if backend == PCL_BACKEND:
        return {
            "ndt_backend": PCL_BACKEND,
            "ndt_threads": "1",
            "ndt_neighborhood": "KDTREE",
        }
    try:
        threads = int(ndt_omp_threads.strip())
    except ValueError as error:
        raise RuntimeError("ndt_omp_threads must be an integer within [1, 64]") from error
    if str(threads) != ndt_omp_threads.strip() or not 1 <= threads <= 64:
        raise RuntimeError("ndt_omp_threads must be an integer within [1, 64]")
    neighborhood = ndt_omp_neighborhood.strip().upper()
    if neighborhood not in NDT_OMP_NEIGHBORHOODS:
        raise RuntimeError(
            "ndt_omp_neighborhood must be KDTREE, DIRECT1, or DIRECT7")
    validate_ndt_omp_receipt(ndt_omp_receipt_path.strip())
    return {
        "ndt_backend": backend,
        "ndt_threads": str(threads),
        "ndt_neighborhood": neighborhood,
    }
