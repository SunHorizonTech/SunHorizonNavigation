"""Pure-stdlib validation for the external FAST-LIVO2 runtime contract."""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path


ROS2_PORT_COMMIT = "a09599a288c56b832114d7ad9e898248a4111d9e"
OFFICIAL_AIRY_REFERENCE_COMMIT = "0d2c0346107b75b59934975adec9a6eeeb913c64"
VIKIT_COMMIT = "6f213c7fe6be1aff1fc79c7bd6103e3e9bc4a02d"
REQUIRED_POINT_FIELDS = ["x", "y", "z", "intensity", "timestamp", "ring"]
REQUIRED_PATCHES = [
    "fast-livo2-ros2-airy-only.patch",
    "fast-livo2-ros2-disable-tf.patch",
    "rpg-vikit-sophus-target.patch",
    "fast-livo2-ros2-sophus-target.patch",
    "fast-livo2-ros2-sensor-qos.patch",
    "rpg-vikit-rational-local-params.patch",
    "fast-livo2-ros2-propagated-odom-contract.patch",
    "fast-livo2-ros2-vio-corrected-odom.patch",
    "fast-livo2-ros2-dedicated-callback-executor.patch",
    "fast-livo2-ros2-live-latest-sync.patch",
    "fast-livo2-ros2-stationary-zupt.patch",
    "fast-livo2-ros2-bounded-visual-map.patch",
    "fast-livo2-ros2-consistent-visual-voxel-key.patch",
    "fast-livo2-ros2-bounded-live-runtime.patch",
    "fast-livo2-ros2-localization-quiet-runtime.patch",
    "fast-livo2-ros2-opencv-thread-budget.patch",
    "fast-livo2-ros2-epoch-timing.patch",
]
REQUIRED_PATCH_SHA256 = {
    "fast-livo2-ros2-epoch-timing.patch": "4698ee083874ee9f5f6abba8e54a171ad4dfa32648a0da9d92d94de88c9bdf15",
    "fast-livo2-ros2-opencv-thread-budget.patch": "77467389978e6e0d6fd45fcac11051a5152ef3604ba34670e87e9c22b6561bf6",
    "fast-livo2-ros2-airy-only.patch": (
        "ae05495ddff95094cc26227304156733b324de754dbfc2cbefed4c539a070c22"
    ),
    "fast-livo2-ros2-disable-tf.patch": (
        "7a8dff0536aa2d4e5f2863ce853cddcb1c30f7664277b18707c7008331323f6c"
    ),
    "rpg-vikit-sophus-target.patch": (
        "d5dcf663b0cc12587ab90a6f1db4a6f60675044fd2a7b88c7e6a27a2eea67421"
    ),
    "fast-livo2-ros2-sophus-target.patch": (
        "87bb450f626f100d1ea8562a51e43f7198191e4aef0bc0166b7e900e59b8e327"
    ),
    "fast-livo2-ros2-sensor-qos.patch": (
        "32dc9cb0af7c972d1dc9366795a03694b1147e4e67bf403b29eabb69b069f2b4"
    ),
    "rpg-vikit-rational-local-params.patch": (
        "ac9451bb57101f24671b67b1430032758332ed8001c7536c484e8827d22604a9"
    ),
    "fast-livo2-ros2-propagated-odom-contract.patch": (
        "5519bf7533978aeb884ad2ffa39f84a68092e600be1efde395933d33cece1bce"
    ),
    "fast-livo2-ros2-vio-corrected-odom.patch": (
        "d0457ef1ce24672bb25db9bb805ea6446ee9868ded65f64a53bd48af7621aa89"
    ),
    "fast-livo2-ros2-dedicated-callback-executor.patch": (
        "d1e5f3901400a30fac8ac737c66d5a0347e97c94ebffb1f73e47d2623349a443"
    ),
    "fast-livo2-ros2-live-latest-sync.patch": (
        "77301d4d45af025178916cb1dde417a25ca6091888c5ca2e71a920921cfb6b64"
    ),
    "fast-livo2-ros2-stationary-zupt.patch": (
        "2df1cd1b8b26450554f255cba3f6a8c347c88689ae94facc756541b92f32e39d"
    ),
    "fast-livo2-ros2-bounded-visual-map.patch": (
        "5eec0c9fd97b008612942ed44e10b436d8f5f409184388411bfbb31b1a9a1a4b"
    ),
    "fast-livo2-ros2-consistent-visual-voxel-key.patch": (
        "6adfe70c0fe07151dc18e91a2232f28a0d5c7c0e2f79c89f5e268320ff9870b9"
    ),
    "fast-livo2-ros2-bounded-live-runtime.patch": (
        "ec5abbbfd80ebaedaf22489b9a52136994c6b31d769316e65d1f83e8dfb703f4"
    ),
    "fast-livo2-ros2-localization-quiet-runtime.patch": (
        "88e6012523a63754ded1dc1210553c0b50518dc4feef277d3fedfcaf3a495d4c"
    ),
}
EXPECTED_PATCHED_SOURCE = {
    "fast_livo2": {
        "base_commit": ROS2_PORT_COMMIT,
        "commit": "fdd113c202e9e37347b78ee89246b54d228fd0cc",
        "tree": "44cd3b80886053579a4bb4cd1c8b4dcdc7fe0167",
    },
    "vikit": {
        "base_commit": VIKIT_COMMIT,
        "commit": "0cf09b16542b84d52b795c75e795ea62a37a13ae",
        "tree": "0f138ff61c6e49ee423249854924dc9489f354c1",
    },
}
RECEIPT_NAME = "sunhorizon_fast_livo2_receipt.json"
SENSOR_QOS_MODE = "point_imu_parameterized_image_reliable_latest_only"
BOUNDED_VISUAL_MAP_CONTRACT = (
    "localization_only_reject_new_candidates_clear_derived_caches_before_delete"
)
BOUNDED_VISUAL_MAP_LIMITS = {
    "voxel_size_m": 0.5,
    "max_points_per_voxel": 32,
    "max_points": 100000,
    "max_reference_age_sec": 180.0,
    "half_window_xy_voxels": 30,
    "half_window_z_voxels": 10,
    "prune_interval_sec": 5.0,
    "diagnostics_interval_sec": 10.0,
    "max_observations_per_point": 30,
}
VISUAL_VOXEL_KEY_CONTRACT = (
    "single_checked_floor_helper_for_visual_and_geometry_keys_with_checked_window_offsets"
)
BOUNDED_LIVE_SENSOR_QUEUE_CONTRACT = (
    "localization_all_qos_time_count_caps_oldest_epoch_imu_bracket_"
    "discontinuity_fail_stop_no_path_history"
)
BOUNDED_LIVE_SENSOR_QUEUE_LIMITS = {
    "lidar_frames": 64,
    "imu_samples": 2048,
    "image_frames": 48,
    "max_age_sec": 4.0,
    "rgb_accumulator_points": 250000,
}
LOCALIZATION_QUIET_RUNTIME_CONTRACT = (
    "localization_only_suppress_per_frame_console_timing_debug_files_and_"
    "geometry_sliding_debug_preserve_errors_startup_and_bounded_diagnostics"
)
RUNTIME_CONFIG_SHA256 = {
    "vendor_params": "bd7060097aa4c7aba53e2bd1a1c96892dd282da2350af6fc2a9e0ed46b7291c9",
    "camera_model": "80a7db469b61e1880596f2196de1382b88fe03b3791dfda4e9c9fd8ecaa0f50d",
}
LOCALIZATION_FILTER_SIZE_SURF_MIN = 0.05
LOCALIZATION_FILTER_SIZE_SURF_MAX = 0.50
INPUT_READINESS_TIMEOUT_MIN_SEC = 1.0
INPUT_READINESS_TIMEOUT_MAX_SEC = 120.0


def validate_mode(mode: str) -> str:
    if mode not in {"mapping", "localization"}:
        raise RuntimeError("mode_must_be_mapping_or_localization")
    return mode


def validate_boolean(value: str, name: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise RuntimeError(f"{name}_must_be_explicit_boolean")


def validate_localization_point_rate(value: str) -> float:
    try:
        rate = float(value)
    except ValueError as error:
        raise RuntimeError("localization_point_rate_hz_must_be_6_8_or_10") from error
    if rate not in {6.0, 8.0, 10.0}:
        raise RuntimeError("localization_point_rate_hz_must_be_6_8_or_10")
    return rate


def validate_localization_image_rate(value: str) -> float:
    try:
        rate = float(value)
    except ValueError as error:
        raise RuntimeError("localization_image_rate_hz_must_be_6_8_or_10") from error
    if rate not in {6.0, 8.0, 10.0}:
        raise RuntimeError("localization_image_rate_hz_must_be_6_8_or_10")
    return rate


def validate_localization_filter_size_surf(value: str) -> float:
    try:
        filter_size = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise RuntimeError(
            "localization_filter_size_surf_must_be_finite_between_0_05_and_0_50"
        ) from error
    if not math.isfinite(filter_size) or not (
        LOCALIZATION_FILTER_SIZE_SURF_MIN
        <= filter_size
        <= LOCALIZATION_FILTER_SIZE_SURF_MAX
    ):
        raise RuntimeError(
            "localization_filter_size_surf_must_be_finite_between_0_05_and_0_50"
        )
    return filter_size


def resolve_localization_filter_size_surf(
    mode: str, value: str
) -> float | None:
    validated_mode = validate_mode(mode)
    filter_size = validate_localization_filter_size_surf(value)
    if validated_mode == "localization":
        return filter_size
    return None


def validate_input_readiness_timeout(value: str) -> float:
    try:
        timeout = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise RuntimeError(
            "input_readiness_timeout_sec_must_be_finite_between_1_and_120"
        ) from error
    if not math.isfinite(timeout) or not (
        INPUT_READINESS_TIMEOUT_MIN_SEC
        <= timeout
        <= INPUT_READINESS_TIMEOUT_MAX_SEC
    ):
        raise RuntimeError(
            "input_readiness_timeout_sec_must_be_finite_between_1_and_120"
        )
    return timeout


def _absolute_directory(value: str, name: str) -> Path:
    if not value:
        raise RuntimeError(f"{name}_must_be_explicit")
    raw = Path(value).expanduser()
    if not raw.is_absolute():
        raise RuntimeError(f"{name}_must_be_absolute")
    path = raw.resolve(strict=True)
    if not path.is_dir():
        raise RuntimeError(f"{name}_must_be_directory")
    return path


def _absolute_file(value: str, name: str) -> Path:
    if not value:
        raise RuntimeError(f"{name}_must_be_explicit")
    raw = Path(value).expanduser()
    if not raw.is_absolute():
        raise RuntimeError(f"{name}_must_be_absolute")
    path = raw.resolve(strict=True)
    if not path.is_file():
        raise RuntimeError(f"{name}_must_be_file")
    return path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_vendor_contract(
    vendor_root_value: str,
    vendor_executable_value: str,
    vendor_params_value: str,
    camera_model_value: str,
) -> dict[str, str]:
    root = _absolute_directory(vendor_root_value, "vendor_root")
    executable = _absolute_file(vendor_executable_value, "vendor_executable")
    params = _absolute_file(vendor_params_value, "vendor_params_path")
    camera = _absolute_file(camera_model_value, "camera_model_path")

    try:
        executable.relative_to(root)
    except ValueError as error:
        raise RuntimeError("vendor_executable_must_be_below_vendor_root") from error
    if not os.access(executable, os.X_OK):
        raise RuntimeError("vendor_executable_must_be_executable")

    receipt_path = root / RECEIPT_NAME
    if not receipt_path.is_file():
        raise RuntimeError("vendor_receipt_missing")
    try:
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise RuntimeError("vendor_receipt_invalid_json") from error

    if receipt.get("ros2_port_commit") != ROS2_PORT_COMMIT:
        raise RuntimeError("vendor_ros2_commit_mismatch")
    if receipt.get("official_airy_reference_commit") != OFFICIAL_AIRY_REFERENCE_COMMIT:
        raise RuntimeError("vendor_airy_reference_commit_mismatch")
    if receipt.get("vikit_commit") != VIKIT_COMMIT:
        raise RuntimeError("vendor_vikit_commit_mismatch")
    if receipt.get("tf_publishing_disabled") is not True:
        raise RuntimeError("vendor_tf_must_be_disabled")
    if receipt.get("measurement_timestamp_source") != "LidarMeasures.last_lio_update_time":
        raise RuntimeError("vendor_measurement_timestamp_contract_mismatch")
    if receipt.get("propagated_odometry_contract") != (
        "camera_init_to_aft_mapped_at_imu_measurement_time"
    ):
        raise RuntimeError("vendor_propagated_odometry_contract_mismatch")
    if receipt.get("vio_corrected_odometry_contract") != (
        "publish_once_after_vio_per_livo_epoch_with_lio_fallback"
    ):
        raise RuntimeError("vendor_vio_corrected_odometry_contract_mismatch")
    if receipt.get("callback_execution_contract") != (
        "dedicated_single_threaded_executor_with_locked_propagation"
    ):
        raise RuntimeError("vendor_callback_execution_contract_mismatch")
    if receipt.get("live_sensor_sync_contract") != (
        "latest_feasible_image_with_bounded_lidar_history_atomic_sensor_deques_"
        "live_warmup_drop_offline_fifo"
    ):
        raise RuntimeError("vendor_live_sensor_sync_contract_mismatch")
    if receipt.get("stationary_zupt_contract") != (
        "uint64_epoch_zero_inactive_corrected_livo_velocity_ekf_"
        "nis_gated_ack_after_apply"
    ):
        raise RuntimeError("vendor_stationary_zupt_contract_mismatch")
    if receipt.get("bounded_visual_map_contract") != BOUNDED_VISUAL_MAP_CONTRACT:
        raise RuntimeError("vendor_bounded_visual_map_contract_mismatch")
    if receipt.get("bounded_visual_map_limits") != BOUNDED_VISUAL_MAP_LIMITS:
        raise RuntimeError("vendor_bounded_visual_map_limits_mismatch")
    if receipt.get("visual_voxel_key_contract") != VISUAL_VOXEL_KEY_CONTRACT:
        raise RuntimeError("vendor_visual_voxel_key_contract_mismatch")
    if receipt.get("bounded_live_sensor_queue_contract") != (
        BOUNDED_LIVE_SENSOR_QUEUE_CONTRACT
    ):
        raise RuntimeError("vendor_bounded_live_sensor_queue_contract_mismatch")
    if receipt.get("bounded_live_sensor_queue_limits") != (
        BOUNDED_LIVE_SENSOR_QUEUE_LIMITS
    ):
        raise RuntimeError("vendor_bounded_live_sensor_queue_limits_mismatch")
    if receipt.get("localization_quiet_runtime_contract") != (
        LOCALIZATION_QUIET_RUNTIME_CONTRACT
    ):
        raise RuntimeError("vendor_localization_quiet_runtime_contract_mismatch")
    if receipt.get("sensor_qos_mode") != SENSOR_QOS_MODE:
        raise RuntimeError("vendor_sensor_qos_contract_mismatch")
    if receipt.get("airy_only") is not True:
        raise RuntimeError("vendor_airy_only_contract_mismatch")
    if receipt.get("point_fields") != REQUIRED_POINT_FIELDS:
        raise RuntimeError("vendor_point_field_contract_mismatch")
    if receipt.get("patches") != REQUIRED_PATCHES:
        raise RuntimeError("vendor_patch_list_mismatch")
    if receipt.get("patch_sha256") != REQUIRED_PATCH_SHA256:
        raise RuntimeError("vendor_patch_sha256_mismatch")
    if receipt.get("patched_source") != EXPECTED_PATCHED_SOURCE:
        raise RuntimeError("vendor_patched_source_identity_mismatch")
    if receipt.get("runtime_config_sha256") != RUNTIME_CONFIG_SHA256:
        raise RuntimeError("vendor_runtime_config_sha256_mismatch")
    if sha256(params) != RUNTIME_CONFIG_SHA256["vendor_params"]:
        raise RuntimeError("vendor_params_sha256_mismatch")
    if sha256(camera) != RUNTIME_CONFIG_SHA256["camera_model"]:
        raise RuntimeError("camera_model_sha256_mismatch")
    if receipt.get("runtime_libraries_resolved") is not True:
        raise RuntimeError("vendor_runtime_libraries_unresolved")
    if receipt.get("runtime_started") is not False:
        raise RuntimeError("vendor_receipt_runtime_state_invalid")
    if receipt.get("executable_sha256") != sha256(executable):
        raise RuntimeError("vendor_executable_sha256_mismatch")

    return {
        "vendor_root": str(root),
        "vendor_executable": str(executable),
        "vendor_params_path": str(params),
        "camera_model_path": str(camera),
        "receipt_path": str(receipt_path),
    }
