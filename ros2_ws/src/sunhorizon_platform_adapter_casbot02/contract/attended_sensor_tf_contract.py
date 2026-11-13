"""Single existing attended prototype geometry validator; no ROS or motion authority."""
import hashlib
import json
import math
from pathlib import Path

ATTENDED_HEADING_ASSET_SHA256 = "e298baf599c8c0757ba75a37269a3a30dcdfa64bb7065c718bf9e00d620f8dd5"
ATTENDED_HEADING_RECEIPT_SHA256 = "0b09bbb96e03e94187f0cf636914cc06168c2110331465bccb6fac71b43cd9e4"

def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def _require_file(path, label):
    path=Path(path)
    if not path.is_absolute() or not path.is_file() or not path.stat().st_size:
        raise RuntimeError(label+"_must_be_absolute_nonempty_file")

def validate_attended_sensor_tf(asset_path, receipt_path, usage, expected_serial):
    if usage not in ("attended_first_click", "attended_heading_correction", "attended_three_meter", "attended_five_meter"):
        raise RuntimeError("attended_navigation_profile_invalid")
    asset_path,receipt_path=Path(asset_path),Path(receipt_path)
    _require_file(asset_path, "attended_first_click_sensor_tf")
    _require_file(receipt_path, "attended_first_click_sensor_tf_receipt")
    with asset_path.open("r", encoding="utf-8") as stream:
        asset = json.load(stream)
    with receipt_path.open("r", encoding="utf-8") as stream:
        receipt = json.load(stream)
    runtime = receipt.get("required_runtime_contract", {})
    receipted_asset = receipt.get("sensor_tf_asset", {})
    if (
        asset.get("schema_version") != 1 or receipt.get("schema_version") != 1 or
        asset.get("robot_scope") != f"Robot A serial {expected_serial}" or
        receipt.get("robot_scope") != f"Robot A serial {expected_serial}" or
        asset.get("allowed_usage") != usage or
        receipt.get("allowed_usage") != [usage] or
        receipted_asset.get("filename") != asset_path.name or
        receipted_asset.get("sha256") != _sha256(asset_path) or
        runtime != {
            "motion_output_enabled": True,
            "operator_attended_required": True,
            "hardware_estop_required": True,
            "waist_pose": "forward, absolute yaw no greater than 0.10 rad",
        }
    ):
        raise RuntimeError("attended_first_click_sensor_tf_contract_invalid")
    transforms = asset.get("transforms")
    if not isinstance(transforms, list) or not transforms:
        raise RuntimeError("attended_first_click_sensor_tf_list_invalid")
    seen_children = set()
    for transform in transforms:
        required = {"parent", "child", "xyz_m", "rpy_rad"}
        values = [*transform.get("xyz_m", []), *transform.get("rpy_rad", [])]
        if (
            not required.issubset(transform) or
            transform["parent"] == transform["child"] or
            transform["child"] in seen_children or
            len(transform["xyz_m"]) != 3 or len(transform["rpy_rad"]) != 3 or
            not all(
                isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
                for value in values
            )
        ):
            raise RuntimeError("attended_first_click_sensor_tf_entry_invalid")
        seen_children.add(transform["child"])
    return transforms

