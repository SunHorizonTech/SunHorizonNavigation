import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE / "internal_optical_tf_contract.py"
SPEC = importlib.util.spec_from_file_location("internal_optical_tf_contract", MODULE_PATH)
CONTRACT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CONTRACT)


PARAMS = """sunhorizon_camera_runtime:
  ros__parameters:
    publish_internal_optical_tf: true
    chest_internal_tf_parent_frame: camera_chest_link
    chest_internal_tf_child_frame: camera_chest_color_optical_frame
    chest_internal_tf_translation_xyz: [0.001, -0.002, 0.003]
    chest_internal_tf_rotation_xyzw: [0.5, -0.5, 0.5, -0.5]
    belly_internal_tf_parent_frame: camera_belly_link
    belly_internal_tf_child_frame: camera_belly_depth_optical_frame
    belly_internal_tf_translation_xyz: [-0.004, 0.005, -0.006]
    belly_internal_tf_rotation_xyzw: [-0.5, 0.5, -0.5, 0.5]
"""


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def repeatability_evidence(root):
    edges = {}
    for label, binding in CONTRACT.REVIEW_EDGE_BINDINGS.items():
        samples = []
        for index in range(3):
            capture = root / f"repeatability-{label.replace('.', '-')}-{index}.json"
            capture.write_text(json.dumps({"edge": label, "sample": index}), encoding="utf-8")
            samples.append({
                "captured_at": f"2026-01-01T00:00:0{index}Z",
                "translation_xyz": [0.0, 0.0, 0.0],
                "rotation_xyzw": [0.0, 0.0, 0.0, -1.0 if index == 1 else 1.0],
                "capture": {"path": str(capture), "filename": capture.name,
                            "sha256": digest(capture)},
            })
        edges[label] = {**binding, "samples": samples}
    evidence = root / "physical-camera-review.json"
    evidence.write_text(json.dumps({
        "schema_version": 1, "kind": CONTRACT.REPEATABILITY_EVIDENCE_KIND,
        "robot_scope": "Robot A", "robot_serial": CONTRACT.EXPECTED_ROBOT_SERIAL,
        "edges": edges,
    }), encoding="utf-8")
    return evidence


def camera_evidence(root, role, serial, transform):
    bag = root / f"{role}-bag"
    bag.mkdir(exist_ok=True)
    metadata = bag / "metadata.yaml"
    metadata.write_text("rosbag2_bagfile_information:\n  storage_identifier: mcap\n",
                        encoding="utf-8")
    segment = bag / f"{role}_0.mcap"
    segment.write_bytes(f"{role}-real-capture".encode())
    def asset(path, relative):
        return {"relative_path": relative, "bytes": path.stat().st_size,
                "sha256": digest(path), "stable_regular_file_checked": True}
    metadata_asset = asset(metadata, "metadata.yaml")
    segment_asset = asset(segment, segment.name)
    timestamp = 1_767_225_600_000_000_000
    record = {
        "filtered_record_index": 1, "storage_timestamp_ns": timestamp,
        "segment_relative_path": segment.name,
        "segment_resolution_method": "rosbag2_py.get_current_file",
        "segment_sha256": digest(segment),
    }
    binding = {
        "schema": CONTRACT.CAPTURE_BINDING_SCHEMA,
        "robot_serial": CONTRACT.EXPECTED_ROBOT_SERIAL,
        "camera_role": role, "camera_serial": serial,
        "observed_by": "camera-capture-operator",
        "chain_parent": CONTRACT.ROLE_FRAMES[role][0],
        "chain_child": CONTRACT.ROLE_FRAMES[role][1],
        "bag_path": str(bag), "metadata": metadata_asset,
        "storage_identifier": "mcap", "tf_static_topic": CONTRACT.TF_STATIC_TOPIC_CONTRACT,
        "read_records": [record], "segments": [segment_asset],
    }
    capture = {
        "path": str(bag), "sha256": CONTRACT._canonical_sha256(binding),
        "sha256_semantics": CONTRACT.CAPTURE_SHA_SEMANTICS,
        "binding_complete": True, "binding_blockers": [], "binding": binding,
        "metadata": metadata_asset,
        "reader_metadata": {"storage_identifier": "mcap",
                            "relative_file_paths": [segment.name], "api_available": True},
        "actual_read_segments": [segment_asset], "read_records": [record],
    }
    source = {key: record[key] for key in (
        "filtered_record_index", "storage_timestamp_ns", "segment_relative_path")}
    evidence = root / f"{role}-tf-static.json"
    tf = {**transform, "source_record": source, "header_stamp_ns": timestamp}
    evidence.write_text(json.dumps({
        "schema_version": 1, "kind": "robot_a_camera_vendor_tf_static_evidence",
        "robot_scope": "Robot A", "robot_serial": CONTRACT.EXPECTED_ROBOT_SERIAL,
        "role": role, "camera_serial": serial, "observed_on": "2026-01-01",
        "observed_by": "camera-capture-operator", "source_capture": capture,
        "transforms": [tf],
    }), encoding="utf-8")
    descriptor = {"path": str(bag), "filename": bag.name, "sha256": capture["sha256"]}
    return evidence, descriptor, segment


class InternalOpticalTfContractTest(unittest.TestCase):
    def prepare(self, root):
        params = root / "camera_runtime.robot-a-motion-accepted.yaml"
        params.write_text(PARAMS, encoding="utf-8")
        receipt = root / "camera_internal_tf.motion-acceptance.receipt.json"
        transforms = CONTRACT._parameter_transforms(PARAMS)
        physical_evidence = repeatability_evidence(root)
        review = {
            "measured_by": "camera-metrologist",
            "measured_on": "2026-01-01",
            "reviewed_by": "camera-physical-reviewer",
            "reviewed_on": "2026-01-02",
            "translation_repeatability_m": 0.0,
            "rotation_repeatability_deg": 0.0,
            "evidence_path": str(physical_evidence),
            "evidence_sha256": digest(physical_evidence),
        }
        evidence_paths = {}
        capture_assets = {}
        capture_segments = {}
        for role, serial in CONTRACT.EXPECTED_CAMERA_SERIALS.items():
            evidence, descriptor, segment = camera_evidence(
                root, role, serial, transforms[role])
            evidence_paths[role] = evidence
            capture_assets[role] = descriptor
            capture_segments[role] = segment
        measurement = root / "robot-a-production-tf-measurements.json"
        measurement.write_text(json.dumps({
            "schema_version": 1,
            "kind": "robot_a_production_tf_measurements",
            "robot_scope": "Robot A",
            "robot_serial": "1421026005616",
            "status": "measured_and_reviewed",
            "motion_authorized": False,
            "sources": {
                f"{role}_vendor_tf_static": {
                    "path": str(evidence_paths[role]),
                    "sha256": digest(evidence_paths[role]),
                }
                for role in ("chest", "belly")
            },
            "camera_internal": {
                role: {
                    "source": f"{role}_vendor_tf_static",
                    "physical_review": review,
                }
                for role in ("chest", "belly")
            },
        }), encoding="utf-8")
        assets = {
            "camera_params": {
                "filename": params.name,
                "sha256": digest(params),
            },
            "measurement_record": {
                "path": str(measurement),
                "filename": measurement.name,
                "sha256": digest(measurement),
            },
        }
        for role in ("chest", "belly"):
            assets[f"{role}_vendor_tf_static"] = {
                "path": str(evidence_paths[role]),
                "filename": evidence_paths[role].name,
                "sha256": digest(evidence_paths[role]),
            }
            assets[f"{role}_source_capture"] = capture_assets[role]
        receipt.write_text(json.dumps({
            "schema_version": 1,
            "kind": "robot_a_camera_internal_optical_tf_acceptance",
            "robot_scope": "Robot A",
            "robot_serial": "1421026005616",
            "allowed_usage": ["motion_acceptance"],
            "assets": assets,
            "tf_ownership": CONTRACT.EXPECTED_OWNERSHIP,
            "transforms": transforms,
            "acceptance": {
                "accepted": True,
                "accepted_by": "independent-reviewer",
                "accepted_on": "2026-01-03",
            },
        }, indent=2), encoding="utf-8")
        return receipt, params

    def test_accepts_exact_hash_bound_motion_parameters(self):
        with tempfile.TemporaryDirectory() as directory:
            receipt, params = self.prepare(Path(directory))
            self.assertEqual(
                CONTRACT.validate(receipt, params, "1421026005616"),
                digest(receipt),
            )
            self.assertTrue(CONTRACT._same_quaternion(
                [0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, -1.0]))

    def test_rejects_shadow_usage_wrong_serial_and_tampering(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            receipt, params = self.prepare(root)
            document = json.loads(receipt.read_text(encoding="utf-8"))
            document["allowed_usage"] = ["live_zero_motion_shadow"]
            receipt.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "usage_invalid"):
                CONTRACT.validate(receipt, params, "1421026005616")

            receipt, params = self.prepare(root)
            with self.assertRaisesRegex(ValueError, "robot_serial_invalid"):
                CONTRACT.validate(receipt, params, "wrong-robot")

            params.write_text(PARAMS.replace("0.001", "0.101"), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "params_hash_mismatch"):
                CONTRACT.validate(receipt, params, "1421026005616")

    def test_rejects_duplicate_or_wrong_frames_and_nonunit_quaternion(self):
        cases = {
            "frame_invalid:chest": PARAMS.replace(
                "camera_chest_color_optical_frame", "camera_belly_depth_optical_frame", 1),
            "quaternion_invalid:belly": PARAMS.replace(
                "[-0.5, 0.5, -0.5, 0.5]", "[0.0, 0.0, 0.0, 0.5]"),
            "publish_not_enabled": PARAMS.replace(
                "publish_internal_optical_tf: true", "publish_internal_optical_tf: false"),
        }
        for reason, text in cases.items():
            with self.subTest(reason=reason):
                with self.assertRaisesRegex(ValueError, reason):
                    CONTRACT._parameter_transforms(text)

    def test_rejects_relative_assets_and_vendor_or_duplicate_ownership(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            receipt, params = self.prepare(root)
            with self.assertRaisesRegex(ValueError, "must_be_absolute"):
                CONTRACT.validate(Path(receipt.name), params, "1421026005616")
            document = json.loads(receipt.read_text(encoding="utf-8"))
            document["tf_ownership"]["vendor_orbbec_tf_publishers_allowed"] = True
            receipt.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "ownership_invalid"):
                CONTRACT.validate(receipt, params, "1421026005616")

    def test_rejects_draft_future_and_nonindependent_final_review(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            receipt, params = self.prepare(root)
            document = json.loads(receipt.read_text(encoding="utf-8"))
            document["draft"] = {"motion_authorized": False}
            receipt.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "draft_receipt_rejected"):
                CONTRACT.validate(receipt, params, "1421026005616")

            receipt, params = self.prepare(root)
            document = json.loads(receipt.read_text(encoding="utf-8"))
            document["acceptance"]["accepted_on"] = "2099-01-01"
            receipt.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "date_in_future"):
                CONTRACT.validate(receipt, params, "1421026005616")

            receipt, params = self.prepare(root)
            document = json.loads(receipt.read_text(encoding="utf-8"))
            document["acceptance"]["accepted_by"] = "CAMERA-METROLOGIST"
            receipt.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "final_review_independence_invalid"):
                CONTRACT.validate(receipt, params, "1421026005616")

    def test_rejects_tampered_capture_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            receipt, params = self.prepare(root)
            document = json.loads(receipt.read_text(encoding="utf-8"))
            (Path(document["assets"]["belly_source_capture"]["path"])
             / "belly_0.mcap").write_bytes(b"tampered-capture")
            with self.assertRaisesRegex(ValueError, "segment:belly:0_asset_hash_mismatch"):
                CONTRACT.validate(receipt, params, "1421026005616")


if __name__ == "__main__":
    unittest.main()
