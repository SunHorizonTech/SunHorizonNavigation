import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("joint_startup_contract", PACKAGE / "contract/joint_startup_contract.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class JointStartupContractTest(unittest.TestCase):
    def models(self):
        # Same measured Robot A local geometry as the tf2 parity gtest.
        text = (PACKAGE / "test/test_joint_kinematics.cpp").read_text()
        xml = text.split('R"urdf(', 1)[1].split(')urdf"', 1)[0]
        legacy = ET.fromstring(xml)
        legacy.find("joint[@name='waist_yaw']").set("name", "waist_yaw_joint")
        production = ET.Element("robot", name="accepted_sensor_fixture")
        for link in ("base_link", "waist_yaw_link"):
            ET.SubElement(production, "link", name=link)
        waist = copy.deepcopy(legacy.find("joint[@name='waist_yaw_joint']"))
        waist.set("name", "waist_yaw")
        production.append(waist)
        names = [joint.get("name") for joint in legacy.findall("joint") if "_leg_" in joint.get("name")]
        return legacy, production, names

    def test_real_leg_edges_are_unchanged_and_waist_alias_is_explicit(self):
        legacy, production, names = self.models()
        source = ET.tostring(legacy)
        result = MODULE.merge_fk_models(source, ET.tostring(production).decode(), names)
        merged = ET.fromstring(result["merged_urdf_xml"])
        self.assertEqual(result["joint_name_aliases"], {"waist_yaw_joint": "waist_yaw"})
        self.assertEqual(len(result["leg_target_frames"]), 12)
        self.assertEqual(hashlib.sha256(result["merged_urdf_xml"].encode()).hexdigest(), result["merged_urdf_sha256"])
        self.assertIsNone(merged.find("joint[@name='waist_yaw_joint']"))
        for name in names:
            original = legacy.find(f"joint[@name='{name}']")
            actual = merged.find(f"joint[@name='{name}']")
            self.assertEqual(actual.attrib, original.attrib)
            self.assertEqual([(child.tag, child.attrib) for child in actual],
                             [(child.tag, child.attrib) for child in original])
        self.assertEqual(ET.tostring(legacy), source)

    def test_changed_waist_geometry_and_guessed_alias_are_rejected(self):
        legacy, production, names = self.models()
        production.find("joint/origin").set("xyz", "1 0 0")
        with self.assertRaisesRegex(ValueError, "waist_geometry_mismatch"):
            MODULE.merge_fk_models(ET.tostring(legacy), ET.tostring(production).decode(), names)
        production.find("joint").set("name", "guessed_waist")
        with self.assertRaisesRegex(ValueError, "explicit_waist_alias_missing"):
            MODULE.merge_fk_models(ET.tostring(legacy), ET.tostring(production).decode(), names)

    def test_already_present_identical_leg_edges_have_exactly_one_owner(self):
        legacy, production, names = self.models()
        first = MODULE.merge_fk_models(ET.tostring(legacy), ET.tostring(production).decode(), names)
        second = MODULE.merge_fk_models(ET.tostring(legacy), first["merged_urdf_xml"], names)
        joints = ET.fromstring(second["merged_urdf_xml"]).findall("joint")
        self.assertEqual(len(joints), 13)
        self.assertEqual(len({joint.get("name") for joint in joints}), 13)

    def test_duplicate_child_owner_and_missing_mapping_are_rejected(self):
        legacy, production, names = self.models()
        collision = copy.deepcopy(legacy.find(f"joint[@name='{names[0]}']"))
        collision.set("name", "another_owner")
        production.append(collision)
        with self.assertRaisesRegex(ValueError, "merged_owner_collision"):
            MODULE.merge_fk_models(ET.tostring(legacy), ET.tostring(production).decode(), names)
        with self.assertRaisesRegex(ValueError, "leg_mapping_shape"):
            MODULE.merge_fk_models(ET.tostring(legacy), ET.tostring(production).decode(), names[:-1])

    def test_fixed_sensor_edges_are_bound_once_without_changing_geometry(self):
        _, production, _ = self.models()
        edges = [("base_footprint", "base_link", (0.1, 0.2, 0.3), (0., 0., 0.)),
                 ("waist_yaw_link", "camera", (0.4, 0.5, 0.6), (0.1, 0.2, 0.3))]
        xml = MODULE.append_accepted_fixed_edges(ET.tostring(production).decode(), edges)
        root = ET.fromstring(xml)
        origin = root.find("joint[@name='accepted_fixed_camera']/origin")
        self.assertEqual(origin.get("xyz"), "0.4 0.5 0.6")
        self.assertEqual(origin.get("rpy"), "0.1 0.2 0.3")
        with self.assertRaisesRegex(ValueError, "fixed_edge_owner_collision"):
            MODULE.append_accepted_fixed_edges(xml, edges)

    def test_attended_vendor_only_model_has_no_invented_base_or_sensor_extrinsics(self):
        legacy, _, names = self.models()
        original = ET.tostring(legacy)
        result = MODULE.attended_fk_model(original, names)
        root = ET.fromstring(result['merged_urdf_xml'])
        self.assertEqual({link.get('name') for link in root.findall('link')},
                         {'base_link', 'waist_yaw_link', *result['leg_target_frames']})
        self.assertEqual(len(root.findall('joint')), 13)
        self.assertIsNone(root.find("link[@name='base_footprint']"))
        self.assertEqual(ET.tostring(legacy), original)

    def test_modes_require_explicit_choice_and_production_never_falls_back(self):
        with self.assertRaisesRegex(ValueError, 'tf_mode_invalid'):
            MODULE.startup_tf_mode({})
        with self.assertRaisesRegex(ValueError, 'production_tf_receipt_required'):
            MODULE.startup_tf_mode({'tf_mode': 'production_direct'})
        self.assertEqual(MODULE.startup_tf_mode({'tf_mode': 'attended_global_tf'}), 'attended_global_tf')
        with self.assertRaisesRegex(ValueError, 'must_not_claim_production_acceptance'):
            MODULE.startup_tf_mode({'tf_mode': 'attended_global_tf', 'production_tf_receipt_path': '/fake'})

    def test_attended_direct_uses_same_original_accepted_eight_edges(self):
        config=PACKAGE.parent/'sunhorizon_nav2_bringup/config'
        asset=config/'robot_a_prototype_sensor_tf.attended-heading-correction.json'
        receipt=config/'robot_a_prototype_sensor_tf.attended-heading-correction.receipt.json'
        options=dict(tf_mode='attended_direct',attended_usage='attended_heading_correction',
                     robot_serial='1421026005616',attended_tf_asset_path=str(asset),
                     attended_tf_receipt_path=str(receipt),
                     attended_tf_asset_sha256=hashlib.sha256(asset.read_bytes()).hexdigest(),
                     attended_tf_receipt_sha256=hashlib.sha256(receipt.read_bytes()).hexdigest())
        self.assertEqual(MODULE.startup_tf_mode(options),'attended_direct')
        edges=MODULE.validated_attended_edges(options)
        self.assertEqual(edges,json.loads(asset.read_text())['transforms'])
        self.assertEqual(len(edges),8)
        waist=next(edge for edge in edges if edge['child']=='waist_yaw_link')
        self.assertEqual(waist['rpy_rad'],[0.,0.,0.])
        options['attended_tf_receipt_sha256']='0'*64
        with self.assertRaisesRegex(ValueError,'original_asset_pin_required'):
            MODULE.validated_attended_edges(options)
        options['attended_usage']='motion_acceptance'
        with self.assertRaisesRegex(ValueError,'attended_direct_evidence_required'):
            MODULE.startup_tf_mode(options)

    def test_runtime_compiles_attended_constants_before_acquisition_and_keeps_budget_checks(self):
        source = (PACKAGE / "src/runtime.cpp").read_text()
        configure = source[source.index("void PerceptionRuntime::configure_direct_transforms("):
                           source.index("std::shared_ptr<JointKinematics> PerceptionRuntime::kinematics()")]
        self.assertIn("std::unique_ptr<const detail::AttendedGeometry> attended_geometry;", source)
        self.assertIn('if (impl_->config.joint_kinematics.tf_mode == "attended_direct")', configure)
        construction = configure[configure.index("geometry=std::make_unique<const detail::AttendedGeometry>"):
                                 configure.index("  impl_->joints->mirror_into(")]
        self.assertIn("impl_->config.joint_kinematics", construction)
        self.assertIn("impl_->config.common_frame", construction)
        self.assertIn("impl_->config.sources[0].frame", construction)
        self.assertIn("impl_->config.sources[1].frame", construction)
        self.assertNotIn("global", construction)
        self.assertNotIn("buffer", construction)
        self.assertLess(configure.index("impl_->attended_geometry=std::move(geometry)"),
                        configure.index("impl_->direct_tf_configured.store(true)"))
        decode = source[source.index("  Output decode("):source.index("  Output decode_and_deliver(")]
        self.assertEqual(decode.count("const auto deadline=fk_end+20000000LL"), 1)
        self.assertIn("config.joint_kinematics.tf_mode,index,stamp(f->source_stamp),query.query_stamp_ns", decode)
        camera = decode[decode.index("        const auto camera=[&]"):
                        decode.index("        tf_detail.camera_end_ns=")]
        spatial = decode[decode.index("        const auto rigid=[&]"):
                         decode.index("        return std::make_pair(")]
        for operation, field in ((camera, "optical_from_base"), (spatial, "common_from_optical")):
            fixed = operation[operation.index("if (fixed) {"):operation.index("return fixed->" + field)]
            self.assertIn("(void)dm::remaining(deadline)", fixed)
        self.assertIn("global->lookupTransform(", camera)
        self.assertIn("sunhorizon_rgbd_adapter::spatial_source_transform(*global", spatial)
        # Completion and source-time FK remain mandatory for either geometry path.
        self.assertLess(decode.index("wait_fk(f,was_qualified)"), decode.index("attended_geometry->select("))
        self.assertIn("model.observations(camera,base_from_links,stamp(f->source_stamp))", decode)

    def test_measured_mode_preserves_neutral_asset_and_cannot_claim_production(self):
        config = PACKAGE.parent / 'sunhorizon_nav2_bringup/config'
        asset = config / 'robot_a_prototype_sensor_tf.attended-heading-correction.json'
        receipt = config / 'robot_a_prototype_sensor_tf.attended-heading-correction.receipt.json'
        options = dict(tf_mode='attended_measured_direct', attended_usage='attended_heading_correction',
            robot_serial='1421026005616', attended_tf_asset_path=str(asset),
            attended_tf_receipt_path=str(receipt),
            attended_tf_asset_sha256=hashlib.sha256(asset.read_bytes()).hexdigest(),
            attended_tf_receipt_sha256=hashlib.sha256(receipt.read_bytes()).hexdigest())
        self.assertEqual(MODULE.startup_tf_mode(options), 'attended_measured_direct')
        self.assertEqual(MODULE.validated_attended_edges(options), json.loads(asset.read_text())['transforms'])
        for key in ('attended_usage', 'attended_tf_asset_path', 'attended_tf_receipt_sha256'):
            invalid = dict(options); invalid.pop(key)
            with self.assertRaisesRegex(ValueError, 'attended_direct_evidence_required'):
                MODULE.startup_tf_mode(invalid)
        invalid = dict(options, production_tf_receipt_path='/not-a-production-receipt')
        with self.assertRaisesRegex(ValueError, 'must_not_claim_production_acceptance'):
            MODULE.startup_tf_mode(invalid)

    def test_actual_file_sha_and_canonical_path_are_required(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary).resolve() / "receipt.json"
            path.write_bytes(b"{}")
            digest = hashlib.sha256(b"{}").hexdigest()
            self.assertEqual(MODULE.pinned_bytes(str(path), digest), b"{}")
            with self.assertRaisesRegex(ValueError, "sha_mismatch"):
                MODULE.pinned_bytes(str(path), "0" * 64)
            alias = path.with_name("alias.json")
            alias.symlink_to(path)
            with self.assertRaisesRegex(ValueError, "canonical_regular"):
                MODULE.pinned_bytes(str(alias), digest)

    def test_pinned_accept_boolean_does_not_bypass_existing_receipt_validator(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary).resolve() / "fake_receipt.json"
            raw = json.dumps({"accepted_for_platform_freshness": True}).encode()
            path.write_bytes(raw)
            digest = hashlib.sha256(raw).hexdigest()
            with self.assertRaisesRegex(ValueError, "live_joint_state_receipt_shape_invalid"):
                MODULE.validate_startup({
                    "source_receipt_path": str(path), "source_receipt_sha256": digest,
                    "mapping_receipt_path": str(path), "mapping_receipt_sha256": digest,
                    "robot_serial": "1421026005616", "release_sha": "a" * 40,
                    "source_review_authority_sha256": "b" * 64,
                    "mapping_review_authority_sha256": "c" * 64})


if __name__ == "__main__":
    unittest.main()
