"""One startup bridge to the existing signed Robot A contracts; no ROS I/O."""
from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ATTENDED_DIRECT_MODES = ("attended_direct", "attended_measured_direct")


def _module(package: str, relative: str):
    candidate = Path(__file__).resolve().parents[2] / package / relative
    if not candidate.is_file():
        from ament_index_python.packages import get_package_share_directory
        candidate = Path(get_package_share_directory(package)) / relative
    spec = importlib.util.spec_from_file_location("joint_startup_" + package + relative.replace("/", "_"), candidate)
    if spec is None or spec.loader is None:
        raise ValueError("joint_startup_contract_module_missing")
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def pinned_bytes(filename: str, digest: str) -> bytes:
    path = Path(filename)
    if not path.is_absolute() or path.resolve(strict=True) != path or not path.is_file():
        raise ValueError("joint_startup_asset_not_canonical_regular")
    before = path.stat()
    if before.st_size <= 0 or before.st_size > 16 * 1024 * 1024:
        raise ValueError("joint_startup_asset_size")
    value = path.read_bytes()
    after = path.stat()
    if (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) != (
        after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns
    ) or len(value) != before.st_size or hashlib.sha256(value).hexdigest() != digest:
        raise ValueError("joint_startup_asset_changed_or_sha_mismatch")
    return value


def merge_fk_models(legacy_xml: bytes, production_xml: str, leg_names: list[str]) -> dict:
    """Called only after both asset/acceptance validators succeed."""
    legacy, production = ET.fromstring(legacy_xml), ET.fromstring(production_xml)
    if len(leg_names) != 12 or len(set(leg_names)) != 12:
        raise ValueError("joint_startup_leg_mapping_shape")
    old_joints = {joint.get("name"): joint for joint in legacy.findall("joint")}
    new_joints = {joint.get("name"): joint for joint in production.findall("joint")}
    if len(old_joints) != len(legacy.findall("joint")) or len(new_joints) != len(production.findall("joint")):
        raise ValueError("joint_startup_duplicate_joint")
    source_waist = old_joints.get("waist_yaw_joint")
    output_waist = new_joints.get("waist_yaw")
    if source_waist is None or output_waist is None:
        raise ValueError("joint_startup_explicit_waist_alias_missing")
    def waist_geometry(joint):
        # The production TF validator independently checks the same vendor
        # origin/type/axis. Name alias is the sole intended naming difference.
        return (joint.get("type"), joint.find("parent").get("link"), joint.find("child").get("link"),
                tuple(float(v) for v in joint.find("origin").get("xyz").split()),
                tuple(float(v) for v in joint.find("origin").get("rpy").split()),
                tuple(float(v) for v in joint.find("axis").get("xyz").split()))
    if waist_geometry(source_waist) != waist_geometry(output_waist):
        raise ValueError("joint_startup_waist_geometry_mismatch")
    parents = {joint.find("child").get("link"): joint for joint in legacy.findall("joint")}
    retained = {}
    targets = []
    for name in leg_names:
        if name not in old_joints:
            raise ValueError("joint_startup_leg_joint_missing")
        joint = old_joints[name]
        targets.append(joint.find("child").get("link"))
        cursor = targets[-1]
        visited = set()
        while cursor != "base_link":
            if cursor in visited or cursor not in parents:
                raise ValueError("joint_startup_leg_path_not_tree")
            visited.add(cursor)
            edge = parents[cursor]
            if edge.get("type") != "fixed" and edge.get("name") not in leg_names:
                raise ValueError("joint_startup_unobserved_leg_joint")
            if edge.find("mimic") is not None:
                raise ValueError("joint_startup_mimic_unsupported")
            retained[edge.get("name")] = edge
            cursor = edge.find("parent").get("link")
    links = {link.get("name") for link in production.findall("link")}
    children = {joint.find("child").get("link") for joint in production.findall("joint")}
    def canonical(element):
        return (element.tag, tuple(sorted(element.attrib.items())),
                tuple(canonical(child) for child in element))
    for name, edge in sorted(retained.items()):
        child = edge.find("child").get("link")
        if name in new_joints and canonical(new_joints[name]) == canonical(edge):
            continue  # An already merged, byte-bound leg edge has one owner.
        if name in new_joints or child in children:
            raise ValueError("joint_startup_merged_owner_collision")
        children.add(child)
        if child not in links:
            # Kinematics needs link identity only. Mesh assets continue to use
            # the unmodified legacy URDF and its original pinned SHA.
            ET.SubElement(production, "link", name=child)
            links.add(child)
        production.append(copy.deepcopy(edge))
    production.set("name", "robot_a_validated_perception_kinematics")
    xml = ET.tostring(production, encoding="unicode")
    return {"merged_urdf_xml": xml, "merged_urdf_sha256": hashlib.sha256(xml.encode()).hexdigest(),
            "leg_target_frames": targets,
            "joint_name_aliases": {"waist_yaw_joint": "waist_yaw"}}


def attended_fk_model(legacy_xml: bytes, leg_names: list[str]) -> dict:
    """Original signed vendor geometry only; grants no production TF acceptance."""
    legacy = ET.fromstring(legacy_xml)
    joints = legacy.findall("joint")
    waist = next((joint for joint in joints if joint.get("name") == "waist_yaw_joint"), None)
    if waist is None or waist.find("parent").get("link") != "base_link":
        raise ValueError("joint_startup_attended_waist_not_base_relative")
    # Copy only the observed waist edge. merge_fk_models copies all original
    # leg path edges verbatim and checks the explicit waist alias geometry.
    observed = ET.Element("robot", name="robot_a_attended_vendor_fk")
    for name in ("base_link", waist.find("child").get("link")):
        ET.SubElement(observed, "link", name=name)
    edge = copy.deepcopy(waist)
    edge.set("name", "waist_yaw")
    observed.append(edge)
    return merge_fk_models(legacy_xml, ET.tostring(observed, encoding="unicode"), leg_names)


def startup_tf_mode(options: dict) -> str:
    mode = options.get("tf_mode", "")
    if mode not in ("attended_global_tf", *ATTENDED_DIRECT_MODES, "production_direct"):
        raise ValueError("joint_startup_tf_mode_invalid")
    if mode != "production_direct" and (
        options.get("production_tf_receipt_path") or options.get("production_tf_receipt_sha256")
    ):
        # Do not ignore a partially supplied or invalid production credential.
        raise ValueError("joint_startup_attended_mode_must_not_claim_production_acceptance")
    if mode == "production_direct" and (
        not options.get("production_tf_receipt_path") or not options.get("production_tf_receipt_sha256")
    ):
        raise ValueError("joint_startup_production_tf_receipt_required")
    attended_fields=("attended_usage", "attended_tf_asset_path", "attended_tf_asset_sha256",
                     "attended_tf_receipt_path", "attended_tf_receipt_sha256")
    if mode in ATTENDED_DIRECT_MODES and (any(not options.get(key) for key in attended_fields) or
        options.get("attended_usage") != "attended_heading_correction"):
        raise ValueError("joint_startup_attended_direct_evidence_required")
    if mode not in ATTENDED_DIRECT_MODES and any(options.get(key) for key in attended_fields):
        raise ValueError("joint_startup_attended_evidence_mode_mismatch")
    return mode


def append_accepted_fixed_edges(xml: str, edges: list[tuple]) -> str:
    root = ET.fromstring(xml)
    links = {link.get("name") for link in root.findall("link")}
    children = {joint.find("child").get("link") for joint in root.findall("joint")}
    for parent, child, translation, rpy in edges:
        if child in children:
            raise ValueError("joint_startup_fixed_edge_owner_collision")
        children.add(child)
        for name in (parent, child):
            if name not in links:
                ET.SubElement(root, "link", name=name)
                links.add(name)
        joint = ET.SubElement(root, "joint", name="accepted_fixed_" + child, type="fixed")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "origin", xyz=" ".join(map(str, translation)), rpy=" ".join(map(str, rpy)))
    return ET.tostring(root, encoding="unicode")


def validated_attended_edges(options: dict) -> list[dict]:
    attended = _module("sunhorizon_platform_adapter_casbot02", "contract/attended_sensor_tf_contract.py")
    # An arbitrary new JSON receipt cannot grant the existing prototype
    # permission. Pin the exact accepted assets already used by the owner.
    if (options["attended_tf_asset_sha256"] != attended.ATTENDED_HEADING_ASSET_SHA256 or
        options["attended_tf_receipt_sha256"] != attended.ATTENDED_HEADING_RECEIPT_SHA256):
        raise ValueError("joint_startup_attended_original_asset_pin_required")
    pinned_bytes(options["attended_tf_asset_path"],options["attended_tf_asset_sha256"])
    pinned_bytes(options["attended_tf_receipt_path"],options["attended_tf_receipt_sha256"])
    edges = attended.validate_attended_sensor_tf(
        Path(options["attended_tf_asset_path"]),Path(options["attended_tf_receipt_path"]),
        options["attended_usage"],options["robot_serial"])
    pinned_bytes(options["attended_tf_asset_path"],options["attended_tf_asset_sha256"])
    pinned_bytes(options["attended_tf_receipt_path"],options["attended_tf_receipt_sha256"])
    return edges


def validate_startup(options: dict) -> dict:
    source_bytes = pinned_bytes(options["source_receipt_path"], options["source_receipt_sha256"])
    mapping_bytes = pinned_bytes(options["mapping_receipt_path"], options["mapping_receipt_sha256"])
    source_contract = _module("sunhorizon_platform_adapter_casbot02", "contract/live_joint_state_source_contract.py")
    mapping_contract = _module("sunhorizon_platform_adapter_casbot02", "contract/leg_joint_mapping_contract.py")
    source = source_contract.validate_live_joint_state_receipt(
        Path(options["source_receipt_path"]), expected_robot_serial=options["robot_serial"],
        expected_release_sha=options["release_sha"],
        expected_review_authority_sha256=options["source_review_authority_sha256"])
    mapping = mapping_contract.validate_mapping_receipt(
        Path(options["mapping_receipt_path"]), expected_robot_serial=options["robot_serial"],
        expected_release_sha=options["release_sha"],
        expected_review_authority_sha256=options["mapping_review_authority_sha256"])
    if (source["selected_source_joint_names"][:12] != mapping["source_joint_names"] or
        source["position_unit"] != mapping["position_unit"] or
        source["value_transform"] != mapping["value_transform"] or
        source["evidence_sha256"] != options["source_evidence_sha256"] or
        source["receipt_sha256"] != options["source_receipt_sha256"] or
        mapping["receipt_sha256"] != options["mapping_receipt_sha256"]):
        raise ValueError("joint_startup_source_mapping_receipt_mismatch")
    mode = startup_tf_mode(options)
    result = {"source": source, "mapping": mapping, "tf_mode": mode,
              "output_joint_names": [*mapping["leg_joint_names"], "waist_yaw"]}
    if mode in ATTENDED_DIRECT_MODES:
        # Both candidates bind the original neutral geometry. The measured
        # mode replaces only the two body/waist edges in the C++ owner; it does
        # not grant production calibration or mutate the accepted asset.
        result["attended_static_edges"] = validated_attended_edges(options)
    if options.get("legacy_urdf_path"):
        legacy = pinned_bytes(options["legacy_urdf_path"], options["legacy_urdf_sha256"])
        mapping_document = json.loads(mapping_bytes)
        if mapping_document["vendor_urdf"]["sha256"] != options["legacy_urdf_sha256"]:
            raise ValueError("joint_startup_legacy_model_not_mapping_bound")
        if mode in ("attended_global_tf", *ATTENDED_DIRECT_MODES):
            result.update(attended_fk_model(legacy, mapping["leg_joint_names"]))
        else:
            production_bytes = pinned_bytes(options["production_tf_receipt_path"], options["production_tf_receipt_sha256"])
            production_receipt = json.loads(production_bytes)
            assets = production_receipt["assets"]
            paths = {name: Path(asset["path"]) for name, asset in assets.items()}
            tf_contract = _module("sunhorizon_platform_adapter_casbot02", "contract/production_sensor_tf_contract.py")
            production_xml = tf_contract.validate(
                Path(options["production_tf_receipt_path"]), paths["robot_urdf"], paths["platform_base_tf_config"],
                paths["measurement_record"], paths["camera_params"], paths["camera_internal_tf_receipt"],
                paths["airy_adapter_config"], paths["airy_frame_receipt"], expected_serial=options["robot_serial"])
            # These separate production owners previously published through TF.
            # Their accepted geometry becomes part of the single in-memory model.
            platform = tf_contract.validated_platform_transform(paths["platform_base_tf_config"])
            cameras = tf_contract.validated_camera_transforms(paths["camera_internal_tf_receipt"], paths["camera_params"])
            edges = [("base_footprint", "base_link", *platform)]
            for role, geometry in cameras.items():
                edges.append((*tf_contract.CAMERA_FRAMES[role], *geometry))
            production_xml = append_accepted_fixed_edges(production_xml, edges)
            result.update(merge_fk_models(legacy, production_xml, mapping["leg_joint_names"]))
            pinned_bytes(options["production_tf_receipt_path"], options["production_tf_receipt_sha256"])
    # The whole validation is over the same source/mapping receipt bytes.
    if pinned_bytes(options["source_receipt_path"], options["source_receipt_sha256"]) != source_bytes or \
       pinned_bytes(options["mapping_receipt_path"], options["mapping_receipt_sha256"]) != mapping_bytes:
        raise ValueError("joint_startup_receipt_changed_during_validation")
    return result
