from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import importlib.util
import os
from pathlib import Path


def _load_mapping_contract():
    path = os.path.join(os.path.dirname(__file__), "..", "contract", "leg_joint_mapping_contract.py")
    spec = importlib.util.spec_from_file_location("leg_joint_mapping_contract", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("leg_joint_mapping_contract_unavailable")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_live_joint_source_contract():
    path = os.path.join(
        os.path.dirname(__file__), "..", "contract",
        "live_joint_state_source_contract.py",
    )
    spec = importlib.util.spec_from_file_location("live_joint_state_source_contract", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("live_joint_state_source_contract_unavailable")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _platform_nodes(
    context, params_file, vendor_joint_log_path, mapping_receipt,
    live_joint_state_source_receipt, live_joint_state_bridge_owner,
):
    bridge_owner = context.perform_substitution(live_joint_state_bridge_owner).strip()
    if bridge_owner not in ("platform", "perception_runtime"):
        raise RuntimeError("invalid_live_joint_state_bridge_owner")
    receipt_path = context.perform_substitution(mapping_receipt).strip()
    mapping = _load_mapping_contract().launch_ros_mapping_parameters(receipt_path)
    platform_mapping = {
        "joint_mapping_valid": mapping["leg_joint_name_mapping_verified"],
    }
    joint_log_mapping = {
        key: mapping[key] for key in (
            "leg_joint_name_mapping_verified", "leg_joint_name_mapping_receipt_path",
            "leg_joint_name_mapping_receipt_sha256",
            "leg_joint_name_mapping_evidence_sha256",
        )
    }
    if "leg_joint_names" in mapping:
        platform_mapping["leg_joint_names"] = mapping["leg_joint_names"]
        joint_log_mapping["leg_joint_names"] = mapping["leg_joint_names"]
    stop_group = Shutdown(reason="platform evidence process exited")
    nodes = []
    live_receipt_path = context.perform_substitution(
        live_joint_state_source_receipt).strip()
    if bridge_owner == "perception_runtime" and not live_receipt_path:
        raise RuntimeError("perception_joint_owner_requires_original_signed_receipts")
    if live_receipt_path:
        if not mapping["leg_joint_name_mapping_verified"]:
            raise RuntimeError(
                "live_joint_state_source_requires_accepted_leg_joint_mapping")
        live_source = _load_live_joint_source_contract().validate_live_joint_state_receipt(
            Path(live_receipt_path),
            expected_robot_serial=os.environ.get("SUNHORIZON_ROBOT_SERIAL", ""),
            expected_release_sha=os.environ.get(
                "SUNHORIZON_PLATFORM_EVIDENCE_RELEASE_SHA", ""),
            expected_review_authority_sha256=os.environ.get(
                "SUNHORIZON_LIVE_JOINT_REVIEW_AUTHORITY_SHA256", ""),
        )
        if (
            live_source["selected_source_joint_names"][:12]
            != mapping["source_joint_names"]
            or live_source["position_unit"] != mapping["position_unit"]
            or live_source["value_transform"] != mapping["value_transform"]
        ):
            raise RuntimeError("live_joint_state_position_semantics_mapping_mismatch")
        # Ownership is selected before launch. No old bridge process or
        # on_exit handler exists in fusion mode; the original proof FQN is
        # owned exclusively by JointIngress inside the perception process.
        if bridge_owner == "platform":
            nodes.append(Node(
                package="sunhorizon_platform_adapter_casbot02",
                executable="live_joint_state_bridge_node",
                name="sunhorizon_live_joint_state_bridge",
                parameters=[{
                    "source_contract_accepted": True,
                    "source_topic": live_source["source_topic"],
                    "source_type": live_source["source_type"],
                    "source_owner_name": live_source["source_owner_name"],
                    "source_owner_namespace": live_source["source_owner_namespace"],
                    "runtime_binding": live_source["runtime_binding"],
                    "expected_source_joint_names": live_source["expected_source_joint_names"],
                    "selected_source_joint_names": live_source["selected_source_joint_names"],
                    "selected_source_joint_indices": live_source[
                        "selected_source_joint_indices"],
                    "output_joint_names": [*mapping["leg_joint_names"], "waist_yaw"],
                    "position_unit": live_source["position_unit"],
                    "value_transform": live_source["value_transform"],
                    "output_topic": live_source["output_topic"],
                    "maximum_source_age_sec": 0.25,
                    "source_future_tolerance_sec": 0.05,
                    "source_owner_discovery_grace_sec": 12.0,
                    "source_receipt_path": live_source["receipt_path"],
                    "source_receipt_sha256": live_source["receipt_sha256"],
                    "source_evidence_sha256": live_source["evidence_sha256"],
                }],
                output="screen",
                on_exit=stop_group,
            ))
    nodes.extend([
        Node(
            package="sunhorizon_platform_adapter_casbot02",
            executable="platform_adapter_node",
            name="sunhorizon_platform_adapter",
            parameters=[params_file, platform_mapping],
            output="screen",
            on_exit=stop_group,
        ),
        Node(
            package="sunhorizon_platform_adapter_casbot02",
            executable="ethercat_evidence_node",
            name="sunhorizon_ethercat_evidence",
            parameters=[params_file],
            output="screen",
            on_exit=stop_group,
        ),
        Node(
            package="sunhorizon_platform_adapter_casbot02",
            executable="joint_log_evidence_node",
            name="sunhorizon_joint_log_evidence",
            parameters=[params_file, {"vendor_joint_log_path": vendor_joint_log_path},
                        joint_log_mapping],
            output="screen",
            on_exit=stop_group,
        ),
    ])
    return nodes


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("sunhorizon_platform_adapter_casbot02"),
        "config",
        "platform_adapter.yaml",
    )
    params_file = LaunchConfiguration("params_file")
    vendor_joint_log_path = LaunchConfiguration("vendor_joint_log_path")
    mapping_receipt = LaunchConfiguration("leg_joint_mapping_receipt")
    live_joint_state_source_receipt = LaunchConfiguration(
        "live_joint_state_source_receipt")
    live_joint_state_bridge_owner = LaunchConfiguration("live_joint_state_bridge_owner")
    return LaunchDescription([
        DeclareLaunchArgument(
            "vendor_joint_log_path",
            description="Absolute path to the active Robot A EtherCAT station log",
        ),
        DeclareLaunchArgument("params_file", default_value=config),
        DeclareLaunchArgument(
            "leg_joint_mapping_receipt",
            default_value="",
            description=(
                "Accepted Robot A canonical JointState-to-URDF name mapping and "
                "raw EtherCAT fault-attribution receipt"
            ),
        ),
        DeclareLaunchArgument(
            "live_joint_state_source_receipt",
            default_value="",
            description=(
                "Accepted Robot A /joint_states source identity/rate receipt; "
                "empty leaves high-rate joint freshness fail-closed"
            ),
        ),
        DeclareLaunchArgument(
            "live_joint_state_bridge_owner", default_value="platform",
            choices=["platform", "perception_runtime"],
            description=("Select the sole proof joint owner; perception_runtime retains "
                         "receipt validation and the other platform evidence processes, "
                         "but never starts the legacy bridge"),
        ),
        OpaqueFunction(
            function=_platform_nodes,
            args=[params_file, vendor_joint_log_path, mapping_receipt,
                  live_joint_state_source_receipt, live_joint_state_bridge_owner],
        ),
    ])
