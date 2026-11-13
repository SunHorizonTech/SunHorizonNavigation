import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


EXPECTED_ROBOT_SERIAL = "1421026005616"


def _contract(package_share):
    path = os.path.join(package_share, "internal_optical_tf_contract.py")
    spec = importlib.util.spec_from_file_location(
        "sunhorizon_camera_internal_optical_tf_contract", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("camera_internal_tf_validator_unavailable")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _compose(context):
    package_share = get_package_share_directory("sunhorizon_camera_runtime")
    params_file = Path(LaunchConfiguration("params_file").perform(context).strip())
    receipt_text = LaunchConfiguration("internal_optical_tf_receipt").perform(context).strip()
    expected_serial = LaunchConfiguration("expected_robot_serial").perform(context).strip()
    parameters = [str(params_file)]
    if receipt_text:
        try:
            receipt_digest = _contract(package_share).validate(
                Path(receipt_text), params_file, expected_serial)
        except (OSError, UnicodeError, ValueError) as error:
            raise RuntimeError(f"camera_internal_tf_contract_rejected:{error}") from error
        parameters.append({
            "internal_optical_tf_acceptance_receipt_sha256": receipt_digest,
        })
    elif expected_serial != EXPECTED_ROBOT_SERIAL:
        raise RuntimeError("camera_expected_robot_serial_invalid")

    transport_profile = LaunchConfiguration("transport_profile").perform(context).strip()
    return [GroupAction(actions=[
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            UnsetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS"),
            SetEnvironmentVariable(
                "FASTRTPS_DEFAULT_PROFILES_FILE", transport_profile),
            Node(
                package="sunhorizon_camera_runtime",
                executable="sunhorizon_camera_runtime_node",
                name="sunhorizon_camera_runtime",
                parameters=parameters,
                output="screen",
            ),
        ], scoped=True)]


def generate_launch_description():
    package_share = get_package_share_directory("sunhorizon_camera_runtime")
    default_params = os.path.join(package_share, "config", "robot_a.yaml")
    default_transport_profile = os.path.join(
        get_package_share_directory("sunhorizon_rgbd_adapter"),
        "config", "robot_a_fastdds.xml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("internal_optical_tf_receipt", default_value=""),
        DeclareLaunchArgument(
            "expected_robot_serial", default_value=EXPECTED_ROBOT_SERIAL),
        DeclareLaunchArgument("transport_profile", default_value=default_transport_profile),
        OpaqueFunction(function=_compose),
    ])
