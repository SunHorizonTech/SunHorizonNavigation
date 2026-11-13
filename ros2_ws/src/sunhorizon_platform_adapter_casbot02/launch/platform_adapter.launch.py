from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("sunhorizon_platform_adapter_casbot02"),
        "config",
        "platform_adapter.yaml",
    )
    return LaunchDescription([
        Node(
            package="sunhorizon_platform_adapter_casbot02",
            executable="platform_adapter_node",
            name="sunhorizon_platform_adapter",
            parameters=[config],
            output="screen",
        ),
        Node(
            package="sunhorizon_platform_adapter_casbot02",
            executable="ethercat_evidence_node",
            name="sunhorizon_ethercat_evidence",
            parameters=[config],
            output="screen",
        ),
        Node(
            package="sunhorizon_platform_adapter_casbot02",
            executable="vendor_velocity_adapter_node",
            name="sunhorizon_vendor_velocity_adapter",
            parameters=[config],
            output="screen",
        ),
    ])
