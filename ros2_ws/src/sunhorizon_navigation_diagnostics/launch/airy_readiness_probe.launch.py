from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                description="Absolute path to an Airy readiness probe parameter file",
            ),
            Node(
                package="sunhorizon_navigation_diagnostics",
                executable="airy_readiness_probe",
                name="airy_readiness_probe",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
