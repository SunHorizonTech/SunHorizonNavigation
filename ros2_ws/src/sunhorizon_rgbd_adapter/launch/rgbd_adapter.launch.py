import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = get_package_share_directory("sunhorizon_rgbd_adapter")
    default_params = os.path.join(package, "config", "rgbd_adapter.yaml")
    default_fastdds_profile = os.path.join(
        package, "config", "robot_a_fastdds.xml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument(
            "fastdds_profile",
            default_value=default_fastdds_profile,
            description="Orin camera transport profile; must be set before ROS participant init",
        ),
        GroupAction(actions=[
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            UnsetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS"),
            SetEnvironmentVariable(
                "FASTRTPS_DEFAULT_PROFILES_FILE",
                LaunchConfiguration("fastdds_profile"),
            ),
            Node(
                package="sunhorizon_rgbd_adapter",
                executable="rgbd_adapter_node",
                name="sunhorizon_rgbd_adapter",
                parameters=[LaunchConfiguration("params_file")],
                output="screen",
            ),
        ], scoped=True),
    ])
