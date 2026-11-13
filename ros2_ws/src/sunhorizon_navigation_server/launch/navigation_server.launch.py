from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("sunhorizon_navigation_server"),
        "config",
        "navigation_server.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "provider_prefix",
            default_value="/sunhorizon/providers/lidar",
            description="Selected lifecycle provider; must end in /visual or /lidar",
        ),
        DeclareLaunchArgument("map_root", default_value="/data/navigation/maps"),
        DeclareLaunchArgument(
            "operator_goal_adapter_enabled",
            default_value="false",
            description="Opt in to the RViz PoseStamped-to-NavigateTo adapter",
        ),
        DeclareLaunchArgument(
            "operator_goal_topic",
            default_value="/sunhorizon/navigation/operator_goal",
            description="PoseStamped topic used only when the operator adapter is enabled",
        ),
        DeclareLaunchArgument(
            "operator_initial_pose_adapter_enabled",
            default_value="false",
            description="Opt in to RViz initial pose through the Localize service",
        ),
        DeclareLaunchArgument(
            "operator_initial_pose_topic",
            default_value="/initialpose",
        ),
        DeclareLaunchArgument("startup_map_id", default_value=""),
        Node(
            package="sunhorizon_navigation_server",
            executable="navigation_server_node",
            name="sunhorizon_navigation_server",
            parameters=[config, {
                "provider_prefix": LaunchConfiguration("provider_prefix"),
                "map_root": LaunchConfiguration("map_root"),
                "operator_goal_adapter_enabled": ParameterValue(
                    LaunchConfiguration("operator_goal_adapter_enabled"),
                    value_type=bool,
                ),
                "operator_goal_topic": LaunchConfiguration("operator_goal_topic"),
                "operator_initial_pose_adapter_enabled": ParameterValue(
                    LaunchConfiguration("operator_initial_pose_adapter_enabled"),
                    value_type=bool,
                ),
                "operator_initial_pose_topic": LaunchConfiguration(
                    "operator_initial_pose_topic"),
                "startup_map_id": LaunchConfiguration("startup_map_id"),
            }],
            output="screen",
        )
    ])
