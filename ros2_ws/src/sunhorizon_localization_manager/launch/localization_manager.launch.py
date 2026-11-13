import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = get_package_share_directory("sunhorizon_localization_manager")
    default_params = os.path.join(package, "config", "localization_manager.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("active_backend", default_value="lidar"),
        DeclareLaunchArgument("comparison_enabled", default_value="true"),
        DeclareLaunchArgument(
            "odometry_timeout_sec", default_value="0.50",
            description="Maximum local receipt gap for correction odometry"),
        DeclareLaunchArgument(
            "tracking_odometry_timeout_sec", default_value="0.25",
            description="Maximum local receipt gap for canonical tracking"),
        DeclareLaunchArgument(
            "map_pose_timeout_sec", default_value="0.50",
            description="Maximum local receipt gap for committed map poses"),
        DeclareLaunchArgument(
            "backend_status_timeout_sec", default_value="0.50",
            description="Maximum local receipt gap for committed status pairs"),
        DeclareLaunchArgument(
            "maximum_measurement_age_sec", default_value="0.50",
            description="Maximum header age at callback receipt"),
        DeclareLaunchArgument(
            "correction_filter_time_constant_sec", default_value="0.0"),
        Node(
            package="sunhorizon_localization_manager",
            executable="localization_manager_node",
            name="sunhorizon_localization_manager",
            parameters=[LaunchConfiguration("params_file"), {
                "active_backend": LaunchConfiguration("active_backend"),
                "comparison_enabled": ParameterValue(
                    LaunchConfiguration("comparison_enabled"), value_type=bool
                ),
                "odometry_timeout_sec": ParameterValue(
                    LaunchConfiguration("odometry_timeout_sec"), value_type=float
                ),
                "tracking_odometry_timeout_sec": ParameterValue(
                    LaunchConfiguration("tracking_odometry_timeout_sec"),
                    value_type=float,
                ),
                "map_pose_timeout_sec": ParameterValue(
                    LaunchConfiguration("map_pose_timeout_sec"), value_type=float
                ),
                "backend_status_timeout_sec": ParameterValue(
                    LaunchConfiguration("backend_status_timeout_sec"),
                    value_type=float,
                ),
                "maximum_measurement_age_sec": ParameterValue(
                    LaunchConfiguration("maximum_measurement_age_sec"),
                    value_type=float,
                ),
                "correction_filter_time_constant_sec": ParameterValue(
                    LaunchConfiguration("correction_filter_time_constant_sec"),
                    value_type=float,
                ),
            }],
            output="screen",
        ),
    ])
