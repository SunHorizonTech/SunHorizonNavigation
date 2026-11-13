"""Offline LIVO backend only; the caller owns recorded-data playback."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory("sunhorizon_livo_bringup")
    online_launch = os.path.join(package_share, "launch", "livo.launch.py")
    default_config = os.path.join(package_share, "config", "livo_robot_a.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="mapping"),
        DeclareLaunchArgument("config_path", default_value=default_config),
        DeclareLaunchArgument("vendor_root"),
        DeclareLaunchArgument("vendor_executable"),
        DeclareLaunchArgument("vendor_params_path"),
        DeclareLaunchArgument("camera_model_path"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(online_launch),
            launch_arguments={
                "mode": LaunchConfiguration("mode"),
                "use_sim_time": "true",
                "reliable_sensor_qos": "true",
                "reliable_odometry_qos": "true",
                "reliable_input_image_qos": "true",
                "reliable_image_qos": "true",
                "direct_camera_input": "false",
                "config_path": LaunchConfiguration("config_path"),
                "vendor_root": LaunchConfiguration("vendor_root"),
                "vendor_executable": LaunchConfiguration("vendor_executable"),
                "vendor_params_path": LaunchConfiguration("vendor_params_path"),
                "camera_model_path": LaunchConfiguration("camera_model_path"),
            }.items(),
        ),
    ])
