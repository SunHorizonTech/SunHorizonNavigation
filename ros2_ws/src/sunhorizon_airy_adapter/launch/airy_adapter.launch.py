from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("sunhorizon_airy_adapter"),
        "config",
        "airy_adapter.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=config),
        DeclareLaunchArgument(
            "reliable_sensor_qos",
            default_value="false",
            description="Use reliable transport for the owned Airy point and IMU streams"),
        Node(
            package="sunhorizon_airy_adapter",
            executable="airy_adapter_node",
            name="sunhorizon_airy_adapter",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "reliable_sensor_qos": ParameterValue(
                        LaunchConfiguration("reliable_sensor_qos"),
                        value_type=bool),
                },
            ],
            output="screen",
        )
    ])
