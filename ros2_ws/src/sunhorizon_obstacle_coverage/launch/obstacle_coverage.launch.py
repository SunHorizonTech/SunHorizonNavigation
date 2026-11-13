from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            description="Absolute path to one obstacle coverage profile YAML file",
        ),
        Node(
            package="sunhorizon_obstacle_coverage",
            executable="obstacle_coverage_node",
            name="sunhorizon_obstacle_coverage",
            parameters=[params_file],
            output="screen",
        ),
    ])
