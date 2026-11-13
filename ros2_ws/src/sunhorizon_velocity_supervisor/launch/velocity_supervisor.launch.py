from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("sunhorizon_velocity_supervisor"),
        "config",
        "velocity_supervisor.yaml",
    )
    return LaunchDescription([
        Node(
            package="sunhorizon_velocity_supervisor",
            executable="velocity_supervisor_node",
            name="sunhorizon_velocity_supervisor",
            parameters=[config],
            output="screen",
        )
    ])
