import os

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bumperbot_tools_dir = get_package_share_directory("bumperbot_tools")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                bumperbot_tools_dir,
                "rviz",
                "display.rviz",
            ),
        ],
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        rviz_node,
    ])