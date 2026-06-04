import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    bumperbot_controllers = os.path.join(
        get_package_share_directory("bumperbot_controller"),
        "config",
        "bumperbot_controllers.yaml"
    )
    
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--param-file", bumperbot_controllers],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--param-file", bumperbot_controllers],
    )

    return LaunchDescription(
        [
            diff_drive_controller_spawner,
            joint_state_broadcaster_spawner
        ]
    )