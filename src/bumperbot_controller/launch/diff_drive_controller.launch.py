import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bumperbot_controllers = os.path.join(
        get_package_share_directory("bumperbot_controller"),
        "config",
        "bumperbot_controllers.yaml",
    )

    bumperbot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--param-file", bumperbot_controllers],
    )

    def propagate_exit_code(event, context):
        exit_code = event.returncode
        sys.exit(exit_code)

    exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=bumperbot_controller_spawner,
            on_exit=propagate_exit_code
        )
    )

    return LaunchDescription([bumperbot_controller_spawner, exit_handler])
