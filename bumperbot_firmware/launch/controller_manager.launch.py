import os
import sys
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bumperbot_description_dir = get_package_share_directory("bumperbot_description")

    robot_state_publisher = IncludeLaunchDescription(
        os.path.join(
            bumperbot_description_dir,
            "launch",
            "robot_state_publisher.launch.py"
        ),
        launch_arguments={
            "is_sim": "False"
        }.items()
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node"
    )

    def propagate_exit_code(event, context):
        exit_code = event.returncode
        sys.exit(exit_code)

    exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager,
            on_exit=propagate_exit_code
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        exit_handler
    ])