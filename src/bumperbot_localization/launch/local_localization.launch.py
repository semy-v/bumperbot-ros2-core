import os
import sys
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0.103",
            "--qx",
            "1",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "0",
            "--frame-id",
            "base_footprint",
            "--child-frame-id",
            "imu_link",
        ],
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("bumperbot_localization"),
                "config",
                "ekf.yaml",
            )
        ],
    )

    def propagate_exit_code(event, context):
        exit_code = event.returncode
        sys.exit(exit_code)

    exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_localization, on_exit=propagate_exit_code
        )
    )

    return LaunchDescription(
        [static_transform_publisher, robot_localization, exit_handler]
    )
