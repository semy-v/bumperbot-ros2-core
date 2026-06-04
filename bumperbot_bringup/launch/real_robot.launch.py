import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    controller_manager = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_firmware"),
            "launch",
            "controller_manager.launch.py"
        )
    )

    imu = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_firmware"),
            "launch",
            "imu.launch.py"
        )
    )

    controllers = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controllers.launch.py"
        )
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "local_localization.launch.py"
        )
    )

    motion = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_motion"),
            "launch",
            "motion_server.launch.py"
        )
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        )
    )
    
    return LaunchDescription([
        controller_manager,
        imu,
        controllers,
        localization,
        motion,
        joystick,
    ])