import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    # controller = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("bumperbot_controller"),
    #         "launch",
    #         "controller.launch.py"
    #     ),
    #     launch_arguments={
    #         "use_simple_controller": "False",
    #         "use_python": "False"
    #     }.items(),
    # )
    
    diff_drive_controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "diff_drive_controller.launch.py"
        )
    )
    
    joint_state_broadcaster = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joint_state_broadcaster.launch.py"
        )
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )
    
    return LaunchDescription([
        gazebo,
        diff_drive_controller,
        joint_state_broadcaster,
        joystick
    ])