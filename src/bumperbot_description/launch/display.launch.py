# Copyright (c) Antonio Brandi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
# Notice: This file has been modified from the original work by Antonio Brandi.
# Changes: Reusage of a standalone robot_state_publisher launch description.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, Shutdown
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bumperbot_description_dir = get_package_share_directory("bumperbot_description")

    robot_state_publisher = IncludeLaunchDescription(
        os.path.join(
            bumperbot_description_dir,
            "launch",
            "robot_state_publisher.launch.py"
        )
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui", executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("bumperbot_description"),
                "rviz",
                "display.rviz",
            ),
        ],
        on_exit=Shutdown()
    )

    return LaunchDescription(
        [robot_state_publisher, joint_state_publisher_gui, rviz_node]
    )
