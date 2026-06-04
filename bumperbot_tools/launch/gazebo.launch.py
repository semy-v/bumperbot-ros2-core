from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from pathlib import Path


def generate_launch_description():
    bumperbot_description_dir = get_package_share_directory("bumperbot_description")

    robot_state_publisher = IncludeLaunchDescription(
        os.path.join(
            bumperbot_description_dir,
            "launch",
            "robot_state_publisher.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(bumperbot_description_dir).parent.resolve())
        ]
    )

    gazebo_partition = SetEnvironmentVariable(
        name="GZ_PARTITION",
        value="default"
    )

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
        "/gz_sim.launch.py"
        ]),
        launch_arguments=[
            ("gz_args", [" -v 4", " -r", " empty.sdf"])
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "bumperbot"]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
        ],
        remappings=[
            ("/imu", "bumperbot/imu")
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_resource_path,
        gazebo_partition,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])