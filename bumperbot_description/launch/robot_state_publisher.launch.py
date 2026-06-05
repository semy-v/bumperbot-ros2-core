import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    bumperbot_description_dir = get_package_share_directory("bumperbot_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            bumperbot_description_dir, "urdf", "bumperbot.urdf.xacro"
        ),
        description="Absolute path to robot URDF file",
    )
    is_sim_arg = DeclareLaunchArgument(
        name="is_sim",
        default_value="True",
    )

    model = LaunchConfiguration("model")
    is_sim = LaunchConfiguration("is_sim")

    robot_description = ParameterValue(
        Command(["xacro ", model, " is_sim:=", is_sim]), value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    def propagate_exit_code(event, context):
        exit_code = event.returncode
        sys.exit(exit_code)

    exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_publisher, on_exit=propagate_exit_code
        )
    )

    return LaunchDescription(
        [model_arg, is_sim_arg, robot_state_publisher, exit_handler]
    )
