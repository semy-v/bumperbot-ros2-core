import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition

def generate_launch_description():
    pkg_share = get_package_share_directory("bumperbot_firmware")

    scenario_file_name_arg = DeclareLaunchArgument(
        "scenario_file_name",
        default_value="pid_test_scenario.yaml",
        description="Name of the test scenario YAML file located inside config/test/"
    )

    full_scenario_path = PathJoinSubstitution([
        pkg_share, "config", "test", LaunchConfiguration("scenario_file_name")
    ])

    # Import and include the standalone serial transceiver launch sequence
    include_serial_transceiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "serial_transceiver.launch.py")
        )
    )

    # Define the Test Runner Execution Node
    velocity_test_runner_node = Node(
        package="bumperbot_firmware",
        executable="velocity_cmd_test_runner_node.py",
        name="velocity_cmd_test_runner",
        output="screen",
        parameters=[{"scenario_file": full_scenario_path}]
    )

    # Event Handler: Wait for the included transceiver to reach 'active' status.
    # This keeps our files decoupled since the Python node object is out of scope.
    trigger_test_runner_handler = RegisterEventHandler(
        OnStateTransition(
            goal_state="active",
            entities=[
                velocity_test_runner_node
            ]
        )
    )

    return LaunchDescription([
        scenario_file_name_arg,
        include_serial_transceiver,
        trigger_test_runner_handler
    ])