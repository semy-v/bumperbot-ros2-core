import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg

def generate_launch_description():
    pkg_share = get_package_share_directory("bumperbot_firmware")

    # Construct the full path to the configuration yaml file
    config_file_path = PathJoinSubstitution([
        pkg_share, "config", "test", "serial_transceiver.yaml"
    ])

    # Define Serial Transceiver as a Lifecycle Node
    serial_transceiver_node = LifecycleNode(
        package="bumperbot_firmware",
        executable="serial_transceiver_node.py",
        name="serial_transceiver",
        namespace="",
        output="screen",
        parameters=[config_file_path]
    )

    # Request automatic transition to 'configure' immediately on launch
    trigger_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == serial_transceiver_node,
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
        )
    )

    # Once configured ('inactive' state reached), trigger 'activate'
    trigger_activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=serial_transceiver_node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == serial_transceiver_node,
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        serial_transceiver_node,
        trigger_configure_event,
        trigger_activate_handler
    ])