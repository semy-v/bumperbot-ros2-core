import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    motion_server_node = LifecycleNode(
        package="bumperbot_motion",
        executable="motion_control_server",
        name="motion_control_server",
        parameters=[
            os.path.join(
                get_package_share_directory("bumperbot_motion"),
                "config",
                "motion_control.yaml")
        ],
        namespace=""
    )

    motion_server_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(motion_server_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    motion_server_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=motion_server_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(motion_server_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        motion_server_node,
        motion_server_configure_event,
        motion_server_activate_event
    ])