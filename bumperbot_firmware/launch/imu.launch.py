import os
import sys
import launch
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    imu_node = LifecycleNode(
        package="bumperbot_firmware",
        executable="mpu6050_node.py",
        name="imu_node",
        namespace=""
    )

    configure_imu_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(imu_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_imu_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=imu_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(imu_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription([       
        imu_node,
        configure_imu_event,
        activate_imu_event
    ])