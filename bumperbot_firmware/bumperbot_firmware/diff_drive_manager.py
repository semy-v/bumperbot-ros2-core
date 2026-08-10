#! /usr/bin/env python3

from rclpy.lifecycle import LifecycleNode
from rclpy.qos import qos_profile_sensor_data
from bumperbot_msgs.msg import TwoWheelsAngularVelocity
from system_messages import *


class DiffDriveManager:
    """Manages ROS 2 interfaces, parameter configuration, and wire translation

    for Differential Drive wheel velocities.
    """

    def __init__(self, node: LifecycleNode, write_serial_cb):
        self.node = node
        self.write_serial_cb = write_serial_cb
        self.pub = None
        self.sub = None
        self.is_configured = False
        self._expected_config = None

    def configure(self):
        """Fetches parameters, transmits DiffDriveConfigMsg, and resets state."""
        self.is_configured = False

        def parameter(name: str):
            return self.node.get_parameter(name).value

        self._expected_config = DiffDriveConfigMsg(
            right_feedforward_ks=parameter("right_wheel.feedforward_ks"),
            right_feedforward_kv=parameter("right_wheel.feedforward_kv"),
            right_feedback_kp=parameter("right_wheel.feedback_kp"),
            right_feedback_ki=parameter("right_wheel.feedback_ki"),
            right_feedback_kd=parameter("right_wheel.feedback_kd"),
            right_max_feedback_pwm=parameter("right_wheel.max_feedback_pwm"),
            left_feedforward_ks=parameter("left_wheel.feedforward_ks"),
            left_feedforward_kv=parameter("left_wheel.feedforward_kv"),
            left_feedback_kp=parameter("left_wheel.feedback_kp"),
            left_feedback_ki=parameter("left_wheel.feedback_ki"),
            left_feedback_kd=parameter("left_wheel.feedback_kd"),
            left_max_feedback_pwm=parameter("left_wheel.max_feedback_pwm"),
            control_rate_hz=parameter("control_rate_hz"),
        )

        self.node.get_logger().info("Transmitting DiffDriveConfigMsg...")
        try:
            self.write_serial_cb(self._expected_config.serialize())
        except Exception as e:
            self.node.get_logger().error(f"Failed to send DiffDriveConfigMsg: {e}")

    def validate_config(self, msg: DiffDriveConfigMsg) -> bool:
        """Validates mirrored hardware configuration against expected parameters."""
        if not self._expected_config:
            return False

        expected = self._expected_config

        def floats_match(actual: float, configured: float) -> bool:
            return abs(actual - configured) < 1e-3

        matches = (
            floats_match(msg.right_feedforward_ks, expected.right_feedforward_ks)
            and floats_match(msg.right_feedforward_kv, expected.right_feedforward_kv)
            and floats_match(msg.right_feedback_kp, expected.right_feedback_kp)
            and floats_match(msg.right_feedback_ki, expected.right_feedback_ki)
            and floats_match(msg.right_feedback_kd, expected.right_feedback_kd)
            and msg.right_max_feedback_pwm == expected.right_max_feedback_pwm
            and floats_match(msg.left_feedforward_ks, expected.left_feedforward_ks)
            and floats_match(msg.left_feedforward_kv, expected.left_feedforward_kv)
            and floats_match(msg.left_feedback_kp, expected.left_feedback_kp)
            and floats_match(msg.left_feedback_ki, expected.left_feedback_ki)
            and floats_match(msg.left_feedback_kd, expected.left_feedback_kd)
            and msg.left_max_feedback_pwm == expected.left_max_feedback_pwm
            and msg.control_rate_hz == expected.control_rate_hz
        )

        if matches:
            self.is_configured = True
            self.node.get_logger().info(
                "Differential drive hardware handshake verified."
            )
        else:
            self.node.get_logger().error(
                "Differential drive parameter mismatch in hardware echo!"
            )

        return self.is_configured

    def activate(self):
        self.pub = self.node.create_lifecycle_publisher(
            TwoWheelsAngularVelocity,
            "/bumperbot/wheels_velocity_out",
            qos_profile_sensor_data,
        )
        self.sub = self.node.create_subscription(
            TwoWheelsAngularVelocity,
            "/bumperbot/wheels_velocity_in",
            self.velocity_cmd_callback,
            qos_profile_sensor_data,
        )
        self.node.get_logger().info("DiffDrive interface activated.")

    def deactivate(self):
        if self.sub:
            self.node.destroy_subscription(self.sub)
            self.sub = None
        if self.pub:
            self.node.destroy_publisher(self.pub)
            self.pub = None
        self.node.get_logger().info("DiffDrive interface deactivated.")

    def velocity_cmd_callback(self, msg: TwoWheelsAngularVelocity):
        cmd_msg = DiffDriveCommandMsg(
            right_wheel_velocity=msg.right_wheel_velocity,
            left_wheel_velocity=msg.left_wheel_velocity,
            response_delay_ms=5,
        )
        try:
            self.write_serial_cb(cmd_msg.serialize())
        except Exception as e:
            self.node.get_logger().error(f"Failed to transmit DiffDriveCommandMsg: {e}")

    def publish_state(self, diff_drive_data: DiffDriveVelocityData):
        if self.pub:
            out_msg = TwoWheelsAngularVelocity()
            out_msg.right_wheel_velocity = float(diff_drive_data.right_wheel_velocity)
            out_msg.left_wheel_velocity = float(diff_drive_data.left_wheel_velocity)
            self.pub.publish(out_msg)
