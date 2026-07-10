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

        pid_rate = self.node.get_parameter("pid_rate").value
        r_kp = self.node.get_parameter("r_wheel_kp").value
        r_ki = self.node.get_parameter("r_wheel_ki").value
        r_kd = self.node.get_parameter("r_wheel_kd").value
        r_db = self.node.get_parameter("r_wheel_deadband").value

        l_kp = self.node.get_parameter("l_wheel_kp").value
        l_ki = self.node.get_parameter("l_wheel_ki").value
        l_kd = self.node.get_parameter("l_wheel_kd").value
        l_db = self.node.get_parameter("l_wheel_deadband").value

        self._expected_config = DiffDriveConfigMsg(
            pid_rate=pid_rate,
            r_wheel_kp=r_kp,
            r_wheel_ki=r_ki,
            r_wheel_kd=r_kd,
            r_wheel_pwm_deadband=r_db,
            l_wheel_kp=l_kp,
            l_wheel_ki=l_ki,
            l_wheel_kd=l_kd,
            l_wheel_pwm_deadband=l_db,
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

        # Verify mirrored parameters match what we transmitted
        matches = (
            abs(msg.pid_rate - self._expected_config.pid_rate) < 1e-3
            and abs(msg.r_wheel_kp - self._expected_config.r_wheel_kp) < 1e-3
            and abs(msg.r_wheel_ki - self._expected_config.r_wheel_ki) < 1e-3
            and abs(msg.r_wheel_kd - self._expected_config.r_wheel_kd) < 1e-3
            and msg.r_wheel_pwm_deadband == self._expected_config.r_wheel_pwm_deadband
            and abs(msg.l_wheel_kp - self._expected_config.l_wheel_kp) < 1e-3
            and abs(msg.l_wheel_ki - self._expected_config.l_wheel_ki) < 1e-3
            and abs(msg.l_wheel_kd - self._expected_config.l_wheel_kd) < 1e-3
            and msg.l_wheel_pwm_deadband == self._expected_config.l_wheel_pwm_deadband
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
