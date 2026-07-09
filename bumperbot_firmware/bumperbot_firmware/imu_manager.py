#! /usr/bin/env python3

from rclpy.lifecycle import LifecycleNode
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from system_messages import *


class ImuManager:
    """Manages ROS 2 lifecycle publishing, calibration configuration, and frame

    translation for IMU sensor data.
    """

    def __init__(self, node: LifecycleNode, write_serial_cb):
        self.node = node
        self.write_serial_cb = write_serial_cb
        self.pub = None
        self.is_configured = False
        self.frame_id = "imu_link"

    def configure(self):
        """Transmits ImuConfigMsg commanding hardware gyro/accel calibration."""
        self.is_configured = False
        self.frame_id = self.node.get_parameter("imu_frame_id").value
        calib_ms = self.node.get_parameter("imu_calibration_ms").value

        config_msg = ImuConfigMsg(
            calibrate_period_ms=int(calib_ms),
            result=False,  # Master sends False; MCU echoes True upon success
        )

        self.node.get_logger().info(f"Commanding IMU calibration ({calib_ms} ms)...")
        try:
            self.write_serial_cb(config_msg.serialize())
        except Exception as e:
            self.node.get_logger().error(f"Failed to send ImuConfigMsg: {e}")

    def validate_config(self, msg: ImuConfigMsg) -> bool:
        """Validates that the microcontroller successfully completed calibration."""
        if msg.result:
            self.is_configured = True
            self.node.get_logger().info("IMU sensor calibration verified by hardware.")
        else:
            self.node.get_logger().error("Hardware reported IMU calibration failure!")

        return self.is_configured

    def activate(self):
        self.pub = self.node.create_lifecycle_publisher(
            Imu,
            "/bumperbot/imu",
            qos_profile_sensor_data,
        )
        self.node.get_logger().info("IMU interface activated.")

    def deactivate(self):
        if self.pub:
            self.node.destroy_publisher(self.pub)
            self.pub = None
        self.node.get_logger().info("IMU interface deactivated.")

    def publish_state(self, imu_data: ImuStateData, status: int):
        if not self.pub:
            return

        if status & SystemStateFlags.ImuUnavailable:
            self.node.get_logger().warning(
                "Hardware flagged IMU sensor as unavailable.", once=True
            )
            return

        imu_msg = Imu()
        imu_msg.header.stamp = self.node.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        imu_msg.angular_velocity.x = float(imu_data.angular_velocity_x)
        imu_msg.angular_velocity.y = float(imu_data.angular_velocity_y)
        imu_msg.angular_velocity.z = float(imu_data.angular_velocity_z)

        imu_msg.linear_acceleration.x = float(imu_data.linear_acceleration_x)
        imu_msg.linear_acceleration.y = float(imu_data.linear_acceleration_y)
        imu_msg.linear_acceleration.z = float(imu_data.linear_acceleration_z)

        imu_msg.orientation_covariance[0] = -1.0
        imu_msg.angular_velocity_covariance[0] = 0.01
        imu_msg.linear_acceleration_covariance[0] = 0.01

        self.pub.publish(imu_msg)
