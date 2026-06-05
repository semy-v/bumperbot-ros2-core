#!/usr/bin/env python3

import rclpy
import sys
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from mpu6050_driver import MPU6050


class MPU6050Node(LifecycleNode):

    def __init__(self):
        super().__init__("mpu6050_node")

        self.declare_parameter("frame_id", "base_footprint")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("calibration_period_ms", 1000)

        self._imu = MPU6050()

        self._publisher = None
        self._timer = None

        self._imu_msg = Imu()

        self.get_logger().info("MPU6050 lifecycle node created.")

    # ========================================================
    # LIFECYCLE
    # ========================================================

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            self._imu.connect()
            self._calibrate_imu()
        except OSError as ex:
            self.get_logger().error(f"Failed to initialize MPU6050: {ex}")
            return TransitionCallbackReturn.ERROR

        self._initialize_imu_message()

        self._publisher = self.create_lifecycle_publisher(
            Imu,
            "bumperbot/imu",
            qos_profile_sensor_data,
        )

        publish_rate = self.get_parameter("publish_rate").value

        self._timer = self.create_timer(
            1.0 / publish_rate,
            self._sensor_read_loop,
        )

        self._timer.cancel()

        self.get_logger().info("MPU6050 configured.")

        return TransitionCallbackReturn.SUCCESS


    def on_activate(self, state: State) -> TransitionCallbackReturn:
        super().on_activate(state)
        self._timer.reset()

        self.get_logger().info("MPU6050 activated.")

        return TransitionCallbackReturn.SUCCESS


    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        super().on_deactivate(state)

        if self._timer:
            self._timer.cancel()

        return TransitionCallbackReturn.SUCCESS


    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self._timer:
            self.destroy_timer(self._timer)
            self._timer = None

        if self._publisher:
            self.destroy_publisher(self._publisher)
            self._publisher = None

        self._imu.disconnect()

        return TransitionCallbackReturn.SUCCESS
    

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().fatal(f"Error occurred in state: {state}. Exiting process...")
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # ========================================================
    # SENSOR READ LOOP
    # ========================================================

    def _sensor_read_loop(self):
        try:
            data = self._imu.read_calibrated()
        except OSError as ex:
            self.get_logger().error(f"MPU6050 read failed: {ex}")
            raise

        msg = self._imu_msg

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.linear_acceleration.x = data.accel_x
        msg.linear_acceleration.y = data.accel_y
        msg.linear_acceleration.z = data.accel_z

        msg.angular_velocity.x = data.gyro_x
        msg.angular_velocity.y = data.gyro_y
        msg.angular_velocity.z = data.gyro_z

        if self._publisher.is_activated:
            self._publisher.publish(msg)

    # ========================================================
    # ROS HELPERS
    # ========================================================

    def _initialize_imu_message(self):
        self._imu_msg.header.frame_id = self.get_parameter("frame_id").value

        #
        # Orientation unavailable
        #
        self._imu_msg.orientation_covariance[0] = -1.0

        #
        # Angular Velocity Covariance (Diagonal elements)
        # Standard MPU6050 gyro noise is roughly 0.005 rad/s
        #
        gyro_var = 0.005
        self._imu_msg.angular_velocity_covariance[0] = gyro_var
        self._imu_msg.angular_velocity_covariance[4] = gyro_var
        self._imu_msg.angular_velocity_covariance[8] = gyro_var

        #
        # Linear Acceleration Covariance (Diagonal elements)
        # Accel noise is typically higher, roughly 0.01 m/s^2
        #
        accel_var = 0.01
        self._imu_msg.linear_acceleration_covariance[0] = accel_var
        self._imu_msg.linear_acceleration_covariance[4] = accel_var
        self._imu_msg.linear_acceleration_covariance[8] = accel_var


    def _calibrate_imu(self):
        sample_interval_ms = 5
        calibration_period_ms = self.get_parameter(
            "calibration_period_ms").get_parameter_value().integer_value
        sample_num = calibration_period_ms // sample_interval_ms
        self.get_logger().info(
            "Calibrating IMU... Keep robot still at flat surface for %.3f seconds."
            % (calibration_period_ms / 1000.0)
        )
        calibration = self._imu.calibrate(sample_num, sample_interval_ms)

        self.get_logger().info(
            "Calibration:"
            f" accel=({calibration.accel_x:.4f}, "
            f"{calibration.accel_y:.4f}, "
            f"{calibration.accel_z:.4f})"
            f" gyro=({calibration.gyro_x:.4f}, "
            f"{calibration.gyro_y:.4f}, "
            f"{calibration.gyro_z:.4f})"
        )
        

# ============================================================
# MAIN
# ============================================================

def main(args=None):
    rclpy.init(args=args)

    node = MPU6050Node()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to KeyboardInterrupt.")
    except Exception as ex:
        node.get_logger().fatal(f"Node crashed with an unhandled exception: {ex}")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
