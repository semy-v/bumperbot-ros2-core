#! /usr/bin/env python3

import threading
import serial
import time
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from lifecycle_msgs.msg import State
from rclpy.qos import qos_profile_sensor_data

from bumperbot_msgs.msg import TwoWheelsAngularVelocity
from diff_drive_messages import *


class SerialTransceiverNode(LifecycleNode):
    def __init__(self):
        # Initialize as a Lifecycle Node
        super().__init__("serial_transceiver")

        # Connection params
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        # Config params (Matching diff_drive_data.hpp ConfigData)
        self.declare_parameter("pid_rate", 25.0)
        self.declare_parameter("r_wheel_kp", 6.0)
        self.declare_parameter("r_wheel_ki", 25.0)
        self.declare_parameter("r_wheel_kd", 0.0)
        self.declare_parameter("r_wheel_deadband", 21)
        self.declare_parameter("l_wheel_kp", 7.2)
        self.declare_parameter("l_wheel_ki", 30.0)
        self.declare_parameter("l_wheel_kd", 0.0)
        self.declare_parameter("l_wheel_deadband", 23)

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        # Interface placeholders
        self.sub_ = None
        self.pub_ = None
        self.transceiver_ = None

        # Threading controls
        self.read_thread_ = None
        self.stop_read_thread_ = threading.Event()
        self.rx_buffer_ = bytearray()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Handle the serial handshake and hardware initialization."""
        self.get_logger().info("Configuring serial port connection...")
        try:
            self.transceiver_ = serial.Serial(self.port_, self.baudrate_, timeout=1.0)

            # Wait for Arduino bootloader reset sequence to complete
            time.sleep(2.0)

            # Fetch current parameters
            pid_rate = self.get_parameter("pid_rate").value
            r_kp = self.get_parameter("r_wheel_kp").value
            r_ki = self.get_parameter("r_wheel_ki").value
            r_kd = self.get_parameter("r_wheel_kd").value
            r_db = self.get_parameter("r_wheel_deadband").value

            l_kp = self.get_parameter("l_wheel_kp").value
            l_ki = self.get_parameter("l_wheel_ki").value
            l_kd = self.get_parameter("l_wheel_kd").value
            l_db = self.get_parameter("l_wheel_deadband").value

            self.get_logger().info(
                f"Sending ConfigData: pid_rate {pid_rate}, "
                f"r_wheel = (kp {r_kp} | ki {r_ki} | kd {r_kd} | deadband {r_db}), "
                f"l_wheel = (kp {l_kp} | ki {l_ki} | kd {l_kd} | deadband {l_db})"
            )

            # Construct and pack ConfigMsg frame binary structure
            config_msg = ConfigMsg(
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

            self.rx_buffer_.clear()

            self.transceiver_.write(config_msg.serialize())
            self.transceiver_.flush()

            start_time = time.time()
            while time.time() - start_time < 5.0:
                parsed_msg = self.parse_incomming_msg()
                if isinstance(parsed_msg, ConfigMsg):
                    # Data successfully mirrored back by Arduino
                    self.get_logger().info(
                        "Hardware configured successfully. Ready for activation."
                    )
                    return TransitionCallbackReturn.SUCCESS
                time.sleep(0.01)

            self.get_logger().error("No valid Config response message received")
            return TransitionCallbackReturn.FAILURE

        except Exception as e:
            self.get_logger().error(f"Failed to configure hardware interface: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Create managed communication topics and launch active workers."""
        self.get_logger().info("Activating ROS interfaces and background processing...")

        # Use lifecycle node variants for publishers
        self.pub_ = self.create_lifecycle_publisher(
            TwoWheelsAngularVelocity,
            "/bumperbot/wheels_velocity_out",
            qos_profile_sensor_data,
        )
        self.sub_ = self.create_subscription(
            TwoWheelsAngularVelocity,
            "/bumperbot/wheels_velocity_in",
            self.velocity_cmd_callback,
            qos_profile_sensor_data,
        )

        # Clear flags and spin up background serial worker thread
        self.stop_read_thread_.clear()
        self.rx_buffer_.clear()
        self.read_thread_ = threading.Thread(target=self.receive_msg_loop, daemon=True)
        self.read_thread_.start()

        # Lifecycle management requires calling super() or manually activating the lifecycle publisher
        self.get_logger().info("Interfaces activated successfully.")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Halt execution workers safely and drop connection queues."""
        self.get_logger().info("Deactivating transceiver workers...")

        # Halt background threads safely
        self.stop_read_thread_.set()
        if self.read_thread_ and self.read_thread_.is_alive():
            self.read_thread_.join()

        # Clean up communication interfaces
        self.destroy_subscription(self.sub_)
        self.destroy_publisher(self.pub_)

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Close communication handles completely."""
        self.get_logger().info("Cleaning up resource handles...")
        if self.transceiver_ and self.transceiver_.is_open:
            self.transceiver_.close()
        return TransitionCallbackReturn.SUCCESS

    def velocity_cmd_callback(self, msg: TwoWheelsAngularVelocity):
        # Guard clause ensures commands are ignored if the lifecycle state is not active
        vel_msg = VelocityMsg(
            right_wheel_velocity=msg.right_wheel_velocity,
            left_wheel_velocity=msg.left_wheel_velocity,
        )
        try:
            if self.transceiver_ and self.transceiver_.is_open:
                self.transceiver_.write(vel_msg.serialize())
                self.transceiver_.flush()
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error during active tracking: {e}")

    def parse_incomming_msg(self):
        if self.transceiver_ and self.transceiver_.in_waiting:
            self.rx_buffer_.extend(
                self.transceiver_.read(self.transceiver_.in_waiting or 1)
            )
            return self.parse_buffer()
        return bytearray()

    def receive_msg_loop(self):
        while not self.stop_read_thread_.is_set():
            try:
                parsed_msg = self.parse_incomming_msg()

                if isinstance(parsed_msg, VelocityMsg):
                    out_msg = TwoWheelsAngularVelocity()
                    out_msg.right_wheel_velocity = float(
                        parsed_msg.right_wheel_velocity
                    )
                    out_msg.left_wheel_velocity = float(parsed_msg.left_wheel_velocity)

                    # Lifecycle publishers safely guard calls based on internal active flags
                    self.pub_.publish(out_msg)
            except Exception as e:
                self.get_logger().error(
                    f"Exception inside background execution loop: {e}"
                )
                time.sleep(0.1)

    def parse_buffer(self):
        """Processes the buffer, validates CRC, and returns instantiated Dataclasses."""
        while len(self.rx_buffer_) >= 4:
            if self.rx_buffer_[0] != START_BYTE:
                self.rx_buffer_.pop(0)
                continue

            msg_id = self.rx_buffer_[1]
            payload_len = self.rx_buffer_[2]
            expected_crc = self.rx_buffer_[3]

            frame_len = 4 + payload_len

            if len(self.rx_buffer_) >= frame_len:
                payload = self.rx_buffer_[4:frame_len]
                header_no_crc = self.rx_buffer_[:3]
                calculated_crc = calculate_lrc8(header_no_crc) ^ calculate_lrc8(payload)

                if calculated_crc == expected_crc:
                    frame_bytes = bytes(self.rx_buffer_[:frame_len])
                    del self.rx_buffer_[:frame_len]  # Consume bytes

                    # Factory: Route to correct Dataclass deserializer
                    if msg_id == MsgId.Config:
                        return ConfigMsg.deserialize(frame_bytes)
                    elif msg_id == MsgId.Velocity:
                        return VelocityMsg.deserialize(frame_bytes)
                    elif msg_id == MsgId.Deactivate:
                        return DeactivateMsg.deserialize(frame_bytes)
                else:
                    self.get_logger().warn("CRC Error during deserialization.")
                    self.rx_buffer_.pop(0)  # Force re-sync
            else:
                break

        return None


def main(args=None):
    rclpy.init(args=args)
    node = SerialTransceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
