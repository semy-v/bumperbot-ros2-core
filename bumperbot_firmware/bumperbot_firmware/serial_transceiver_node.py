#! /usr/bin/env python3

import threading
import serial
import time
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from lifecycle_msgs.msg import State

from system_messages import *
from diff_drive_manager import *
from imu_manager import *


class SerialTransceiverNode(LifecycleNode):
    """Lifecycle Node orchestrating hardware serial communication and dispatching

    incoming protocol frames to dedicated domain managers.
    """

    def __init__(self):
        super().__init__("serial_transceiver")

        # Connection & Frame params
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("imu_calibration_ms", 2000)

        # Config params matching C++ DiffDriveConfigData
        self.declare_parameter("pid_rate", 50.0)
        self.declare_parameter("r_wheel_kp", 15.5)
        self.declare_parameter("r_wheel_ki", 39.0)
        self.declare_parameter("r_wheel_kd", 0.0)
        self.declare_parameter("r_wheel_deadband", 17)
        self.declare_parameter("l_wheel_kp", 14.0)
        self.declare_parameter("l_wheel_ki", 43.0)
        self.declare_parameter("l_wheel_kd", 0.0)
        self.declare_parameter("l_wheel_deadband", 18)

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        # Instantiate Domain Managers
        self.diff_drive_manager_ = DiffDriveManager(self, self.send_serial_payload)
        self.imu_manager_ = ImuManager(self, self.send_serial_payload)

        # Communication interfaces
        self.transceiver_ = None
        self.read_thread_ = None
        self.stop_read_thread_ = threading.Event()
        self.rx_buffer_ = bytearray()

    def send_serial_payload(self, data: bytes):
        """Thread-safe serial transmission handler."""
        if self.transceiver_ and self.transceiver_.is_open:
            self.transceiver_.write(data)
            self.transceiver_.flush()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Handle the serial handshake and parallel hardware initialization."""
        self.get_logger().info("Configuring serial port connection...")
        try:
            self.transceiver_ = serial.Serial(self.port_, self.baudrate_, timeout=0.1)

            # Wait for Arduino bootloader reset sequence to finish
            time.sleep(2.0)
            self.rx_buffer_.clear()

            # Trigger parallel configuration transmission
            self.diff_drive_manager_.configure()
            self.imu_manager_.configure()

            # Calculate dynamic timeout based on IMU calibration period + safety buffer
            calib_ms = self.get_parameter("imu_calibration_ms").value
            timeout_sec = (calib_ms / 1000.0) + 3.0
            start_time = time.time()

            self.get_logger().info(
                f"Waiting up to {timeout_sec:.1f}s for hardware handshakes..."
            )

            while time.time() - start_time < timeout_sec:
                parsed_msg = self.parse_incoming_msg()

                if isinstance(parsed_msg, DiffDriveConfigMsg):
                    self.diff_drive_manager_.validate_config(parsed_msg)
                elif isinstance(parsed_msg, ImuConfigMsg):
                    self.imu_manager_.validate_config(parsed_msg)

                # Both responses must be successfully received to complete configuration
                if (
                    self.diff_drive_manager_.is_configured
                    and self.imu_manager_.is_configured
                ):
                    self.get_logger().info(
                        "All hardware subsystems configured successfully."
                    )
                    return TransitionCallbackReturn.SUCCESS

                time.sleep(0.01)

            # Detail which subsystem failed if timeout occurs
            if not self.diff_drive_manager_.is_configured:
                self.get_logger().error(
                    "Timeout: DiffDrive hardware configuration failed."
                )
            if not self.imu_manager_.is_configured:
                self.get_logger().error(
                    "Timeout: IMU calibration/configuration failed."
                )

            return TransitionCallbackReturn.FAILURE

        except Exception as e:
            self.get_logger().error(f"Failed to configure hardware interface: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate managers and launch background processing."""
        self.get_logger().info("Activating ROS interfaces and background workers...")

        self.diff_drive_manager_.activate()
        self.imu_manager_.activate()

        self.stop_read_thread_.clear()
        self.rx_buffer_.clear()
        self.read_thread_ = threading.Thread(target=self.receive_msg_loop, daemon=True)
        self.read_thread_.start()

        self.get_logger().info("Transceiver node activated successfully.")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Halt execution workers safely and drop connection queues."""
        self.get_logger().info("Deactivating transceiver workers...")

        self.stop_read_thread_.set()
        if self.read_thread_ and self.read_thread_.is_alive():
            self.read_thread_.join()

        self.diff_drive_manager_.deactivate()
        self.imu_manager_.deactivate()

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Close communication handles completely."""
        self.get_logger().info("Cleaning up resource handles...")
        if self.transceiver_ and self.transceiver_.is_open:
            self.transceiver_.close()
        return TransitionCallbackReturn.SUCCESS

    def parse_incoming_msg(self):
        if self.transceiver_ and self.transceiver_.in_waiting:
            self.rx_buffer_.extend(
                self.transceiver_.read(self.transceiver_.in_waiting or 1)
            )
            return self.parse_buffer()
        return None

    def receive_msg_loop(self):
        while not self.stop_read_thread_.is_set():
            try:
                parsed_msg = self.parse_incoming_msg()

                # Dispatch incoming composite state frame to respective managers
                if isinstance(parsed_msg, SystemStateMsg):
                    self.diff_drive_manager_.publish_state(parsed_msg.diff_drive)
                    self.imu_manager_.publish_state(parsed_msg.imu, parsed_msg.status)

            except Exception as e:
                self.get_logger().error(
                    f"Exception inside background execution loop: {e}"
                )
                time.sleep(0.1)

    def parse_buffer(self):
        """Processes the rx buffer, validates CRC16, and dispatches to deserializers."""
        while len(self.rx_buffer_) >= 5:
            if self.rx_buffer_[0] != START_BYTE:
                self.rx_buffer_.pop(0)
                continue

            msg_id = self.rx_buffer_[1]
            payload_len = self.rx_buffer_[2]

            expected_crc = int.from_bytes(
                self.rx_buffer_[3:5], byteorder="little", signed=False
            )

            frame_len = 5 + payload_len

            if len(self.rx_buffer_) >= frame_len:
                payload = self.rx_buffer_[5:frame_len]
                header_no_crc = self.rx_buffer_[:3]
                calculated_crc = calculate_crc16(header_no_crc + payload)

                if calculated_crc == expected_crc:
                    frame_bytes = bytes(self.rx_buffer_[:frame_len])
                    del self.rx_buffer_[:frame_len]

                    if msg_id in MESSAGE_DESERIALIZERS:
                        return MESSAGE_DESERIALIZERS[msg_id](frame_bytes)
                    else:
                        self.get_logger().warning(
                            f"Unknown MsgId received: 0x{msg_id:02X}"
                        )
                else:
                    self.get_logger().warning(
                        f"CRC16 check failed (Expected: 0x{expected_crc:04X}, Got: 0x{calculated_crc:04X})."
                    )
                    self.rx_buffer_.pop(0)
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
