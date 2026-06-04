#! /usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node
from bumperbot_msgs.msg import TwoWheelsAngularVelocity
from rclpy.qos import qos_profile_sensor_data


class VelocityCmdTestRunnerNode(Node):
    def __init__(self):
        super().__init__("velocity_cmd_test_runner")

        # Declare ROS 2 parameters
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("publish_rate", 100.0)  # Hz

        self.publish_rate_ = self.get_parameter("publish_rate").value
        scenario_file_path = self.get_parameter("scenario_file").value

        # Setup publisher
        self.pub_ = self.create_publisher(
            TwoWheelsAngularVelocity,
            "/bumperbot/wheels_velocity_in",
            qos_profile_sensor_data,
        )

        # State management tracking
        self.steps_ = []
        self.current_step_idx_ = 0
        self.step_elapsed_time_ = 0.0
        self.timer_period_ = 1.0 / self.publish_rate_
        self.timer_ = None

        # Load and parse the test steps
        if not scenario_file_path:
            self.get_logger().error(
                "Parameter 'scenario_file' is empty! Please provide a valid YAML path."
            )
            return

        if not os.path.exists(scenario_file_path):
            self.get_logger().error(
                f"Scenario configuration file not found at: {scenario_file_path}"
            )
            return

        if self.load_scenario(scenario_file_path):
            self.get_logger().info(
                f"Loaded scenario successfully with {len(self.steps_)} steps."
            )
            self.get_logger().info(
                f"Starting test runner execution at {self.publish_rate_} Hz..."
            )

            # Create execution loop timer
            self.timer_ = self.create_timer(self.timer_period_, self.timer_callback)

    def load_scenario(self, file_path: str) -> bool:
        try:
            with open(file_path, "r") as file:
                config_data = yaml.safe_load(file)

                # Check for alternative override of publish rate in scenario file
                if config_data and "publish_rate" in config_data:
                    self.publish_rate_ = float(config_data["publish_rate"])
                    self.timer_period_ = 1.0 / self.publish_rate_
                    self.get_logger().info(
                        f"Overriding publish rate from file config: {self.publish_rate_} Hz"
                    )

                if config_data and "steps" in config_data:
                    self.steps_ = config_data["steps"]
                    if not self.steps_:
                        self.get_logger().error(
                            "The loaded scenario file contains an empty steps list."
                        )
                        return False
                    return True
                else:
                    self.get_logger().error(
                        "Invalid scenario format. Root key 'steps' missing."
                    )
                    return False
        except Exception as e:
            self.get_logger().error(f"Failed to read/parse scenario configuration: {e}")
            return False

    def timer_callback(self):
        # Stop condition: check if we executed all steps
        if self.current_step_idx_ >= len(self.steps_):
            self.get_logger().info("Test scenario completed! Stopping robot safely.")
            self.publish_stop_command()
            if self.timer_:
                self.timer_.cancel()
            # Shut down the node execution gracefully
            rclpy.shutdown()
            return

        current_step = self.steps_[self.current_step_idx_]

        try:
            right_vel = float(current_step["right_wheel_velocity"])
            left_vel = float(current_step["left_wheel_velocity"])
            duration = float(current_step["duration"])
        except KeyError as e:
            self.get_logger().error(
                f"Missing required parameter {e} in step index {self.current_step_idx_}. Skipping step."
            )
            self.current_step_idx_ += 1
            self.step_elapsed_time_ = 0.0
            return

        # Construct and fill TwoWheelsAngularVelocity message fields
        msg = TwoWheelsAngularVelocity()
        msg.right_wheel_velocity = right_vel
        msg.left_wheel_velocity = left_vel

        # Publish to the serial receiver target
        self.pub_.publish(msg)

        # Track elapsed step execution window
        self.step_elapsed_time_ += self.timer_period_

        # Transition validation
        if self.step_elapsed_time_ >= duration:
            self.get_logger().info(
                f"Finished step {self.current_step_idx_ + 1}/{len(self.steps_)} "
                f"(R: {right_vel:.2f} rad/s, L: {left_vel:.2f} rad/s) for {duration}s"
            )
            self.current_step_idx_ += 1
            self.step_elapsed_time_ = 0.0

    def publish_stop_command(self):
        msg = TwoWheelsAngularVelocity()
        msg.right_wheel_velocity = 0.0
        msg.left_wheel_velocity = 0.0
        self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityCmdTestRunnerNode()
    try:
        if rclpy.ok() and node.timer_ is not None:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency catch if killed early: attempt to send zero-speeds
        try:
            node.publish_stop_command()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
