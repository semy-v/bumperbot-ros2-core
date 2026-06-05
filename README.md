# Bumperbot ROS 2 Core Stack

**ROS 2 software stack for **BumperBot**, an open-source, 3D-printed autonomous differential-drive mobile robot.

BumperBot is a complete, hardware-proven robotics platform built with modern C++20 and ROS 2 Jazzy. It features a modular architecture covering hardware abstraction, real-time control, localization, motion planning, and simulation — ready for both simulation and physical deployment on a Raspberry Pi 5 + Arduino Nano setup.

> **⚙️ Companion OS Repository** > This software stack is designed to be deployed natively onto a custom embedded Linux image. The Yocto Project configuration and board support package (BSP) used to build the OS for this robot can be found in the companion repository: **[meta-bumperbot](https://github.com/semy-v/meta-bumperbot)**.

---

## ✨ Features

- **Full ROS 2 Jazzy** integration with hardware and simulation support
- **Custom differential-drive motion control** with Nav2-compatible `FollowPath` action server and plugin-based PD Pure Pursuit controller
- **High-performance hardware interface** using a custom binary serial protocol between Raspberry Pi and Arduino
- **Sensor fusion** via EKF (wheel odometry + MPU6050 IMU)
- **ROS 2 Control** integration with velocity commands, joint states, and joystick teleoperation
- **Production-ready deployment** on a custom Yocto Linux image
- **Comprehensive Gazebo simulation** environment
- **Modern C++20** with strong typing, concepts, and clean modular architecture

## 📦 Packages

| Package                    | Purpose |
|---------------------------|--------|
| **bumperbot_bringup**     | Top-level launch files for simulation and real robot |
| **bumperbot_firmware**    | Arduino firmware, ROS 2 serial transceiver, IMU driver, and custom binary protocol |
| **bumperbot_motion**      | Custom motion control framework with PD Pure Pursuit plugin |
| **bumperbot_controller**  | ROS 2 Control configuration and teleoperation |
| **bumperbot_localization**| EKF-based state estimation |
| **bumperbot_description** | URDF/Xacro models, meshes, and Gazebo configuration |
| **bumperbot_msgs**        | Custom message definitions |
| **bumperbot_tools**       | RViz configurations, Gazebo utilities, and visualization helpers |

## 🤖 Hardware Architecture

Unlike simulation-only projects, this stack is actively deployed and proven on a custom physical hardware setup. The architecture utilizes the following components:

* **Main Compute:** Raspberry Pi 5
* **Low-Level Controller:** Arduino Nano (quadrature encoders + closed-loop motor PID)
* **Actuators:** DC motors with L298N H-bridge
* **Sensors:** MPU6050 IMU (I2C)
* **Interface:** Custom high-frequency binary serial protocol

---

## 🚀 Quick Start

### 1. Prerequisites

- ROS 2 Jazzy Jalisco ([Installation](https://docs.ros.org/en/jazzy/Installation.html))
- Ubuntu 24.04 (recommended)

### 2. Setup Workspace

```bash
# Create workspace
mkdir -p ~/bumperbot_ws/src
cd ~/bumperbot_ws/src

# Clone the repository

git clone https://github.com/semy-v/bumperbot-ros2-core.git

# Install dependencies
cd ..
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
```

### 3. Build

```bash
. /opt/ros/jazzy/setup.bash
colcon build
```

---

## 🎮 Running the Robot

### Simulation (Recommended for Development)

```bash
source install/setup.bash
ros2 launch bumperbot_bringup simulated_robot.launch.py
```

This launches:

- Gazebo simulation
- RViz visualization
- ROS 2 controllers
- Localization stack
- Motion control server

### Real Hardware

#### 1. Flash Arduino Firmware

Open and upload the following sketch to the Arduino Nano:

```text
bumperbot_firmware/arduino/diff_drive_controller/diff_drive_controller.ino
```

#### 2. Deploy Operating System

Build and flash the custom Yocto image from the **[meta-bumperbot](https://github.com/semy-v/meta-bumperbot)** repository.

#### 3. Launch on the Robot

Power on the robot. The system will boot, connect to the configured Wi-Fi network, and start the bumperbot packages as systemd services autonomously.

---

## 🧪 Development & Testing

- Individual launch files are available in each package for isolated testing.
- Firmware includes utilities for serial communication validation and PID tuning.
- `bumperbot_tools` provides resources for RViz and Gazebo development workflows.
- Components can be developed and tested independently using ROS 2 launch files and lifecycle-managed nodes.

---

## 🙏 Acknowledgements

This project was heavily inspired by the outstanding course:

**[Self Driving and ROS 2 – Learn by Doing! Odometry & Control](https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-odometry-control/)** by **Antonio Brandi**.

While the overall robot concept and some mechanical/URDF design elements were influenced by the course, **this repository is an independent implementation** developed from scratch. The entire software stack — including hardware interfaces, custom binary communication protocol, motion control framework, localization system, ROS 2 Control integration, and Yocto-based embedded Linux deployment — was designed and written independently.

Special thanks to Antonio Brandi for creating an excellent hands-on course that helped spark this project.