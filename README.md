# Bumperbot ROS 2 Core Stack

ROS 2 software stack for **BumperBot**, an open-source, 3D-printed autonomous differential-drive mobile robot.

The project is implemented with **ROS 2 Jazzy**, modern **C++20**, Python, and a modular embedded/ROS 2 architecture. The same stack supports both **Gazebo simulation** and deployment to the physical robot built around a **Raspberry Pi 5 + Arduino Nano ESP32**.

> **⚙️ Companion OS Repository**
>
> The ROS 2 stack is designed to run natively on a custom embedded Linux image. The Yocto Project configuration and BSP used to build that image are maintained in the companion repository:
>
> **[meta-bumperbot](https://github.com/semy-v/meta-bumperbot)**

---

## ✨ Features

- **ROS 2 Jazzy** integration for simulation and physical hardware
- **ROS 2 Control** based hardware integration through a custom `SystemInterface`
- **Custom USB binary communication protocol** between Raspberry Pi 5 and Arduino Nano ESP32
- **Closed-loop differential-drive wheel control** on the ESP32
- **Wheel velocity estimation** from motor FG pulse feedback
- **Configurable feed-forward + feedback wheel controllers** and diagnostic calibration utilities
- **MPU6050 IMU** integration over I2C
- **EKF-based localization** combining wheel odometry and IMU measurements
- **Custom motion-control framework** with plugin-based PD Pure Pursuit
- **Nav2-compatible `FollowPath` action server**
- **Square trajectory round-trip action** exposed through `bumperbot_msgs`
- **Joystick teleoperation**
- **Gazebo simulation** and RViz visualization
- **Custom embedded Linux deployment** using Yocto
- Modern **C++20** architecture using strong typing, concepts, and modular interfaces

---

## 🏗️ System Architecture

The current architecture is split into four major layers:

```text
┌──────────────────────────────────────────────────────────────────┐
│                         ROS 2 Application                        │
│                                                                  │
│  Motion Control Server   Localization / EKF   Teleoperation      │
│          │                       │                  │             │
│          ▼                       ▼                  ▼             │
│  FollowPath / Square     robot_localization      /cmd_vel        │
└──────────────────────────────┬───────────────────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────────────┐
│                    ROS 2 Control / Hardware                      │
│                                                                  │
│  controller_manager                                             │
│       │                                                          │
│       └── bumperbot_firmware::RobotSystemInterface               │
│              │              │                                    │
│              │              └── IMU sensor handler               │
│              │                                                   │
│              └── Differential-drive handler                     │
└──────────────────────────────┬───────────────────────────────────┘
                               │
                               │ custom binary protocol over USB
                               ▼
┌──────────────────────────────────────────────────────────────────┐
│                       Arduino Nano ESP32                         │
│                                                                  │
│  Serial processing                                               │
│        │                                                         │
│        ├── Differential-drive control task                       │
│        │      ├── left/right wheel controllers                   │
│        │      ├── FG pulse counters                              │
│        │      └── wheel velocity estimators                      │
│        │                                                         │
│        └── Sensor read task                                      │
│               └── MPU6050                                       │
└──────────────────────────────────────────────────────────────────┘
```

The separation between ROS 2 application logic, ROS 2 Control, transport/protocol handling, and MCU real-time control allows each layer to be developed and tested independently.

---

## 📦 Packages

| Package | Purpose |
|---|---|
| **`bumperbot_bringup`** | Top-level launch files for simulation and real-robot startup |
| **`bumperbot_description`** | URDF/Xacro robot model, ROS 2 Control integration, meshes, and Gazebo description |
| **`bumperbot_controller`** | ROS 2 Control controller configuration and joystick teleoperation |
| **`bumperbot_firmware`** | ROS 2 hardware interface, USB serial transceiver, protocol handling, MCU firmware, sensor interfaces, and diagnostic tools |
| **`bumperbot_localization`** | EKF configuration and localization launch |
| **`bumperbot_motion`** | Motion-control framework, PD Pure Pursuit plugin, `FollowPath`, and square round-trip control |
| **`bumperbot_msgs`** | Custom ROS 2 messages and actions |
| **`bumperbot_tools`** | RViz configuration and Gazebo/display launch utilities |

### `bumperbot_motion`

The motion package is organized around reusable control layers:

```text
MotionControlServer
        │
        ├── FollowPathControl
        │
        └── SquareRoundtripControl
                │
                └── MotionControlPlugin
                        │
                        └── PD Pure Pursuit
```

The motion controller is plugin-oriented so that the path-following strategy can be replaced or extended without changing the motion-control server architecture.

### `bumperbot_msgs`

Currently provides:

- `TwoWheelsAngularVelocity.msg`
- `SquareRoundtrip.action`

The `SquareRoundtrip` action drives the robot around a square trajectory and returns it to its starting position. The square side length is supplied by the action goal.

---

## 🤖 Hardware Architecture

### Main hardware

| Component | Current implementation |
|---|---|
| Main computer | **Raspberry Pi 5** |
| MCU | **Arduino Nano ESP32** |
| Drive motors | **24 mm brushless DC gear motors** with FG speed feedback |
| IMU | **MPU6050**, I2C |
| Host ↔ MCU link | **USB** |
| Wheel feedback | FG pulse signal + ESP32 pulse counting |
| Wheel control | Closed-loop velocity control executed on the ESP32 |

The ESP32 performs the time-sensitive wheel-control and measurement tasks locally. The Raspberry Pi runs the ROS 2 stack, localization, motion control, and the ROS 2 Control integration.

### Firmware architecture

The MCU firmware is structured as reusable PlatformIO libraries:

```text
bumperbot_diff_drive
├── bldc2430_motor
├── bldc2430_encoder
├── bldc2430_pulse_counter
├── wheel_controller
└── wheel_velocity_estimator

bumperbot_imu
└── mpu6050_driver

bumperbot_protocol
├── message_serialize
├── message_deserialize
├── message_traits
├── system_data
└── system_messages

bumperbot_hal
└── wire_i2c_bus
```

This keeps hardware-specific implementation details separate from application tasks and the communication protocol.

---

## 🔌 Communication Architecture

The Raspberry Pi and ESP32 communicate through a **custom binary message protocol over USB**.

On the ROS 2 side, the transport is implemented by the `bumperbot_firmware` package:

```text
ROS 2 Controllers / Nodes
          │
          ▼
RobotSystemInterface
          │
          ├── DiffDriveHandler
          └── ImuSensorHandler
          │
          ▼
SerialMessageTransceiver
          │
          ▼
SerialMessageProtocol
          │
          ║ USB
          ▼
ESP32 Serial Process Task
          │
          ▼
SerialMessageProcessor
          │
          ├── Differential-drive control
          └── Sensor processing
```

The protocol code is shared conceptually between both sides through strongly typed message structures, serialization/deserialization helpers, and message traits.

---

## 🧠 Localization

The localization stack uses an Extended Kalman Filter (**EKF**) to combine:

- differential-drive wheel odometry
- MPU6050 IMU measurements

Configuration is maintained in:

```text
bumperbot_localization/config/ekf.yaml
```

The localization launch file is:

```bash
ros2 launch bumperbot_localization local_localization.launch.py
```

The robot model and sensor frames are defined in the description package so that the TF tree used in simulation and real hardware remains consistent.

---

## 🎮 Running the Robot

### Prerequisites

- **ROS 2 Jazzy Jalisco**
- **Ubuntu 24.04** for the development host
- `colcon`
- `rosdep`
- PlatformIO for ESP32 firmware development
- Gazebo / RViz for simulation and visualization

Install ROS 2 Jazzy according to the official documentation:

https://docs.ros.org/en/jazzy/Installation.html

### Create the workspace

```bash
mkdir -p ~/bumperbot_ws/src
cd ~/bumperbot_ws/src

git clone https://github.com/semy-v/bumperbot-ros2-core.git
```

### Install dependencies

```bash
cd ~/bumperbot_ws

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro jazzy \
  -y
```

### Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🧪 Simulation

Simulation is the recommended starting point for development of the ROS 2 application layer.

```bash
source ~/bumperbot_ws/install/setup.bash

ros2 launch bumperbot_bringup simulated_robot.launch.py
```

The simulation bringup integrates:

- Gazebo
- robot state publication
- ROS 2 controllers
- localization
- motion-control server
- RViz visualization

For more focused debugging, individual launch files are available in the corresponding packages.

---

## 🦾 Real Robot

### 1. Build and flash the ESP32 firmware

The current firmware project is a PlatformIO project located at:

```text
bumperbot_firmware/arduino/esp32_controller
```

Build:

```bash
cd bumperbot_firmware/arduino/esp32_controller
pio run
```

Upload:

```bash
pio run -t upload
```

The firmware contains:

- the main application
- serial communication processing
- differential-drive control
- sensor acquisition
- wheel velocity estimation
- diagnostic applications
- reusable PlatformIO libraries

### 2. Build the Yocto operating system

The robot is intended to run the ROS 2 stack natively on a custom embedded Linux image.

Build the image using:

**[meta-bumperbot](https://github.com/semy-v/meta-bumperbot)**

### 3. Launch the real robot

The companion Yocto image is designed to support autonomous service startup. After the robot power on the system will boot, connect to the configured Wi-Fi network, and start the bumperbot packages as systemd services autonomously.

---

## 🧰 Useful Launch Files

### Bringup

```text
bumperbot_bringup/
├── launch/
│   ├── simulated_robot.launch.py
│   └── real_robot.launch.py
└── scripts/
    └── run_env_setup.sh
```

### Controllers

```text
bumperbot_controller/launch/
├── controllers.launch.py
├── diff_drive_controller.launch.py
├── imu_sensor_broadcaster.launch.py
├── joint_state_broadcaster.launch.py
└── joystick_teleop.launch.py
```

### Firmware / hardware interface

```text
bumperbot_firmware/launch/
├── controller_manager.launch.py
├── serial_transceiver.launch.py
└── test_runner.launch.py
```

### Motion

```text
bumperbot_motion/launch/
└── motion_server.launch.py
```

### Visualization

```text
bumperbot_tools/launch/
├── display_model.launch.py
├── display_real_robot.launch.py
└── gazebo.launch.py
```

---

## 🧪 Development & Diagnostics

The repository includes dedicated diagnostic applications instead of relying only on the full robot stack.

### 🔄 Differential-Drive Rotation Test

The ESP32 firmware also includes a dedicated **differential-drive rotation test** diagnostic application for exercising controlled wheel-rotation sequences. This utility is intended for validating wheel direction/control behavior and repeatable rotation sequences independently of the normal robot runtime.

The diagnostic implementation is located under:

```text
bumperbot_firmware/arduino/esp32_controller/src/diagnostic/diff_drive_rotation/
├── main.cpp
├── README_diff_drive_rotation.md
└── wheel_rotation_sequence_runner.hpp
```

Refer to [`README_diff_drive_rotation.md`](bumperbot_firmware/arduino/esp32_controller/src/diagnostic/diff_drive_rotation/README_diff_drive_rotation.md) for the detailed test procedure, sequence definition, and expected behavior.

### Feed-forward calibration

The ESP32 firmware contains a dedicated **feed-forward calibration** diagnostic application for the differential-drive system. It is separated from the normal robot control application so calibration experiments can be performed without changing the production runtime.

The calibration implementation is located under:

```text
bumperbot_firmware/arduino/esp32_controller/src/diagnostic/feedforward_calibration/
├── feedforward_calibration_runner.cpp
├── feedforward_calibration_runner.hpp
├── main.cpp
└── README_feedforward_calibration.md
```

The diagnostic application provides the calibration workflow and supporting test runner, while the resulting feed-forward parameters are intended to improve the relationship between commanded wheel velocity and the motor drive command before the closed-loop velocity controller applies feedback correction.

Refer to [`README_feedforward_calibration.md`](bumperbot_firmware/arduino/esp32_controller/src/diagnostic/feedforward_calibration/README_feedforward_calibration.md) for the detailed calibration procedure, experiment setup, and interpretation of the collected results.


### IMU diagnostic

```text
bumperbot_firmware/arduino/esp32_controller/src/diagnostic/imu
```

### Wi-Fi diagnostic

```text
bumperbot_firmware/arduino/esp32_controller/src/diagnostic/wifi_connect
```

### ROS 2 velocity test runner

The `bumperbot_firmware` ROS 2 package includes:

```text
velocity_cmd_test_runner_node.py
```

for repeatable wheel-velocity test scenarios, including observation of measured wheel angular velocity and controller response.

Test configuration is stored under:

```text
bumperbot_firmware/config/test/
├── robot_move_scenario.yaml
└── serial_transceiver.yaml
```

---

## 📁 Repository Structure

```text
bumperbot-ros2-core/
├── bumperbot_bringup/
├── bumperbot_controller/
├── bumperbot_description/
├── bumperbot_firmware/
│   ├── arduino/
│   │   └── esp32_controller/
│   │       ├── include/
│   │       ├── lib/
│   │       │   ├── bumperbot_diff_drive/
│   │       │   ├── bumperbot_hal/
│   │       │   ├── bumperbot_imu/
│   │       │   ├── bumperbot_protocol/
│   │       │   └── bumperbot_wireless-console/
│   │       ├── src/
│   │       │   ├── diagnostic/
│   │       │   ├── diff_drive_control_task.cpp
│   │       │   ├── sensor_read_task.cpp
│   │       │   ├── serial_message_processor.cpp
│   │       │   └── serial_process_task.cpp
│   │       └── test/
│   ├── include/
│   ├── launch/
│   ├── config/
│   └── src/
├── bumperbot_localization/
├── bumperbot_motion/
├── bumperbot_msgs/
├── bumperbot_tools/
├── LICENSE
└── README.md
```

---

## 🙏 Acknowledgements

This project was heavily inspired by the outstanding course:

**[Self Driving and ROS 2 – Learn by Doing! Odometry & Control](https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-odometry-control/)** by **Antonio Brandi**.

While the overall robot concept and some mechanical/URDF design elements were influenced by the course, **this repository is an independent implementation** developed from scratch.

The current software stack — including the custom ROS 2 Control hardware interface, USB binary communication protocol, ESP32 wheel-control firmware, wheel velocity estimation, MPU6050 integration, motion-control framework, localization configuration, diagnostic tools, and Yocto-based embedded Linux deployment — is independently designed and implemented.

Special thanks to Antonio Brandi for creating an excellent hands-on course that helped spark this project.

---
