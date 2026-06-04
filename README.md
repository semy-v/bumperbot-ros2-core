# Bumperbot ROS 2 Core Stack

A production-grade ROS 2 software stack for **BumperBot**, an open-source, 3D-printed autonomous differential-drive robot.

This repository contains the robot application layer, motion-control framework, hardware interfaces, and system integration components required to operate both the physical robot and its simulation environment. The software was developed from the ground up using modern C++20 and ROS 2 Jazzy, with a focus on modular architecture, hardware deployment, and real-time control.

> **⚙️ Companion OS Repository** > This software stack is designed to be deployed natively onto a custom embedded Linux image. The Yocto Project configuration and board support package (BSP) used to build the OS for this robot can be found in the companion repository: **[meta-bumperbot](https://github.com/semy-v/meta-bumperbot)**.

---

## 📦 Packages Overview
* **bumperbot_bringup:** Top-level launch package for starting the complete robot stack in either hardware or simulation environments.

* **bumperbot_firmware:** Custom hardware interfaces bridging the ROS 2 control loop with the Arduino-based physical motor drivers and encoders using serial communication. Also includes the I2C IMU drivers.

* **bumperbot_motion:** Custom ROS 2 motion-control framework featuring a Nav2-compatible FollowPath action server and plugin-based PD Pure Pursuit controller which computes and sends linear and angular velocity commands for differential-drive robot.

* **bumperbot_controller:** ROS 2 Control configuration package for differential-drive control, odometry generation, wheel-state publishing, and joystick teleoperation.

* **bumperbot_localization:** EKF-based localization package that fuses wheel odometry and IMU data to provide filtered robot state estimation and pose tracking.

* **bumperbot_description:** Robot description package containing the URDF/Xacro models, meshes, kinematic structure, and simulation definitions.

* **bumperbot_msgs:** Custom ROS 2 message definitions used throughout the BumperBot software stack.

* **bumperbot_tools:** Development utilities for RViz visualization, Gazebo simulation, robot spawning, and ROS–Gazebo integration.

## 🤖 Hardware Architecture

Unlike simulation-only projects, this stack is actively deployed and proven on a custom physical hardware setup. The architecture utilizes the following components:

* **Main Compute:** Raspberry Pi 5
* **Low-Level Microcontroller:** Arduino Nano (handles high-frequency quadrature encoder reading and closed-loop motor PID control)
* **Actuation:** DC Motors driven by an L298N dual H-bridge motor driver
* **Sensors:** MPU6050 IMU (I2C) for orientation and acceleration tracking
* **Interface:** A custom binary serial protocol bridges the ROS 2 `diff_drive_interface` running on the Pi with the Arduino firmware.

---

## 🛠️ Installation & Build

### Prerequisite: Install ROS 2 Jazzy
Before setting up this workspace, ensure you have a working installation of ROS 2 Jazzy Jalisco on your development machine. 
* Official Installation Guide: [Install ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)

### Setup the Workspace
Clone this repository into the `src` directory of your ROS 2 workspace.

### Install Dependencies
Install all necessary ROS 2 package dependencies automatically using the `rosdep` tool:
```bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
```

### Build the Workspace
```bash
. /opt/ros/jazzy/setup.bash
colcon build
```

## 🚀 Running the Robot

### Simulation (Gazebo)

To validate the robot model, controllers, localization, and motion-control stack without physical hardware, launch the complete simulation environment:

```bash
source install/setup.bash

ros2 launch bumperbot_bringup simulated_robot.launch.py
```

This launches the robot in Gazebo and starts the required ROS 2 nodes for simulation, control, localization, and visualization.

---

### Real Hardware Deployment

Deploying BumperBot on physical hardware consists of programming the microcontroller and installing the operating system on the Raspberry Pi.

#### 1. Flash the Arduino Firmware

Compile and upload the custom firmware to the Arduino Nano responsible for wheel encoder processing and low-level motor control.

Firmware location:

```text
src/bumperbot_firmware/arduino/diff_drive_controller/diff_drive_controller.ino
```

#### 2. Install the Raspberry Pi OS Image

The ROS 2 application stack is designed to run natively on a custom Yocto Linux image built using the companion **meta-bumperbot** repository.

Build the image using the Yocto configuration provided in:

```text
https://github.com/semy-v/meta-bumperbot
```

Flash the generated image to the Raspberry Pi 5 and connect the robot hardware.

---

## 🙏 Acknowledgements

The BumperBot platform was inspired by the course **[Self Driving and ROS 2 – Learn by Doing! Odometry & Control](https://www.udemy.com/course/self-driving-and-ros-2-learn-by-doing-odometry-control/)** by **Antonio Brandi**.

This repository is an independent personal project and is not a fork of the course codebase. While the overall robot concept and portions of the mechanical/URDF design were inspired by the course, the software implementation contained in this repository—including the hardware interfaces, communication protocols, motion-control framework, localization stack, controller integration, build system, deployment infrastructure—was developed independently from scratch.

Special thanks to Antonio Brandi for providing an excellent foundation for exploring ROS 2 robotics and autonomous mobile robot development.