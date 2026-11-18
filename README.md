# UVOT

UVOT is a low-cost, modular and reconfigurable mobile robot platform for research and education.

This repository contains the ESP32 firmware, the ROS 2 packages for the physical robot and the Gazebo simulation of the three supported locomotion configurations:

- **Omnidirectional (`omni`)**: four Mecanum wheels.
- **Skid-steer (`skid`)**: four conventional driven wheels.
- **Differential drive (`diff`)**: two rear driven wheels and a passive front caster.

The current simulation stack targets **Ubuntu 24.04**, **ROS 2 Jazzy** and **Gazebo Harmonic / Gazebo Sim 8**.

## Repository structure

```text
UVOT/
├── firmware/
├── uvot_bringup/
├── uvot_description/
├── uvot_gazebo/
├── uvot_lidar/
├── uvot_navigation/
├── COMMANDS.md
├── .gitignore
└── README.md
```

## Package overview

### `firmware/uvot_esp32`

ESP32 firmware (micro-ROS, motor control and odometry EKF) for the three locomotion configurations.

### `uvot_description`

Contains the common URDF/Xacro description and mesh resources used by the robot model.

### `uvot_gazebo`

Contains the Gazebo simulation files, `gz_ros2_control` integration, controller configuration, simulation launch file and the custom kinematic Gazebo system plugin used by the Mecanum configuration.

### `uvot_bringup`

Package reserved for physical-robot bringup and system launch configuration.

### `uvot_lidar`

Contains LiDAR launch and filtering configuration.

### `uvot_navigation`

Contains SLAM and navigation configuration. These parts are still being prepared and are not documented here yet.

## Requirements

Recommended environment:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic / Gazebo Sim 8

Install the available package dependencies with:

```bash
cd ~/ros2_ws
rosdep install --from-paths src/UVOT --ignore-src -r -y
```

## Build

Clone or place the repository inside a ROS 2 workspace, for example:

```text
~/ros2_ws/src/UVOT
```

Then build:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

In a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Usage

All the commands to run the platform, both on the physical robot (ESP32 firmware, micro-ROS, bringup, Nav2) and in Gazebo simulation (launch, motion commands, teleoperation), are collected in [COMMANDS.md](COMMANDS.md).

## License

Apache-2.0.
