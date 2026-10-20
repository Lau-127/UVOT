# UVOT

UVOT is a low-cost, modular and reconfigurable mobile robot platform for research and education.

This repository currently contains the ROS 2 description and Gazebo simulation of the three supported locomotion configurations:

- **Omnidirectional (`omni`)**: four Mecanum wheels.
- **Skid-steer (`skid`)**: four conventional driven wheels.
- **Differential drive (`diff`)**: two rear driven wheels and a passive front caster.

The current simulation stack targets **Ubuntu 24.04**, **ROS 2 Jazzy** and **Gazebo Harmonic / Gazebo Sim 8**.

## Repository structure

```text
UVOT/
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

## Gazebo simulation

The three locomotion configurations are selected with the `robot_type` launch argument.

### Omnidirectional / Mecanum

```bash
ros2 launch uvot_gazebo simulation.launch.py \
  namespace:=robot1 \
  robot_type:=omni
```

### Skid-steer

```bash
ros2 launch uvot_gazebo simulation.launch.py \
  namespace:=robot1 \
  robot_type:=skid
```

### Differential drive

```bash
ros2 launch uvot_gazebo simulation.launch.py \
  namespace:=robot1 \
  robot_type:=diff
```

The simulation launch automatically starts the joint state broadcaster and the drive controller corresponding to the selected configuration.

## Manual simulation commands

### Omnidirectional: longitudinal motion

```bash
ros2 topic pub --once \
  /robot1/mecanum_controller/reference \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

### Omnidirectional: lateral motion

```bash
ros2 topic pub --once \
  /robot1/mecanum_controller/reference \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

### Omnidirectional: rotation

```bash
ros2 topic pub --once \
  /robot1/mecanum_controller/reference \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}}"
```

### Skid-steer: longitudinal motion

```bash
ros2 topic pub --once \
  /robot1/skid_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

### Differential drive: longitudinal motion

```bash
ros2 topic pub --once \
  /robot1/diff_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

## Check active controllers

```bash
ros2 control list_controllers \
  -c /robot1/controller_manager
```

## Validation plan

The planned experimental validation will compare the physical platform and the digital twin using equivalent inputs and measurements.

| Level | Experiment | Omni | Skid | Diff |
|---|---|:---:|:---:|:---:|
| Wheels raised | Wheel-speed validation | ✓ | ✓ | ✓ |
| Open loop | Straight motion | ✓ | ✓ | ✓ |
| Open loop | Lateral motion | ✓ | — | — |
| Open loop | In-place rotation | ✓ | ✓ | ✓ |
| Open loop | Arc / combined motion | ✓ | ✓ | ✓ |
| Open loop | Diagonal motion | ✓ | — | — |
| Closed loop | Square trajectory | ✓ | ✓ | ✓ |
| Navigation | Common map and waypoint sequence | ✓ | ✓ | ✓ |

The real-robot and navigation procedures will be documented once their final implementation and validation are completed.

## License

Apache-2.0.
