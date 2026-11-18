# UVOT Commands

Quick reference for the main commands used with the UVOT platform, on the physical robot and in simulation (Gazebo).

The examples use the namespace `robot1`. Replace it when necessary.

The workspace path below is generic; adapt it to the ROS 2 workspace used on the target computer.

- [Real robot](#real-robot)
- [Simulation (Gazebo Harmonic)](#simulation-gazebo-harmonic)

---

## Real robot

### Upload robot configuration to the ESP32

```bash
cd ~/ros2_ws/src/UVOT/firmware/uvot_esp32
```

Select the kinematic configuration and the robot namespace:

```bash
./upload.sh <diff|skid|omni> <namespace>
```

Examples:

```bash
./upload.sh diff robot1
./upload.sh skid robot1
./upload.sh omni robot1
```

### Launch micro-ROS

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32 --baudrate 921600
```

### Launch the robot

This launches `robot_state_publisher`, `odom_to_tf` and `lidar_with_filter`:

```bash
ros2 launch uvot_bringup robot.launch.py namespace:=robot1 robot_type:=diff
```

Use `robot_type:=skid` or `robot_type:=omni` for the other locomotion configurations.

#### Launch components separately

```bash
ros2 launch uvot_bringup robot_state_publisher.launch.py namespace:=robot1 robot_type:=diff
```

```bash
ros2 launch uvot_bringup odom_to_tf.launch.py namespace:=robot1
```

```bash
ros2 launch uvot_lidar lidar_with_filter.launch.py namespace:=robot1
```

### Move the robot

With the micro-ROS agent and the bringup running, the ESP32 listens on `/robot1/cmd_vel` with `geometry_msgs/msg/Twist` (not `TwistStamped`, unlike the simulation). Publish continuously with `-r 10` and stop with Ctrl+C.

Diff / skid:

```bash
ros2 topic pub -r 10 /robot1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1}, angular: {z: 0.0}}"
```

Omni (`linear.y` is the lateral velocity, positive to the left):

```bash
ros2 topic pub -r 10 /robot1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.1}, angular: {z: 0.0}}"
```

### Keyboard teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/robot1/cmd_vel
```

In omni, hold Shift for lateral motion.

### Sensors

- Odometry: `/robot1/odom`
- IMU: `/robot1/imu`
- Wheel states: `/robot1/joint_states`
- Firmware information: `/robot1/robot_info`

### Launch Nav2

```bash
ros2 launch uvot_navigation navigation_launch.py namespace:=robot1 robot_type:=diff map:=$(ros2 pkg prefix --share uvot_navigation)/maps/quiro_save.yaml
```

Use `robot_type:=skid` or `robot_type:=omni` as required.

---

## Simulation (Gazebo Harmonic)

The simulation does not need the micro-ROS agent or `robot.launch.py`: the Gazebo launch starts `robot_state_publisher`, `ros2_control` and the bridges.

### Launch the simulation

```bash
ros2 launch uvot_gazebo simulation.launch.py \
  namespace:=robot1 \
  robot_type:=diff
```

Use `robot_type:=skid` or `robot_type:=omni` for the other locomotion configurations.

The launch starts the joint state broadcaster and the drive controller of the selected configuration. Check that it is active:

```bash
ros2 control list_controllers -c /robot1/controller_manager
```

### Move the robot

The controllers listen under `/robot1/...` and expect `geometry_msgs/msg/TwistStamped`, not `Twist`. They stop the robot if no message arrives for 0.5 s, so publish continuously with `-r 10` and stop with Ctrl+C.

Diff:

```bash
ros2 topic pub -r 10 /robot1/diff_controller/cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {frame_id: 'robot1/base_footprint'}, twist: {linear: {x: 0.1}, angular: {z: 0.0}}}"
```

Skid:

```bash
ros2 topic pub -r 10 /robot1/skid_controller/cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {frame_id: 'robot1/base_footprint'}, twist: {linear: {x: 0.1}, angular: {z: 0.0}}}"
```

Omni (the topic is `reference`; `linear.y` is the lateral velocity, positive to the left):

```bash
ros2 topic pub -r 10 /robot1/mecanum_controller/reference geometry_msgs/msg/TwistStamped \
"{header: {frame_id: 'robot1/base_footprint'}, twist: {linear: {x: 0.0, y: 0.1}, angular: {z: 0.0}}}"
```

Omni rotation:

```bash
ros2 topic pub -r 10 /robot1/mecanum_controller/reference geometry_msgs/msg/TwistStamped \
"{header: {frame_id: 'robot1/base_footprint'}, twist: {linear: {x: 0.0, y: 0.0}, angular: {z: 0.2}}}"
```

If the robot does not move, check the launch log: with `stamp` at 0 the controller uses the current time, but some versions reject the message as too old.

### Keyboard teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p frame_id:=robot1/base_footprint \
  -r cmd_vel:=/robot1/diff_controller/cmd_vel
```

For skid use `-r cmd_vel:=/robot1/skid_controller/cmd_vel`. For omni use `-r cmd_vel:=/robot1/mecanum_controller/reference` and hold Shift for lateral motion.

### Sensors

- Controller odometry: `/robot1/diff_controller/odom`, `/robot1/skid_controller/odom` or `/robot1/mecanum_controller/odometry`
- IMU: `/robot1/imu` (100 Hz)
- Wheel states: `/robot1/joint_states`
- Ground-truth pose: `/model/robot1/odometry_gt` (50 Hz)
