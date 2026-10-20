# UVOT Commands

Quick reference for the main commands used with the physical UVOT platform.

The examples use the namespace `robot1`. Replace it when necessary.

The workspace path below is generic; adapt it to the ROS 2 workspace used on the target computer.

## Upload robot configuration to the ESP32

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

## Launch micro-ROS

```bash
ros2 run micro_ros_agent micro_ros_agent serial   --dev /dev/esp32   --baudrate 921600
```

## Launch the robot

This launches `robot_state_publisher`, `odom_to_tf` and `lidar_with_filter`:

```bash
ros2 launch uvot_bringup robot.launch.py   namespace:=robot1   robot_type:=diff
```

Use `robot_type:=skid` or `robot_type:=omni` for the other locomotion configurations.

### Launch components separately

```bash
ros2 launch uvot_bringup robot_state_publisher.launch.py   namespace:=robot1   robot_type:=diff
```

```bash
ros2 launch uvot_bringup odom_to_tf.launch.py   namespace:=robot1
```

```bash
ros2 launch uvot_lidar lidar_with_filter.launch.py   namespace:=robot1
```

## Launch Nav2

```bash
ros2 launch uvot_navigation navigation_launch.py   namespace:=robot1   robot_type:=diff   map:=$(ros2 pkg prefix --share uvot_navigation)/maps/quiro_save.yaml
```

Use `robot_type:=skid` or `robot_type:=omni` as required.
