# 4-Wheel Differential Drive Robot Simulation (Gazebo)

## About

This package provides a (custom) 4-wheel skid-steer robot for simulation in gazebo. 
It also provides scripts for launching the robot. World generation is not included
in this package.

## Robot
This is a custom robot based on the Waveshare UGV02. The base robot is augmented with:
- NVIDIA Orin compute
- Goal Zero 100 Wh battery (Orin supply)
- U-blox 7 GPS (USB)
- LDRobot STL-19P Lidar (USB)
- Intel D4535 RGBD Camera (USB)

The robot XACRO file uses geometric primitives to define the robot. This should accelerate physics and rendering computations involving the robot. This robot is supported in other simulators as well (e.g. IsaacSim). See those repos for robot model development/translation.

## Requirements

This package was developed on Ubuntu 22.04/ROS2 Humble/Gazebo Garden.

#### Install Required ROS 2 Packages

Many ROS2 packages are required. Keep installing until it works.

## Usage

### Clone the Repository

### Build the Package

Source the ROS 2 environment and build the package. In your ros2 ws directory:


```bash
source /opt/ros/humble/setup.bash #(if necessary)
colcon build
colcon source install/setup.bash
```

### Launch the Robot

After building the package, and given world and robot files with accompanying launch metadata:

```bash
ros2 launch gazebo_differential_drive_robot_4wheel robot.launch.py world:=<world>.sdf robot:=<robot>.xacro
```

This script assumes an accompanying "\<world\>.yml" file such as:

```yml
area:
- 40.0
- 20.0
resolution: 2.0
robot_start_xy:
- -2.0
- 0.0
robot_stop_xy:
- 10.0
- -8.0
scale: 0.4
seed: 41
threshold: 0.0
```

The important data here is `robot_start_xy`.

### Control the Robot

#### Using a Keyboard

You can control the robot using the ```teleop_twist_keyboard``` package:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Note that open-loop robot behavior (i.e. teleop with unit gains on linear and angular velocity) are quite good with gazebo. See the section on Open-loop robot behavior.

## Sensor Support

Current sensors are lidar (2d), rgbd camera, and IMU. GPS support will be added.

## Transform Tree

Launching the robot and running:

`ros2 run tf2_tools view_frames`

generates:

## Open-loop robot behavior

Robot behavior is described in the Open-loop robot behavior section in the test repo/ROS2 package:

[differential_drive_test](https://github.com/StuartGJohnson/differential_drive_test)

## Gazebo Issues
- Extracting ground truth poses from gazebo takes some extra work. See `src/gt_bridge_node.cpp`.
- There is no python support for `gz` published topics (as opposed to ROS2 topics). This necessitated C++ for nodes like `src/gt_bridge_node.cpp`.
- Gazebo garden does not support a clean simulation reset. For example, `src/reset_and_respawn_node.cpp` causes a crash of gazebo garden. See also: [gazebo reset issues](https://github.com/gazebosim/gz-sim/issues/1107)