# Controls

The **controls** package provides the AUV control loops for depth and attitude. It computes control efforts as `geometry_msgs/Wrench` messages and combines them into a single effort for downstream actuation.

This process occurs in three stages:

1. **Depth control** - compute a vertical force using a PID controller and a feedforward term.
2. **Attitude control** - compute body torques from IMU orientation and target quaternion with a buoyancy feedforward term.
3. **Effort superposition** - Rotate the depth effort into the body frame. Sum the efforts (plus optional biases) into a combined wrench.


## Table of Contents
- [Controls](#controls)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Usage](#usage)
  - [Nodes](#nodes)
    - [Published Topics](#published-topics)
    - [Subscribed Topics](#subscribed-topics)
  - [Installation](#installation)
    - [Dependencies](#dependencies)
    - [Building](#building)
    - [Running](#running)
    - [License](#license)


## Overview
The controls package implements separate depth and attitude controllers. The depth controller uses a PID loop on `/processed/depth` and publishes a vertical effort on `/controls/depth_effort`. The attitude controller uses IMU orientation (`processed/imu`) and a target quaternion (`quaternion_setpoint`) to publish torques on `/controls/attitude_effort`. The superimposer node sums both efforts, applies optional bias terms, and publishes `/controls/combined_effort` for propulsion.


## Usage
The controls package is not for direct use, it is used through publishing setpoints.

Publishing a depth setpoint onto `/controls/depth_setpoint`:


        ros2 topic pub /controls/depth_setpoint std_msgs/msg/Float64 "{data: 1.5}" 


Publishing an attitude setpoint onto `/controls/quaternion_setpoint`:

        ros2 topic pub /controls/quaternion_setpoint geometry_msgs/msg/Quaternion "{x: 0, y: 0, z: 0.7071, w: 0.7071}"


## Nodes
The package provides three ROS nodes: `depth_controller`, `attitude_controller`, and `superimposer`.

- `depth_controller` input: `/processed/depth`, `/controls/depth_setpoint`

- `depth_controller` output: `/controls/depth_effort`

- `attitude_controller` input: `processed/imu`, `quaternion_setpoint`

- `attitude_controller` output: `/controls/attitude_effort`

- `superimposer` input: `/controls/depth_effort`, `/controls/attitude_effort`, `processed/imu`

- `superimposer` output: `/controls/combined_effort`


### Published Topics

 Topic | Message | Description |
| ------ | ------- | ---------- |
| `/controls/depth_effort` | `geometry_msgs/Wrench` | Depth controller effort (force.z) in the pool frame |
| `/controls/attitude_effort` | `geometry_msgs/Wrench` | Attitude controller effort (torques) |
| `/controls/combined_effort` | `geometry_msgs/Wrench` | Sum of depth and attitude efforts with optional biases |


### Subscribed Topics

| Topic | Message | Description |
| ------ | ------- | ---------- |
| `/processed/depth` | `std_msgs/Float64` | Current depth estimate |
| `processed/imu` | `sensor_msgs/Imu` | Orientation and angular velocity for attitude control |
| `/controls/depth_setpoint` | `std_msgs/Float64` | Desired depth setpoint |
| `/controls/quaternion_setpoint` | `geometry_msgs/Quaternion` | Desired vehicle orientation |
| `/controls/depth_effort` | `geometry_msgs/Wrench` | Depth effort input to superimposer |
| `/controls/attitude_effort` | `geometry_msgs/Wrench` | Attitude effort input to superimposer |


## Installation

### Dependencies

- `rclcpp` - ROS 2 C++ client library

- `rclpy` - ROS 2 Python client library

- `geometry_msgs` - for `Wrench` and `Quaternion` messages

- `sensor_msgs` - for `Imu` messages

- `std_msgs` - for `Float64` setpoints

- `message_filters` - for C++ subscriptions

- `eigen3_cmake_module` - Eigen headers for matrix math

- `numpy` - used by the PID controller

### Building

	source /opt/ros/humble/setup.bash
	cd <AUV-2026>/ros2_ws
	colcon build --packages-select controls

After build is complete, make the packages visible to ROS

	source install/setup.bash

### Running

Launch all package nodes

	ros2 launch controls controls.launch.py


### License

The source code is released under a GPLv3 license.
