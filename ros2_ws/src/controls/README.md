# Controls

The **controls** package provides the AUV control loops for depth and attitude. It computes control efforts as `geometry_msgs/Wrench` messages and combines them into a single effort for downstream actuation.

This process occurs in three stages:

1. **Depth control** - compute a vertical force using a PID controller and a feedforward term.
2. **Attitude control** - compute body torques from IMU orientation and target quaternion with a buoyancy feedforward term. Refer to this [Control law derivation (PDF)](docs/Attitude_Controller.pdf) for more details.
3. **Effort superposition** - Rotate the depth effort into the body frame. Sum the efforts (plus optional biases) into a combined wrench.


## Table of Contents
- [Controls](#controls)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Usage](#usage)
    - [Raw Setpoint Publishing](#raw-setpoint-publishing)
    - [Navigation Server (Action Client)](#navigation-server-action-client)
    - [Automated PID Tuning with CMA-ES (SITL)](#automated-pid-tuning-with-cma-es-sitl)
  - [Nodes](#nodes)
    - [Published Topics](#published-topics)
    - [Subscribed Topics](#subscribed-topics)
  - [Installation](#installation)
    - [Dependencies](#dependencies)
    - [Building](#building)
    - [Running](#running)
    - [License](#license)


## Overview
The controls package implements separate depth, planar, and attitude controllers. The depth controller uses a PID loop on `/processed/depth` and publishes a vertical effort on `/controls/depth_effort`. The planar controller handles X and Y translation by subscribing to DVL position data (`auv_frame/dvl/position`) and publishing efforts on `/controls/x_effort` and `/controls/y_effort`. The attitude controller uses IMU orientation (`processed/imu`) and a target quaternion (`quaternion_setpoint`) to publish torques on `/controls/attitude_effort`. The superimposer node sums all efforts, applies optional bias terms, and publishes `/controls/combined_effort` for propulsion.

Additionally, the **Navigation Server** acts as a high-level orchestrator. It hosts an Action Server (`/motion/navigate`) that accepts goal poses and handles computing errors, resolving coordinates, publishing setpoints to the underlying PID controllers, and reporting convergence.


## Usage
The controls package is not for direct use, it is used through publishing setpoints or via the Navigation Server. By default, the controllers are disabled at startup. 

To activate the attitude controller:

        ros2 param set attitude_controller enabled true

To activate the depth controller:

        ros2 param set depth_controller enabled true

To activate the X-axis controller:

        ros2 param set x_controller enabled true

To activate the y-axis controller:

        ros2 param set y_controller enabled true

### Raw Setpoint Publishing

Publishing a depth setpoint onto `/controls/depth_setpoint`:

        ros2 topic pub /controls/depth_setpoint std_msgs/msg/Float64 "{data: 1.5}" 

Publishing a depth setpoint onto `/controls/x_setpoint`:


        ros2 topic pub /controls/x_setpoint std_msgs/msg/Float64 "{data: 2.0}" 


Publishing a depth setpoint onto `/controls/y_setpoint`:


        ros2 topic pub /controls/y_setpoint std_msgs/msg/Float64 "{data: 2.0}" 


Publishing an attitude setpoint onto `/controls/quaternion_setpoint`:

        ros2 topic pub /controls/quaternion_setpoint geometry_msgs/msg/Quaternion "{x: 0, y: 0, z: 0.7071, w: 0.7071}"

### Navigation Server (Action Client)
To send a test 3D goal to the Navigation Server via the ROS 2 Action CLI:

        ros2 action send_goal /motion/navigate auv_msgs/action/AUVNavigate "{
          target_pose: {
            position: {x: 1.0, y: 1.0, z: 1.0},
            orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}
          },
          do_x: true,
          do_y: true,
          do_z: true,
          do_yaw: true,
          is_relative: false,
          is_robot_centric: false,
          position_tolerance: 0.1,
          yaw_tolerance: 0.1,
          hold_time: 2.0,
          timeout: 30.0
        }" --feedback

To send a goal programmatically using Python, utilize the `goal_helpers` library included in `controls/goal_helpers.py`. See `test_nav.py` for examples.


### Automated PID Tuning with CMA-ES (SITL)

The package includes an automated SITL PID tuning pipeline (`cmaes_tuner.py`) utilizing Covariance Matrix Adaptation Evolution Strategy (CMA-ES). It interfaces directly with our Unity physics engine running in fast-forward via `/clock` and `/simulation/reset`.

#### Prerequisites
- **Unity Simulation** must be running and publishing `/clock`.
- **ROS-TCP-Endpoint** must be running to bridge Unity ↔ ROS 2.
- **No Clamping During Tuning**: Ensure maximum effort/torque clamping is disabled while tuning so CMA-ES observes unclipped mathematical dynamics. Once tuning is complete, add torque/thrust limits for hardware safety and apply slew-rate limiting (ramped setpoints) during mission navigation to prevent camera blur and DVL loss.

#### Exact Commands to Run

**Terminal 1: Launch the controls stack (inside Docker)**
```bash
source /home/douglas/AUV-2026/ros2_ws/install/setup.bash
ros2 launch controls controls.launch.py sim:=true
```

**Terminal 2: Run the CMA-ES tuner (inside Docker)**

*Phase 1 — Attitude (pick one axis at a time):*
```bash
source /home/douglas/AUV-2026/ros2_ws/install/setup.bash
ros2 run controls cmaes_tuner.py --tune pitch
```
```bash
ros2 run controls cmaes_tuner.py --tune roll
```
```bash
ros2 run controls cmaes_tuner.py --tune yaw
```
*(After Phase 1 completes, update `attitude_controller` default parameters in `Controller_params_sim.yaml` with the winning gains before proceeding.)*

*Phase 2 — Depth:*
```bash
ros2 run controls cmaes_tuner.py --tune depth
```

*Phase 3 — Planar (pick one):*
```bash
ros2 run controls cmaes_tuner.py --tune surge
```
```bash
ros2 run controls cmaes_tuner.py --tune sway
```

*Phase 4 — Full joint optimization (all 18 params at once):*
```bash
ros2 run controls cmaes_tuner.py --tune full
```

#### Key Automation Features
- **Dedicated Tuner Configuration (`cmaes_tuner_params.yaml`)**: All algorithm parameters (`popsize`, `maxiter`, `sim_duration_sec`, penalty weights, absolute overshoot floors) and mode definitions are cleanly defined in [cmaes_tuner_params.yaml](file:///home/sohaib/projects/AUV-2026/ros2_ws/src/controls/params/cmaes_tuner_params.yaml).
- **Automatic YAML Initial Guesses**: The tuner automatically reads `Controller_params_sim.yaml` upon launch to populate the starting PID gains (`init_kp`, `init_ki`, `init_kd`).
- **Synchronized Setpoint Randomization**: To ensure robust PID tuning across varying step sizes, the tuner enables `randomize_setpoints:=true` by default, testing candidates against randomized target setpoints within mode-dependent bounds while enforcing absolute overshoot tolerance floors ($1.5^\circ$ for attitude, $0.05\text{m}$ for translation).


## Nodes
The package provides seven ROS nodes: `depth_controller`, `attitude_controller`, `x_controller`, `y_controller`, `superimposer`, `navigation_server`, and `cmaes_tuner`.

- `depth_controller` input: `/auv_frame/depth`, `/controls/depth_setpoint`

- `depth_controller` output: `/controls/depth_effort`
- `x_controller` input: `auv_frame/dvl/position`, `/controls/x_setpoint`
- `x_controller` output: `/controls/x_effort`
- `y_controller` input: `auv_frame/dvl/position`, `/controls/y_setpoint`
- `y_controller` output: `/controls/y_effort`

- `attitude_controller` input: `/auv_frame/imu`, `quaternion_setpoint`

- `attitude_controller` output: `/controls/attitude_effort`

- `superimposer` input: `/controls/depth_effort`, `/controls/attitude_effort`, `/controls/x_effort`, `/controls/y_effort`, `processed/imu`

- `superimposer` output: `/controls/combined_effort`

- `navigation_server` input: `/state/pose` (geometry_msgs/PoseStamped)

- `navigation_server` output: Action Server `/motion/navigate`, publishers pointing to `/controls/depth_setpoint`, `/controls/x_setpoint`, `/controls/y_setpoint`, `/controls/quaternion_setpoint`

- `cmaes_tuner` input: `/clock`, `/auv_frame/depth`, `/auv_frame/imu`, `auv_frame/dvl/position`, `/controls/depth_effort`, `/controls/attitude_effort`, `/controls/x_effort`

- `cmaes_tuner` output: `/simulation/reset`, setpoint publishers, and dynamic parameter client calls to active/auxiliary controllers


### Published Topics

 Topic | Message | Description |
| ------ | ------- | ---------- |
| `/controls/depth_effort` | `geometry_msgs/Wrench` | Depth controller effort (force.z) in the **pool frame** |
| `/controls/x_effort` | `geometry_msgs/Wrench` | X-axis controller effort in the **pool frame** |
| `/controls/y_effort` | `geometry_msgs/Wrench` | Y-axis controller effort in the **pool frame** |
| `/controls/attitude_effort` | `geometry_msgs/Wrench` | Attitude controller effort (torques) in the **body frame** |
| `/controls/total_effort` | `geometry_msgs/Wrench` | Sum of depth and attitude efforts with optional biases in the **body frame** |


### Subscribed Topics

| Topic | Message | Description |
| ------ | ------- | ---------- |
| `/auv_frame/depth` | `std_msgs/Float64` | Current depth estimate |
| `/auv_frame/imu` | `sensor_msgs/Imu` | Orientation and angular velocity for attitude control |
| `auv_frame/dvl/position` | `geometry_msgs/Float64` | AUV's position in the pool frame from the DVL | 
| `/controls/depth_setpoint` | `std_msgs/Float64` | Desired vehicle depth |
| `/controls/quaternion_setpoint` | `geometry_msgs/Quaternion` | Desired vehicle orientation |
| `/controls/x_setpoint` | `std_msgs/Float64` | Desired vehicle position along the X-axis | 
| `/controls/y_setpoint` | `std_msgs/Float64` | Desired vehicle position along the Y-axis | 
| `/controls/depth_effort` | `geometry_msgs/Wrench` | Depth effort input to superimposer |
| `/controls/attitude_effort` | `geometry_msgs/Wrench` | Attitude effort input to superimposer |
| `/controls/x_effort` | `geometry_msgs/Wrench` | X-axis effort input to superimposer |
| `/controls/y_effort` | `geometry_msgs/Wrench` | Y-axis effort input to superimposer |

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

- `sensors` - for utility functions

- `scipy` - Used by `navigation_server` and `controls.utils` for 3D coordinate transformations.

- `cma` - Covariance Matrix Adaptation Evolution Strategy library for automated SITL PID tuning (`cmaes_tuner.py`).

### Building

	source /opt/ros/humble/setup.bash
	cd <AUV-2026>/ros2_ws
	colcon build --packages-select controls

After build is complete, make the packages visible to ROS

	source ros2_ws/install/setup.bash

### Running

Launch all package nodes

	ros2 launch controls controls.launch.py


### License

The source code is released under a GPLv3 license.
