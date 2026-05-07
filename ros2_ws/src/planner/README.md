# Planner

The **planner** package provides a high-level usage of the other packages to accomplish competition tasks. To accomplish this task (hah), the library
py_trees, an implementation of behaviour trees, is utilized. 

Currently, there is only skeleton functionality and the pre-qual missions. It is projected to add the various tasks from RoboSub 2026 to the planner package.

**Note: In the source code and the following documentation, the term "Behaviour" refers to a Tree Node (i.e. a Square or Parallelogram on the tree diagram)**

## Tree Graph

*TODO*, put a high-level diagram representing the Behaviour Tree

## Competition Tasks

The autonomous missions are designed based on the official **RoboSub 2026** challenge descriptions. For detailed technical specifications, scoring rules, and task elements, refer to the official handbook:

🔗 **[RoboSub Task Descriptions](https://robonation.gitbook.io/robosub-resources/section-3-autonomy-challenge/3.2-task-descriptions)**

## Usage
To use the planner, configure the parameters inside `config/behaviour_tree_params.yaml`. It is recommended to maintain separate configuration files for significantly different trees (e.g., `pre_qual_params.yaml` vs `competition_2026_params.yaml`).

Note that while `sim` and `use_ground_truth` can be toggled via launch arguments, most mission parameters are hierarchical (namespaced with dots) and should be modified directly in the configuration file.

### General Parameters

| Parameter | Type | Default | Description |
| ------ | ------- | ------- | ---------- |
| `tick_rate` | `float` | `3.0` | The tick rate of the behaviour tree in Hz. Higher tick rates increase responsiveness but consume more compute. |
| `sim` | `bool` | `true` | Whether to use simulation time. |
| `use_ground_truth` | `bool` | `true` | Whether to use ground truth pose and twist data from the simulator. Only relevant if `sim` is true. | 

### Pre-Qualification Parameters

| Parameter | Type | Default | Description |
| ------ | ------- | ------- | ---------- |
| `pre_qual.yaw_tolerance` | `float` | `0.5` | Allowed yaw error in radians. |
| `pre_qual.positional_tolerance` | `float` | `0.2` | Allowed positional error in meters. |
| `pre_qual.hold_time` | `float` | `1.0` | Required stabilization time within tolerances. |
| `pre_qual.timeout` | `float` | `20.0` | Maximum time allowed to complete the task before failure. |

#### Orbit Pre-Qualification
| Parameter | Type | Default | Description |
| ------ | ------- | ------- | ---------- |
| `pre_qual.orbit.yaw_tolerance_scale` | `float` | `0.5` | Scaling factor for yaw tolerance based on trajectory curvature. |
| `pre_qual.orbit.positional_tolerance_scale` | `float` | `0.5` | Scaling factor for positional tolerance based on orbit radius. |
| `pre_qual.orbit.hold_time_initial` | `float` | `1.0` | Hold time for the initial orbit segment. |
| `pre_qual.orbit.hold_time_segments` | `float` | `0.2` | Hold time for subsequent orbit segments. |
| `pre_qual.orbit.timeout` | `float` | `30.0` | Total timeout for the orbit mission. |

### Steps
(If using sim, follow step 1 and 2, otherwise move on to step 3)
1. **Launch Simulator**:
   - Launch the sim and toggle the **connect to ROS** button.
   - **Stream Data**: Navigate to `Config -> Sensors` and toggle the following:
     - `IMU / AHRS`
     - `Doppler Velocity`
     - `Depth Sensor`
2. **Setup Environment**:
   - Follow the steps in [AUV-2026/Docker/dev](../../../Docker/dev/README.md) to start the development environment.
3. **Build & Source**:
   - Build the project in the root directory and source the workspace:
```bash
./build.sh
source ros2_ws/install/setup.bash
```
4. Launch bringup (with `sim:=true` if using sim), then launch planner:
```bash
ros2 launch planner planner.launch.py sim:=true
```

5. Upon launching the planner, you will be prompted with a mission selection dialogue. To start a specific mission, manually publish an Int32 message, matching the mission choice, to the mission selector topic.

Selection of implemented missions:
```bash
1: Orbit Prequal
2: Rectangle Prequal
3: Basic Move forward (1.0m relative to Dougie)
4: Basic Dive (Down 1.5m)
5: Basic Yaw (180 deg)
6: Translation Rectangle (no yaw)
7: Test Service Call (reset dead reckoning)
8: Gate Task
9: Slalom Task
10: Bins Task
11: Torpedo Task
12: Table & Octagon Task
13: FULL COMPETITION RUN
```
```bash
ros2 topic pub --once /mission_selector std_msgs/msg/Int32 "{data: 1}"
```

To write a new Mission, consult the [README](missions/README.md) in the missions directory.

### Visualization
To live debug the behaviour tree, open a new terminal and run:
```bash
py-trees-tree-viewer
```
Select the node namespace (e.g., `/planner_root_tree`) and check **Blackboard Data** and **Periodic** in the stream settings.
*CLI Fallback:* `py-trees-tree-watcher -b`

## ROS Nodes
The package provides a single ROS Node: `RootNode` 

### Subscribed Topics

| Topic | Message | Description |
| ------ | ------- | ---------- |
| `/state_estimation/pose` | `geometry_msgs/PoseStamped` | State estimation pose used to write related information to the blackboard when sim = false |
| `/state_estimation/twist` | `geometry_msgs/TwistStamped` | State estimation twist used to write related information to the blackboard when sim = false|
| `/auv/ground_truth/pose"` | `geometry_msgs/PoseStamped` | Sim's ground truth pose used to write related information to the blackboard when sim = true |
| `/auv/ground_truth/pose"` | `geometry_msgs/PoseStamped` | Sim's ground truth twist used to write related information to the blackboard when sim = true |
| `/vision/object_map` | `auv_msgs/VisionObjectArray` | Object map information to be used in specific Behaviours |
| `/mission_selector` | `std_msgs/Int32` | User input for mission selection. Int32 value matches missions associated with the mission dashboard.


## Blackboard keys
Similar to ROS topics, PyTrees uses a Blackboard where we can write keys to serve as shared memory between different Behaviours. Topics are not used by each individual Behaviour. To achieve consistent performance, every Behaviour should have access to the same information. This would not be possible if  ROS callbacks were used in each one of them as they could be out of sync leading to undesired actions.

A key is a variable registered on the Blackboard. These variables can be of any type and each Behaviour can have `WRITE`, `READ` access or both or none at all to them. Determining which keys of the Blackboard a Behaviour can access is determined in the source code for said Behaviour.
To maintain control of Blackboard 

Here is a list of each Blackboard key and which Behaviour posseses READ/WRITE access to them:

 Key | Type | Description | READ access | WRITE access |
| ------ | ------- | ---------- | --------| --------- | 
| `/sensors/pose` | `geometry_msgs/PoseStamped` | State estimation pose used to determine actions of the planner | | SensorsBehaviour
| `/sensors/twist` | `geometry_msgs/TwistStamped` | State estimation twist used to determine actions of the planner| | SensorsBehaviour|
| `/vision/object_map` | `geometry_msgs/PoseStamped` | Object Map detections used to determine actions of the planner | | SensorsBehaviour | 
| `/navigation_client` | `rclpy.action.ActionClient` | Navigation client singleton. Manages all navigation goal requests given by the Behaviour nodes. | BasicActionBehaviour, OrbitActionBehaviour | Root
| `/navigation_client/ongoing_goal`| `rclpy.action.client.ClientGoalHandle` | Holds the current goal being handled by the action client. Used for monitoring what the current navigation goal of the behaviour tree is. (Currently unused) | | Root
