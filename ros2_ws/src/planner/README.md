# Planner

The **planner** package provides a high-level usage of the other packages to accomplish competition tasks. To accomplish this task (hah), the library
py_trees, an implementation of behaviour trees, is utilized. 

Currently, there is only skeleton functionality and the pre-qual missions. It is projected to add the various tasks from RoboSub 2026 to the planner package.

**Note: In the source code and the following documentation, the term "Behaviour" refers to a Tree Node (i.e. a Square or Parallelogram on the tree diagram)**

## Tree Graph

*TODO*, put a high-level diagram representing the Behaviour Tree

## Usage
To use the planner, configurate the parameters inside the config folder:
Here is a description of the available parameters:

| Parameter | Type | Description |
| ------ | ------- | ---------- |
| `tick_rate` | `float` | The tick rate of the behaviour tree in Hz. Determines how often the tree is ticked and thus how often each behaviour is updated. Higher tick rates can lead to more responsive behaviour but at a tradeoff of more compute usage. |
| `sim` | `bool` | Determines whether to use simulation time. Should be true when running in simulation and false when running on the real AUV. |
| `use_ground_truth` | `bool` | Determines whether to use ground truth pose and twist data from the simulator. Parameter is only relevant if sim is true. | 
`pre_qual_yaw_tolerance` | float | 
`pre_qual_positional_tolerance` 
`pre_qual_hold_time`
`pre_qual_timeout`
`orbit_pre_qual_yaw_tolerance_scale`
`orbit_pre_qual_positional_tolerance_scale`
`orbit_pre_qual_hold_time_initial`
`orbit_pre_qual_hold_time_segments`
`orbit_pre_qual_timeout`

### Steps
(If using sim, follow step 1 and 2, otherwise move on to step 3)
1. Launch the sim and toggle the **connect to ROS** button and stream IMU data
2. Follow the steps inside AUV-2026/Docker/dev to start up the Docker development environment
3. Build the project in AUV-2026/ directory and source the work space
```bash
./build.sh
source ros2_ws/install/setup.bash
```
4. Launch the planner launch file
```bash
ros2 launch planner planner.launch.py
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
7: Test Service Call (reset dead reckoning)"
```
```bash
ros2 topic pub --once /mission_selector std_msgs/msg/Int32 "{data: 1}"
```

To write a new Mission, consult the [README](missions/README.md) in the missions directory.

## ROS Nodes
The package provides a single ROS Node: `RootNode` 

### Subscribed Topics

 Topic | Message | Description |
| ------ | ------- | ---------- |
| `/state_estimation/pose` | `geometry_msgs/PoseStamped` | State estimation pose used to write related information to the blackboard when sim = false |
| `/state_estimation/twist` | `geometry_msgs/TwistStamped` | State estimation twist used to write related information to the blackboard when sim = false|
| `/auv/ground_truth/pose"` | `geometry_msgs/PoseStamped` | Sim's ground truth pose used to write related information to the blackboard when sim = true |
| `/auv/ground_truth/pose"` | `geometry_msgs/PoseStamped` | Sim's ground truth twist used to write related information to the blackboard when sim = true |
| `/vision/object_map` | `auv_msgs/VisionObjectArray` | Object map information to be used in specific Behaviours |
| `/mission_selector` | `std_msgs/Int32` | User input for mission selection. Int32 value matches missions associated with the mission dashboard.


### Published Topics

 Topic | Message | Description |
| ------ | ------- | ---------- |
| None | None | None|


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
