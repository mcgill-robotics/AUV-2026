# Planner

The **planner** package provides a high-level usage of the other packages to accomplish competition tasks. To accomplish this task (hah), the library
py_trees, an implementation of behaviour trees, is utilized. 

Currently, there is only skeleton functionality as the competition tasks have not been released. It is in plan to replicate robosub 2025's objectivees
in the foreseeable future while waiting for the new ones to be unveiled.

**Note: In the source code and the following documentation, Behaviour refers to a Tree Node (i.e. a Square or Parallelogram on the tree diagram)**

## Skeleton Implementation

*TODO*, put the sketch of the skeleton tree here

## Usage
To use the planner, configurate the parameters inside the config folder:
Here is a description of the available parameters:

| Parameter | Type | Description |
| ------ | ------- | ---------- |
| `tick_rate` | `float` | The tick rate of the behaviour tree in Hz. Determines how often the tree is ticked and thus how often each behaviour is updated. Higher tick rates can lead to more responsive behaviour but at a tradeoff of more compute usage. |
| `sim` | `bool` | Determines whether to use simulation time. Should be true when running in simulation and false when running on the real AUV. |
| `use_ground_truth` | `bool` | Determines whether to use ground truth pose and twist data from the simulator. Parameter is only relevant if sim is true. | 



### Steps

1. Launch the sim and toggle the **connect to ROS** button and stream IMU data
2. Follow the steps inside AUV-2026/Docker/dev to start up the Docker development environment
3. Launch the planner launch file
```bash
ros2 launch planner planner_launch.py
```

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
