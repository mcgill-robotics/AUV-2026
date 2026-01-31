# Planner

The **planner** package provides a high-level usage of the other packages to accomplish competition tasks. To accomplish this task (hah), the library
py_trees, an implementation of behaviour trees, is utilized. 

Currently, there is only skeleton functionality as the competition tasks have not been released. It is in plan to replicate robosub 2025's objectivees
in the foreseeable future while waiting for the new ones to be unveiled.

## Skeleton Implementation

*TODO*, put the sketch of the skeleton tree here

## Usage
To use the basic yaw behaviour tree, whose objective is to simply rotate to a target yaw. Here are the steps to run it in the sim.

### Steps

1. Launch the sim and toggle the **connect to ROS** button and stream IMU data
2. Follow the steps inside AUV-2026/Docker/dev to start up the Docker development environment
3. Run the YawBehaviourTree 
```bash
ros2 run planner yaw_behaviour_tree
```
4. Follow the prompt the input Yaw (degrees)