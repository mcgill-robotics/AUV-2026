# ROS 1 & ROS 2 Bridge Docker

This directory contains ROS 1 noetic and ROS 2 humble dockerfiles for aiding in the transition from ROS 1 to ROS 2. It supports custom messages for the AUV and is designed to simplify the transition from **AUV-2025** to **AUV-2026**.

> ⚠️ **Note:** This Docker environment is temporary and will be phased out once the transition to ROS 2 is complete and all subteams confirm that the bridge is no longer required.

## Dockerfiles

This repository contains **two Dockerfiles**:

1. **`Dockerfile.ros1`**  
   * Base image derived from the **AUV-2025** repository.  
   * Provides ROS 1 Noetic and standard dependencies.  
   * Does **not** include CUDA or ZED SDK dependencies.

2. **`Dockerfile.ros2`**  
   * Builds upon a ROS 2 Humble bridge builder from [TommyChangUMD/ros-humble-ros1-bridge-builder](https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder).
   * Contains both ROS 1 and ROS 2 environments (both with colcon) but designed to only run the ROS 2 portion, in parallel with the other Dockerfile in ROS 1.
   * Builds the **ROS 1 ↔ ROS 2 bridge** from source and integrates custom messages between ROS 1 Noetic (AUV-2025 repo) and ROS 2 Humble (AUV-2026 repo).
   * The bridge itself is cloned from our fork of the original repository:  
     ```bash
     git clone -b action_bridge_humble --depth=1 https://github.com/mcgill-robotics/ros1_bridge.git
     ```
     This fork is based on:  
     * [smith-doug/ros1_bridge](https://github.com/smith-doug/ros1_bridge.git) (fork with fixes)  
     * Which itself is based on [hsd-dev/ros1_bridge](https://github.com/hsd-dev/ros1_bridge)  
     * And ultimately [ros2/ros1_bridge](https://github.com/ros2/ros1_bridge)  
   * This forking occurs because there is no official humble version of the bridge.

## Usage
### System Setup
Build and start dockerfiles:
```bash
docker compose build
docker compose up -d
```

### ROS 1 Noetic Setup
1. Exec into ROS 1 dockerfile:
   ```bash
   docker exec -it ros1-noetic bash
   ```
2. Build ROS 1 environment:
   ```bash
   cd AUV-2025/catkin_ws
   catkin build
   source devel/setup.bash
   ```
3. Launch the ROS master:
   ```bash
   roscore
   ```

### ROS 2 Humble Setup
1. Exec into ROS 2 dockerfile:
   ```bash
   docker exec -it ros2-humble bash
   ```
2. Build ROS 2 environment:
   ```bash
   cd AUV-2026/ros2_ws
   colcon build
   source install/setup.bash
   ```
3. Start the bridge
   ```bash
   ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
   ```

### Next Steps
You can now run any code in the ROS 1 or ROS 2 dockerfiles and the topics should automatically bridge between both dockerfiles.

## Supported Platforms

The Dockerfiles have been tested on:
* Windows 11 (with WSL2)
* Ubuntu 20.04 (x86_64)
* NVIDIA L4T 35.4.1 and 36.4.4 (ARM64)

The bridge works consistently across these environments, but it is theoretically possible to integrate the bridge with the sim. This is currently not needed given that the sim has been transitioned to ROS 2.
