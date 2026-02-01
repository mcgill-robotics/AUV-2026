# ROS 2 Docker Container for Jetson Orin Nano

This repository contains a Dockerized ROS 2 Humble development environment for the **NVIDIA Jetson Orin Nano**, pre-configured with the ZED SDK, CUDA-accelerated OpenCV, and other essential tools. It is designed to provide a consistent, performant, and isolated workspace for our projects. It has been tested with:
*   **Hardware:** NVIDIA Jetson Orin Nano Developer Kit.
*   **Host OS:** L4T 36.4.0
*   **ZED SDK:** 5.0.0
  
## 🚀 Quick Start
1.  Give the container to access EGL display resources (required only once):
    ```bash
    sudo xhost +si:localuser:root
    ```
2.  **Launch the Container:**
    ```bash
    docker compose up -d
    ```
3.  **Open a Shell inside the Container:**
    ```bash
    docker exec -it jetson-douglas-1 bash
    ```

## ⚠️ Warnings and pitfalls

- When build the ROS package, make sure to source the ROS setup files first:
    ```bash
    source /opt/ros/humble/setup.bash
    source /root/dependencies_ws/install/setup.bash
    source /root/AUV-2026/ros2_ws/install/setup.bash
    ```
This is done by default when launching the container or opening a new bash shell. 

**DO NOT** run `source /opt/ros/humble/install/setup.bash` (notice the additional `install` in the path). This ROS environment was generated when building ROS from source in the [`base image`](https://hub.docker.com/layers/dustynv/ros/humble-desktop-l4t-r36.4.0/images/sha256-b8ee30b1ae189cfeeea755a7fd6b8aea74267f5c1bc0cfa4f19a6acec9d941e5), and does not contain any of the installed packages (e.g. via `apt-get install`).

- **OpenCV issue:**
NVIDIA’s OpenCV version (4.10) which comes from dustynv's base image and provides CUDA acceleration are not the same as Ubuntu’s OpenCV version (4.5.4) Any ROS package that depends on vision_opencv (like cv-bridge, image-view) must be built from source against NVIDIA’s OpenCV. You can check the dependencies by running `apt-get install -s ros-humble-[package name]` inside the clean base container. Problematic packages will have a line like: Conf libopencv-core4.5d (4.5.4+dfsg-9ubuntu4 Ubuntu:22.04/jammy [arm64]).

To address this issue, we are building all packages from source against NVIDIA's OpenCV version.
## 🏗️ Building from Source 

This container is built upon the [`dustynv/ros:humble-desktop-l4t-r36.4.0`](https://hub.docker.com/layers/dustynv/ros/humble-desktop-l4t-r36.4.0/images/sha256-b8ee30b1ae189cfeeea755a7fd6b8aea74267f5c1bc0cfa4f19a6acec9d941e5) image. The core of our setup is derived from the [ZED ROS2 Wrapper Dockerfile](https://github.com/stereolabs/zed-ros2-wrapper/blob/master/docker/Dockerfile.l4t-humble), which we have extended with our own packages.

To rebuild the container from scratch, run the following command:

```bash
docker compose build
```


