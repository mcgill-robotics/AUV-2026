This is the Docker container for the Jetson Orin Nano

L4T Version of the Nano: 36.4.4
Jetpack
ZED SDK: 5.0.0

To run the container, do `docker compose up -d`
Then `docker exec -it jetson-douglas-1 bash`

We used the ZED Dockerfile as the base and added our own packages. https://github.com/stereolabs/zed-ros2-wrapper/tree/master/docker 

To rebuild this container, use 
```
# example w/ Jetson with JP6.2 and ZED SDK v5.0.0 --> change if need be
./jetson_build_dockerfile_from_sdk_and_l4T_version.sh l4t-r36.4.0 zedsdk-5.0.0
```
For more information, checkout the link above.

OpenCV issue:
NVIDIA’s OpenCV version (4.10) which comes from dustynv's base image and provides CUDA acceleration are not the same as Ubuntu’s OpenCV version (4.5.4) Any ROS package that depends on vision_opencv (like cv-bridge, image-view) must be built from source against NVIDIA’s OpenCV. You can check the dependencies by running `apt-get install -s ros-humble-[package name]` inside the clean base container. Problematic packages will have a line like: Conf libopencv-core4.5d (4.5.4+dfsg-9ubuntu4 Ubuntu:22.04/jammy [arm64]).

To address this issue, we are building all packages from source against NVIDIA's OpenCV version.