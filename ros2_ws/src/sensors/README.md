# Integrate Down Cam in Ros2
## Description of issue
To integrate downcam-facing USB camera into the ROS2 worksplace using the usb_cam package so that its image stream and camera info can be accessed through standardized ROS2 topics. We confirmed that the camera publishes data to the correct ROS2 topic and that the data can be visualized (e.g., using ros2 topic echo or rqt_image_view).

## Setup Instructions
- Connect the down cam to the jetson nano
## Launch Instructions
- Run the launch file inside the Docker container (see documentation about how to use the Docker container)
- Open a terminal, ensure that you are in the correct branch and run the following commands:
```
1. cd AUV-2026/Docker/jetson
2. docker compose up -d
3. docker exec -it jetson-douglas-1 bash
4. cd ros2_ws
5. colcon build --> this should build successfully without errors
6. source install/setup.bash
7. ros2 launch sensors down_cam.launch.py 
```
## Results:
- Image and camera info topics are active with ros2 topic list
- ROS2 topic is displayed with echo /down_cam/camera_info
- In a separate terminal, visualize images with rqt_image_view /down_cam/image_raw or using foxglove.