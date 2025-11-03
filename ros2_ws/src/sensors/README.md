# Down-Facing Camera Integration

## ```usb_cam``` package
This node uses the ```usb_cam``` ROS2 package to read frames from a USB camera and publish them over ROS with configurable resolution and frame ID.

## Topics
- Image: ``` /down_cam/image_raw ``` (```sensor_msgs/msg/Image```)
- Camera info: ``` /down_cam/camera_info ``` (```sensor_msgs/msg/CameraInfo```)

## Running
```
ros2 launch sensors down_cam_launch.py 
```

## Verifying it's publishing
```
ros2 topic list
ros2 topic echo /down_cam/camera_info
```