# AUV Dashboard

Foxglove Studio layouts and telemetry configuration for AUV monitoring during pool tests and competition.

# Launch Foxglove Bridge
ros2 launch telemetry dashboard.launch.py

## Layouts

Import layouts from `foxglove/` folder in Foxglove Studio (File → Import Layout):

| Layout | Purpose |
|--------|---------|
| `mission_control.json` | **Competition**: 3D scene, tabbed cameras, logs, and health status |
| `sensors.json` | **Sensor Validation**: Tabbed quadrants for IMU, Depth, DVL (Velocity/Odom), and State (Raw/3D) |
| `controls.json` | **PID Tuning**: Stacked Depth/Attitude plots, Control Efforts, and Tabbed Propulsion (Forces/PWM) |
| `teleop.json` | **Manual Control**: 3D visualizer + tabs for Teleop (Wrench), Thrusters (PWM), and Setpoints |
| `perception_debug.json` | **Vision**: Maximized top-down 3D scene + tabbed debug images (YOLO, Depth, Ref) |

## Topic Configuration

All topic names are centralized in `config/topics.yaml`. Reference this file when:
- Adding new layouts
- Writing nodes that subscribe/publish to standard topics
- Debugging topic name mismatches

## Launch Arguments

```bash
# Custom port
ros2 launch telemetry dashboard.launch.py port:=9090

# Bind to specific address
ros2 launch telemetry dashboard.launch.py address:=192.168.1.100
```

## Sharing Layouts

When you modify a layout in Foxglove:
1. Export: Layout icon (top left) → Export to file
2. Save to `foxglove/` folder, overwriting the existing JSON
3. Commit changes to git

## CLI Backup Commands

In the event Foxglove is unavailable, you can use the following terminal commands to manually interact with the AUV.

### Publishing Setpoints

```bash
# Depth setpoint (Note: Z is down in traditional ENU, so positive depth = negative Z visually)
ros2 topic pub /controls/depth_setpoint std_msgs/msg/Float64 "{data: 1.5}" 

# X, Y planar setpoints
ros2 topic pub /controls/x_setpoint std_msgs/msg/Float64 "{data: 2.0}" 
ros2 topic pub /controls/y_setpoint std_msgs/msg/Float64 "{data: 0.0}" 

# Attitude setpoint (Quaternion)
ros2 topic pub /controls/quaternion_setpoint geometry_msgs/msg/Quaternion "{x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}"
```

### Image Collection (Vision)

```bash
# Manually trigger a single frame capture for the front camera
ros2 service call /vision/image_collection/toggle_manual_front_collection std_srvs/srv/Trigger

# Manually trigger a single frame capture for the downward camera
ros2 service call /vision/image_collection/toggle_manual_down_collection std_srvs/srv/Trigger

# Start automatic front collection (1 image every 2.5 seconds)
ros2 service call /image_collection/toggle_front_collection auv_msgs/srv/AutomaticCapture "{data: true, time_interval: 2.5}"

# Stop automatic front collection
ros2 service call /image_collection/toggle_front_collection auv_msgs/srv/AutomaticCapture "{data: false, time_interval: 0.0}"

scp jetson@ubuntu.local:~/AUV-2026/data_front_cam/* .
```

