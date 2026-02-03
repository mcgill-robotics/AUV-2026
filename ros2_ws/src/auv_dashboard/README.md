# AUV Dashboard

Foxglove Studio layouts and telemetry configuration for AUV monitoring during pool tests and competition.

# Launch Foxglove Bridge
ros2 launch auv_dashboard dashboard.launch.py

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
ros2 launch auv_dashboard dashboard.launch.py port:=9090

# Bind to specific address
ros2 launch auv_dashboard dashboard.launch.py address:=192.168.1.100
```

## Sharing Layouts

When you modify a layout in Foxglove:
1. Export: Layout icon (top left) → Export to file
2. Save to `foxglove/` folder, overwriting the existing JSON
3. Commit changes to git
