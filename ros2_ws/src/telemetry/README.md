# AUV Dashboard

Foxglove Studio layouts and telemetry configuration for AUV monitoring during pool tests and competition.

# Launch Foxglove Bridge
ros2 launch telemetry dashboard.launch.py

## Foxglove Extensions

To fully utilize the dashboards, make sure to install our custom Foxglove extensions. You can download the latest pre-compiled `.foxe` releases from the links below and simply drag-and-drop them into your Foxglove Studio window to install them instantly:

- [Foxglove Custom Depth Decoder](https://github.com/mcgill-robotics/foxglove-custom-depth-decoder/releases): Natively decodes compressed 16-bit PNG depth maps, enabling Foxglove's Image Panel to display colormaps (Turbo/Rainbow) and exact millimeter depth readings on mouse hover.
- [Foxglove PyTrees Viewer](https://github.com/mcgill-robotics/foxglove-py-trees-viewer/releases): Provides an interactive, auto-layout canvas for `py_trees_ros` behavior trees, featuring live status colors, success-state latching, and embedded feedback messages.

## Layouts

Import layouts from `foxglove/` folder in Foxglove Studio (File → Import Layout):

| Layout                   | Purpose                                                                                           |
| ------------------------ | ------------------------------------------------------------------------------------------------- |
| `mission_control.json`   | **Competition**: 3D scene, tabbed cameras, logs, and health status                                |
| `sensors.json`           | **Sensor Validation**: Tabbed quadrants for IMU, Depth, DVL (Velocity/Odom), and State (Raw/3D)   |
| `controls.json`          | **PID Tuning**: Stacked Depth/Attitude plots, Control Efforts, and Tabbed Propulsion (Forces/PWM) |
| `teleop.json`            | **Manual Control**: 3D visualizer + tabs for Teleop (Wrench), Thrusters (PWM), and Setpoints      |
| `perception_debug.json`  | **Vision**: Maximized top-down 3D scene + tabbed debug images (YOLO, Depth, Ref)                  |
| `electrical_status.json` | **Electrical**: Real-time monitoring of battery levels, PWM, and raw sensor depth                 |

## Topic Configuration

All topic names are centralized in `config/topics.yaml`. 

For the 3D visualizer, settings such as the 3D AUV model offsets and performance toggles (like `publish_auv_model` to disable the 3D model if your machine is lagging) are located in `config/vision_foxglove.yaml`.

Reference these files when:
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

# This command securely copies all files from the remote Jetson's front camera data folder into your current local directory.
scp jetson@ubuntu.local:~/AUV-2026/data_front_cam/* .

```

## Rosbag Recording

The `telemetry` package includes a `rosbag_manager_node` that provides a service interface to start and stop high-performance `mcap` bag recordings with `zstd_fast` compression. It works alongside `topic_tools throttle` nodes to provide 5Hz image streams for recording without consuming massive disk space.

Configuration for profiles and the base save directory is handled in `config/rosbag_profiles.yaml`.

### Recording Profiles

Profiles dictate which topics are included or excluded via regex.
- `all`: Records everything. Excludes raw 30Hz images/depth and relies on the 5Hz throttled streams.
- `all_no_vision`: Records everything except vision and ZED topics.
- `vision`: Records only vision and ZED topics (using throttled images).
- `nav_and_controls`: Records core navigation, IMU, DVL, and propulsion data.

### CLI Recording Commands

Start recording a bag (e.g., using the `all` profile):
```bash
ros2 service call /rosbag_manager/control auv_msgs/srv/RosbagControl "{action: 0, bag_name: '', profile: 'all'}"
```
*(If `bag_name` is left empty, it will auto-generate a timestamped folder name).*

You can also automatically start recording when launching the AUV by passing these arguments to the `bringup` script:
```bash
ros2 launch bringup bringup.launch.py auto_start_rosbag:=true rosbag_prefix:=my_bag
```

Stop the current recording:
```bash
ros2 service call /rosbag_manager/control auv_msgs/srv/RosbagControl "{action: 1}"
```

> **Note**: To playback a recorded `.mcap` bag, we highly recommend simply dragging and dropping the file directly into Foxglove Studio. This allows for instant timeline scrubbing without network bottlenecks or topic collisions. **When viewing the bag in Foxglove, make sure to select the `*_throttled` topics (e.g. `.../compressed_throttled`) for the front and down camera RGB images**, since the massive raw 30Hz topics are intentionally excluded from the rosbag to save disk space!

If you need to playback the bag into the live ROS network for Software-in-the-Loop (SITL) testing (e.g. running the perception or planning nodes against recorded sensor data), run this in a terminal:

```bash
# Play a bag, publish the /clock topic for simulation time, and loop endlessly
ros2 bag play rosbags/<bag_name> --clock --loop
```
*(Make sure to set `use_sim_time:=true` on any nodes you run against the playback data).*

> **Behavior Tree Playback Gotcha**: The `py_trees_ros_viewer` GUI relies on active ROS Services to connect to a running tree. Since rosbags only play back topics, the GUI will show up empty during playback. To view a recorded behavior tree, use the command-line watcher instead, which taps directly into the recorded snapshot topic: 

```bash
py-trees-tree-watcher /planner_root_tree/snapshots -b
```

> **Note**: With the [extension](https://github.com/mcgill-robotics/foxglove-py-trees-viewer) you can natively visualize it in foxglove :)


