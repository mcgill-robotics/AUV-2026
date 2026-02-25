# Teleop

Manual teleoperation for AUV via Xbox controller.

## Quick Start

```bash
# With deadman switch (LB must be held)
ros2 launch teleop teleop.launch.py

# Without deadman (for testing)
ros2 launch teleop teleop.launch.py require_deadman:=false
```

## Controls (Xbox One S)

| Input | Action |
|-------|--------|
| **Left Stick** | Surge / Sway |
| **Right Stick** | Yaw |
| **LT / RT** | Heave down / up |
| **D-pad ↑↓** | Pitch |
| **D-pad ←→** | Roll |
| **LB (Hold)** | Deadman switch |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/joy` | `sensor_msgs/Joy` | Input |
| `/controls/effort` | `geometry_msgs/Wrench` | Output |

## Configuration

Parameters are stored in `config/teleop.yaml`. Edit defaults there or override via launch:

```bash
ros2 launch teleop teleop.launch.py require_deadman:=false
```

## Safety

- **Deadman**: Must hold LB to output force
- **Timeout**: Zeros output after 500ms of no input
