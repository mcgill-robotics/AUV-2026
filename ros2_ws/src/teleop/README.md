# Teleop

Manual teleoperation for the AUV via Gamepad or Keyboard.
The system features a dual-mode control architecture: **Controls Mode** (position/setpoint following) and **Propulsion Mode** (raw wrench output).

## Quick Start

### 1. Gamepad (Default)
Plug in any standard gamepad (Xbox, PS4, Logitech) and launch:
```bash
ros2 launch teleop teleop.launch.py
```

### 2. Keyboard
If you don't have a gamepad or want to test locally on your laptop, you can use your keyboard. Because the keyboard listener requires a raw terminal input stream (TTY), you must run it in a separate terminal from the launch file.

**Terminal 1 (Background Services):**
```bash
ros2 launch teleop teleop.launch.py use_keyboard:=true
```

**Terminal 2 (Controller Input):**
```bash
ros2 run teleop keyboard_to_joy
```
*(Keep Terminal 2 focused to send keyboard commands)*

## Control Modes

You must hold either the **LB** or **RB** buttons (or the equivalent keyboard toggles) to move the AUV.

### Controls Mode (`LB` or Keyboard `1`)
When engaged, the node latches the AUV's current position and operates in **Setpoint Proportional Lead** mode.
Using the joysticks/keyboard will push the setpoints in front of the robot. When you release the stick, the setpoint securely latches to hold the robot perfectly still.

### Propulsion Mode (`RB` or Keyboard `2`)
When engaged, the node bypasses all PID loops and outputs raw `Wrench` effort directly to the thrusters. The setpoints will seamlessly track the robot's movement in the background so there are no sudden jumps when you switch back to Controls mode.

## Mappings

### Gamepad (SDL2 Standard)
| Input | Action |
|-------|--------|
| **Left Stick** | Surge / Sway |
| **Right Stick (X)** | Yaw |
| **LT / RT** | Heave down / up |
| **D-pad ↑↓** | Pitch |
| **D-pad ←→** | Roll |
| **LB (Hold)** | **Controls Mode** |
| **RB (Hold)** | **Propulsion Mode** |

### Keyboard
| Key | Action |
|-----|--------|
| **W/S** | Surge Forward/Backward |
| **A/D** | Sway Left/Right |
| **E/Q** | Heave Up/Down |
| **J/L** | Yaw Left/Right |
| **I/K** | Pitch Forward/Backward |
| **U/O** | Roll Left/Right |
| **1 (Toggle)**| **Controls Mode** |
| **2 (Toggle)**| **Propulsion Mode** |
| **Space** | Stop all movement |

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/joy` | `sensor_msgs/Joy` | Input |
| `/state/pose` | `geometry_msgs/PoseStamped` | Input |
| `/controls/total_effort` | `geometry_msgs/Wrench` | Output (Propulsion) |
| `/controls/*_setpoint` | `std_msgs/Float64` | Output (Controls) |
| `/controls/quaternion_setpoint` | `geometry_msgs/Quaternion`| Output (Controls) |

## Configuration

Parameters, scale rates, and max offsets are stored in `config/teleop.yaml`.
