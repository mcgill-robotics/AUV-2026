# Image Collection Node

Collects training images from ZED2i (front) and down-facing cameras for YOLO object detection.

## Quick Start

### Testing (Dev Container)

```bash
# Terminal 1: Start fake camera
ros2 run vision fake_camera_node

# Terminal 2: Start image collection
ros2 run vision image_collection
```

### Production (Jetson Nano)

```bash
# ZED2i should already be running
# Just start collection:
ros2 run vision image_collection
```

## Usage

### Interactive Commands

```
m - Manual capture
a - Automatic capture (every X seconds)
s - Show statistics
q - Quit
```

### Manual Capture Example

```
Enter command: m
Select camera [f]ront or [d]own: f

Action: c  (capture)
Action: v  (view latest - Jetson only)
Action: b  (back to menu)
```

### Automatic Capture Example

```
Enter command: a
Select camera [f]ront or [d]own: d
Interval in seconds: 2

(Press Enter to stop)
```

## Output

Images saved to:
- `data_front_cam/image_YYYYMMDD_HHMMSS_microseconds.jpg`
- `data_down_cam/image_YYYYMMDD_HHMMSS_microseconds.jpg`

## Configuration

### Custom Data Directories

```bash
ros2 run vision image_collection --ros-args \
  -p front_cam_data_dir:=/path/to/front \
  -p down_cam_data_dir:=/path/to/down
```

### Custom Topics

```bash
ros2 run vision image_collection --ros-args \
  -p front_cam_topic:=/custom/topic \
  -p down_cam_topic:=/another/topic
```

### Via Launch File

**Note:** Launch files don't support interactive input well. Use `ros2 run` for manual collection. Launch file is provided for:
- Integration with larger launch systems
- Setting consistent parameters
- Future automated collection modes

```bash
ros2 launch vision image_collection.launch.py \
  front_cam_data_dir:=/custom/path
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `front_cam_data_dir` | `data_front_cam` | Directory for front camera images |
| `down_cam_data_dir` | `data_down_cam` | Directory for down camera images |
| `front_cam_topic` | `/zed2i/zed_node/stereo/image_rect_color` | Front camera topic |
| `down_cam_topic` | `/vision/down_cam/image_raw` | Down camera topic |

## Notes

- **View feature** (`v` command) only works on Jetson with display
- Images saved regardless of view capability
- Works offline (no WiFi required)
- Thread-safe operation
- Press `Ctrl+C` or `q` to exit cleanly