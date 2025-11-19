# Image Collection

Collects training images from ZED2i (front) and down-facing cameras for YOLO object detection.

## Usage

### Jetson Nano

```bash
# Ensure ZED2i camera and/ror down cam node are running in seperate terminals, then:
ros2 run vision image_collection
```

## Commands

Once running, use these interactive commands:

- `m` - Manual capture mode
- `a` - Automatic capture mode (timed intervals)
- `s` - Show statistics
- `q` - Quit

### Manual Capture Workflow

```
Enter command: m
Select camera [f]ront or [d]own: f

Action: c  (capture image)
Action: v  (view latest - Jetson only)
Action: b  (back to main menu)
```

### Automatic Capture Workflow

```
Enter command: a
Select camera [f]ront or [d]own: d
Interval in seconds: 2

(Press Enter to stop)
```

## Output

Images are saved with timestamps:
- `data_front_cam/image_YYYYMMDD_HHMMSS_microseconds.jpg`
- `data_down_cam/image_YYYYMMDD_HHMMSS_microseconds.jpg`

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `front_cam_data_dir` | `data_front_cam` | Directory for front camera images |
| `down_cam_data_dir` | `data_down_cam` | Directory for down camera images |
| `front_cam_topic` | `/zed/zed_node/stereo/image_rect_color` | Front camera topic |
| `down_cam_topic` | `/down_cam/image_raw` | Down camera topic |

### Custom Configuration Example

```bash
# Organize by date/location
ros2 run vision image_collection --ros-args \
  -p front_cam_data_dir:=~/datasets/pool_2024_11_02/front \
  -p down_cam_data_dir:=~/datasets/pool_2024_11_02/down
```
