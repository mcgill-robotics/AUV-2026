# Vision Package

## Launch File Usage
In most cases, the [vision pipeline](#vision-pipeline) launch file will be the main entry point for running vision nodes. All other launch files are intended for testing and development of individual nodes, and are run as intermediate steps in the vision pipeline. It expects models to already be trained (see [Steps to train a model](./model_pipeline/README.md)) and placed in the `ros2_ws/vision/models` directory.
### VIsion Pipeline
Launches the entire vision pipeline, image enhancement -> object detection -> object mapping.

```bash
ros2 launch vision vision_pipeline.launch.py
```

Configurations for the launch file are pulled from [config/vision_pipeline.yaml](config/vision_pipeline.yaml) by default. Parameters are listed below:

| Category | Parameter | Default | Description |
|----------|-----------|---------|-------------|
| `camera` | `front_cam_topic` | `/zed/zed_node/stereo/image_rect_color` | Input topic for front camera images |
| `camera` | `down_cam_topic` | `/sensors/down_cam/image_raw` | Input topic for down camera images |
| `image_enhancement` | `front_enhanced_topic` | `/vision/front_cam/image_enhanced` | Output topic for enhanced front camera images |
| `image_enhancement` | `down_enhanced_topic` | `/vision/down_cam/image_enhanced` | Output topic for enhanced down camera images |
| `object_detection` | `front_detections_topic` | `/vision/front_cam/detections` | Output topic for front camera object detections |
| `object_detection` | `down_detections_topic` | `/vision/down_cam/detections` | Output topic for down camera object detections |
| `object_detection` | `front_model_relative_path` | `frontcam.pt` | Path to front camera object detection model file, relative to the `src` directory of the vision package |
| `object_detection` | `down_model_relative_path` | `downcam.pt` | Path to down camera object detection model file, relative to the `ros2_ws/vision/models` directory |
| `object_map` | `object_map_topic` | `/vision/object_map` | Output topic for object map |
| `object_map` | `vio_pose_topic` | `/vision/vio_pose` | Output topic for VIO pose computed by ZED SDK|
| `general` | `sim` | `false` | Set to true to enable simulation mode (for Unity sim) |
| `general` | `debug` | `false` | Set to true to enable debug logging in all nodes (there should only be debug logging in object map) |

Parameters can be overriden via command line:

```bash
ros2 launch vision vision_pipeline.launch.py PARAM_NAME:=VALUE
```

Currently, the overridable parameters are:
- "sim"
- "front_model_relative_path"
- "down_model_relative_path"

All remaining parameters can be set by editing the [config/vision_pipeline.yaml](config/vision_pipeline.yaml) file directly, to avoid CLI clutter.

When working with the [Unity Simulation](https://github.com/mcgill-robotics/auv-sim-unity/), the messages arriving to the `ros_tcp_endpoint` should be compressed images. This is what simulation mode send by default.

## Information on Individual Nodes

### Image Collection

Collects training images from ZED2i (front) and down-facing cameras for YOLO object detection.

#### Usage

##### Jetson Nano

```bash
# Ensure ZED2i camera and/ror down cam node are running in seperate terminals, then:
ros2 run vision image_collection
```

#### Commands

Once running, use these interactive commands:

- `m` - Manual capture mode
- `a` - Automatic capture mode (timed intervals)
- `s` - Show statistics
- `q` - Quit

##### Manual Capture Workflow

```
Enter command: m
Select camera [f]ront or [d]own: f

Action: c  (capture image)
Action: v  (view latest - Jetson only)
Action: b  (back to main menu)
```

##### Automatic Capture Workflow

```
Enter command: a
Select camera [f]ront or [d]own: d
Interval in seconds: 2

(Press Enter to stop)
```

#### Output

Images are saved with timestamps:
- `data_front_cam/image_YYYYMMDD_HHMMSS_microseconds.jpg`
- `data_down_cam/image_YYYYMMDD_HHMMSS_microseconds.jpg`

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `front_cam_data_dir` | `data_front_cam` | Directory for front camera images |
| `down_cam_data_dir` | `data_down_cam` | Directory for down camera images |
| `front_cam_topic` | `/zed/zed_node/stereo/image_rect_color` | Front camera topic |
| `down_cam_topic` | `/down_cam/image_raw` | Down camera topic |

##### Custom Configuration Example

```bash
# Organize by date/location
ros2 run vision image_collection --ros-args \
  -p front_cam_data_dir:=~/datasets/pool_2024_11_02/front \
  -p down_cam_data_dir:=~/datasets/pool_2024_11_02/down
```

### Image Enhancement

Enhances images using various interchangeable algorithms from both front and down cameras. Output encoding will match input encoding.

#### Usage

```bash
colcon build --packages-select vision --symlink-install
source install/setup.bash
# Ensure ZED2i camera and/or down cam node are running in separate terminals, then:
ros2 launch vision image_enhancement.launch.xml
```

This will launch the image enhancement nodes for both cameras with default parameters. Arguments can be set in the launch file or via command line:

```bash
ros2 launch vision image_enhancement.launch.xml PARAM_NAME:=VALUE
```

Algorithms can be set in the node scripts (`front_image_enhancement.py` and `down_image_enhancement.py`). See the [wiki](https://github.com/mcgill-robotics/AUV-2026/wiki/3.1-Vision) for a list of algorithms.

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `front_cam_topic` | `/zed/zed_node/stereo/image_rect_color` | Input front camera topic |
| `down_cam_topic` | `/sensors/down_cam/image_raw` | Input down camera topic |
| `front_cam_enhanced_topic` | `/vision/front_cam/image_enhanced` | Output enhanced front camera topic |
| `down_cam_enhanced_topic` | `/vision/down_cam/image_enhanced` | Output enhanced down camera topic |
| `sim` | `false` | Enable simulation mode |

### Object Map

The `object_map` node aggregates 2D object detections from the YOLO inference node (`object_detection`) and projects them into a persistent 3D world map relative to the AUV. It uses the ZED SDK neural depth sensing for 3D estimation and a custom `ObjectTracker` pipeline to maintain object permanence across complex frames.

#### Tracking Architecture
The tracking system operates using a multi-step data association pipeline designed to handle noise, occlusions, and false positives in underwater environments:

1. **3D Projection (ZED SDK Integration):** 
   - 2D Bounding Boxes published by the YOLO node are intercepted by `ObjectMapNode`.
   - These 2D bounds are ingested into the ZED SDK (`ZEDDetection::process_detections`), which leverages neural depth sensing and stereo triangulation to extract real-world 3D coordinates relative to the left camera lens.
   - The coordinates are then transformed into the global `world` frame using the VIO positional data published by the ZED SDK.

2. **Persistent Tracking (Kalman Filter + Hungarian Algorithm):**
   - **State Estimation:** Every tracked object is assigned a constant-velocity Kalman Filter (`KalmanFilter.cpp`). This allows the tracker to predict where an object *should* be in the next frame based on its previous trajectory.
   - **Cost Matrix:** When new detections arrive, the system computes the Mahalanobis Distance between the predicted positions of existing tracks and the new incoming measurements.
   - **Data Association:** [The Hungarian Matcher](https://github.com/mcximing/hungarian-algorithm-cpp) (`ObjectTracker::match_tracks`) solves the linear assignment problem using this cost matrix, pairing incoming detections to existing tracks while minimizing overall distance error.
   - **Track Lifecycle:** Unmatched detections spawn new `TENTATIVE` tracks. If a track is consistently matched across several frames, it becomes `CONFIRMED`. If a confirmed track loses sight of the object, the Kalman Filter continues predicting its location for a configured `max_age` duration before the track is finally purged.

3. **Categorization:** 
   - Dynamically groups objects into `unique_objects` (only one instance allowed to exist globally), `floor_objects` (locked to pool floor), and `surface_objects` (locked to pool surface).

#### Physical Constraints
Applies game-specific logic to improve tracking accuracy and patch noisy vision data:
- **Gate Top Crop:** Ignores the bottom portion of gate bounding boxes to prevent depth errors caused by the pool floor.
- **Z-Axis Locking:** Forces specific objects (e.g., bins, tables) to snap to the known depths of the pool floor or surface.
- **Midpoint Refinement:** Centers the gate position between detected shark and sawfish markers.
- **Octagon Inheritance:** Inherits the XY position of the octagon directly from the more reliably detected table.

#### Usage

The object map node is launched automatically as part of the `vision_pipeline.launch.py`. It subscribes to the 2D detections publised by the YOLO node and the camera images/depth.

#### Parameters

All configuration parameters for the `object_map` node, including Kalman/Hungarian tracker tuning thresholds and constraint feature toggles, are found in `config/vision_pipeline.yaml`.