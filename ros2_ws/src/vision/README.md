# Vision Package

This package contains the front-camera detection pipeline, the persistent 3D object map, optional image enhancement nodes, and the model-training/export helpers used to build the deployed detectors.

The current runtime center of gravity is:
- [vision_pipeline.launch.py](launch/vision_pipeline.launch.py)
- [front_cam_object_detection_node.py](vision/object_detection/front_cam_object_detection_node.py)
- [object_map.cpp](vision/object_map.cpp)
- [object_tracker.cpp](vision/object_map/object_tracker.cpp)

## Current Architecture

```mermaid
flowchart LR
    A["Front camera / ZED<br/>RGB + depth source"] --> B["front_cam_object_detection<br/>infer + ZED 3D + frame sync"]
    S["sensors / state_aggregator<br/>official pose + TF"] -->|"state/pose"| B
    B -->|"VisionDetectionFrame"| C["object_map<br/>camera->world + tracking"]
    B -->|"front camera feed + CameraInfo"| F["Foxglove / debug<br/>visualization"]
    B -->|"optional /vision/vio_pose"| E["debug / future fusion<br/>vision pose measurement"]
    T["Static TF<br/>auv_link -> zed_camera_center -> zed_left_camera_frame -> optical"] --> C
    S -->|"official TF: pool_link -> auv_link"| C
    C -->|"VisionObjectArray"| D["/vision/object_map<br/>persistent world map"]
    D --> F
```

Node summary:
- `Front camera / ZED`: Provides the raw RGB images and ZED depth data used to build 3D detections.
- `sensors / state_aggregator`: Publishes the official aggregated AUV pose and the official `pool_link -> auv_link` TF.
- `front_cam_object_detection`: Runs the detector, asks ZED for 3D object positions, and packages each frame with one synchronized AUV pose.
- `Static TF`: Describes the fixed mounting relationship between the AUV body frame and the front camera frames.
- `object_map`: Converts camera-frame detections into world-frame objects and tracks them over time.
- `/vision/object_map`: Exposes the persistent world-frame map as `VisionObjectArray`.
- `Foxglove / debug`: Visualizes the front camera feed, camera calibration, and mapped detections.
- `debug / future fusion`: Carries the optional vision-only VIO pose output for debugging or later estimator fusion.

### Front camera path
The front camera node is responsible for:
- running model inference
- ingesting 2D boxes into the ZED SDK
- recovering 3D detections in the camera frame
- snapshotting one AUV pose per grabbed frame
- publishing one synchronized detection-frame message
- optionally publishing a front-camera debug feed, `CameraInfo`, depth images, and a VIO measurement/debug TF

The synchronized message types live in `auv_msgs`:
- [VisionDetection.msg](../auv_msgs/msg/VisionDetection.msg)
- [VisionDetectionFrame.msg](../auv_msgs/msg/VisionDetectionFrame.msg)

`VisionDetectionFrame` is the input to `object_map`. It carries:
- `header.stamp`: frame timestamp
- `header.frame_id`: 3D detection frame, currently `zed_left_camera_frame`
- `auv_pose`: AUV pose snapshot for that frame, typically from `state/pose`
- `detections`: camera-frame 3D detections plus the original model 2D bbox

### Object map path
The object map node:
- subscribes to `VisionDetectionFrame`
- resolves the TF chain from `auv_link` to `zed_left_camera_frame`
- transforms detections from camera frame to world frame in C++
- tracks objects over time with a Kalman + Hungarian pipeline
- publishes the persistent map as:
  - [VisionObject.msg](../auv_msgs/msg/VisionObject.msg)
  - [VisionObjectArray.msg](../auv_msgs/msg/VisionObjectArray.msg)

### How the front camera node works
At a high level, the front camera node does this each frame:
1. `grab()` one ZED frame
2. snapshot one AUV pose for that same frame
3. run detector inference on the RGB image
4. optionally filter detections near image borders
5. optionally crop the lower half of gate boxes before depth ingestion
6. ingest the 2D boxes into the ZED SDK
7. retrieve 3D detections in `zed_left_camera_frame`
8. publish one `VisionDetectionFrame`

This keeps the expensive 2D-to-3D step close to the ZED SDK while still leaving world-frame conversion to `object_map`.

### How object_map works
At a high level, `object_map` does this for each `VisionDetectionFrame`:
1. read the synchronized AUV pose from the message
2. resolve `auv_link -> zed_left_camera_frame` from TF
3. transform all camera-frame detections into `pool_link`
4. filter and track them with `ObjectTracker`
5. apply game-specific constraints and refinements
6. publish the persistent world-frame map as `VisionObjectArray`

### Sensors / pose ownership
The official vehicle pose and TF should come from the `sensors` package:
- `state/pose`
- `pool_link -> auv_link`

Vision can still publish a VIO measurement for debugging or future fusion:
- `/vision/vio_pose`
- optionally `pool_link -> auv_vio_link`

This is intentionally separate from the official `auv_link` TF so Foxglove and downstream nodes do not confuse vision VIO with the fused vehicle pose.

## Frames And TF

The front-camera pipeline currently uses:
- `pool_link`: world / inertial frame
- `auv_link`: official AUV frame
- `auv_vio_link`: optional debug VIO frame from vision
- `zed_camera_center`: static camera mount frame
- `zed_left_camera_frame`: 3D detection frame used by the ZED SDK and `VisionDetectionFrame`
- `zed_left_camera_frame_optical`: optical frame used for image visualization and `CameraInfo`

The launch file publishes the static chain:
- `auv_link -> zed_camera_center`
- `zed_camera_center -> zed_left_camera_frame`
- `zed_left_camera_frame -> zed_left_camera_frame_optical`

This split matters:
- 3D detections stay in `zed_left_camera_frame`
- image-like topics use `zed_left_camera_frame_optical`

That layout is what Foxglove expects for correct camera visualization.

## Main Entry Point

Launch the package with:

```bash
ros2 launch vision vision_pipeline.launch.py
```

Configuration is loaded from:
- [config/vision_pipeline.yaml](config/vision_pipeline.yaml)

Common launch overrides:

```bash
ros2 launch vision vision_pipeline.launch.py sim:=true
ros2 launch vision vision_pipeline.launch.py compressed:=false
ros2 launch vision vision_pipeline.launch.py front_model_relative_path:=models/local_rfdetr_onnx_test
```

## What The Main Launch Actually Starts

The main launch currently focuses on the front-camera path:
- optional front image enhancement
- front camera object detection
- static TF publishers for the front camera
- object map

The down-camera nodes still exist, but they are not part of the default runtime pipeline in `vision_pipeline.launch.py`.

## Important Topics

### Front camera topics
- `/vision/front_cam/detection_frame`
  - synchronized `VisionDetectionFrame`
- `/vision/front_cam/detection_frame/annotated`
  - single published front-camera feed topic
  - can show annotated or raw frames on the same topic
- `/vision/front_cam/detection_frame/camera_info`
  - `CameraInfo` built from ZED left-camera calibration
- `/vision/front_cam/detection_frame/depth`
  - optional depth image output
- `/vision/vio_pose`
  - optional VIO-based AUV pose measurement from vision

### Object map
- `/vision/object_map`
  - `VisionObjectArray` containing the persistent world-frame map

### Down camera
- `/vision/down_cam/detections`
  - `vision_msgs/Detection2DArray`

## Runtime Services

### Front camera
- `/vision/front_cam/toggle_annotated_image`
  - `std_srvs/Trigger`
  - toggles the front-camera feed topic between annotated and raw

- `/vision/front_cam/toggle_collection`
  - `auv_msgs/AutomaticCapture`
  - enables/disables timed collection and sets the interval

- `/image_collection/toggle_manual_front_collection`
  - compatibility alias for the same `AutomaticCapture` behavior

Example request:

```json
{ "data": true, "time_interval": 1.0 }
```

### Down camera
- `/down_cam_object_detection/toggle_collection`
  - `auv_msgs/AutomaticCapture`

## Front Camera Debug Feed

The front debug feed intentionally uses one topic:
- `/vision/front_cam/detection_frame/annotated`

Behavior:
- if annotation is enabled, detections are drawn on the image
- if annotation is disabled, the same topic publishes the raw image

This keeps Foxglove simple because you do not need to switch image topics to compare raw vs annotated output.

## 2026 Tracking / Mapping Logic

The tracker has several 2026-specific constraints:
- gate refinement from `search_rescue` and `survey_repair`
- board refinement from `ambulance`, `firetruck`, `fire`, and `blood`
- observer-facing yaw chosen using the current AUV position
- large-structure spawn filtering near pipes and other large structures
- configurable table/octagon publish-time inheritance modes
- floor/surface Z locking for selected classes

The main tuning and counts are configured in:
- [config/vision_pipeline.yaml](config/vision_pipeline.yaml)

## Front Camera Pose Modes

`front_cam_object_detection` still supports two pose modes:
- `pose_source: auv_pose`
- `pose_source: zed_vio`

Current recommended setup:
- `enable_vio: false`
- `pose_source: auv_pose`
- use `state/pose` from `sensors`

If VIO is enabled:
- the node can publish `/vision/vio_pose`
- it can optionally publish `pool_link -> auv_vio_link`

If `enable_vio` is false, the node skips ZED positional tracking entirely.

## Configuration Notes

Some especially important front-camera parameters are:
- `model_detection_threshold`
- `depth_confidence_threshold`
- `zed_depth_maximum_distance`
- `zed_depth_minimum_distance`
- `zed_positional_tracking_depth_min_range`
- `zed_depth_stabilization`
- `zed_camera_resolution_sim`
- `zed_camera_resolution_real`
- `zed_camera_fps_sim`
- `zed_camera_fps_real`
- `zed_camera_flip_mode`
- `zed_self_calib`
- `zed_enable_right_side_measure`
- `zed_sdk_verbose`
- `zed_sdk_gpu_id`
- `zed_enable_image_enhancement`
- `zed_open_timeout_sec`
- `zed_async_grab_camera_recovery`
- `zed_grab_compute_capping_fps`
- `zed_enable_image_validity_check`
- `zed_optional_opencv_calibration_file`
- `zed_brightness`
- `zed_contrast`
- `zed_hue`
- `zed_saturation`
- `zed_sharpness`
- `zed_gamma`
- `zed_gain`
- `zed_exposure`
- `zed_auto_exposure_gain`
- `zed_auto_whitebalance`
- `zed_whitebalance_temperature`
- `zed_led_status`
- `enable_gate_top_crop`
- `gate_top_crop_ratio`
- `enable_border_exclusion`
- `border_exclusion_margin_px`
- `border_exclusion_labels`
- `publish_camera_info`
- `publish_annotated_image`
- `publish_depth_image`
- `collection_interval_seconds`
- `enable_vio`
- `pose_source`

`enable_border_exclusion` is a simple filter applied before ZED ingestion. For configured labels such as `gate`, any 2D detection touching the image border within `border_exclusion_margin_px` is dropped. This helps reject partial detections at the left/right image edges that often produce unstable depth or incorrect clipped 3D positions.

The ZED camera-control parameters are split into two groups:
- init-time controls such as resolution, FPS, and flip mode
- runtime ISP controls such as brightness, exposure, gain, gamma, white balance, and LED status

The current defaults intentionally start close to the ZED ROS 2 wrapper defaults for a ZED 2i:
- real camera: `HD1080` at `15 FPS`
- common stereo video tuning: saturation `4`, sharpness `4`, gamma `8`
- auto exposure/gain and auto white balance enabled by default

For the current front-camera pipeline, these controls are intentionally limited to settings that keep the left rectified camera model intact. The node still assumes:
- detections come from the left camera
- `VisionDetectionFrame` uses `zed_left_camera_frame`
- `CameraInfo` describes the left rectified camera

Some especially important object-map parameters are:
- `new_object_min_distance_threshold`
- `large_structure_labels`
- `pipe_labels`
- `min_large_structure_separation_m`
- `min_large_structure_pipe_separation_m`
- `enable_gate_midpoint_refinement`
- `enable_board_icon_refinement`
- `enable_octagon_from_table_xy`
- `enable_table_from_octagon_xy`
- `enable_table_octagon_xy_midpoint`
- `max_pipe_distance`
- `unique_objects`
- `floor_objects`
- `surface_objects`
- `max_per_class`

The table/octagon pair has three mutually exclusive publish-time strategies:
- `enable_octagon_from_table_xy`: suppress direct octagon output and publish the octagon at the table XY
- `enable_table_from_octagon_xy`: suppress direct table output and publish the table at the octagon XY
- `enable_table_octagon_xy_midpoint`: publish both at the midpoint XY when both are visible, or fall back to the seen object’s XY for both when only one is visible

These modes only change the published `VisionObjectArray`. They do not modify the underlying `ObjectTracker` state for table or octagon.

Only one of those three flags should be enabled at a time. If more than one is enabled, `object_map` will fail at startup with a configuration error.

## Package Scope

This repository package is currently a mixed runtime + model-development package:
- runtime ROS nodes live under `vision/`
- launch/config live under `launch/` and `config/`
- training/export helpers live under `model_pipeline/` and `models/`

That is convenient for development, but it also means the package is heavier than a pure runtime deployment package.

## Training And Model Export

The training/export helpers are still under:
- [model_pipeline/README.md](model_pipeline/README.md)
- [models/export_model.py](models/export_model.py)

Those tools are not part of the runtime launch path, but they are still the source of the deployed detector packages under `models/`.
