# AUV-2026

This project is currently under development

Ahoy! This project contains software intended to run on the custom-built Douglas AUV to compete at RoboSub2026 on behalf of McGill Robotics.

This project is maintained by the McGill Robotics Club and was developed by its members - students of McGill University.

## Packages Architecture

This repository contains several ROS 2 packages that make up the AUV software stack. Click on any package below to read its specific documentation:

- **[auv_msgs](ros2_ws/src/auv_msgs/README.md)**: Custom ROS 2 message, service, and action definitions.
- **[bringup](ros2_ws/src/bringup/README.md)**: Top-level launch files for starting the entire AUV system or major subsystems.
- **[controls](ros2_ws/src/controls/README.md)**: Provides the AUV control loops for depth, attitude, and planar motion (X/Y), computing control efforts as `geometry_msgs/Wrench` messages.
- **[planner](ros2_ws/src/planner/README.md)**: Provides high-level mission planning and competition task execution using `py_trees` behavior trees.
- **[propulsion](ros2_ws/src/propulsion/README.md)**: Handles low-level actuation by transforming high-level control efforts into PWM signals for each thruster.
- **[sensors](ros2_ws/src/sensors/README.md)**: Handles low-level processing, coordinate transformations, and conversion of raw sensor outputs (IMU, Depth, DVL).
- **[telemetry](ros2_ws/src/telemetry/README.md)**: Foxglove Studio layouts and telemetry configurations for AUV monitoring during pool tests and competition.
- **[teleop](ros2_ws/src/teleop/README.md)**: Manual teleoperation for the AUV via an Xbox controller.
- **[vision](ros2_ws/src/vision/README.md)**: Contains the front-camera detection pipeline, the persistent 3D object map, image enhancement nodes, and model-training helpers.

## 1. Setting Up the Dev Environment

### Prerequisites
Before setting up the dev environment, you should have the following software installed:
-   `Docker` (either through [Docker Desktop](https://www.docker.com/products/docker-desktop/) or **CLI**)
-   `git` installed.
-   `Visual Studio Code`
-   [`Visual Studio Code Remote Containers Plugin`](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

---

### Method 1: VS Code Dev Containers (CPU Only)


If you have the software installed, follow these instructions:

1. `git clone` this repository on your local environment.
   
2. Open the repo in **VS Code**.

3. In the bottom left, you should see a little blue/green box. This will bring up the VSCode dialogue.
  (Keyboard Shortcut: `CTRL + SHIFT + P` or `CMD + SHIFT + P`)

![image](https://github.com/user-attachments/assets/69cfa5b7-9513-4aa1-b797-e9adcc8aa68a)

4. Select ***"Dev Containers: Reopen in container"***
   
5. Select the option that matches your hardware
   -  `AUV Dev (CPU)`: CPU or Integrated Graphics
   -  `AUV Dev (NVIDIA)`: NVIDIA Graphics Card

6. VS Code will automatically load necessary files and configure the dev environment. This can take minutes to load.

Once loading is complete, you're ready to develop! 


### Method 2: Docker CLI (Advanced)
*Use this when not using VS Code as your primary text editor*

1.  **Navigate to the dev folder:**
    ```bash
    cd Docker/dev
    ```

2.  **Allow GUI windows (Linux only):**
    ```bash
    xhost +si:localuser:root
    ```

3.  **Start the environment:**
    Choose the command matching your hardware:

    *   **NVIDIA GPU Users** (Requires [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)):
        ```bash
        docker compose up -d --build nvidia
        ```
    *   **Standard CPU / Integrated Graphics**:
        ```bash
        docker compose up -d --build cpu
        ```

4.  **Enter the container:**
    ```bash
    # For NVIDIA
    docker exec -it auv-dev-nvidia bash
    
    # For CPU
    docker exec -it auv-dev-cpu bash
    ```
For more details, see [Docker/dev/README.md](Docker/dev/README.md).

## 2. Building ROS Packages
Once inside the container, use the build script. It automatically detects if you have the ZED SDK and builds accordingly.

```bash
    cd /home/douglas/AUV-2026
    ./build.sh
    source ros2_ws/install/setup.bash
```

Note the script can be run from anywhere in the container and will always build inside the `AUV-2026/ros2_ws` folder.

Multiple build flags are available:
-  `./build.sh` : Default Release Build, works for most cases
-  `./build.sh -c` : Clean Build, removes previous build artifacts (`build/`, `install/`, `log/` folders)
-  `./build.sh -d` : Debug Build, compiles with debug symbols and no optimizations and serial compilation
-  `./build.sh -o` : Offline Build, prevents CMake from attempting to download missing dependencies
-  `./build.sh -p <package_name>` : Build a specific package and its required dependencies (as determined by colcon)

Since the debug `-d` build is single-threaded for easier debugging, it may take very long to compile. As such, it should only be used in tandem with the `-p` flag to build a specific package. A typical debug build command would be `./build.sh -cd -p <package_name>`. Note that the `-c` will only clean the build artifacts for the specified package and its dependencies.

## 3. Running the AUV

To launch the entire AUV software stack, use the `bringup.launch.py` file:

```bash
ros2 launch bringup bringup.launch.py
```

Currently, this bringup script launches the following subsystems:
- **Sensors** (`sensors.launch.py`): Aggregates and processes raw data from the IMU, Depth Sensor, and DVL.
- **Propulsion** (`propulsion.launch.py`): Handles thruster allocation and hardware communication.
- **Controls** (`controls.launch.py`): Manages the vehicle's PID controllers and superimposer.
- **Vision** (`vision_pipeline.launch.py`): Runs the object detection, point cloud mapping, and TF pipeline.
- **Telemetry** (`dashboard.launch.py`): Launches the Foxglove bridge for remote monitoring and debugging.
- **ROS TCP Endpoint** (`endpoint.py`): Facilitates communication with the Unity Simulator (only when `sim:=true`).

**⚠️ Keep in mind that the bringup script is designed to launch all subsystems by default, including the following packages for which special care must be taken:**

- The `sensors` package launches drivers for the DVL. The DVL must always be underwater when configured by the driver during operation, otherwise it may overheat
- The `propulsion` package re-arms all thrusters on launch, which can be dangerous if the AUV is not properly secured.
- The `controls` package attempts to maintain the AUV's underwater position on launch, which may lead to unexpected thruster activation and movement.

See the [Launch Options](#launch-options) section below for instructions on how to launch the bringup script with specific subsystems disabled, such as when running in simulation or when testing specific packages.

### Launch Options

- `sim:=true|false` (default: `false`): Launch the AUV in simulation mode. *(Note: To set up and run the Unity simulator itself, please refer to the [auv-sim-unity repository](https://github.com/mcgill-robotics/auv-sim-unity).)*
- `vision:=true|false` (default: `true`): Launch the vision pipeline. Set this to `false` when running the simulation on a machine without an NVIDIA GPU, as the simulator will directly provide the object map.
- `enable_object_detection:=true|false` (default: `true`): Enable object detection inference globally. If set to `false`, the vision pipeline will still launch and publish raw camera feeds, but no AI inference models will be loaded or run.
- `sensors:=true|false` (default: `true`): Launch the sensors package with real hardware drivers. 
- `propulsion:=true|false` (default: `true`): Launch the propulsion package.
- `controls:=true|false` (default: `true`): Launch the controls package.
- `telemetry:=true|false` (default: `true`): Launch the telemetry dashboard.
- `planner:=true|false` (default: `false`): Launch the mission planner. This is set to `false` by default as the mission planner is not yet fully integrated with the rest of the system.
- `auto_start_rosbag:=true|false` (default: `false`): Automatically start recording a rosbag when launching.
- `rosbag_prefix:=<string>` (default: `mission_bag`): The prefix used for the auto-started rosbag name.

Note that the `sim` flag is also passed to the launched packages, so for instance if `sim:=true`, no hardware drivers will be launched by the sensors package.

**Example: Running simulation without vision**
```bash
ros2 launch bringup bringup.launch.py sim:=true vision:=false
```

**Example: Full real pooltest bringup with rosbag**
```bash
ros2 launch bringup bringup.launch.py auto_start_rosbag:=true rosbag_prefix:=my_bag_name
```