# Local Development Environment (Docker)

This directory contains the Docker setup for local development on x86_64 machines (Laptops/Desktops). It supports both **CPU-only** workflows and **NVIDIA GPU** accelerated workflows (ZED SDK, PyTorch).

## 1. Prerequisites

### For Everyone
*   **Docker Engine** & **Docker Compose**.
*   **Disk Space:** Ensure you have at least **25GB** free (CUDA and PyTorch are large).

### For NVIDIA GPU Users
If you have an NVIDIA GPU, you must install the **NVIDIA Container Toolkit** to allow Docker to access your hardware.

```bash
# Verify drivers are installed on host
nvidia-smi
```

### Install Toolkit (Ubuntu)
Follow the instructions on the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) page.

## 2. Choosing Your Mode

We have defined two services in `docker-compose.yml`. Choose the one matching your hardware:

| Service Name | Hardware Requirement | Features |
| :--- | :--- | :--- |
| `cpu` | Intel/AMD CPU, Integrated Graphics, Apple Silicon (slow) | ROS 2 Core, Sensors, Vision (CPU), No ZED Wrapper. |
| `nvidia` | NVIDIA GPU (RTX/GTX/Quadro) | **Full Support.** ZED SDK 5.1, CUDA 12.8, PyTorch GPU, Hardware Encoding. |

## 3. Quick Start

### Step 1: Allow GUI Access (Linux Only)
To view GUI tools like `rqt` or `rviz` from inside the container, run this **on your host terminal**:
```bash
xhost +si:localuser:root
```

### Step 2: Build & Run
Run the command corresponding to your hardware:

**Option A: NVIDIA GPU**
```bash
cd Docker/dev
docker compose up -d --build nvidia
docker exec -it auv-dev-nvidia bash
```

**Option B: CPU Only**
```bash
cd Docker/dev
docker compose up -d --build cpu
docker exec -it auv-dev-cpu bash
```

#### Speed up the setup (Optional)
If you haven't modified the Dockerfiles and want to skip the build process, you can pull the latest pre-built image from our DockerHub (updated whenever `main` is updated):

```bash
# Pull the latest stable image
docker compose pull nvidia  # or 'cpu'

# Start the container using the pulled image
docker compose up -d nvidia
```
*Note: If you have made changes to the Dockerfiles themselves, you must run `docker compose up -d --build` to see your changes.*

### Step 3: Build the Workspace
Once inside the container, use the build script. It automatically detects if you have the ZED SDK and builds accordingly.

```bash
cd /root/AUV-2026
./build.sh
source ros2_ws/install/setup.bash
```

## 4. Connecting to Unity Simulation & Networking

### Standardized ROS Domain
We standardize **`ROS_DOMAIN_ID=0`** across all our Docker containers (Dev, Jetson, Simulation). This ensures seamless communication between:
*   The Docker container and the Host machine.
*   The Docker container and the Unity Simulation (which defaults to ID 0).
*   Multiple robots/machines on the same network (e.g., Topside <-> AUV).

This setup uses **Host Networking**, meaning the container shares `localhost` with your computer.

### Windows Users (WSL 2)

To ensure the container can communicate with the simulation running on Windows, you must update your networking settings.

1.  **Enable Host Networking in Docker Desktop:**
    
    ![windows-docker-desktop-network-settings](https://github.com/user-attachments/assets/158a659b-9e18-4b9b-9e66-2231692e144a)

2.  **Set Networking Mode to Mirrored & Enable Host Address Loopback:**
    
    ![windows-wsl-network-settings](https://github.com/user-attachments/assets/ca6500b4-c4c0-4e9c-a0c6-30b789014e84)

### Instructions


1.  **Start Unity** on your host machine.
2.  **Inside Docker**, launch the TCP Endpoint:
    ```bash
    ros2 launch ros_tcp_endpoint endpoint.py
    ```
3.  **(NVIDIA Only)** To use ZED Simulation mode:
    ```bash
    ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx stream_address:=127.0.0.1 sim_mode:=true stream_port:=30000 ros_params_override_path:=Docker/dev/unity_sim_zed_config.yaml
    ```

## 5. Troubleshooting

*   **"could not select device driver nvidia":** You forgot to install the NVIDIA Container Toolkit (see Prerequisites).
*   **GUI not opening:** Ensure you ran `xhost +si:localuser:root` on the host before starting the container.
