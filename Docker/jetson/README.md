# ROS 2 Docker Container for Jetson AGX Orin

This repository contains a Dockerized ROS 2 Humble development environment for the **NVIDIA Jetson AGX Orin**, pre-configured with CUDA-accelerated OpenCV, PyTorch, the ZED SDK, and other essential tools. It uses a **two-stage Docker build** for fast iteration.

## Architecture

The container stack uses two layers:

| Layer | Image | Rebuild Frequency |
|---|---|---|
| **Stage 1 (Base)** | `mcgillrobotics/auv_2026:isaac-ros-base` | Once per season / dependency upgrade |
| **Stage 2 (Application)** | `mcgillrobotics/auv_2026:ros2` | Every code change |

**Stage 1 (`Dockerfile.base`)** extends the official NVIDIA Isaac ROS Dev Base image and caches all slow/volatile dependencies:
- OpenCV 4.10 compiled from source with CUDA (Compute Capability 8.7)
- PyTorch, torchvision, torchaudio from Jetson AI Lab
- cuSPARSELt + cuDSS
- ZED SDK 5.1.1 + Python API
- Custom `cv_bridge` / `vision_opencv` compiled from source against CUDA OpenCV
- numpy==1.26.4 (tested with ZED + PyTorch)

**Stage 2 (`Dockerfile`)** extends the base image and handles workspace-specific setup (ROS packages, micro-ROS, Foxglove, Python ML deps, user config).

### Isaac ROS Base Image

```dockerfile
# Maps to: Isaac ROS 3.2 + JetPack 6.2 (L4T R36.4.3/R36.4.4)
# Tag date: 11/12/2025
# Do NOT upgrade JetPack without updating this tag.
FROM nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c
```

### Hardware
- **Hardware:** NVIDIA Jetson AGX Orin
- **Host OS:** JetPack 6.2 (L4T R36.4.3/R36.4.4), Ubuntu 22.04
- **ZED SDK:** 5.1.1

## 🚀 Quick Start

1. Give the container access to EGL display resources (required only once):
    ```bash
    sudo xhost +si:localuser:root
    ```
2. **Launch the Container:**
    ```bash
    docker compose up -d
    ```
3. **Open a Shell inside the Container:**
    ```bash
    docker exec -it jetson-douglas-1 bash
    ```

## 🏗️ Building

### First Time: Build the Base Image

This only needs to be done once (or when upgrading OpenCV, PyTorch, ZED SDK, etc.).

#### Option A: Standard Build (Default base)
```bash
docker build -f Dockerfile.base -t mcgillrobotics/auv_2026:isaac-ros-base ../../
docker push mcgillrobotics/auv_2026:isaac-ros-base
```

#### Option B: Version-Tagged Base Image (For Archiving / Upgrades)
If you want to tag the base image with a specific version or date (e.g., `2026-05-21`) to keep an archive:
1. Build and push the base image with your custom suffix:
   ```bash
   docker build -f Dockerfile.base -t mcgillrobotics/auv_2026:isaac-ros-base-2026-05-21 ../../
   docker push mcgillrobotics/auv_2026:isaac-ros-base-2026-05-21
   ```
2. When building the regular Stage 2 application container, override the base image using the `BASE_IMAGE` build argument:
   * **Via Docker CLI:**
     ```bash
     docker build --build-arg BASE_IMAGE=mcgillrobotics/auv_2026:isaac-ros-base-2026-05-21 -t mcgillrobotics/auv_2026:latest-jetson -f Docker/jetson/Dockerfile ../../
     ```
   * **Via Docker Compose:**
     ```bash
     docker compose build --build-arg BASE_IMAGE=mcgillrobotics/auv_2026:isaac-ros-base-2026-05-21
     ```


### Regular Builds

```bash
docker compose build
```

This only runs the Stage 2 Dockerfile.

### CI/CD Manual Builds & Version Tagging

To build and push custom-tagged images to Docker Hub (for archiving specific dates, releases, or milestones):
1. Navigate to the **Actions** tab of the GitHub repository.
2. Select the **Docker Images Main CI** workflow.
3. Click the **Run workflow** dropdown on the right.
4. Input your custom tag name in the `Docker Tag (e.g., 2026-01 or dev)` field (e.g., `2026-05-21` or `v1.2.0`).
5. Choose the branch and click **Run workflow**.

This manually triggered build will build and push the images for all matrix platforms to Docker Hub under the tag format `mcgillrobotics/auv_2026:<custom-tag>-<suffix>`, where the `<suffix>` corresponds to:
* `-jetson` (NVIDIA Jetson AGX Orin build)
* `-amd64` (Standard x86 CPU build)
* `-nvidia-amd64` (x86 NVIDIA GPU build)
* `-arm64` (Standard ARM64 CPU build)

For example, triggering the workflow with the tag `2026-05-21` will generate and push `mcgillrobotics/auv_2026:2026-05-21-jetson` for the Orin AGX.

### Network Configuration
We standardize **`ROS_DOMAIN_ID=0`** across all our Docker containers. This is pre-configured in the `docker-compose.yml` environment variables.

### User Permissions
- The container runs as the `douglas` user (UID 1000) with passwordless sudo.
- The entrypoint ([entrypoint.sh](../entrypoint.sh)) dynamically aligns GIDs for `render` and `video` groups to match the host, ensuring GPU access without manual permission adjustments.

## ⚠️ Important Notes

- **OpenCV:** CUDA-accelerated OpenCV 4.10 is source-compiled in the base image. The Python bindings are registered via a pip wheel built from the same source tree (`python_loader`). Do NOT install `opencv-python` via pip - it will overwrite the CUDA version.

- **`cv_bridge` (vision_opencv):** Do NOT install `cv_bridge` or `vision_opencv` via `apt-get` or standard pip. Doing so installs a CPU-only OpenCV library (4.5.4) that conflicts with our custom CUDA OpenCV (4.10.0), resulting in immediate segmentation faults at runtime. The custom-compiled, CUDA-linked `cv_bridge` workspace is pre-baked into the base image at `/opt/ros/vision_opencv_ws` and is automatically sourced by `entrypoint.sh` and `~/.bashrc`.

- **PyTorch:** Installed from the Jetson AI Lab index (`pypi.jetson-ai-lab.io/jp6/cu126`). The index prunes old versions, so the base image should be treated as an **immutable artifact** - never casually rebuild it.

- **NumPy:** Pinned to `1.26.4` in `constraints.txt`. This version is tested with both PyTorch and ZED SDK 5.1.1. The ZED Python API is installed with `--no-deps` to prevent NumPy version conflicts.
