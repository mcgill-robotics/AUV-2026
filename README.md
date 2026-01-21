# AUV-2026

This project is currently under development

Ahoy! This project contains software intended to run on the custom-built Douglas AUV to compete at RoboSub2026 on behalf of McGill Robotics.

This project is maintained by the McGill Robotics Club and was developed by its members - students of McGill University.

## Setting Up the Dev Environment

### Prerequisites
Before setting up the dev environment, you should have the following software installed:
-   `Docker` (either through [Docker Desktop](https://www.docker.com/products/docker-desktop/) or **CLI**)
-   `git` installed.
-   `Visual Studio Code`
-   [`Visual Studio Code Remote Containers Plugin`](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

---

### Method 1: VS Code Dev Containers (CPU Only)
*Best for: Quick logic changes, non-graphical coding, Windows/Mac users without NVIDIA GPUs.*

If you have the software installed, follow these instructions:

1. `git clone` this repository on your local environment.
   
2. Open the repo in **VS Code**.

3. In the bottom left, you should see a little blue/green box. This will bring up the VSCode dialogue.
  (Keyboard Shortcut: `CTRL + SHIFT + P` or `CMD + SHIFT + P`)

![image](https://github.com/user-attachments/assets/69cfa5b7-9513-4aa1-b797-e9adcc8aa68a)

4. Select ***"Dev Containers: Reopen in container"***
   
5. VS Code will automatically load necessary files and configure the dev environment. This can take minutes to load.

Once loading is complete, you're ready to develop! 


### Method 2: Docker CLI (GPU & Advanced)
*Best for: Running ZED simulations, neural networks (YOLO), visualization (RQT/Rviz), and Linux users.*

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

Once inside the container, use the build script. It automatically detects if you have the ZED SDK and builds accordingly.

```bash
    cd /root/AUV-2026
    ./build.sh
    source ros2_ws/install/setup.bash
```

For more details, see [Docker/dev/README.md](Docker/dev/README.md).