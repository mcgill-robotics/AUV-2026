# AUV-2026

This project is currently under development

Ahoy! This project contains software intended to run on the custom-built Douglas AUV to compete at RoboSub2026 on behalf of McGill Robotics.

This project is maintained by the McGill Robotics Club and was developed by its members - students of McGill University.

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
    cd /root/AUV-2026
    ./build.sh
    source ros2_ws/install/setup.bash
```

Multiple build flags are available:
-  `./build.sh` : Default Release Build, works for most cases
-  `./build.sh -c` : Clean Build, removes previous build artifacts (`build/`, `install/`, `log/` folders)
-  `./build.sh -d` : Debug Build, compiles with debug symbols and no optimizations and serial compilation
-  `./build.sh -p <package_name>` : Build a specific package and its required dependencies (as determined by colcon)

Since the debug `-d` build is single-threaded for easier debugging, it may take very long to compile. As such, it should only be used in tandem with the `-p` flag to build a specific package. A typical debug build command would be `./build.sh -cd -p <package_name>`. Note that the `-c` will only clean the build artifacts for the specified package and its dependencies.