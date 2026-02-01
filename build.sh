#!/usr/bin/env bash
set -euo pipefail


# Do we want to remove install/build/log directories before building?
CLEAN_BUILD=false

while getopts ":c" flag; do
    case "${flag}" in
        c) 
            echo "Flag -c was set. ros2_ws packages will be built from scratch"
            CLEAN_BUILD=true
            ;;
        *)
            echo "Usage: $0 [-c]"
            exit 1
            ;;
    esac
done
shift $((OPTIND -1)) 

# ---------------------------------------------------------
# 0. Permission & User Detection
# ---------------------------------------------------------
# If we are NOT root (e.g. running on Host), use sudo for system commands
SUDO=""
if [ $(id -u) -ne 0 ]; then
    SUDO="sudo"
    echo "Running on Host as user: $(whoami) (will use sudo where needed)"
else
    echo "Running as Root (likely inside Docker)"
fi

# ---------------------------------------------------------
# 1. Environment Setup
# ---------------------------------------------------------
ROS_DISTRO=humble
ROS_INSTALL=/opt/ros/$ROS_DISTRO/setup.bash

# Initialize all git submodules
git config --global --add safe.directory "$(pwd)" || $SUDO git config --system --add safe.directory "$(pwd)"
git config --global --add safe.directory $(pwd)/ros2_ws/src/Xsens_MTi_Driver || $SUDO git config --system --add safe.directory $(pwd)/ros2_ws/src/Xsens_MTi_Driver
git config --global --add safe.directory $(pwd)/ros2_ws/src/ros-tcp-endpoint || $SUDO git config --system --add safe.directory $(pwd)/ros2_ws/src/ros-tcp-endpoint
git config --global --add safe.directory $(pwd)/ros2_ws/src/zed-ros2-wrapper || $SUDO git config --system --add safe.directory $(pwd)/ros2_ws/src/zed-ros2-wrapper

git submodule update --init --recursive

if [ -f "$ROS_INSTALL" ]; then
  echo -e "\n=== Sourcing ROS 2 base ($ROS_DISTRO) ==="
  set +u
  source "$ROS_INSTALL"
  set -u
else
  echo "ERROR: cannot find ROS install at $ROS_INSTALL" >&2
  exit 1
fi

if [ -d "ros2_ws" ]; then
    cd ros2_ws
else
    echo "ERROR: 'ros2_ws' directory not found. Run this from the repo root." >&2
    exit 1
fi

# ---------------------------------------------------------
# 2. Hardware/SDK Detection
# ---------------------------------------------------------
ZED_DIR="src/zed-ros2-wrapper"
CAN_BUILD_ZED=false

# Check 1: Is this a Jetson? (Hardware check)
if [[ -f /proc/device-tree/model ]] && grep -qi "NVIDIA Jetson" /proc/device-tree/model; then
    echo "✅ Detected Jetson hardware."
    CAN_BUILD_ZED=true
fi

# Check 2: Is ZED SDK installed? (Software check for x86 GPU)
if [ -d "/usr/local/zed" ]; then
    echo "✅ Detected ZED SDK installation."
    CAN_BUILD_ZED=true
fi

# ---------------------------------------------------------
# 3. Configure Ignore Rules
# ---------------------------------------------------------
if [ "$CAN_BUILD_ZED" = false ]; then
    echo "⚠️  No ZED SDK or Jetson detected. Ignoring ZED packages."
    SKIP_KEYS="zed zed_msgs zed_components"
    if [ -d "$ZED_DIR" ]; then
        touch "$ZED_DIR/AMENT_IGNORE"
    fi
else
    echo "🚀 ZED Environment Ready. Including ZED packages."
    # Skip OpenCV to avoid overwriting Jetson/CUDA optimized version
    SKIP_KEYS="zed zed_msgs zed_components opencv libopencv-dev python3-opencv opencv-python"
    if [ -f "$ZED_DIR/AMENT_IGNORE" ]; then
        rm "$ZED_DIR/AMENT_IGNORE"
    fi
fi

# ---------------------------------------------------------
# 4. Dependency Installation (Rosdep)
# ---------------------------------------------------------
echo -e "\n=== Checking Rosdep ==="

# 4a. Initialize if missing
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "   -> Initializing rosdep..."
    $SUDO rosdep init
fi

# 4b. Update if needed
# We run this as the current user (NO SUDO) so permissions stay correct
echo "   -> Updating rosdep cache..."
rosdep update

# echo -e "\n=== Installing Dependencies ==="
# # Uses sudo if on host, no sudo if in docker
# $SUDO apt-get update

# # Note: rosdep install handles sudo internally/interactively
# # Skipped as per user request (dependencies assumed present in Docker)
# # rosdep install --from-paths src --ignore-src -r -y \
# #    --skip-keys="$SKIP_KEYS"

# ---------------------------------------------------------
# 5. Build (colcon)
# ---------------------------------------------------------

echo -e "\n=== Building Workspace ==="

if [ "$CLEAN_BUILD" = true ]; then
    echo "    -> Removing ros2_ws/build ros2_ws/install ros2_ws/log for clean build"
    rm -rf build log install
fi

colcon build \
    --symlink-install \
    --parallel-workers $(nproc) \
    --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---------------------------------------------------------
# 6. Cleanup
# ---------------------------------------------------------
if [ "$CAN_BUILD_ZED" = false ] && [ -f "$ZED_DIR/AMENT_IGNORE" ]; then
    rm "$ZED_DIR/AMENT_IGNORE"
fi

echo -e "\n✅ Build Complete. Don't forget to source:"
echo "source ros2_ws/install/setup.bash"
