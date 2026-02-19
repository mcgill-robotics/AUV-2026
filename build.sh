#!/usr/bin/env bash
set -euo pipefail


# Do we want to remove install/build/log directories before building?
CLEAN_BUILD=false
DEBUG_BUILD=false
PACKAGE_TO_BUILD=""

while getopts ":cdp:" flag; do
    case "${flag}" in
        c) 
            echo "Flag -c was set. ros2_ws packages will be built from scratch"
            CLEAN_BUILD=true
            ;;
        d)
            echo "Flag -d was set. Debug build enabled."
            DEBUG_BUILD=true
            ;;
        p)
            echo "Flag -p was set. Building up to package: ${OPTARG}"
            PACKAGE_TO_BUILD="${OPTARG}"
            ;;
        *)
            echo "Usage: $0 [-c] [-d] [-p <package_name>]"
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
# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Set ROS distribution and install path
ROS_DISTRO=humble
ROS_INSTALL=/opt/ros/$ROS_DISTRO/setup.bash

# Initialize all git submodules
# Skip in CI since GitHub Actions checkout already fetches submodules (avoids permission issues)
if [ -z "${CI:-}" ]; then
    git config --global --add safe.directory "$(pwd)" || $SUDO git config --system --add safe.directory "$(pwd)"
    git config --global --add safe.directory $(pwd)/ros2_ws/src/Xsens_MTi_Driver || $SUDO git config --system --add safe.directory $(pwd)/ros2_ws/src/Xsens_MTi_Driver
    git config --global --add safe.directory $(pwd)/ros2_ws/src/ros-tcp-endpoint || $SUDO git config --system --add safe.directory $(pwd)/ros2_ws/src/ros-tcp-endpoint
    git config --global --add safe.directory $(pwd)/ros2_ws/src/zed-ros2-wrapper || $SUDO git config --system --add safe.directory $(pwd)/ros2_ws/src/zed-ros2-wrapper
    # do not exit on error
    set +e
    git submodule update --init --recursive
    SUBMODULE_ERROR_CODE=$?
    set -e
else
    echo "Running in CI, skipping git submodule update (already handled by checkout action)."
fi

if [ -f "$ROS_INSTALL" ]; then
  echo -e "\n=== Sourcing ROS 2 base ($ROS_DISTRO) ==="
  set +u
  source "$ROS_INSTALL"
  set -u
else
  echo "ERROR: cannot find ROS install at $ROS_INSTALL" >&2
  exit 1
fi

# Source dependencies workspace if available 
DEPS_WS_SETUP="/opt/dependencies_ws/install/setup.bash"
if [ -f "$DEPS_WS_SETUP" ]; then
  echo "   -> Sourcing dependencies_ws ..."
  set +u
  source "$DEPS_WS_SETUP"
  set -u
fi

# move into script directory
cd "$SCRIPT_DIR"
if [ -d "ros2_ws" ]; then
    cd ros2_ws
else
    echo "ERROR: ros2_ws directory not found in $SCRIPT_DIR." >&2
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

# the jetson container CI cannot have the ZED SDK installed since it the SDK will looks for symbols that are jetson specific which we cannot replicate in CI, so we just skip, so we define a variable that the CMakeLists can check to skip trying to find the ZED SDK in that case 
# IS_JETSON is defined in the Dockerfile for the jetson container builds
IS_JETSON_CI=false
if [ -n "${CI:-}" ] && [ "$IS_JETSON" = true ]; then
    echo "⚠️  CI jetson environment detected. Skipping linking ZED SDK to any packages since it cannot be installed in CI (zed-ros-wrapper will still be built)."
    IS_JETSON_CI=true
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
# echo -e "\n=== Checking Rosdep ==="

# # 4a. Initialize if missing
# if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
#     echo "   -> Initializing rosdep..."
#     $SUDO rosdep init
# fi

# 4b. Update if needed
# We run this as the current user (NO SUDO) so permissions stay correct
# echo "   -> Updating rosdep cache..."
# rosdep update

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

if [ -n "$PACKAGE_TO_BUILD" ]; then
    echo "    -> Building up to package: $PACKAGE_TO_BUILD"
    # Determine dependencies + packages to build, colcon build mapped to PKGS bash array
    mapfile -t PKGS < <( colcon list --packages-up-to "$PACKAGE_TO_BUILD" --names-only )
else
    # Set PKGS to all packages
    mapfile -t PKGS < <( colcon list --names-only )
fi


if [ "$CLEAN_BUILD" = true ]; then
    PACKAGES_TO_CLEAN=""
    echo "    -> Cleaning packages: ${PKGS[*]}"
    for pkg in "${PKGS[@]}"; do
        rm -rf "build/$pkg" "install/$pkg" "log/$pkg"
    done
fi

if [ "$DEBUG_BUILD" = true ]; then
    echo "    -> Performing Debug Build of packages: ${PKGS[*]}"
    # Output to console, interleaving build output for better visibility
    # Generate compile commands for use with linters e.g. VSCode
    # Run everything sequentially to find exact failure point
    colcon build \
        --event-handlers console_direct+ \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=RelWithDebInfo  \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            -DIS_JETSON_CI=$IS_JETSON_CI \
        --executor sequential \
        $([ -n "$PACKAGE_TO_BUILD" ] && echo "--packages-up-to $PACKAGE_TO_BUILD")
else
    echo "    -> Performing Release Build of packages: ${PKGS[*]}"
    # Output with cohesion, groups output by package
    # Utilize all available CPU cores
    colcon build \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DIS_JETSON_CI=$IS_JETSON_CI \
        --event-handlers console_cohesion+ \
        --parallel-workers $(nproc) \
        $([ -n "$PACKAGE_TO_BUILD" ] && echo "--packages-up-to $PACKAGE_TO_BUILD")
fi


# ---------------------------------------------------------
# 6. Cleanup
# ---------------------------------------------------------
if [ "$CAN_BUILD_ZED" = false ] && [ -f "$ZED_DIR/AMENT_IGNORE" ]; then
    rm "$ZED_DIR/AMENT_IGNORE"
fi

if [ SUBMODULE_ERROR_CODE -ne 0 ]; then
    echo -e "\n⚠️  Warning: git submodule update failed with code $SUBMODULE_ERROR_CODE. Submodules may not be properly initialized."
    echo "Please run 'git submodule update --init --recursive' manually to ensure all submodules are correctly set up."
fi

echo -e "\n✅ Build Complete. Don't forget to source:"
echo "source ros2_ws/install/setup.bash"
