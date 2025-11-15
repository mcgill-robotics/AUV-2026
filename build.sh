set -euo pipefail

ROS_DISTRO=humble
ROS_INSTALL=/opt/ros/$ROS_DISTRO/setup.bash

if [ -f "$ROS_INSTALL" ]; then
  echo -e "\n=== Sourcing ROS 2 base ($ROS_DISTRO) ==="
  set +u
  source "$ROS_INSTALL"
  set -u
else
  echo "ERROR: cannot find ROS install at $ROS_INSTALL" >&2
  exit 1
fi

# Check if running on Jetson
if [[ -f /proc/device-tree/model ]] && grep -qi "NVIDIA Jetson" /proc/device-tree/model; then
  echo "Detected Jetson environment. Proceeding to build zed-ros2-wrapper."
  # Place your zed-ros2-wrapper build commands here, e.g.:
  # colcon build --packages-select zed-ros2-wrapper
else
  echo "Not running on Jetson. Skipping build of zed-ros2-wrapper."
fi

if [ -d "ros2_ws/src" ]; then
  echo -e "\n=== Building all ROS2 packages in ros2_ws except zed-ros2-wrapper ==="
  # Find all packages except zed-ros2-wrapper
  EXCLUDE_PKGS=()
  if [ -d "ros2_ws/src/zed-ros2-wrapper" ]; then
    EXCLUDE_PKGS+=(--packages-skip zed-ros2-wrapper zed_ros2 zed_components zed_msgs zed_wrapper)
  fi
  cd ros2_ws
  rosdep install --from-paths src --ignore-src -r -y --skip-keys="zed zed_msgs zed_components"
  colcon build "${EXCLUDE_PKGS[@]}"
  cd -
else
  echo "ERROR: ros2_ws/src directory not found. Skipping colcon build." >&2
fi
