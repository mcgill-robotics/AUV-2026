#!/bin/bash
# Original script from dusty-nv/jetson-containers
# https://github.com/dusty-nv/jetson-containers/blob/master/packages/ros/ros_environment.sh
# Author: Dustin Franklin (dusty-nv) — NVIDIA
#
# We keep a local copy rather than fetching from upstream at build time for:
#   - Reproducibility: pinned to a known-good version, no surprise breakages
#   - Reliability: no dependency on GitHub availability during CI builds
#   - Patchability: modifications can be applied cleanly in-place

export CUDA_HOME="/usr/local/cuda"
export NVCC_PATH="$CUDA_HOME/bin/nvcc"
export PYTHONPATH="/opt/venv/lib/python${PYTHON_VERSION}/site-packages:/usr/lib/python3/dist-packages:${PYTHONPATH:-}"

function ros_source_env()
{
	if [ -f "$1" ]; then
		echo "sourcing   $1"
		source "$1"
  fi
}

if [[ "$ROS_DISTRO" == "melodic" || "$ROS_DISTRO" == "noetic" ]]; then
	ros_source_env "/opt/ros/$ROS_DISTRO/setup.bash"
else
	ros_source_env "$ROS_ROOT/install/setup.bash"

	#echo "ROS_PACKAGE_PATH   $ROS_PACKAGE_PATH"
	#echo "COLCON_PREFIX_PATH $COLCON_PREFIX_PATH"
	#echo "AMENT_PREFIX_PATH  $AMENT_PREFIX_PATH"
	#echo "CMAKE_PREFIX_PATH  $CMAKE_PREFIX_PATH"
fi

echo "ROS_DISTRO $ROS_DISTRO"
echo "ROS_ROOT   $ROS_ROOT"

export AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH}:${CMAKE_PREFIX_PATH}
