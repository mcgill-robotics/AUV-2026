#!/bin/bash
# Original script from dusty-nv/jetson-containers
# https://github.com/dusty-nv/jetson-containers/blob/master/packages/ros/ros_environment.sh
# Author: Dustin Franklin (dusty-nv) — NVIDIA
# License: 
#
# Copyright (c) 2026, NVIDIA CORPORATION. All rights reserved.

# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

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
