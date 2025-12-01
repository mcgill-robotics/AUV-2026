# Define the L4T_VERSION argument
ARG L4T_VERSION=l4t-r36.4.0
ARG IMAGE_NAME=dustynv/ros:humble-desktop-l4t-r36.4.0

FROM ${IMAGE_NAME}

ARG ZED_SDK_MAJOR=5
ARG ZED_SDK_MINOR=0
ARG ZED_SDK_PATCH=0
ARG L4T_MAJOR=36
ARG L4T_MINOR=4
# Optional: Override ZED SDK URL
ARG CUSTOM_ZED_SDK_URL=""

# ROS2 distribution
ARG ROS2_DIST=humble

# ZED ROS2 Wrapper dependencies version
ARG XACRO_VERSION=2.0.8
ARG DIAGNOSTICS_VERSION=4.0.0
ARG AMENT_LINT_VERSION=0.12.11
ARG ROBOT_LOCALIZATION_VERSION=3.5.3
ARG ZED_MSGS_VERSION=5.0.0
ARG NMEA_MSGS_VERSION=2.0.0
ARG ANGLES_VERSION=1.15.0
ARG GEOGRAPHIC_INFO_VERSION=1.0.6
ARG POINTCLOUD_TRANSPORT_VERSION=1.0.18
ARG POINTCLOUD_TRANSPORT_PLUGINS_VERSION=1.0.11
ARG RMW_CYCLONEDDS_VERSION=1.3.4
ARG BACKWARD_ROS_VERSION=1.0.7
ARG RQT_GUI_VERSION=1.1.7
ARG ROBOT_STATE_PUBLISHER_VERSION=3.5.2
ARG USB_CAM_VERSION=0.8.1
ARG IMAGE_COMMON_VERSION=3.1.12
ARG MAVROS_VERSION=2.10.1

# Make pip work
ENV PYPI_URL=https://pypi.jetson-ai-lab.io/jp6/cu126
ENV PIP_INDEX_URL=https://pypi.jetson-ai-lab.io/jp6/cu126 
ENV PIP_TRUSTED_HOST=pypi.jetson-ai-lab.io

# protobuf is a pip package needed by zed sdk, for some reason nvidias pip server killed protobuf in its available packages so you need 
# the default non nvidia pip index just to download protobuf
ENV PIP_INDEX_FOR_NON_NVIDIA=https://pypi.python.org/simple

# Set up environment (we in paris because official docs are in paris)
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Paris 
ENV ROS_DISTRO=${ROS2_DIST}

# ZED SDK link
ENV ZED_SDK_URL="https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/5.0/ZED_SDK_Tegra_L4T36.4_v5.0.0.zstd.run"

# Check that this SDK exists
RUN if [ "$(curl -L -I "${ZED_SDK_URL}" -o /dev/null -s -w '%{http_code}\n' | head -n 1)" = "200" ]; then \
        echo "The URL points to something."; \
    else \
        echo "The URL does not point to a .run file or the file does not exist."; \
        exit 1; \
    fi

# Disable apt-get warnings
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt-get update || true && apt-get install -y --no-install-recommends apt-utils dialog && \
    rm -rf /var/lib/apt/lists/*

# Re install opencv and cv_bridge so we can have opencv contrib algos for image enhancement, code from https://github.com/AastaNV/JEP/blob/master/script/install_opencv4.10.0_Jetpack6.1.sh
RUN rm -rf /usr/lib/aarch64-linux-gnu/libopencv* \
           /usr/include/opencv4
#RUN sudo apt -y purge *libopencv*, try above way because in docker build, libopencv not detected by apt. only detected when inside container after build

# Set arguments for OpenCV version and folder
USER root
ARG OPENCV_VERSION=4.10.0
ARG OPENCV_FOLDER=/opt/opencv

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/usr/local/lib/python3.10/site-packages/:$PYTHONPATH

# Install required dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git curl unzip \
        libgtk2.0-dev pkg-config \
        libavcodec-dev libavformat-dev libswscale-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        python3.10-dev python3-numpy \
        libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev \
        libv4l-dev v4l-utils qv4l2 libunwind-dev && \
    rm -rf /var/lib/apt/lists/*

# Download OpenCV and OpenCV contrib
RUN mkdir -p ${OPENCV_FOLDER} && cd ${OPENCV_FOLDER} && \
    chmod -R 777 /usr/local/ && \
    curl -L https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip -o opencv-${OPENCV_VERSION}.zip && \
    curl -L https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip -o opencv_contrib-${OPENCV_VERSION}.zip && \
    unzip opencv-${OPENCV_VERSION}.zip && \
    unzip opencv_contrib-${OPENCV_VERSION}.zip && \
    rm opencv-${OPENCV_VERSION}.zip opencv_contrib-${OPENCV_VERSION}.zip

# Build and install OpenCV
RUN mkdir /root/tmp && cd ${OPENCV_FOLDER}/opencv-${OPENCV_VERSION} && \
    mkdir -p release && cd release && \
    cmake -D WITH_CUDA=ON \
          -D WITH_CUDNN=ON \
          -D CUDA_ARCH_BIN="8.7" \
          -D CUDA_ARCH_PTX="" \
          -D OPENCV_GENERATE_PKGCONFIG=ON \
          -D OPENCV_EXTRA_MODULES_PATH=${OPENCV_FOLDER}/opencv_contrib-${OPENCV_VERSION}/modules \
          -D WITH_GSTREAMER=ON \
          -D WITH_LIBV4L=ON \
          -D BUILD_opencv_python3=ON \
          -D BUILD_TESTS=OFF \
          -D BUILD_PERF_TESTS=OFF \
          -D BUILD_EXAMPLES=OFF \
          -D CMAKE_BUILD_TYPE=RELEASE \
          -D CMAKE_INSTALL_PREFIX=/root/tmp .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# Re install cv_bridge
RUN pip3 uninstall cv_bridge cv-bridge
WORKDIR /root/cv_bridge/src
RUN git clone -b "humble" https://github.com/ros-perception/vision_opencv.git                  
WORKDIR /root/cv_bridge/
RUN colcon build --symlink-install

# Source env variables
RUN echo "source /opt/ros/humble/install/setup.bash && source /root/dependencies_ws/install/setup.bash && source /root/AUV-2026/ros2_ws/install/setup.bash && source /root/cv_bridge/install/setup.bash" >> ~/.bashrc
CMD ["bash"]
