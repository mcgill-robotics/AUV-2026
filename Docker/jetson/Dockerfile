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
# Make pip work
ENV PYPI_URL=https://pypi.jetson-ai-lab.io/jp6/cu126
ENV PIP_INDEX_URL=https://pypi.jetson-ai-lab.io/jp6/cu126 
ENV PIP_TRUSTED_HOST=pypi.jetson-ai-lab.io


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

# Install SSH server and set up SSH access
# Hardcoding a password is not secure but eh
RUN apt-get update && apt-get install -y \
   openssh-server \
   xauth \
   && mkdir /var/run/sshd \
   && mkdir /root/.ssh \
   && chmod 700 /root/.ssh \
   && echo "Port 222" >> /etc/ssh/sshd_config \
   && echo "PermitRootLogin yes" >> /etc/ssh/sshd_config \
   && echo 'root:jetson' | chpasswd \
   && rm -rf /var/lib/apt/lists/*
 
RUN echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}" > /etc/nv_tegra_release && \
  apt-get update -y || true && apt-get install -y --no-install-recommends \
  build-essential \
  cmake \
  curl \
  gnupg2 \
  less \
  libdraco-dev \
  libgeographic-dev \
  libusb-1.0-0-dev \
  python3 \
  python3-dev \
  python3-pip \
  python3-setuptools \
  wget \
  zlib1g-dev \
  zstd && \
  export PIP_INDEX_URL="${PYPI_URL}" && \
  pip install protobuf && \
  wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run \
  ${ZED_SDK_URL} && \
  chmod +x ZED_SDK_Linux_JP.run ; ./ZED_SDK_Linux_JP.run silent skip_python && \
  rm -rf ZED_SDK_Linux_JP.run && \
  rm -rf /var/lib/apt/lists/*

# Install other common development tools
# Keep these in alphabetical
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \ 
    apt-get update && apt-get install -y --no-install-recommends \
    git \
    git-lfs \
    iproute2 \
    iputils-ping \
    jq \
    libpq-dev \
    lsb-release \
    nano \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstools \
    python3-wheel \
    sudo \
    tar \
    tmux \
    udev \
    usbutils \
    v4l-utils \
    vim \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

#Install graphics, video, c++ dev libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
   g++ \
   libc++-dev \
   libavcodec-dev \
   libavdevice-dev \
   libavformat-dev \
   libavutil-dev \
   libegl1-mesa-dev \
   libepoxy-dev \
   libgl1-mesa-dev \
   libglew-dev \
   libjpeg-dev \
   libpng-dev \
   libswscale-dev \
   libwayland-dev \
   libxkbcommon-dev \
   ninja-build \
   && add-apt-repository ppa:deadsnakes/ppa -y \
   && rm -rf /var/lib/apt/lists/* \
   && apt-get clean

# copy our configs for zed2i
COPY SN33688213.conf /usr/local/zed/settings/

# Install missing dependencies and wanted packages from source
WORKDIR /root/dependencies_ws/src
RUN wget https://github.com/ros/xacro/archive/refs/tags/${XACRO_VERSION}.tar.gz -O - | tar -xvz && mv xacro-${XACRO_VERSION} xacro && \
    wget https://github.com/ros/diagnostics/archive/refs/tags/${DIAGNOSTICS_VERSION}.tar.gz -O - | tar -xvz && mv diagnostics-${DIAGNOSTICS_VERSION} diagnostics && \
    wget https://github.com/ament/ament_lint/archive/refs/tags/${AMENT_LINT_VERSION}.tar.gz -O - | tar -xvz && mv ament_lint-${AMENT_LINT_VERSION} ament-lint && \
    wget https://github.com/cra-ros-pkg/robot_localization/archive/refs/tags/${ROBOT_LOCALIZATION_VERSION}.tar.gz -O - | tar -xvz && mv robot_localization-${ROBOT_LOCALIZATION_VERSION} robot-localization && \
    wget https://github.com/stereolabs/zed-ros2-interfaces/archive/refs/tags/${ZED_MSGS_VERSION}.tar.gz -O - | tar -xvz && mv zed-ros2-interfaces-${ZED_MSGS_VERSION} zed-ros2-interfaces && \
    wget https://github.com/ros-drivers/nmea_msgs/archive/refs/tags/${NMEA_MSGS_VERSION}.tar.gz -O - | tar -xvz && mv nmea_msgs-${NMEA_MSGS_VERSION} nmea_msgs && \
    wget https://github.com/ros/angles/archive/refs/tags/${ANGLES_VERSION}.tar.gz -O - | tar -xvz && mv angles-${ANGLES_VERSION} angles && \
    wget https://github.com/ros-perception/point_cloud_transport/archive/refs/tags/${POINTCLOUD_TRANSPORT_VERSION}.tar.gz -O - | tar -xvz && mv point_cloud_transport-${POINTCLOUD_TRANSPORT_VERSION} point_cloud_transport && \
    wget https://github.com/ros-perception/point_cloud_transport_plugins/archive/refs/tags/${POINTCLOUD_TRANSPORT_PLUGINS_VERSION}.tar.gz -O - | tar -xvz && mv point_cloud_transport_plugins-${POINTCLOUD_TRANSPORT_PLUGINS_VERSION} point_cloud_transport_plugins && \
    wget https://github.com/ros2/rmw_cyclonedds/archive/refs/tags/${RMW_CYCLONEDDS_VERSION}.tar.gz -O - | tar -xvz && mv rmw_cyclonedds-${RMW_CYCLONEDDS_VERSION} rmw_cyclonedds && \
    wget https://github.com/ros-geographic-info/geographic_info/archive/refs/tags/${GEOGRAPHIC_INFO_VERSION}.tar.gz -O - | tar -xvz && mv geographic_info-${GEOGRAPHIC_INFO_VERSION} geographic-info && \
    wget https://github.com/pal-robotics/backward_ros/archive/refs/tags/${BACKWARD_ROS_VERSION}.tar.gz -O - | tar -xvz && mv backward_ros-${BACKWARD_ROS_VERSION} backward_ros && \
    wget https://github.com/ros-drivers/usb_cam/archive/refs/tags/${USB_CAM_VERSION}.tar.gz -O - | tar -xvz && mv usb_cam-${USB_CAM_VERSION} usb_cam && \
    wget https://github.com/mavlink/mavros/archive/refs/tags/2.10.1.tar.gz -O - | tar -xvz && mv mavros-2.10.1 mavros && \
    cp -r geographic-info/geographic_msgs/ . && \
    cp -r mavros/mavros_msgs/ . && \
    rm -rf mavros && \
    rm -rf geographic-info

# ROS packages that do not depend on OpenCV/that ZED wrapper doesn't depend on can be installed here
RUN apt-get update && apt-get install -y --no-install-recommends \
   ros-$ROS_DISTRO-joy \
   ros-$ROS_DISTRO-joy-teleop \
   ros-$ROS_DISTRO-foxglove-bridge \
   ros-$ROS_DISTRO-mavros \
   && rm -rf /var/lib/apt/lists/* \
   && apt-get clean

# Build the dependencies from source against NVIDIA OpenCV
# There are 2 workspaces so that all the dependencies installed source
# do not clutter our ros2_ws
WORKDIR /root/dependencies_ws

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && \
  colcon build --parallel-workers $(nproc) --symlink-install \
  --event-handlers console_direct+ --base-paths src \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' \
  ' --no-warn-unused-cli' "

# Get CUDA dependencies to install torch and torchvision
WORKDIR /root
RUN wget https://developer.download.nvidia.com/compute/cusparselt/0.8.1/local_installers/cusparselt-local-tegra-repo-ubuntu2204-0.8.1_0.8.1-1_arm64.deb && \
    sudo dpkg -i cusparselt-local-tegra-repo-ubuntu2204-0.8.1_0.8.1-1_arm64.deb && \
    sudo cp /var/cusparselt-local-tegra-repo-ubuntu2204-0.8.1/cusparselt-*-keyring.gpg /usr/share/keyrings/ && \
    sudo apt-get update && \
    sudo apt-get -y install cusparselt-cuda-12 && \
	rm -rf cusparselt-local-tegra-repo-ubuntu2204-0.8.1_0.8.1-1_arm64.deb

ARG CUSPARSE_SOLVER_NAME="libcudss-linux-sbsa-0.6.0.5_cuda12-archive"
RUN mkdir -p tmp_cudss && cd tmp_cudss && \
    curl -L -O https://developer.download.nvidia.com/compute/cudss/redist/libcudss/linux-sbsa/${CUSPARSE_SOLVER_NAME}.tar.xz && \
    tar xf ${CUSPARSE_SOLVER_NAME}.tar.xz && \
    sudo cp -a ${CUSPARSE_SOLVER_NAME}/include/* /usr/local/cuda/include/ && \
    sudo cp -a ${CUSPARSE_SOLVER_NAME}/lib/* /usr/local/cuda/lib64/ && \
	rm -rf ${CUSPARSE_SOLVER_NAME} ${CUSPARSE_SOLVER_NAME}.tar.xz

# Install ultralytics, avoid reinstalling numpy and opencv (torch, tensorflow and torchvision installed in the .toml file, cuda supported cause pip index is Jetpack6 one)
RUN pip install --ignore-install sympy && pip install cython --upgrade
RUN git clone https://github.com/ultralytics/ultralytics.git && \
    cd ultralytics && \
	sed -i '/"numpy>=1.23.0"/d; /"opencv-python>=4.6.0"/d; /"numpy<2.0.0"/d' pyproject.toml && \
	pip install -e . && \
    cd .. && \
	rm -rf ultralytics


# Source env variables
RUN echo "source /opt/ros/humble/install/setup.bash && source /root/dependencies_ws/install/setup.bash && source /root/AUV-2026/ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "tmux" >> ~/.bashrc
CMD ["bash"]
