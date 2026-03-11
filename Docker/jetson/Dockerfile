# ==============================================================================
# Jetson Dockerfile for AUV-2026
# Base: NVIDIA Isaac ROS (apt-based ROS 2 Humble) on L4T r36.4 / JetPack 6
# ==============================================================================

# 1. Official Isaac ROS Base (Pinned by digest for stability)
ARG IMAGE_NAME=nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c
FROM ${IMAGE_NAME}

SHELL ["/bin/bash", "-lc"]

# ===== Core Environment =====
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Paris
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV IS_JETSON=true

# NVIDIA's jetson pip server variables
ENV PYPI_URL=https://pypi.jetson-ai-lab.io/jp6/cu126
ENV PIP_INDEX_URL=https://pypi.jetson-ai-lab.io/jp6/cu126 
ENV PIP_TRUSTED_HOST=pypi.jetson-ai-lab.io
ENV PIP_INDEX_FOR_NON_NVIDIA=https://pypi.python.org/simple

ARG UBUNTU_RELEASE=22
ARG UBUNTU_MINOR_RELEASE=04

# ===== ZED SDK Configuration =====
ARG ZED_SDK_MAJOR=5
ARG ZED_SDK_MINOR=1
ARG ZED_SDK_PATCH=1
ARG L4T_MAJOR=36
ARG L4T_MINOR=4
ENV ZED_SDK_URL="https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}.${ZED_SDK_PATCH}/l4t${L4T_MAJOR}.${L4T_MINOR}/jetsons"
ARG ZED_SDK_RUN="ZED_SDK_installer.run"

# Fail fast: check ZED SDK URL
RUN if [ "$(curl -L -I "${ZED_SDK_URL}" -o /dev/null -s -w '%{http_code}\n' | head -n 1)" = "200" ]; then \
    echo "ZED SDK URL is valid."; \
    else \
    echo "ERROR: ZED SDK URL invalid."; exit 1; \
    fi

# ===== System Dependencies =====
# Fix Isaac ROS base quirk: remove expired yarn key that breaks apt update
RUN rm -f /etc/apt/sources.list.d/yarn.list && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    curl \
    git \
    git-lfs \
    gnupg2 \
    iproute2 \
    iputils-ping \
    jq \
    less \
    lsb-release \
    nano \
    sudo \
    tar \
    tmux \
    udev \
    usbutils \
    v4l-utils \
    vim \
    wget \
    python3 \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-wheel \
    libdraco-dev \
    libgeographic-dev \
    libusb-1.0-0-dev \
    libpq-dev \
    zlib1g-dev \
    zstd \
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
    && rm -rf /var/lib/apt/lists/* && apt-get clean

# ===== ZED SDK Installation =====
RUN echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}" > /etc/nv_tegra_release
RUN PIP_INDEX_URL="${PIP_INDEX_FOR_NON_NVIDIA}" pip install --no-cache-dir protobuf

RUN cd /tmp && \
    wget -q --no-check-certificate -O ${ZED_SDK_RUN} ${ZED_SDK_URL} && \
    chmod +x ${ZED_SDK_RUN} && \
    ./${ZED_SDK_RUN} silent skip_python skip_cuda && \
    chmod -R a+rX /usr/local/zed && ldconfig && \
    rm -rf ${ZED_SDK_RUN}
COPY Docker/jetson/SN33688213.conf /usr/local/zed/settings/

# ===== ROS 2 Dependencies =====
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-diagnostics \
    ros-$ROS_DISTRO-ament-lint \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-nmea-msgs \
    ros-$ROS_DISTRO-angles \
    ros-$ROS_DISTRO-point-cloud-transport \
    ros-$ROS_DISTRO-point-cloud-transport-plugins \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-geographic-msgs \
    ros-$ROS_DISTRO-backward-ros \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-mavros \
    ros-$ROS_DISTRO-mavros-msgs \
    ros-$ROS_DISTRO-image-common \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-joy-teleop \
    ros-$ROS_DISTRO-rosidl-default-generators \
    ros-$ROS_DISTRO-vision-msgs \
    ros-$ROS_DISTRO-zed-msgs \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

# ===== micro-ROS Setup =====
WORKDIR /opt/ros/micro_ros_ws
# add repository and 
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
RUN . /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build \
        --parallel-workers $(nproc) \
        --merge-install \
        --event-handlers console_direct+ \
        --base-paths src \
        --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    . /opt/ros/micro_ros_ws/install/setup.sh && \
    MICROROS_PREFIX=$(ros2 pkg prefix micro_ros_setup) && \
    ros2 run micro_ros_setup create_ws.sh src "$MICROROS_PREFIX/config/agent_ros2_packages.txt" "$MICROROS_PREFIX/config/agent_uros_packages.repos" && \
    colcon build \
        --packages-up-to micro_ros_agent $@ \
        --parallel-workers $(nproc) \
        --merge-install \
        --event-handlers console_direct+ \
        --cmake-args \
            "-DUAGENT_BUILD_EXECUTABLE=OFF" \
            "-DUAGENT_P2P_PROFILE=OFF" \
            "--no-warn-unused-cli" && \
    rm -rf build src log

# ===== CUDA Sparse Libraries & Python AI Stack =====
WORKDIR /root
RUN CUSPARSE_LIBRARY_NAME="cusparselt-local-tegra-repo-ubuntu${UBUNTU_RELEASE}${UBUNTU_MINOR_RELEASE}-0.8.1_0.8.1-1_arm64.deb" && \
    wget https://developer.download.nvidia.com/compute/cusparselt/0.8.1/local_installers/${CUSPARSE_LIBRARY_NAME} && \
    dpkg -i ${CUSPARSE_LIBRARY_NAME} && \
    cp /var/cusparselt-local-tegra-repo-ubuntu${UBUNTU_RELEASE}${UBUNTU_MINOR_RELEASE}-0.8.1/cusparselt-*-keyring.gpg /usr/share/keyrings/ && \
    apt-get update && \
    apt-get -y install cusparselt-cuda-12 && \
    rm -rf ${CUSPARSE_LIBRARY_NAME} && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

ARG CUSPARSE_SOLVER_NAME="libcudss-linux-sbsa-0.6.0.5_cuda12-archive"
RUN mkdir -p /tmp/cudss && cd /tmp/cudss && \
    curl -L -O https://developer.download.nvidia.com/compute/cudss/redist/libcudss/linux-sbsa/${CUSPARSE_SOLVER_NAME}.tar.xz && \
    tar xf ${CUSPARSE_SOLVER_NAME}.tar.xz && \
    cp -a ${CUSPARSE_SOLVER_NAME}/include/* /usr/local/cuda/include/ && \
    cp -a ${CUSPARSE_SOLVER_NAME}/lib/* /usr/local/cuda/lib64/ && \
    rm -rf /tmp/cudss
RUN pip install --no-cache-dir --extra-index-url ${PIP_INDEX_FOR_NON_NVIDIA} --ignore-installed sympy cython --upgrade
RUN pip install --no-cache-dir torch torchvision

# Pinned to v8.2.0 for reproducible builds
ARG ULTRALYTICS_REF=v8.2.0
RUN git clone --depth 1 --branch ${ULTRALYTICS_REF} https://github.com/ultralytics/ultralytics.git && \
    cd ultralytics && \
    sed -i '/"numpy>=1.23.0"/d; /"opencv-python>=4.6.0"/d; /"numpy<2.0.0"/d' pyproject.toml && \
    pip install --no-cache-dir --extra-index-url ${PIP_INDEX_FOR_NON_NVIDIA} . && \
    cd .. && \
    rm -rf ultralytics

# Install dependencies for vision
RUN pip install --no-cache-dir --extra-index-url ${PIP_INDEX_FOR_NON_NVIDIA} notebook \
    roboflow albumentations pandas && \
    pip install --no-cache-dir --ignore-install --upgrade typing_extensions

# ===== Environment Setup =====
ARG USERNAME=douglas
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN if getent group $USER_GID; then \
    groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1); \
    else \
    groupadd --gid $USER_GID $USERNAME; \
    fi \
    # 2. Create the user, dialout is IMU driver permissions
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -aG video,render,dialout -s /bin/bash $USERNAME 

ARG FIXUID_VERSION=0.6.0
RUN ARCH="$(dpkg --print-architecture)" && \
    curl -fsSL "https://github.com/boxboat/fixuid/releases/download/v${FIXUID_VERSION}/fixuid-${FIXUID_VERSION}-linux-${ARCH}.tar.gz" | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USERNAME\ngroup: $USERNAME\n" > /etc/fixuid/config.yml

USER $USERNAME

# set colcon home after adding user
ENV COLCON_HOME=/home/$USERNAME/.colcon
# move repo defaults.yaml to container, CI is run from Docker/jetson so need to go up 2 levels
COPY .colcon ${COLCON_HOME}/

# add ultralytics config dir to avoid storing settings in temp
RUN mkdir -p /home/$USERNAME/.config/Ultralytics

WORKDIR /home/$USERNAME/AUV-2026

# Beautifully simplified sourcing (.bashrc)
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /opt/ros/micro_ros_ws/install/local_setup.bash && \
    source /home/$USERNAME/AUV-2026/ros2_ws/install/setup.bash" >> ~/.bashrc 

ENTRYPOINT ["fixuid", "-q"]

CMD ["bash"]
