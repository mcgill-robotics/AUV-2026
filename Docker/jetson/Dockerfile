# Define the L4T_VERSION argument
ARG L4T_VERSION=l4t-r36.4.0
ARG IMAGE_NAME=dustynv/ros:humble-desktop-l4t-r36.4.0

FROM ${IMAGE_NAME}

# Get uv
COPY --from=ghcr.io/astral-sh/uv:0.11.9 /uv /uvx /bin/

ARG ZED_SDK_MAJOR=5
ARG ZED_SDK_MINOR=1
ARG ZED_SDK_PATCH=1
ARG L4T_MAJOR=36
ARG L4T_MINOR=4
# Optional: Override ZED SDK URL
ARG CUSTOM_ZED_SDK_URL=""

# ZED SDK link
ENV ZED_SDK_URL=${CUSTOM_ZED_SDK_URL:-"https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}.${ZED_SDK_PATCH}/l4t${L4T_MAJOR}.${L4T_MINOR}/jetsons"}
ARG ZED_SDK_RUN="ZED_SDK_installer.run"

# ROS2 distribution
ARG ROS2_DIST=humble

# UBUNTU Versions
ARG UBUNTU_RELEASE=22
ARG UBUNTU_MINOR_RELEASE=04

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
# CI has a different runner than the jetson (notably git permissions are different and there is no GPU)
# Unlike the nvidia dev container, we cannot just install stubs to satisfy the linker, so we just forego installing the zed sdk in CI for the jetson, which is fine since we should test it locally anyways
ARG CI
RUN if [ "$CI" = "true" ]; then \
        echo "Building container for CI"; \
    else \
        echo "Building container locally"; \
    fi
# set it as env var so build scripts can see it
ENV CI=${CI}

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
# assume anything trying to build this container is for jetson, whether CI or physical jetson hardware
ENV IS_JETSON=true
# Standard Jetson CUDA paths
ENV CUDA_ROOT=/usr/local/cuda
ENV PATH=${CUDA_ROOT}/bin:${PATH}
ENV CUDA_INC_DIR=${CUDA_ROOT}/include
ENV LD_LIBRARY_PATH=${CUDA_ROOT}/lib64:${LD_LIBRARY_PATH}

# Set UV to system mode and use Jetpack index
ENV UV_SYSTEM_PYTHON=1 \
    UV_EXTRA_INDEX_URL=${PIP_INDEX_FOR_NON_NVIDIA}

# Check that this SDK URL exists
RUN if [ "$(curl -L -I "${ZED_SDK_URL}" -o /dev/null -s -w '%{http_code}\n' | head -n 1)" = "200" ]; then \
    echo "The URL points to something."; \
    else \
    echo "The URL does not point to a .run file or the file does not exist."; \
    exit 1; \
    fi

# Disable apt-get warnings
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt-get update || true && apt-get install -y --no-install-recommends apt-utils dialog && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

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
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean


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
    export PIP_INDEX_URL="${PIP_INDEX_FOR_NON_NVIDIA}" && \
    pip install  --no-cache-dir protobuf && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

RUN cd /tmp && \
    wget -q --no-check-certificate -O ${ZED_SDK_RUN} \
    ${ZED_SDK_URL} && \
    chmod +x ${ZED_SDK_RUN} && \
    ./${ZED_SDK_RUN} silent skip_python skip_cuda && \
    # since the installer installs to /usr/local/zed we need to set permissions for lib access
    # a for all user r for read and X for conditional execute (only allow executing if it is an actual executable)
    chmod -R a+rX /usr/local/zed && \
    # Update library cache
    ldconfig && \
    rm -rf ${ZED_SDK_RUN} && \
    ldconfig
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
    python3-vcstool \
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
    v4l-utils \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# copy our configs for zed2i
COPY Docker/jetson/SN33688213.conf /usr/local/zed/settings/

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-foxglove-msgs \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-joy-teleop \
    ros-$ROS_DISTRO-rosidl-default-generators \
    ros-$ROS_DISTRO-vision-msgs \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-nmea-msgs \
    ros-$ROS_DISTRO-mavros-msgs \
    ros-$ROS_DISTRO-actionlib-msgs \
    ros-$ROS_DISTRO-diagnostic-msgs \
    ros-$ROS_DISTRO-message-filters \
    ros-$ROS_DISTRO-eigen3-cmake-module \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    # Planner packages
    ros-$ROS_DISTRO-py-trees \
    ros-$ROS_DISTRO-py-trees-ros-interfaces \
    ros-$ROS_DISTRO-py-trees-ros \
    ros-$ROS_DISTRO-py-trees-ros-tutorials \
    ros-$ROS_DISTRO-py-trees-ros-viewer \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# install micro-ROS
WORKDIR /opt/ros/micro_ros_ws
# add repository and 
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
# build micro-ROS package and agent
RUN . /opt/ros/$ROS_DISTRO/install/setup.bash && \
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

# ----------------------------------------------------------
# Install Foxglove Bridge from source
# ----------------------------------------------------------
WORKDIR /opt/ros/foxglove_ws
RUN git clone --depth 1 --branch sdk/v0.21.0 https://github.com/foxglove/foxglove-sdk.git src/foxglove-sdk && \
    apt-get update && \
    rm -rf /etc/ros/rosdep/sources.list.d/* && \
    rosdep init && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select foxglove_bridge \
    --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log src /var/lib/apt/lists/*

# Get CUDA dependencies to install torch and torchvision
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
RUN mkdir -p tmp_cudss && cd tmp_cudss && \
    curl -L -O https://developer.download.nvidia.com/compute/cudss/redist/libcudss/linux-sbsa/${CUSPARSE_SOLVER_NAME}.tar.xz && \
    tar xf ${CUSPARSE_SOLVER_NAME}.tar.xz && \
    cp -a ${CUSPARSE_SOLVER_NAME}/include/* /usr/local/cuda/include/ && \
    cp -a ${CUSPARSE_SOLVER_NAME}/lib/* /usr/local/cuda/lib64/ && \
    rm -rf ${CUSPARSE_SOLVER_NAME} ${CUSPARSE_SOLVER_NAME}.tar.xz

# Copy dependency lists
COPY Docker/jetson/requirements.txt Docker/jetson/constraints.txt /tmp/

# Install everything in one clean step
# --no-build-isolation helps with some Jetson-specific wheels
RUN uv pip install \
    -v \ 
    --index-strategy unsafe-best-match \
    --no-cache \ 
    --no-build-isolation \
    --extra-index-url https://pypi.jetson-ai-lab.io/jp6/cu126 \
    -r /tmp/requirements.txt \  
    -c /tmp/constraints.txt

RUN uv pip install \
    -v \
    --extra-index-url https://pypi.jetson-ai-lab.io/jp6/cu126 \ 
    --index-strategy unsafe-best-match \
    --no-deps \
    "inference-models[torch-jp6-cu126,onnx-jp6-cu126]" 

# Install ZED (still separate as it's a local build from git)
RUN git clone -b v${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}.${ZED_SDK_PATCH} https://github.com/stereolabs/zed-python-api.git  /tmp/zed-python-api && \
    uv pip install -v --no-deps --no-build-isolation /tmp/zed-python-api/src && \
    rm -rf /tmp/zed-python-api

# Environment Setup
ARG USERNAME=douglas
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# 1. Create or rename the group
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

# Install fixuid for dynamic UID/GID remapping at runtime
ARG FIXUID_VERSION=0.6.0
RUN ARCH="$(dpkg --print-architecture)" && \
    curl -fsSL "https://github.com/boxboat/fixuid/releases/download/v${FIXUID_VERSION}/fixuid-${FIXUID_VERSION}-linux-${ARCH}.tar.gz" | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USERNAME\ngroup: $USERNAME\n" > /etc/fixuid/config.yml

# Copy entrypoint
COPY Docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Switch to the User and configure the environment
USER $USERNAME

# set colcon home after adding user
ENV COLCON_HOME=/home/$USERNAME/.colcon
# move repo defaults.yaml to container, CI is run from Docker/jetson so need to go up 2 levels
COPY .colcon ${COLCON_HOME}/

# add ultralytics config dir to avoid storing settings in temp
RUN mkdir -p /home/$USERNAME/.config/Ultralytics

WORKDIR /home/$USERNAME/AUV-2026

# Source env variables
# We use /opt/dependencies_ws which is readable by all
RUN echo "source /opt/ros/humble/setup.bash && source /opt/ros/micro_ros_ws/install/local_setup.bash && source /opt/ros/foxglove_ws/install/local_setup.bash && source /home/$USERNAME/AUV-2026/ros2_ws/install/setup.bash" >> ~/.bashrc 
RUN echo 'AMENT_PREFIX_PATH=$(echo $AMENT_PREFIX_PATH | sed -E "s|/opt/ros/humble/install:/opt/ros/humble$|/opt/ros/humble:/opt/ros/humble/install|")' >> ~/.bashrc

# Entrypoint to fix Group IDs to allow GPU access 
ENTRYPOINT ["entrypoint.sh"]

CMD ["bash"]
