# Stage 2: Application Dockerfile
# =============================================================================
# Extends the base image which caches: OpenCV CUDA, PyTorch, ZED SDK,
# cuSPARSELt, cuDSS, and numpy. This stage handles workspace-specific
# setup for fast iteration.
#
# Build the base image first:
#   docker build -f Dockerfile.base -t mcgillrobotics/auv_2026:isaac-ros-base ../../
# =============================================================================
ARG BASE_IMAGE=mcgillrobotics/auv_2026:isaac-ros-base
FROM ${BASE_IMAGE}

# ROS2 distribution
ARG ROS2_DIST=humble

# CI has a different runner than the jetson (notably git permissions are different and there is no GPU)
ARG CI

# Set up pip index as a build argument so it can be updated easily
ARG JETSON_PIP_INDEX=https://pypi.jetson-ai-lab.io/jp6/cu126

# Set up environment
ENV CI=${CI} \
    TZ=America/Montreal \
    ROS_DISTRO=${ROS2_DIST} \
    # assume anything trying to build this container is for jetson, whether CI or physical jetson hardware
    IS_JETSON=true \
    UV_SYSTEM_PYTHON=1

# =============================================================================
# 1. ZED Config
# =============================================================================
# copy our configs for zed2i
COPY Docker/jetson/SN33688213.conf /usr/local/zed/settings/

# =============================================================================
# 2. System Packages
# =============================================================================

# Disable apt-get warnings and install common development tools
# Install graphics, video, c++ dev libraries
# Keep packages in alphabetical order within each group
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    btop \
    dialog \
    g++ \
    git-lfs \
    iproute2 \
    iputils-ping \
    jq \
    libavcodec-dev \
    libavdevice-dev \
    libavformat-dev \
    libavutil-dev \
    libc++-dev \
    libegl1-mesa-dev \
    libepoxy-dev \
    libgl1-mesa-dev \
    libglew-dev \
    libjpeg-dev \
    libpng-dev \
    libpq-dev \
    libswscale-dev \
    libwayland-dev \
    libxkbcommon-dev \
    lsb-release \
    nano \
    ninja-build \
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
    && rm -rf /var/lib/apt/lists/* && apt-get clean

# =============================================================================
# 3. ROS 2 Packages
# =============================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-actionlib-msgs \
    ros-$ROS_DISTRO-ament-index-cpp \
    ros-$ROS_DISTRO-diagnostic-msgs \
    ros-$ROS_DISTRO-eigen3-cmake-module \
    ros-$ROS_DISTRO-foxglove-msgs \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-joy-teleop \
    ros-$ROS_DISTRO-mavros-msgs \
    ros-$ROS_DISTRO-message-filters \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-nmea-msgs \
    ros-$ROS_DISTRO-py-trees \
    ros-$ROS_DISTRO-py-trees-ros \
    ros-$ROS_DISTRO-py-trees-ros-interfaces \
    ros-$ROS_DISTRO-py-trees-ros-tutorials \
    ros-$ROS_DISTRO-py-trees-ros-viewer \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-rclcpp-lifecycle \
    ros-$ROS_DISTRO-rosidl-default-generators \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-vision-msgs \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

# =============================================================================
# 4. micro-ROS (clone + build in single layer)
# =============================================================================
WORKDIR /opt/ros/micro_ros_ws
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    . /opt/ros/$ROS_DISTRO/setup.bash && \
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

# =============================================================================
# 5. Foxglove Bridge
# =============================================================================
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

# =============================================================================
# 6. Python Dependencies (single layer for both installs)
# =============================================================================
WORKDIR /root

# Copy dependency lists
COPY Docker/jetson/requirements.txt Docker/jetson/constraints.txt /tmp/

# Install Python packages. torch/torchvision/torchaudio and opencv are already
# in the base image - uv will see them as satisfied and skip them.
RUN uv pip install \
    -v \
    --index-strategy unsafe-best-match \
    --no-cache \
    --no-build-isolation \
    --system \
    --extra-index-url ${JETSON_PIP_INDEX} \
    -r /tmp/requirements.txt \
    -c /tmp/constraints.txt && \
    uv pip install \
    -v \
    --extra-index-url ${JETSON_PIP_INDEX} \
    --index-strategy unsafe-best-match \
    --no-deps \
    --system \
    "inference-models[torch-jp6-cu126,onnx-jp6-cu126]"

# =============================================================================
# 7. User Setup + Entrypoint (consolidated to reduce layers)
# =============================================================================
ARG USERNAME=douglas
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG FIXUID_VERSION=0.6.0

# Create user, install fixuid, and set up entrypoint in a single layer
RUN if getent group $USER_GID; then \
        groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1); \
    else \
        groupadd --gid $USER_GID $USERNAME; \
    fi && \
    useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    usermod -aG video,render,dialout -s /bin/bash $USERNAME && \
    ARCH="$(dpkg --print-architecture)" && \
    curl -fsSL "https://github.com/boxboat/fixuid/releases/download/v${FIXUID_VERSION}/fixuid-${FIXUID_VERSION}-linux-${ARCH}.tar.gz" | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USERNAME\ngroup: $USERNAME\n" > /etc/fixuid/config.yml

# copy entrypoint script and set permissions
COPY --chmod=755 Docker/entrypoint.sh /usr/local/bin/entrypoint.sh

# Switch to the user and configure the environment
USER $USERNAME

# set colcon home after adding user
ENV COLCON_HOME=/home/$USERNAME/.colcon
# move repo defaults.yaml to container, CI is run from Docker/jetson so need to go up 2 levels
COPY .colcon ${COLCON_HOME}/

# Create ultralytics config dir and set up bashrc in one layer
WORKDIR /home/$USERNAME/AUV-2026
RUN mkdir -p /home/$USERNAME/.config/Ultralytics && \
    { \
        echo "source /opt/ros/humble/setup.bash"; \
        echo "source /opt/ros/micro_ros_ws/install/local_setup.bash"; \
        echo "source /opt/ros/foxglove_ws/install/local_setup.bash"; \
        echo "source /opt/ros/vision_opencv_ws/install/local_setup.bash"; \
        echo "source /home/$USERNAME/AUV-2026/ros2_ws/install/setup.bash"; \
    } >> ~/.bashrc

# Entrypoint to fix Group IDs to allow GPU access
ENTRYPOINT ["entrypoint.sh"]

CMD ["bash"]
