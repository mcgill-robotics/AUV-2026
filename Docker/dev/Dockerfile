# ---------------------------------------------------------
# Base Image: ROS 2 Humble Dev (Ubuntu 22.04)
# CPU-only development environment without NVIDIA/ZED support
# ---------------------------------------------------------
FROM althack/ros2:humble-dev

# CI has a different runner than the jetson (notably git permissions are different and there is no GPU)
ARG CI
RUN if [ "$CI" = "true" ]; then \
        echo "Building container for CI"; \
    else \
        echo "Building container locally"; \
    fi
# set it as env var so build scripts can see it
ENV CI=${CI}
# Set environment variables
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    IS_JETSON=false

# ---------------------------------------------------------
# 1. Install System Dependencies & Tools
# ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build Tools
    apt-utils \
    build-essential \
    cmake \
    # Version Control
    git \
    git-lfs \
    # Networking & Utils
    curl \
    wget \
    gnupg2 \
    lsb-release \
    sudo \
    iputils-ping \
    net-tools \
    # Editors & Terminal
    nano \
    vim \
    less \
    tmux \
    # Python
    python3 \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-opencv \
    # ROS Build Tools
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    tar \
    v4l-utils \
    usbutils \
    libusb-1.0-0-dev \
    libboost-all-dev \
    # GUI Support (RQT/Rviz)
    mesa-utils \
    x11-apps \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# ---------------------------------------------------------
# 2. Install ROS 2 Packages
# ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Bridge & Teleop
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-joy-teleop \
    ros-${ROS_DISTRO}-foxglove-msgs \
    # Navigation & Localization
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-nmea-msgs \
    ros-${ROS_DISTRO}-marine-sensor-msgs \
    ros-${ROS_DISTRO}-marine-acoustic-msgs \
    ros-${ROS_DISTRO}-vision-msgs \
    # Image & Transport
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-compressed-image-transport \
    # Visualization Tools
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt-gui \
    ros-${ROS_DISTRO}-rqt-image-view \
    # Robot Description & Drivers
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-usb-cam \
    ros-${ROS_DISTRO}-zed-msgs \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# ----------------------------------------------------------
# Install micro-ROS Agent (for microcontroller communication)
# ------------------------------------------
WORKDIR /opt/ros/micro_ros_ws
RUN git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
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
WORKDIR /root

# ---------------------------------------------------------
# 3. Install Python ML & Vision Dependencies (CPU)
# ---------------------------------------------------------
COPY Docker/dev/requirements.txt Docker/dev/constraints.txt /tmp/

RUN pip3 install --no-cache-dir -c /tmp/constraints.txt torch torchvision --index-url https://download.pytorch.org/whl/cpu

RUN pip3 install --no-cache-dir -c /tmp/constraints.txt "opencv-python==4.10.0.84"

# Install other requirements (Pip will skip torch since it's already done)
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install --ignore-installed -c /tmp/constraints.txt -r /tmp/requirements.txt

# DOWNGRADE Setuptools for ROS 2
# Restores compatibility with 'colcon build'
RUN pip3 install --no-cache-dir --force-reinstall "setuptools==65.5.1"

# we extract only the libraries we need
RUN wget -q -O /tmp/adept.deb https://files.digilent.com/Software/Adept2%20Runtime/2.30.1/digilent.adept.runtime_2.30.1_amd64.deb && \
    wget -q -O /tmp/waveforms.deb https://files.digilent.com/Software/Waveforms/3.24.4/digilent.waveforms_3.24.4_amd64.deb && \
    # extract libs from .deb packages
    dpkg-deb -x /tmp/adept.deb /tmp/adept-extracted && \
    dpkg-deb -x /tmp/waveforms.deb /tmp/waveforms-extracted && \
    # copy all .so libraries into the container
    cp -r /tmp/adept-extracted/usr/lib/digilent /usr/lib/digilent && \
    cp /tmp/waveforms-extracted/usr/lib/*.so* /usr/local/lib/ && \
    # add digilent adept path to ldconfig
    echo "/usr/lib/digilent/adept" > /etc/ld.so.conf.d/digilent-adept.conf && \
    ldconfig && \
    rm -rf /tmp/adept-extracted /tmp/waveforms-extracted

# ---------------------------------------------------------
# 4. Environment Setup
# ---------------------------------------------------------
# The base image already has a 'ros' user with sudo. 
ARG USERNAME=douglas
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Rename the default 'ros' user to our desired username/uid
# We keep UID 1000 to match host permissions
RUN groupmod --gid $USER_GID --new-name $USERNAME ros \
    && usermod --uid $USER_UID --gid $USER_GID --login $USERNAME --move-home --home /home/$USERNAME ros \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && groupadd -f -r render \
    && usermod -aG video,render -s /bin/bash $USERNAME

# Install fixuid for dynamic UID/GID remapping at runtime
# This allows the container to match the host user's UID when running with --user
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
# move repo defaults.yaml to container
COPY .colcon ${COLCON_HOME}/

WORKDIR /home/$USERNAME/AUV-2026

# Add source commands to bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/local_setup.bash && source /opt/ros/micro_ros_ws/install/local_setup.bash && source /opt/ros/foxglove_ws/install/local_setup.bash" >> ~/.bashrc && \
    echo "if [ -f ~/AUV-2026/ros2_ws/install/setup.bash ]; then source ~/AUV-2026/ros2_ws/install/setup.bash; fi" >> ~/.bashrc

# Entrypoint uses fixuid to remap UID/GID if running with --user
ENTRYPOINT ["entrypoint.sh"]

# Keep container alive
CMD ["bash"]
