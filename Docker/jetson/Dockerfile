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
# Set up environment (we in paris because ZED official docs are in paris)
ENV TZ=Europe/Paris
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV IS_JETSON=true

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

# NVIDIA's jetson pip server variables
ENV PYPI_URL=https://pypi.jetson-ai-lab.io/jp6/cu126
ENV PIP_INDEX_URL=https://pypi.jetson-ai-lab.io/jp6/cu126 
ENV PIP_TRUSTED_HOST=pypi.jetson-ai-lab.io
ENV PIP_INDEX_FOR_NON_NVIDIA=https://pypi.python.org/simple

ARG UBUNTU_RELEASE=22
ARG UBUNTU_MINOR_RELEASE=04
# assume anything trying to build this container is for jetson, whether CI or physical jetson hardware
ENV IS_JETSON=true

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
# Non-ROS apt packages: build tools, libraries, and utilities.

# The base image ships with a Yarn apt repo whose GPG key is expired.
# We don't need Yarn, so remove it to prevent apt-get update from failing.
RUN rm -f /etc/apt/sources.list.d/yarn.list && \
    # Refresh the ROS apt GPG key in case it has expired
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

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


# Install common development tools
# Keep these in alphabetical
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
    wget \
    # Python
    python3 \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstools \
    python3-wheel \
    # Libraries for ZED SDK and ROS package compilation
    libusb-1.0-0-dev \
    libgeographic-dev \
    libdraco-dev \
    libpq-dev \
    zlib1g-dev \
    zstd \
    # Graphics, video encoding, and C++ development
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
    libopenblas-dev \
    xauth \
    ninja-build \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# ===== SSH Server Setup =====
# We use port 222 so it doesn't conflict with the host's SSH
# Password is hardcoded — not secure, but fine for our LAN-only dev setup
RUN mkdir -p /var/run/sshd /root/.ssh && \
    chmod 700 /root/.ssh && \
    echo "Port 222" >> /etc/ssh/sshd_config && \
    echo "PermitRootLogin yes" >> /etc/ssh/sshd_config && \
    echo 'root:jetson' | chpasswd

# ===== ZED SDK Installation =====
# The ZED installer checks /etc/nv_tegra_release to detect L4T version
RUN echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}" > /etc/nv_tegra_release

# protobuf is needed by ZED SDK but NVIDIA's pip index dropped it, so use the standard index
RUN PIP_INDEX_URL="${PIP_INDEX_FOR_NON_NVIDIA}" pip install --no-cache-dir protobuf

# Download and install the ZED SDK
# skip_python: we manage our own Python environment
# skip_cuda: CUDA is already in the base image
# chmod a+rX: let non-root users read ZED libs (r) and execute actual binaries only (X)
RUN cd /tmp && \
    wget -q --no-check-certificate -O ${ZED_SDK_RUN} ${ZED_SDK_URL} && \
    chmod +x ${ZED_SDK_RUN} && \
    ./${ZED_SDK_RUN} silent skip_python skip_cuda && \
    chmod -R a+rX /usr/local/zed && \
    ldconfig && \
    rm -f ${ZED_SDK_RUN}

# Copy our ZED2i camera calibration config
COPY Docker/jetson/SN33688213.conf /usr/local/zed/settings/

# ===== ROS 2 Dependencies (apt binaries) =====
# Since Isaac ROS uses standard apt-based ROS 2, we install packages as binaries.
# This guarantees ABI compatibility — no more rosidl symbol mismatches.
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
# build micro-ROS package and agent
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

# ===== CUDA Sparse Libraries (for torch/torchvision) =====
WORKDIR /root

# Install cuSPARSELt (sparse matrix operations on tensor cores)
RUN CUSPARSE_LIBRARY_NAME="cusparselt-local-tegra-repo-ubuntu${UBUNTU_RELEASE}${UBUNTU_MINOR_RELEASE}-0.8.1_0.8.1-1_arm64.deb" && \
    wget https://developer.download.nvidia.com/compute/cusparselt/0.8.1/local_installers/${CUSPARSE_LIBRARY_NAME} && \
    dpkg -i ${CUSPARSE_LIBRARY_NAME} && \
    cp /var/cusparselt-local-tegra-repo-ubuntu${UBUNTU_RELEASE}${UBUNTU_MINOR_RELEASE}-0.8.1/cusparselt-*-keyring.gpg /usr/share/keyrings/ && \
    apt-get update && \
    apt-get -y install cusparselt-cuda-12 && \
    rm -rf ${CUSPARSE_LIBRARY_NAME} && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

# Install cuDSS (CUDA direct sparse solver)
ARG CUSPARSE_SOLVER_NAME="libcudss-linux-sbsa-0.6.0.5_cuda12-archive"
RUN mkdir -p /tmp/cudss && cd /tmp/cudss && \
    curl -L -O https://developer.download.nvidia.com/compute/cudss/redist/libcudss/linux-sbsa/${CUSPARSE_SOLVER_NAME}.tar.xz && \
    tar xf ${CUSPARSE_SOLVER_NAME}.tar.xz && \
    cp -a ${CUSPARSE_SOLVER_NAME}/include/* /usr/local/cuda/include/ && \
    cp -a ${CUSPARSE_SOLVER_NAME}/lib/* /usr/local/cuda/lib64/ && \
    rm -rf /tmp/cudss

# ===== Python ML/Vision Stack =====
# torch, tensorflow, and torchvision are installed via the .toml file
# pip index points to the Jetpack6 CUDA-aware server so we get GPU-enabled wheels
RUN pip install --no-cache-dir --extra-index-url ${PIP_INDEX_FOR_NON_NVIDIA} \
    --ignore-installed \
    sympy \
    cython \
    --upgrade

# Install PyTorch and Torchvision from the Jetson index (CUDA enabled)
RUN pip install --no-cache-dir torch torchvision --index-url ${PIP_INDEX_URL}

# Ultralytics YOLO — pinned to a tag so builds are reproducible (git --branch works for tags too)
# We strip numpy/opencv constraints because the base image ships Jetson-optimized versions
# NOTE: installed non-editable (no -e) so we can delete the clone afterwards
ARG ULTRALYTICS_REF=v8.2.0
RUN git clone --depth 1 --branch ${ULTRALYTICS_REF} https://github.com/ultralytics/ultralytics.git && \
    cd ultralytics && \
    sed -i '/"numpy>=1.23.0"/d; /"opencv-python>=4.6.0"/d; /"numpy<2.0.0"/d' pyproject.toml && \
    pip install --no-cache-dir --extra-index-url ${PIP_INDEX_FOR_NON_NVIDIA} . && \
    cd .. && \
    rm -rf ultralytics

# Computer vision tooling
RUN pip install --no-cache-dir --extra-index-url ${PIP_INDEX_FOR_NON_NVIDIA} "numpy<2.0.0" notebook \
    roboflow albumentations pandas \
    rfdetr supervision pytorch_lightning pycocotools faster-coco-eval onnx onnxruntime-gpu && \
    pip install --no-cache-dir --ignore-install --upgrade typing_extensions

# ===== User Setup =====
ARG USERNAME=douglas
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create or rename the group/user. dialout group is needed for IMU serial access.
RUN if getent group $USER_GID; then \
    groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1); \
    else \
    groupadd --gid $USER_GID $USERNAME; \
    fi && \
    useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    usermod -aG video,render,dialout -s /bin/bash $USERNAME

# fixuid remaps UID/GID at runtime when using docker run --user
# This keeps file permissions sane between host and container
ARG FIXUID_VERSION=0.6.0
RUN ARCH="$(dpkg --print-architecture)" && \
    curl -fsSL "https://github.com/boxboat/fixuid/releases/download/v${FIXUID_VERSION}/fixuid-${FIXUID_VERSION}-linux-${ARCH}.tar.gz" \
    | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USERNAME\ngroup: $USERNAME\n" > /etc/fixuid/config.yml

# Copy entrypoint
COPY Docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
# ===== User Environment =====
# Switch to the User and configure the environment
USER $USERNAME

# colcon defaults (e.g. parallel workers, build type) — copied from repo root
ENV COLCON_HOME=/home/$USERNAME/.colcon
# move repo defaults.yaml to container, CI is run from Docker/jetson so need to go up 2 levels
COPY .colcon ${COLCON_HOME}/

# add ultralytics config dir to avoid storing settings in temp
RUN mkdir -p /home/$USERNAME/.config/Ultralytics

WORKDIR /home/$USERNAME/AUV-2026

# Source the standard apt-installed ROS environment and our project workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash && source /opt/ros/micro_ros_ws/install/local_setup.bash && source /opt/ros/foxglove_ws/install/local_setup.bash && source /home/$USERNAME/AUV-2026/ros2_ws/install/setup.bash" >> ~/.bashrc 

# Entrypoint to fix Group IDs to allow GPU access 
ENTRYPOINT ["entrypoint.sh"]

CMD ["bash"]
