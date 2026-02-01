# Define the L4T_VERSION argument
ARG L4T_VERSION=l4t-r36.4.0
ARG IMAGE_NAME=dustynv/ros:humble-desktop-l4t-r36.4.0

FROM ${IMAGE_NAME}

ARG ZED_SDK_MAJOR=5
ARG ZED_SDK_MINOR=1
ARG ZED_SDK_PATCH=0
ARG L4T_MAJOR=36
ARG L4T_MINOR=4
# Optional: Override ZED SDK URL
ARG CUSTOM_ZED_SDK_URL=""

# ZED SDK link
ENV ZED_SDK_URL=${CUSTOM_ZED_SDK_URL:-"https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}.${ZED_SDK_PATCH}/l4t${L4T_MAJOR}.${L4T_MINOR}/jetsons"}
ARG ZED_SDK_RUN="ZED_SDK_installer.run"

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
    v4l-utils \
    && add-apt-repository ppa:deadsnakes/ppa -y \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# copy our configs for zed2i
COPY SN33688213.conf /usr/local/zed/settings/

# Install missing dependencies and wanted packages from source
WORKDIR /opt/dependencies_ws/src
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
    wget https://github.com/mavlink/mavros/archive/refs/tags/${MAVROS_VERSION}.tar.gz -O - | tar -xvz && mv mavros-${MAVROS_VERSION} mavros && \
    wget https://github.com/ros-perception/image_common/archive/refs/tags/${IMAGE_COMMON_VERSION}.tar.gz -O - | tar -xvz && mv image_common-${IMAGE_COMMON_VERSION} image_common && \
    # copy needed packages to src
    cp -r geographic-info/geographic_msgs/ . && \
    cp -r mavros/mavros_msgs/ . && \
    # clean up unneeded source folders
    rm -rf geographic-info && \
    rm -rf mavros && \
    rm -rf /opt/dependencies_ws/src/.cache

# ROS packages that do not depend on OpenCV/that ZED wrapper doesn't depend on can be installed here
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-joy-teleop \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Build the dependencies from source against NVIDIA OpenCV
# There are 2 workspaces so that all the dependencies installed source
# do not clutter our ros2_ws
WORKDIR /opt/dependencies_ws

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && \
    colcon build --parallel-workers $(nproc) --symlink-install \
    --event-handlers console_direct+ --base-paths src \
    --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
    ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
    ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' \
    ' --no-warn-unused-cli' " && \
    rm -rf build install logs || true

# Get CUDA dependencies to install torch and torchvision
WORKDIR /root
RUN CUSPARSE_LIBRARY_NAME="cusparselt-local-tegra-repo-ubuntu2204-0.8.1_0.8.1-1_arm64.deb" && \
    wget https://developer.download.nvidia.com/compute/cusparselt/0.8.1/local_installers/${CUSPARSE_LIBRARY_NAME} && \
    dpkg -i ${CUSPARSE_LIBRARY_NAME} && \
    cp /var/cusparselt-local-tegra-repo-ubuntu2204-0.8.1/cusparselt-*-keyring.gpg /usr/share/keyrings/ && \
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

# Install ultralytics, avoid reinstalling numpy and opencv (torch, tensorflow and torchvision installed in the .toml file, cuda supported cause pip index is Jetpack6 one)
RUN pip install --no-cache-dir --extra-index-url ${PIP_INDEX_FOR_NON_NVIDIA} --ignore-install sympy \ 
    cython --upgrade

RUN git clone https://github.com/ultralytics/ultralytics.git && \
    cd ultralytics && \
    sed -i '/"numpy>=1.23.0"/d; /"opencv-python>=4.6.0"/d; /"numpy<2.0.0"/d' pyproject.toml && \
    pip install --no-cache-dir --extra-index-url ${PIP_INDEX_FOR_NON_NVIDIA} -e . && \
    cd .. && \
    rm -rf ultralytics

# Install dependencies for vision
RUN pip install --no-cache-dir --extra-index-url ${PIP_INDEX_FOR_NON_NVIDIA} notebook \
    roboflow albumentations pandas && \
    pip install --no-cache-dir --ignore-install --upgrade typing_extensions


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
    # 2. Create the user
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -aG video,render -s /bin/bash $USERNAME

# Install fixuid for dynamic UID/GID remapping at runtime
ARG FIXUID_VERSION=0.6.0
RUN ARCH="$(dpkg --print-architecture)" && \
    curl -fsSL "https://github.com/boxboat/fixuid/releases/download/v${FIXUID_VERSION}/fixuid-${FIXUID_VERSION}-linux-${ARCH}.tar.gz" | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USERNAME\ngroup: $USERNAME\n" > /etc/fixuid/config.yml

USER $USERNAME

WORKDIR /home/$USERNAME/AUV-2026

# Source env variables
# We use /opt/dependencies_ws which is readable by all
RUN echo "source /opt/ros/humble/setup.bash && source /opt/dependencies_ws/install/setup.bash && source /home/$USERNAME/AUV-2026/ros2_ws/install/setup.bash" >> ~/.bashrc

# Entrypoint uses fixuid to remap UID/GID if running with --user
ENTRYPOINT ["fixuid", "-q"]

CMD ["bash"]
