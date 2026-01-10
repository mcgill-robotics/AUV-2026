# Use the official ROS 2 Humble image as the base image
FROM althack/ros2:humble-dev

# Set up environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# 1. Install System Dependencies & Tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    build-essential \
    cmake \
    curl \
    git \
    git-lfs \
    gnupg2 \
    less \
    lsb-release \
    mesa-utils \
    nano \
    python3 \
    python3-dev \
    python3-opencv \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstools \
    sudo \
    tar \
    tmux \
    v4l-utils \
    usbutils \
    vim \
    wget \
    x11-apps \
    iputils-ping \
    net-tools \
    libboost-all-dev \
    libusb-1.0-0-dev \
    # GL libraries for GUI support (RQT/Rviz)
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# 2. Install ROS 2 Packages
# Added packages specifically for Unity Sim interaction (Image transport, RQT)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-joy-teleop \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-mavros \
    ros-$ROS_DISTRO-mavros-msgs \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-rqt-gui \
    ros-$ROS_DISTRO-rqt-image-view \
    ros-$ROS_DISTRO-nmea-msgs \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-compressed-image-transport \
    ros-$ROS_DISTRO-marine-sensor-msgs \
    ros-$ROS_DISTRO-marine-acoustic-msgs \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-zed-msgs \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# 3. Install Python ML & Vision Dependencies (CPU Support)
RUN pip3 install --no-cache-dir \
    ultralytics \
    pandas \
    scipy \
    notebook \
    roboflow \
    albumentations \
    typing_extensions \
    sympy \
    cython

# Create workspace
WORKDIR /root/AUV-2026

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Setup bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/AUV-2026/ros2_ws/install/setup.bash ]; then source /root/AUV-2026/ros2_ws/install/setup.bash; fi" >> /root/.bashrc

CMD ["bash"]