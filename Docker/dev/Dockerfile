# ---------------------------------------------------------
# Base Image: ROS 2 Humble Dev (Ubuntu 22.04)
# CPU-only development environment without NVIDIA/ZED support
# ---------------------------------------------------------
FROM althack/ros2:humble-dev

# Set environment variables
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble

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
    python3-vcstools \
    # Hardware & Devices
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
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# ---------------------------------------------------------
# 2. Install ROS 2 Packages
# ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Bridge & Teleop
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-joy-teleop \
    ros-${ROS_DISTRO}-foxglove-bridge \
    # Navigation & Localization
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-nmea-msgs \
    ros-${ROS_DISTRO}-marine-sensor-msgs \
    ros-${ROS_DISTRO}-marine-acoustic-msgs \
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

# Initialize rosdep
RUN rosdep init || true && rosdep update

# ---------------------------------------------------------
# 3. Install Python ML & Vision Dependencies (CPU)
# ---------------------------------------------------------
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

# ---------------------------------------------------------
# 4. Environment Setup
# ---------------------------------------------------------
# Add source commands to bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/AUV-2026/ros2_ws/install/setup.bash ]; then source /root/AUV-2026/ros2_ws/install/setup.bash; fi" >> /root/.bashrc

# Default directory
WORKDIR /root/AUV-2026

# Keep container alive
CMD ["bash"]