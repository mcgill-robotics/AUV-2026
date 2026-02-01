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
COPY Docker/dev/requirements.txt Docker/dev/constraints.txt /tmp/

# Upgrade pip/setuptools (Required to build modern ML wheels)
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel

RUN pip3 install --no-cache-dir -c /tmp/constraints.txt torch torchvision --index-url https://download.pytorch.org/whl/cpu

RUN pip3 install --no-cache-dir -c /tmp/constraints.txt "opencv-python==4.10.0.84"

# Install other requirements (Pip will skip torch since it's already done)
RUN pip3 install --no-cache-dir -c /tmp/constraints.txt -r /tmp/requirements.txt

# DOWNGRADE Setuptools for ROS 2
# Restores compatibility with 'colcon build'
RUN pip3 install --no-cache-dir "setuptools==65.5.1"

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

USER $USERNAME

WORKDIR /home/$USERNAME/AUV-2026

# Add source commands to bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "if [ -f ~/AUV-2026/ros2_ws/install/setup.bash ]; then source ~/AUV-2026/ros2_ws/install/setup.bash; fi" >> ~/.bashrc

# Keep container alive
CMD ["bash"]