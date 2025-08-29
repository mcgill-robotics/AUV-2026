# Use the correct Jetson ROS base image (Ensure JetPack matches L4T version)
FROM dustynv/ros:humble-desktop-l4t-r36.4.0

# Install system dependencies
RUN apt-get update && apt-get install -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# Set up environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO=humble

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

# Install common development tools
# Keep these in alphabetical order
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    git-lfs \
    iproute2 \
    iputils-ping \
    nano \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstools \
    tar \
    tmux \
    v4l-utils \
    vim \
    wget \
    zlib1g-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

#Install graphis, video, c++ dev libraries
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
    libusb-dev \
    libwayland-dev \
    libxkbcommon-dev \
    ninja-build \
    wayland-protocols \
    && add-apt-repository ppa:deadsnakes/ppa -y \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# bootstrap rosdep
RUN rosdep init || true && \
    rosdep update --rosdistro $ROS_DISTRO

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-joy-teleop \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-view \
    ros-$ROS_DISTRO-rqt-gui \
    ros-$ROS_DISTRO-smach-ros \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-nmea-msgs \
    ros-$ROS_DISTRO-mavros-msgs && \
#    --fix-broken || true \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Create New User for Jetson
RUN adduser jetson --disabled-password --gecos "" 

# https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/5.0/ZED_SDK_Tegra_L4T36.4_v5.0.0.zstd.run <-- this downloads smth
#https://download.stereolabs.com/zedsdk/5.0/l4t36.4/jetsons
# WORKDIR /root/
# RUN wget "https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/5.0/ZED_SDK_Tegra_L4T36.4_v5.0.0.zstd.run" -O ZED_SDK_Installer.run && \
#     chmod +x ZED_SDK_Installer.run && \
#     ./ZED_SDK_Installer.run -- silent && \
#     chown -R jetson:jetson /usr/local/zed && \
#     rm ZED_SDK_Installer.run

# COPY SN33688213.conf /usr/local/zed/settings/

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Set up environment variables
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set default shell to Bash
SHELL ["/bin/bash", "-c"]
RUN echo "tmux" >> ~/.bashrc
