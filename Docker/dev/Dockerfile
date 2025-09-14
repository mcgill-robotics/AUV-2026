# Use the official ROS 2 Humble image as the base image
FROM althack/ros2:humble-dev

# Set up environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

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
    python3 \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstools \
    sudo \
    tar \
    tmux \
    v4l-utils usbutils \ 
    vim \
    wget \
    x11-apps \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# ROS packages 
RUN apt-get update && apt-get install -y --no-install-recommends \
   ros-$ROS_DISTRO-joy \
   ros-$ROS_DISTRO-joy-teleop \
   ros-$ROS_DISTRO-foxglove-bridge \
   ros-$ROS_DISTRO-mavros \
   ros-$ROS_DISTRO-rviz2 \
   && rm -rf /var/lib/apt/lists/* \
   && apt-get clean

# Create workspace
WORKDIR /root/AUV-2026

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/AUV-2026/ros2_ws/install/setup.bash ]; then source /root/AUV-2026/ros2_ws/install/setup.bash; fi" >> /root/.bashrc
CMD ["bash"]