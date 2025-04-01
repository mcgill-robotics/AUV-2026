# Use the correct Jetson ROS base image (Ensure JetPack matches L4T version)
FROM dustynv/ros:humble-ros-base-l4t-r36.3.0 

# Install system dependencies
RUN apt-get update && apt-get install -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# Set up environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO humble

# Install SSH server and set up SSH access
RUN apt-get update && apt-get install -y \
    openssh-server \
    xauth \
    && mkdir /var/run/sshd \
    && mkdir /root/.ssh \
    && chmod 700 /root/.ssh \
    && echo "Port 222" >> /etc/ssh/sshd_config \
    && echo "PermitRootLogin yes" >> /etc/ssh/sshd_config \
    && echo 'root:jetson' | chpasswd

# Install additional tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstools \
    libusb-dev \
    v4l-utils \
    tmux \
    vim \
    iputils-ping \
    iproute2 \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-usb-cam \
    --fix-broken \
    || true && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Set up environment variables
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set default shell to Bash
SHELL ["/bin/bash", "-c"]

# Default command
CMD ["bash"]
