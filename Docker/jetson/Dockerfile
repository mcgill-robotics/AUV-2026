# Use the ROS 2 Humble base image for Jetson
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

# Bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# Install ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
ros-humble-ros-core \
ros-humble-ros-base \
ros-humble-cv-bridge \
ros-humble-image-transport \
ros-humble-camera-info-manager \
ros-humble-usb-cam \
# ros-humble-robot-localization \
&& rm -rf /var/lib/apt/lists/*

# Clone and build the SBG ROS 2 driver
RUN mkdir -p /ros2_ws/src && \
    cd /ros2_ws/src && \
    git clone https://github.com/SBG-Systems/sbg_ros2_driver.git && \
    cd /ros2_ws && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build

# # Install Python packages
# COPY requirements.txt requirements.txt
# RUN pip3 install --no-cache-dir -r requirements.txt \
#     && pip3 install --no-deps --no-cache-dir ultralytics==8.0.207 \
#     && rm -f /usr/local/bin/cmake && rm -rf /root/.cache/pip && rm requirements.txt

# # Install PyTorch Vision
# RUN git clone --branch release/0.16 --recursive --depth=1 https://github.com/pytorch/vision torchvision && \
#     cd torchvision && \
#     git checkout release/0.16 && \
#     python3 setup.py bdist_wheel && \
#     cp dist/torchvision*.whl /opt && \
#     pip3 install --no-cache-dir --verbose /opt/torchvision*.whl && \
#     cd ../ && \
#     rm -rf torchvision

# Set up environment variables
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "source /AUV-2026/ros2_ws/install/setup.bash" >> ~/.bashrc 
    # && echo "source /AUV-2026/scripts/set_env.bash" >> ~/.bashrc \
    # && echo "source /AUV-2026/scripts/jetson/docker_aliases.bash" >> ~/.bashrc

# Install additional tools
# RUN apt-get update && apt-get install -y \
#     docker.io \
#     && pip3 install sklearn

# Set default command
RUN echo "tmux" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]
CMD ["bash"]