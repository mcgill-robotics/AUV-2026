# Define the L4T_VERSION argument
ARG L4T_VERSION=l4t-r36.4.0
ARG IMAGE_NAME=dustynv/ros:humble-desktop-l4t-r36.4.0

FROM ${IMAGE_NAME}

# Disable apt-get warnings
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
   apt-get update || true && apt-get install -y --no-install-recommends apt-utils dialog && \
   rm -rf /var/lib/apt/lists/*


# ZED SDK link
ENV ZED_SDK_URL="https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/5.0/ZED_SDK_Tegra_L4T36.4_v5.0.0.zstd.run"

# Check that this SDK exists
RUN if [ "$(curl -L -I "${ZED_SDK_URL}" -o /dev/null -s -w '%{http_code}\n' | head -n 1)" = "200" ]; then \
        echo "The URL points to something."; \
    else \
        echo "The URL does not point to a .run file or the file does not exist."; \
        exit 1; \
    fi


RUN apt-get update -y && apt install -y \
    vim
ENV PIP_INDEX_URL=https://pypi.jetson-ai-lab.io/jp6/cu126
 
RUN pip install --ignore-install sympy && pip install cython --upgrade

RUN echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}" > /etc/nv_tegra_release && \
  apt-get update -y || true && apt-get install -y --no-install-recommends \
  zstd && \
  export PIP_INDEX_URL="${PYPI_URL}" && \
  pip install protobuf && \
  wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run \
 ${ZED_SDK_URL} && \
  chmod +x ZED_SDK_Linux_JP.run ; ./ZED_SDK_Linux_JP.run silent skip_python && \
  rm -rf ZED_SDK_Linux_JP.run && \
  rm -rf /var/lib/apt/lists/*

# Get CUDA dependencies to install torch and torchvision
RUN wget https://developer.download.nvidia.com/compute/cusparselt/0.8.1/local_installers/cusparselt-local-tegra-repo-ubuntu2204-0.8.1_0.8.1-1_arm64.deb && \
    sudo dpkg -i cusparselt-local-tegra-repo-ubuntu2204-0.8.1_0.8.1-1_arm64.deb && \
    sudo cp /var/cusparselt-local-tegra-repo-ubuntu2204-0.8.1/cusparselt-*-keyring.gpg /usr/share/keyrings/ && \
    sudo apt-get update && \
    sudo apt-get -y install cusparselt-cuda-12

ARG CUSPARSE_SOLVER_NAME="libcudss-linux-sbsa-0.6.0.5_cuda12-archive"
RUN mkdir -p tmp_cudss && cd tmp_cudss && \
    curl -L -O https://developer.download.nvidia.com/compute/cudss/redist/libcudss/linux-sbsa/${CUSPARSE_SOLVER_NAME}.tar.xz && \
    tar xf ${CUSPARSE_SOLVER_NAME}.tar.xz && \
    sudo cp -a ${CUSPARSE_SOLVER_NAME}/include/* /usr/local/cuda/include/ && \
    sudo cp -a ${CUSPARSE_SOLVER_NAME}/lib/* /usr/local/cuda/lib64/

#ARG TORCH_INSTALL=https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
#RUN python3 -m pip install --no-cache $TORCH_INSTALL

# Install ultralytics, avoid reinstalling numpy and opencv (torch, tensorflow and torchvision installed in the .toml file, cuda supported cause pip index is Jetpack6 one)
RUN git clone https://github.com/ultralytics/ultralytics.git
RUN cd ultralytics && sed -i '/"numpy>=1.23.0"/d; /"opencv-python>=4.6.0"/d; /"numpy<2.0.0"/d' pyproject.toml && pip install -e .
