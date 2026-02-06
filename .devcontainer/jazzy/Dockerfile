FROM nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04 AS base

## Set ROS distribution as an environment variable
ENV ROS_DISTRO=jazzy
ENV DEBIAN_FRONTEND=noninteractive

## Check and update locale settings
RUN apt-get update && apt-get upgrade -y && apt-get install --no-install-recommends -y \
    locales \
    gnupg2 \
    ca-certificates \
    pkg-config \
    lsb-release 
    
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

## Add ROS 2 apt repository and keys
RUN apt-get update && apt-get install --no-install-recommends -y software-properties-common && \
    add-apt-repository universe

RUN apt-get update && apt-get install --no-install-recommends -y curl && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb

## Install ROS 2 base and development tools
RUN apt-get update && apt-get upgrade -y && apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-ros-base \
    ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

## Add dependencies ROS development
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    ninja-build \
    libomp-dev \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-ur \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
    python3 \
    python3-pip \
    python3-dev \
    libopenblas-dev \
    && rm -rf /var/lib/apt/lists/*

## Upgrade pip and build tooling
RUN python3 -m pip install --upgrade pip setuptools wheel

## install PyTorch with CUDA support (if you have an NVIDIA GPU and the appropriate drivers installed)
RUN python3 -m pip install --no-cache-dir \
    torch \
    torchvision \
    numpy \
    ninja

## Install MinkowskiEngine.
WORKDIR /dependencies
ENV CUDA_HOME=/usr/local/cuda
ENV CXX=c++
ENV TORCH_CUDA_ARCH_LIST="7.5;8.0;8.6;8.9"
RUN git clone https://github.com/chenxi-wang/MinkowskiEngine.git

WORKDIR /dependencies/MinkowskiEngine
RUN git checkout cuda-12-1 && \
    python3 setup.py install --blas=openblas --force_cuda -j 4

## Install GraspNET API
WORKDIR /dependencies
RUN git clone https://github.com/CollaborativeRoboticsLab/graspnetAPI.git

WORKDIR /dependencies/graspnetAPI
RUN python3 -m pip install .

## Install anygrasp_sdk.
WORKDIR /
RUN git clone https://github.com/graspnet/anygrasp_sdk.git

WORKDIR /anygrasp_sdk
RUN python3 -m pip install --no-cache-dir -r requirements.txt

## Install pointnet2.
WORKDIR /anygrasp_sdk/pointnet2
RUN python3 setup.py install

## Move library file
WORKDIR /anygrasp_sdk/grasp_detection

RUN cp gsnet_versions/gsnet.cpython-312-x86_64-linux-gnu.so gsnet.so && \
    cp ../license_registration/lib_cxx_versions/lib_cxx.cpython-312-x86_64-linux-gnu.so lib_cxx.so

WORKDIR /anygrasp_sdk/grasp_tracking

RUN cp tracker_versions/tracker.cpython-312-x86_64-linux-gnu.so tracker.so && \
    cp ../license_registration/lib_cxx_versions/lib_cxx.cpython-312-x86_64-linux-gnu.so lib_cxx.so

# Add vscode user with same UID and GID as your host system
# (modified from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
# ubuntu now has a 1000 user ubuntu
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
    sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Initialize rosdep as root
RUN rosdep init && rosdep update

# Switch from root to user
USER $USERNAME

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# make sure folders exist
WORKDIR /colcon_ws/src

RUN git clone https://github.com/CollaborativeRoboticsLab/grippers.git && \
    git clone https://github.com/CollaborativeRoboticsLab/anygrasp_ros.git
