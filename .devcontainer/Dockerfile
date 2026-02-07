FROM pytorch/pytorch:2.10.0-cuda12.6-cudnn9-devel AS base

############################################################################
####        Install MinkowskiEngine
############################################################################

## Set environment variables for CUDA and PyTorch
ENV TORCH_CUDA_ARCH_LIST="6.0 6.1 6.2 7.0 7.2 7.5 8.0 8.6" 
ENV TORCH_NVCC_FLAGS="-Xfatbin -compress-all"
ENV MAX_JOBS=4
ENV CUDA_HOME=/usr/local/cuda-12.6
ENV PATH=/opt/conda/bin:${PATH}
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV CPATH=${CUDA_HOME}/include
ENV LIBRARY_PATH=${CUDA_HOME}/lib:${LIBRARY_PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib:${LD_LIBRARY_PATH}
ENV CXX=c++

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        ninja-build \
        libopenblas-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN python -m pip config set global.break-system-packages true

## Fix issues per https://github.com/Julie-tang00/Common-envs-issues/blob/main/Cuda12-MinkowskiEngine
RUN sed -i 's/\bauto __raw = __to_address(__r.get());/auto __raw = std::__to_address(__r.get());/' /usr/include/c++/13/bits/shared_ptr_base.h

WORKDIR /dependencies
RUN git clone --recursive https://github.com/chenxi-wang/MinkowskiEngine.git

WORKDIR /dependencies/MinkowskiEngine
RUN git checkout cuda-12-1 && python setup.py install --blas=openblas --force_cuda

############################################################################
####        Install GraspNET API
############################################################################

WORKDIR /dependencies
RUN git clone https://github.com/CollaborativeRoboticsLab/graspnetAPI.git

WORKDIR /dependencies/graspnetAPI
RUN python -m pip install --no-cache-dir .

############################################################################
####        Install AnyGrasp SDK
############################################################################

WORKDIR /dependencies
RUN git clone https://github.com/graspnet/anygrasp_sdk.git

WORKDIR /dependencies/anygrasp_sdk
RUN python -m pip install --no-cache-dir -r requirements.txt

############################################################################
####        Install pointnet2
############################################################################

## pip uses PEP 517 build isolation by default. It creates a temporary build environment and only installs 
## what the project declares for building. The pointnet2 setup.py/pyproject appears to import torch during 
## “get requirements to build wheel”, but doesn’t declare it in build requirements, so the isolated env 
## doesn’t have torch, leading to ModuleNotFoundError. So  --no-build-isolation is needed.

WORKDIR /dependencies/anygrasp_sdk/pointnet2
RUN python -m pip install --no-cache-dir --no-build-isolation .

############################################################################
####        Move library files
############################################################################

WORKDIR /dependencies/anygrasp_sdk/grasp_detection
RUN cp gsnet_versions/gsnet.cpython-312-x86_64-linux-gnu.so gsnet.so && \
    cp ../license_registration/lib_cxx_versions/lib_cxx.cpython-312-x86_64-linux-gnu.so lib_cxx.so

WORKDIR /dependencies/anygrasp_sdk/grasp_tracking
RUN cp tracker_versions/tracker.cpython-312-x86_64-linux-gnu.so tracker.so && \
    cp ../license_registration/lib_cxx_versions/lib_cxx.cpython-312-x86_64-linux-gnu.so lib_cxx.so

############################################################################
####        Install ROS2 Jazzy and ROS development tools
############################################################################

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

# Initialize rosdep as root
RUN rosdep init && rosdep update

## Add dependencies ROS development
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-ur \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
    python3 \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

############################################################################
####        Add vscode user
############################################################################

# Add vscode user with same UID and GID as your host system
# (modified from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
# ubuntu now has a 1000 user ubuntu

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install  --no-install-recommends -y \
    sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# make sure folders exist
WORKDIR /home/ubuntu/colcon_ws/src

RUN git clone https://github.com/CollaborativeRoboticsLab/grippers.git && \
    git clone https://github.com/CollaborativeRoboticsLab/anygrasp_ros.git

WORKDIR /