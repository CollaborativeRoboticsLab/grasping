FROM ros:jazzy

# Add vscode user with same UID and GID as your host system
# (modified from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
# ubuntu now has a 1000 user ubuntu
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# RUN groupadd --gid $USER_GID $USERNAME \
#     && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \

RUN apt-get update && \
    apt-get upgrade -y \
    && apt-get install -y \
    sudo \
    git \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

## Add dependencies ROS development
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-ur \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
    python3 \
    python3-pip \
    python3-dev \
    libopenblas-dev \
    && rm -rf /var/lib/apt/lists/*

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
RUN git clone https://github.com/NVIDIA/MinkowskiEngine.git

WORKDIR /dependencies/MinkowskiEngine
RUN sed -i 's/\bauto __raw = __to_address(__r.get());/auto __raw = std::__to_address(__r.get());/' /usr/include/c++/11/bits/shared_ptr_base.h && \
    python3 setup.py install --blas=openblas --force_cuda

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

# Switch from root to user
USER $USERNAME

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# make sure folders exist
RUN mkdir -p ~/colcon_ws/src
