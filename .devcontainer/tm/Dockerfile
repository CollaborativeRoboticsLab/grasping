FROM ros:humble AS base

############################################################################
####        Install Additional packages for ROS2
############################################################################

## Install ROS 2 development tools
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-dev-tools  \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

## Set RMW implementation to CycloneDDS as it is the default and recommended for ROS 2 Humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Initialize rosdep as root
RUN rosdep init && rosdep update

############################################################################
####        Install Realsense packages
############################################################################

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

############################################################################
####        Install OMRON robot packages
############################################################################

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-vision-opencv \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/ubuntu/colcon_ws/src/

RUN git clone https://github.com/CollaborativeRoboticsLab/tmr_ros2.git -b humble

############################################################################
####        Install grippers packages
############################################################################

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    python3-serial \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/ubuntu/colcon_ws/src/

RUN git clone https://github.com/CollaborativeRoboticsLab/grippers.git

############################################################################
####      Add vscode user
############################################################################

# Add vscode user with same UID and GID as your host system
# (modified from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
# ubuntu now has a 1000 user ubuntu
RUN apt-get update && \
    apt-get install  --no-install-recommends -y \
    sudo  \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
    
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG dialout $USERNAME

RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN mkdir -p /home/ubuntu/colcon_ws/src /home/ubuntu/colcon_ws/build /home/ubuntu/colcon_ws/install /home/ubuntu/colcon_ws/log && \
    touch /home/ubuntu/.bashrc && \
    chown -R $USERNAME:$USERNAME /home/ubuntu

# Switch from root to user
USER $USERNAME

# Source the ROS setup file and set a custom PS1 prompt in the user's bashrc
RUN cat <<'EOF' >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash
PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
EOF

WORKDIR /
