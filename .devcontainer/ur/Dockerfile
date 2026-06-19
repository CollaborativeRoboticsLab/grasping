FROM ros:humble AS base

############################################################################
####        Install Additional packages for ROS2
############################################################################

## Install ROS 2 base and development tools
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
####        Install UR robot packages
############################################################################

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-ur \
    ros-${ROS_DISTRO}-moveit \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Patch the UR external control script for CB3 PolyScope 3.13.x compatibility.
# Newer ur_client_library releases use list * float in extrapolate(), which
# raises "Unsupported operand type(s) for *: list and Float" on older robots.
RUN perl -0pi -e 's#def extrapolate\(\):\n  cmd_servo_q_last = cmd_servo_q\n  cmd_servo_q = cmd_servo_q \+ \(get_target_joint_speeds\(\) \* steptime\)\n  return cmd_servo_q\nend#def extrapolate():\n  local qd = get_target_joint_speeds()\n  cmd_servo_q_last = cmd_servo_q\n  cmd_servo_q = [\n    cmd_servo_q[0] + qd[0] * steptime,\n    cmd_servo_q[1] + qd[1] * steptime,\n    cmd_servo_q[2] + qd[2] * steptime,\n    cmd_servo_q[3] + qd[3] * steptime,\n    cmd_servo_q[4] + qd[4] * steptime,\n    cmd_servo_q[5] + qd[5] * steptime\n  ]\n  return cmd_servo_q\nend#s' /opt/ros/${ROS_DISTRO}/share/ur_client_library/resources/external_control.urscript && \
    grep -q 'local qd = get_target_joint_speeds()' /opt/ros/${ROS_DISTRO}/share/ur_client_library/resources/external_control.urscript

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
