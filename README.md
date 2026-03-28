# Grasping Stack

This package provides grasping functionality for [UR robots](https://www.universal-robots.com/) and [TM Robots](https://www.tm-robot.com/en/) using [AnyGrasp](https://github.com/graspnet/anygrasp_sdk) with the help of a [RealSense Camera](https://github.com/realsenseai/realsense-ros). 

The devcontainer is based on [pytorch/pytorch:2.10.0-cuda12.6-cudnn9-devel](https://hub.docker.com/layers/pytorch/pytorch/2.10.0-cuda12.6-cudnn9-devel/images/sha256-df80e10d07cd114c5f33380e3df7b6c5a3caab8481f68509ea652a7c0908316e) image and provides
- Pytorch 2.10
- CUDA 12.6
- CUDNN9
- ROS Jazzy (Base container is ubuntu 24.04)
- [chenxi-wang/MinkowskiEngine](https://github.com/chenxi-wang/MinkowskiEngine.git)
- [CollaborativeRoboticsLab/graspnetAPI](https://github.com/CollaborativeRoboticsLab/graspnetAPI.git)
- [graspnet/anygrasp_sdk](https://github.com/graspnet/anygrasp_sdk.git)
- [Realsense packages](https://github.com/realsenseai/realsense-ros)
- [UR packages](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

Read here for [known issues and fixes](./docs/issues.md)

## System Architecture

![System Architecture](./docs/images/system.png)

### Camera Driver

As show in the system architecture diagram, we utilize RGB and Depth images to identify objects and their grasping poses. In this graping framework, instructions related to setup, configuration and customization are in the linked file.

- [Realsense Camera](./docs/camera/realsense.md)

### Arm Controller

In this grasping framework, we utilize following manipulators. Instructions related to setup, configuration and customization are in the linked file.

- [UR Manipulator](./docs/manipulator/universal.md)
- [TM Manipulator](./docs/manipulator/techman.md)

### Gripper Controller

In this grasping framework, we evaluate different gripper types. Due to this we focus on custom built grippers and our gripper controller revolves around different types for servos used to build the grippers and over controllers are availble in [CollaborativeRoboticsLab/grippers](https://github.com/CollaborativeRoboticsLab/grippers). Instructions related to setup, configuration and customization are in the linked file.

- [Dynamixel Grippers](https://github.com/CollaborativeRoboticsLab/grippers/blob/main/docs/dynamixel.md)
- [Feetech Grippers](https://github.com/CollaborativeRoboticsLab/grippers/blob/main/docs/feetech.md)

### Anygrasp Node

In this grasping framework we utilize anygrasp for grasp pose detection. We have configured the devcontainer to install the anygraph along with its dependencies. License need to be requested and loaded as described following instructions [docs/anygrasp.md](./docs/anygrasp.md).

### Grasping Node

Grasping node is reponsible for guiding the robot arm to selected grasp pose (from Anygrasp Node) and Controlling the gripper to grasp. More information about the implementation can be found in [docs/grasping.md](./docs/grasping.md)

## Building container

Install VSCode and add the [DevContainer addon](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

Clone this repo and open using VSCode. Generally VScode should auto detect, if not press Shift+Ctrl+P to open the command palette and select "DevContainer: Rebuild and Reopen the container" option.

Following are quick commands to match our specific setup. Read relevant sections under `System architecture` to find other supported commands.

## Quick Commands

### Starting the camera

Use the following command to start the realsense D435 camera. For other systems, look at [documentation](./docs/camera/)

```bash
source install/setup.bash
ros2 launch grasping_camera d435.launch.py
```

### Start the Manipulator

Use the following command to start the UR robot control

```bash
source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=10.0.0.89
```

or use the following command to start the TM robot control

```bash
source install/setup.bash
ros2 launch tm12x_moveit_config tm12x_run_move_group.launch.py
```

### Start the gripper

Use the following command to start the gripper controller

```bash
source install/setup.bash
ros2 launch gripper_ros dynamixel.launch.py
```

### Starting the anygrasp detection system

Use the following command to start the anygrasp system

```bash
source install/setup.bash
ros2 launch anygrasp_ros detection.launch.py
```

### Start the gripping process

Use the following command to start the gripping process

```bash
source install/setup.bash
ros2 launch grasping grip.launch.py server_mode:=false
```