# Grasping Stack

This package provides grasping functionality for [UR robots](https://www.universal-robots.com/) using [AnyGrasp](https://github.com/graspnet/anygrasp_sdk) with the help of a [RealSense Camera](https://github.com/realsenseai/realsense-ros). 

The devcontainer is based on [nvidia/cuda:12.6.0-cudnn-devel-ubuntu22.04](https://hub.docker.com/layers/nvidia/cuda/12.6.0-cudnn-devel-ubuntu22.04/images/sha256-3814ef2c9d46ca559e601374029a576596f016e33ddf48d6e2ad778d21bfa3f0) image and provides the following software stack:

- Pytorch 2.10
- CUDA 12.6
- CUDNN9
- ROS Humble (Base container is ubuntu 22.04)
- [chenxi-wang/MinkowskiEngine](https://github.com/chenxi-wang/MinkowskiEngine.git)
- [CollaborativeRoboticsLab/graspnetAPI](https://github.com/CollaborativeRoboticsLab/graspnetAPI.git)
- [graspnet/anygrasp_sdk](https://github.com/graspnet/anygrasp_sdk.git)
- [Realsense packages](https://github.com/realsenseai/realsense-ros)
- [UR packages](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

Read here for [known issues and how we fixed them](./docs/issues.md)

## System Architecture

![System Architecture](./docs/images/system.png)

### Camera Driver

In this graping framework, instructions related to Camera setup, configuration and customization are in the linked file. We currebtly support following devices,

- [Realsense Camera](./docs/camera/realsense.md)

### RGBD to Pointcloud

As show in the system architecture diagram, as a preprocessing step, we combine the RGB and Depth images to create a colored pointcloud. This is done using the `anygrasp_ros/rgbd_to_pointcloud_node` node which subscribes to RGB and Depth image topics, synchronizes them, and publishes the resulting colored pointcloud for use in grasp pose detection.

- [RGBD to Pointcloud Node](https://github.com/CollaborativeRoboticsLab/anygrasp_ros/blob/main/docs/rgbd_to_pointcloud.md)

### Anygrasp Node

In this grasping framework we utilize anygrasp for grasp pose detection. The anygrasp has been interfaced with ROS2 via `anygrasp_ros/anygrasp_detection_node` and `anygrasp_ros/anygrasp_tracking_node`. These two node expects a colored pointcloud as input (which we provide via the `rgbd_to_pointcloud_node`). We have configured the devcontainer to install the anygraph along with its dependencies. Read following related documentation.

- [License requesting and loading, node customization and starting](https://github.com/CollaborativeRoboticsLab/anygrasp_ros/blob/main/README.md)
- [Testing the anygrasp installation](https://github.com/CollaborativeRoboticsLab/anygrasp_ros/blob/main/docs/testing.md)
- [Anygrasp Detection Node](https://github.com/CollaborativeRoboticsLab/anygrasp_ros/blob/main/docs/detection.md)
- [Anygrasp Tracking Node](https://github.com/CollaborativeRoboticsLab/anygrasp_ros/blob/main/docs/tracking.md)


### Gripper Controller

In this grasping framework, we evaluate different gripper types. Due to this we focus on custom built grippers and our gripper controller revolves around different types for servos used to build the grippers and are availble in [CollaborativeRoboticsLab/grippers](https://github.com/CollaborativeRoboticsLab/grippers). Instructions related to setup, configuration and customization are in the linked file.

- [Dynamixel Grippers](https://github.com/CollaborativeRoboticsLab/grippers/blob/main/docs/dynamixel.md)
- [Feetech Grippers](https://github.com/CollaborativeRoboticsLab/grippers/blob/main/docs/feetech.md)

### Arm Controller

In this grasping framework, we utilize the ur10 manipulator. Instructions related to setup, configuration and calibration are in the linked files.

- [UR10 and Devcontainer connection](./docs/manipulator/connection.md)
- [UR10 calibration](./docs/manipulator/calibration.md)
- [UR10 startup](./docs/manipulator/universal.md)
- [UR10 TF frames for gripper compatibility](./docs/manipulator/tf_frames.md)
- [UR10 attaching new gripper and components](./docs/manipulator/adding_new_components.md)

### Arm Control and Workspace Creation

This component transforms grasp poses, applies workspace obstacles to MoveIt, visualizes the calibrated workspace area, and rejects poses outside that area.

- [Workspace Creation](./docs/workspace/creation.md)
- [Arm Control](./docs/control/arm_control.md)

### Grasping Pipeline

The grasping pipeline is the main component that orchestrates the grasping process: it requests a grasp pose from AnyGrasp, calls the arm-control action, closes the gripper, and optionally runs a post-grasp move. 

- [Control stack overview](./docs/control/control_stack_overview.md)
- [Grasping pipeline](./docs/control/grasping_pipeline.md)

## Building container

Install VSCode and add the [DevContainer addon](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

Clone this repo and open using VSCode. Generally VScode should auto detect, if not press Shift+Ctrl+P to open the command palette and select "DevContainer: Rebuild and Reopen the container" option.

Following are quick commands to match our specific setup. Read relevant sections under `System architecture` to find other supported commands.

## Connecting with extenral docker containers/devcontainers

To connect with external devices running other docker containers, please follow following guidelines

- [External container using network=host](./docs/external/container.md)

## Quick Commands

Following commands are for quick launching our system. Use relevant commands if components differ.

### Starting the camera

Use the following command to start the realsense D435 camera.

```bash
source install/setup.bash
ros2 launch grasping_camera d435.launch.py
```

### Starting the anygrasp detection system

Use the following command to start the anygrasp system

```bash
source install/setup.bash
ros2 launch anygrasp_ros detection.launch.py
```

### Start the UR10 Manipulator and gripper with MoveIt

#### For `UR10 with soft two-finger gripper`

```bash
source install/setup.bash
ros2 launch ur10_soft_two_fingers_moveit_config hardware.launch.py
```

### Start the gripping process

Use the following command to start the gripping process

```bash
source install/setup.bash
ros2 launch grasping grip.launch.py server_mode:=false
```