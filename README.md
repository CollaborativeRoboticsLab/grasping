# ur10-grasping

This package provides grasping functionality for a [UR robot](https://www.universal-robots.com/) using [AnyGrasp](https://github.com/graspnet/anygrasp_sdk) with the help of a [RealSense Camera](https://github.com/realsenseai/realsense-ros). 

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

## Setup

### Creating docker network

Create the docker [mavclan](https://docs.docker.com/engine/network/drivers/macvlan/) network following [these instructions](./docs/macvlan.md). This limits the host to Linux OS (Windows, WSL and MacOS not supported) but AnyGrasp Licensing requires this apprach when using a docker based implementation.

### Building container

Install VSCode and add the [DevContainer addon](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

Clone this repo and open using VSCode. Generally VScode should auto detect, if not press Shift+Ctrl+P to open the command palette and select "DevContainer: Rebuild and Reopen the container" option.

### Adding license

Once the Container is built, run the `license_checker` function from anygrasp_sdk and apply for the license following the steps from [here](https://github.com/graspnet/anygrasp_sdk/blob/main/license_registration/README.md). 

Following commands will help to run the `license_checker` within the dev container.

```bash
cd /dependencies/anygrasp_sdk/license_registration/
./license_checker -f
```

Once you fill the form and receive the license zip file, unzip and copy it to the `/license` folder within the cloned repo (Not inside the container). Devcontainer has been configured to mount the license folder into the following locations of the container,

- `/home/ubuntu/colcon_ws/license`

To check the license run following command

```bash
cd /dependencies/anygrasp_sdk/license_registration/
./license_checker -c /dependencies/precompiled/license/licenseCfg.json
```

### Adding model weights

Copy the detection and tracking model weights into `weights/detection` and `weights/tracking` folders respectively. These will be mounted into following folders inside the container. 

- `/dependencies/precompiled/weights/detection`             allows to run the ros2 packages
- `/dependencies/precompiled/weights/tracking`              allows to run the ros2 packages

This can also be done alongside the prior `Adding Licesne` step.

### Basic testing

Try running the `grasp_detection/demo.py` and `grasp_tracking/demo.py` to confirm the process pipeline is working

## Usage

### Starting the camera

Use the following command to start the camera

```bash
ros2 launch ur_launch camera.launch.py
```

### Starting the anygrasp detection system

Use the following command to start the anygrasp system

```bash
ros2 launch anygrasp_ros detection.launch.py
```

### Start the gripper

Use the following command to start the gripper controller

```bash
ros2 launch gripper_ros dynamixel.launch.py
```

### Start the UR robot

Use the following command to start the ur moveit control

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=192.168.56.101
```

## Start the gripping process

Use the following command to start the gripping process

```bash
ros2 launch ur_launch grip.launch.py server_mode:=false
```