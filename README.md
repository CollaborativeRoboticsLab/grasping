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

Once you fill the form and receive the license zip file, unzip and copy it to the `/license` folder within the cloned repo (Not inside the container). Then rebuild the container. This will automatically copy the license content into the following locations of the container,

- `/dependencies/anygrasp_sdk/license_registration/license`  allows to run license checker
- `/dependencies/anygrasp_sdk/grasp_detection/license`       allows to run the grasp detection
- `/dependencies/anygrasp_sdk/grasp_tracking/license`        allows to run the grasp tracking
- `/home/ubuntu/colcon_ws/license`                           allows to run the ros2 packages

To check the license run following command

```bash
cd /dependencies/anygrasp_sdk/license_registration/
./license_checker -c license/licenseCfg.json
```

### Adding model weights

Copy the detection and tracking model weights into `weights/detection` and `weights/tracking` folders respectively and Rebuild the container.
These will be loaded into following folders inside the container. 

- `/dependencies/anygrasp_sdk/grasp_detection/log`       allows to run the grasp detection
- `/dependencies/anygrasp_sdk/grasp_tracking/log`        allows to run the grasp tracking
- `/home/ubuntu/colcon_ws/weights/detection`             allows to run the ros2 packages
- `/home/ubuntu/colcon_ws/weights/tracking`              allows to run the ros2 packages

This can also be done alongside the prior `Adding Licesne` step.

### Basic testing

Try running the `grasp_detection/demo.py` and `grasp_tracking/demo.py` to confirm the process pipeline is working


## Usage

### Starting the camera

Use the following command to start the camera

```bash

```