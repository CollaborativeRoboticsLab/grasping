# ur10-grasping

This package provides grasping functionality for a [UR robot](https://www.universal-robots.com/) using [AnyGrasp](https://github.com/graspnet/anygrasp_sdk) with the help of a [RealSense Camera](https://github.com/realsenseai/realsense-ros). 

Current configurations are focused towards a UR10 robot and a D435 Realsense Camera

## Setup

### Building container

Install VSCode and add the [DevContainer addon](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

Clone this repo and open using VSCode. Generally VScode should auto detect, if not press Shift+Ctrl+P to open the command palette and select "DevContainer: Rebuild and Reopen the container" option.

### Adding license

Once the Container is built, run the `license_checker` function from anygrasp_sdk and apply for the license following the steps from [here](https://github.com/graspnet/anygrasp_sdk/blob/main/license_registration/README.md). 

Following commands will help to locate and run the `license_checker` within the dev container.

```bash
cd /dependencies/anygrasp_sdk/license_registration/license
./license_checker -f
```

Once you fill the form and receive the license zip file, unzip and copy it to the `/license` folder within the repo (Not inside the container). Then rebuild the container.
This will automatically load the license into the following locations of the container,

- `/anygrasp_sdk/license_registration/license`  allows to run license checker
- `/anygrasp_sdk/grasp_detection/license`       allows to run the grasp detection
- `/anygrasp_sdk/grasp_tracking/license`        allows to run the grasp tracking

To check the license run following command

```bash
cd /anygrasp_sdk/license_registration/
./license_checker -c license/licenseCfg.json
```

### Adding model weights

Copy the detection and tracking model weights into `weights/detection` and `weights/tracking` folders respectively and Rebuild the container.
These will be loaded into the `grasp_detection/log` and `grasp_tracking/log` folders inside the container. 

This can also be done alongside the prior `Adding Licesne` step.

### Basic testing

Try running the `grasp_detection/demo.py` and `grasp_tracking/demo.py` to confirm the process pipeline is working
