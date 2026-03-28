# Anygrasp Node

## Setup Information

### Creating docker network

To have a stable feature id for the anygrasp license, we utilize built-in docker network `bridge` and a fixed mac address. For the dev container, this is represented by following config. Change the given mac address as required.

```json
  "runArgs": [
    "--network=bridge",
    "--mac-address=02:42:de:ad:be:ef"
  ]
```

### Adding license

Once the Container is built, run the `license_checker` function from anygrasp_sdk and apply for the license following the steps from [here](https://github.com/graspnet/anygrasp_sdk/blob/main/license_registration/README.md). 

Following commands will help to run the `license_checker` within the dev container.

```bash
/dependencies/anygrasp_sdk/license_registration/license_checker -f
```

Once you fill the form and receive the license zip file, unzip and copy it to the `/license` folder within the cloned repo (Not inside the container). Devcontainer has been configured to mount the license folder into the following location of the container,

- `/dependencies/precompiled/license`

To check the license run following command

```bash
/dependencies/anygrasp_sdk/license_registration/license_checker -c /dependencies/precompiled/license/licenseCfg.json
```

### Adding model weights

Copy the detection and tracking model weights into `weights/detection` and `weights/tracking` folders within the cloned repo (Not inside the container) respectively. These will be mounted into following folders inside the container. 

- `/dependencies/precompiled/weights/detection`             allows to run the ros2 packages
- `/dependencies/precompiled/weights/tracking`              allows to run the ros2 packages

This can also be done alongside the prior `Adding Licesne` step.

[Read more information on testing the anygrasp installation](./testing.md)

## Starting the anygrasp detection system

Use the following command to start the anygrasp system

```bash
source install/setup.bash
ros2 launch anygrasp_ros detection.launch.py
```