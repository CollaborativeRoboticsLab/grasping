# Adding new Manipulator Components

In this repository we test multiple grippers with the same manipulator. 

If you want to add a new gripper, you start can following the instructions in [Adding New Grippers](https://github.com/CollaborativeRoboticsLab/grippers/blob/main/docs/gripper/adding-new-grippers.md) document.

Once you have a new gripper ready, you can add it to the manipulator by following the steps discussed in this document.

## Preparing the combined robot description

Combine the description files of the components you need to use into a single xacro file. For example, if you want to add a new gripper to the UR10 manipulator, you can create a new xacro file that includes both the UR10 and the new gripper descriptions.

We recommend creating a new xacro file in the `grasping_description/xacro` folder for each new combination of manipulator and gripper, for example [`ur10_with_gripper.urdf.xacro`](../../grasping_description/xacro/ur10-soft-two-fingers.urdf.xacro). This way you can keep the original component descriptions unchanged and easily switch between different combinations.

In this project we follow this and use the following naming convention for the combined descriptions:

```text
<manipulator>-<gripper>.urdf.xacro
```

Currently we have,

| Description | Manipulator and Gripper |
|-------------|------------------------|
| `ur10-soft-two-fingers.urdf.xacro` | UR10 manipulator and the softtwo-finger gripper |

## Updating the MoveIt configuration

Once you have the combined description ready, you need to update/create the MoveIt configuration to use it. 

This allows you to define self-collision and other properties for the new combination of manipulator and gripper.

You can create a new MoveIt configuration package for the new combination, or you can update an existing one if it already exists. 

Recommended approach is to use the Moveit Setup Assistant to create a new MoveIt configuration package for the new combination. This way you can easily define the necessary properties and configurations for the new setup.

To create a new MoveIt configuration package using the MoveIt Setup Assistant, follow these steps:

1. Build your workspace and source the setup file to make sure all your packages are available:

```bash
colcon build
source install/setup.bash
```

1. Open the MoveIt Setup Assistant by running the following command in your terminal:

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```  

2. Follow the instructions [here](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html) to complete the setup.


Recommended naming convention for the MoveIt configuration packages is:

```text
<manipulator>_<gripper>_moveit_config
``` 

Currently we have,

- `ur10_soft_two_fingers_moveit_config` which is the MoveIt configuration package for the UR10 with two-finger gripper.

| Description | MoveIt Configuration Package | Manipulator and Gripper |
|-------------|-------------------------------|------------------------|
| `ur10-soft-two-fingers.urdf.xacro` | `ur10_soft_two_fingers_moveit_config` | UR10 manipulator and the softtwo-finger gripper |

## Updating the launch files

Finally, you need to update the launch files to use the new MoveIt configuration and the combined description.

You can create a new launch file for the new combination, or you can update an existing one if it already exists.

Recommended approach is to create a new launch file for the new combination. This way you can easily switch between different setups by launching the corresponding launch file. Additionally, you can use the same naming convention for the launch files as well:

```text
<manipulator>_<gripper>_moveit.launch.py
```

Currently we have,

| Description | Launch File | Manipulator and Gripper |
|-------------|-------------|------------------------|
| `ur10-soft-two-fingers.urdf.xacro` | `ur10_soft_two_fingers_moveit.launch.py` | UR10 manipulator and the softtwo-finger gripper |

To create a new launch file, you can copy an existing one and modify it to use the new MoveIt configuration and the combined description.