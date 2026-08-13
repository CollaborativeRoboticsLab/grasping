# Grasping Control

This package contains the UR10 arm control node, workspace creation flow, and related launch files. It applies workspace obstacles to MoveIt and rejects poses outside the calibrated workspace area.


## Starting the system

This package now exposes multiple launch files to facilitate different combinations of arm and gripper launch configurations.

### Base UR10 arm launch

```bash
source install/setup.bash
ros2 launch ur10_moveit_config hardware_with_moveit.launch.py
```

### UR10 arm with soft two-finger gripper launch

```bash
source install/setup.bash
ros2 launch ur10_soft_two_fingers_moveit_config hardware_with_moveit.launch.py
```

## Teleop utilities

Keyboard teleop and pose-reading helpers now live in the `grasping_teleop` package.
