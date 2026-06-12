# Grasping Arm Control

This package contains the UR10 arm control node and related launch files. It also includes the workspace creation component that applies workspace obstacles to MoveIt and rejects poses outside the calibrated workspace area.


## Starting the system

This package now exposes multiple launch files to facilitate different combinations of arm and gripper launch configurations.

### Base UR10 arm launch

```bash
source install/setup.bash
ros2 launch grasping_arm_control ur10.launch.py
```

### UR10 arm with soft two-finger gripper launch

```bash
source install/setup.bash
ros2 launch grasping_arm_control ur10_soft_two_fingers_gripper.launch.py
```
