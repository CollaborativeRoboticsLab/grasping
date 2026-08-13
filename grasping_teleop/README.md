# Grasping Teleop

This package contains operator-facing teleoperation utilities for the grasping stack.

## Keyboard Servo Teleop

With MoveIt Servo already running, start the keyboard teleop:

```bash
source install/setup.bash
ros2 run grasping_teleop servo_teleop_node
```

## Read Current Pose Or Joints

Read the current transform:

```bash
source install/setup.bash
ros2 run grasping_teleop read_pose_node --ros-args -p from:=base_link -p to:=camera_link
```

Read the current joint positions:

```bash
source install/setup.bash
ros2 run grasping_teleop read_pose_node --ros-args -p mode:=joint
```