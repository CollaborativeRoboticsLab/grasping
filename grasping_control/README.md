# Grasping Control

This package contains the UR10 arm control node and related launch files. It also includes the workspace creation component that applies workspace obstacles to MoveIt and rejects poses outside the calibrated workspace area.


## Starting the system

This package now exposes multiple launch files to facilitate different combinations of arm and gripper launch configurations.

### Base UR10 arm launch

```bash
source install/setup.bash
ros2 launch ur10_moveit_config hardware_with_moveit.launch.launch.py
```

### UR10 arm with soft two-finger gripper launch

```bash
source install/setup.bash
ros2 launch ur10_soft_two_fingers_moveit_config hardware_with_moveit.launch.launch.py
```

## Servo keyboard teleop

With MoveIt Servo already running, start the keyboard teleop:

```bash
source install/setup.bash
ros2 run grasping_control servo_teleop
```

Default key bindings:

- `w/s`: tool-frame `+x / -x`
- `a/d`: tool-frame `+y / -y`
- `q/e`: tool-frame `+z / -z`
- `u/o`: `+roll / -roll`
- `i/k`: `+pitch / -pitch`
- `j/l`: `+yaw / -yaw`
- `space`: stop
- `x`: quit
