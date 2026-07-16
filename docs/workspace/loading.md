# Loading the workspace for planning

To run the system without a robot, use the `use_demo:=true` launch parameter.

## UR10 arm with soft two-finger gripper

Start the arm control node with the calibrated workspace file and robot

```bash
source install/setup.bash
ros2 launch grasping_control ur10_soft_two_fingers.launch.py
```