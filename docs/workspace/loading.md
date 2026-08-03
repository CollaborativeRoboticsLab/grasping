# Loading the workspace for planning

To run the system without a robot, use the `use_demo:=true` launch parameter.

This checkout currently only includes `motion_execution.launch.py` under `grasping_control/launch`

Each robot stack (TM, UR, etc.) should provide its own launch file for motion execution. The launch file should include the `motion_execution.launch.py` from this package and have its own motion configuration and workspace configuration files.

Following are some examples,

- [UR10 Manipulator launch](https://github.com/CollaborativeRoboticsLab/grasping_ur/blob/main/grasping_ur/launch/ur10_soft_two_fingers.launch.py)
- [TM12s Manipulator launch](https://github.com/CollaborativeRoboticsLab/grasping_tm/blob/main/grasping_tm/launch/tm12s_soft_two_fingers.launch.py)
- [LD250 & TM12x Mobile Manipulator launch](https://github.com/CollaborativeRoboticsLab/grasping_omron_moma/blob/main/grasping_omron_moma/launch/ld250_tm12x_soft_two_fingers.launch.py)