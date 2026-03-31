# Universal Robot Manipulator

## Installation

We have configured the devcontainer to install the drivers required by the ur robots during the build process. 

If you are setting this up yourself (outside the devcontainer) and using a Techman Robot Manipulator, follow the official instructions [here](https://github.com/CollaborativeRoboticsLab/tmr_ros2).

## Calibration

We are using a `tm12m` mnaipulator in our system and we utilize standard launch files to start the robot.

## Start the TM robot

Use the following command to start the ur moveit control

```bash
source install/setup.bash
ros2 launch tm12x_moveit_config tm12x_run_move_group.launch.py
```