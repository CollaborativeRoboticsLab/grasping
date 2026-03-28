# Universal Robot Manipulator

## Installation

We have configured the devcontainer to install the drivers required by the ur robots during the build process. 

If you are setting this up yourself (outside the devcontainer) and using a Universal Robot Manipulator, follow the official instructions [here](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) or [here](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/installation.html)

## Calibration

We are using a `ur10` mnaipulator in our system and we utilize standard launch files to start the robot. 

Since the calibration step depends on each robot, follow the instruction here to [calibrate](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/robot_setup.html#extract-calibration-information).


## Start the UR robot

Use the following command to start the ur moveit control

```bash
source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=10.0.0.89
```

## Start UR10 control with rviz

```bash
source install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10 launch_rviz:=true
```