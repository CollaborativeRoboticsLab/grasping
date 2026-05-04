# Universal Robot Manipulator

## Installation

We have configured the devcontainer to install the drivers required by the ur robots during the build process. 

If you are setting this up yourself (outside the devcontainer) and using a Universal Robot Manipulator, follow the official instructions [here](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) or [here](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/installation.html)

## Calibration

We are using a `ur10` mnaipulator in our system and we utilize standard launch files to start the robot. 

Since the calibration step depends on each robot, follow the instruction here to [calibrate](./../control/ur_calibration.md).

## Start the UR robot

Use the following command to start the UR robot control

```bash
source install/setup.bash
ros2 launch grasping_arm_control ur10.launch.py
```

This wrapper launch file includes the default `ur_robot_driver` and `ur_moveit_config` launches with these defaults:

- `ur_type:=ur10`
- `robot_ip:=10.0.0.89`
- `kinematics_params_file:=/home/ubuntu/colcon_ws/src/ur_grasping/grasping_arm_control/config/ur_kinematics.yaml`
- `launch_rviz:=true`
- `initial_joint_controller:=scaled_joint_trajectory_controller`

Override any of them on the command line when needed, for example

```bash
source install/setup.bash
ros2 launch grasping_arm_control ur10.launch.py robot_ip:=10.0.0.89 launch_rviz:=false
```

if the execution fails, try the following command

```bash
source install/setup.bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['scaled_joint_trajectory_controller'], deactivate_controllers: [], strictness: 1, activate_asap: true, timeout: {sec: 5, nanosec: 0}}"
```