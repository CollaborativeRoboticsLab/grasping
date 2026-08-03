# Universal Robot Manipulator

## Installation

We have configured the devcontainer to install the drivers required by the ur robots during the build process. 

If you are setting this up yourself (outside the devcontainer) and using a Universal Robot Manipulator, follow the official instructions [here](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) or [here](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/installation.html)

## Calibration

We are using a `ur10` manipulator in our system and we utilize standard launch files to start the robot. 

Since the calibration step depends on each robot, follow the instructions here to [calibrate](./ur10_calibration.md).

## Start the UR robot

Use the following command to start the UR robot control

```bash
source install/setup.bash
ros2 launch grasping_control ur10_soft_two_fingers.launch.py
```

This wrapper launch file includes the default `ur10_soft_two_fingers_moveit_config` hardware launch when `use_demo:=false`, or the package demo launch when `use_demo:=true`. It also starts the `motion_execution_node` from `grasping_control`.

The wrapper itself defines these defaults:

- `use_demo:=false`
- `workspace_file:=workspace_empty.yaml`

Motion execution defaults, including action names, planning group, planning frame, TCP link, MoveIt tuning, and named poses such as `workspace_center_pose`, `pre_grasp_pose`, and `post_grasp_pose`, are loaded into `motion_execution_node` from `<grasping_control_share>/config/motion_config.yaml` as ROS parameters. The calibrated table/workspace scene is selected with `workspace_file` and loaded as a ROS parameter file.

The included `ur10_soft_two_fingers_moveit_config` launch still provides its own driver, MoveIt, and soft two-finger gripper action server on `/gripper_command`. It also provides arguments such as `robot_ip`, `launch_rviz`, and `initial_joint_controller`.

Do not start `gripper_ros gripper_soft_two_fingers.launch.py` separately with this stack unless you intentionally want an independent gripper-only session.

Override any of them on the command line when needed, for example

```bash
source install/setup.bash
ros2 launch grasping_control ur10_soft_two_fingers.launch.py robot_ip:=10.0.0.89 launch_rviz:=false
```

Or switch the wrapper to demo mode:

```bash
source install/setup.bash
ros2 launch grasping_control ur10_soft_two_fingers.launch.py use_demo:=true
```

if the execution fails, try the following command

```bash
source install/setup.bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['scaled_joint_trajectory_controller'], deactivate_controllers: [], strictness: 1, activate_asap: true, timeout: {sec: 5, nanosec: 0}}"
```