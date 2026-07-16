# TechMann Robot Manipulator

## Installation

We have configured the devcontainer to install the drivers required by the TM robots during the build process. 
    
If you are setting this up yourself (outside the devcontainer) and using a TechMann Robot Manipulator, follow the official instructions [here](https://github.com/CollaborativeRoboticsLab/tm2_ros2)

## Calibration

We are using a `tm12s` manipulator in our system and we utilize standard launch files to start the robot. 

## Start the TM12S robot

Use the following command to start the TM12S robot control

```bash
source install/setup.bash
ros2 launch grasping_control tm12s.launch.py
```

This wrapper launch file includes the default `tm12s_moveit_config` hardware launch when `use_demo:=false`, or the package demo launch when `use_demo:=true`. It also starts the `motion_execution_node` from `grasping_control`.

The wrapper itself defines these defaults:

- `use_demo:=false`
- `arm_action_name:=move_arm_to_pose`
- `move_group_action_name:=move_action`
- `planning_group:=tmr_arm`
- `planning_frame:=base`
- `end_effector_link:=flange`
- `allowed_planning_time:=5.0`
- `num_planning_attempts:=5`
- `max_velocity_scaling:=0.2`
- `max_acceleration_scaling:=0.2`
- `position_tolerance_m:=0.005`
- `orientation_tolerance_rad:=0.1`
- `planning_pipeline_id:=''`
- `planner_id:=''`
- `workspace_config_path:=''`

The included `tm12s_moveit_config` launch still provides its own driver and MoveIt arguments, such as `robot_ip` and `launch_rviz`.

Override any of them on the command line when needed, for example

```bash
source install/setup.bash
ros2 launch grasping_control tm12s.launch.py robot_ip:=10.0.0.89 launch_rviz:=false
```

Or switch the wrapper to demo mode:

```bash
source install/setup.bash
ros2 launch grasping_control tm12s.launch.py use_demo:=true
```

if the execution fails, try the following command

```bash
source install/setup.bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['scaled_joint_trajectory_controller'], deactivate_controllers: [], strictness: 1, activate_asap: true, timeout: {sec: 5, nanosec: 0}}"
```