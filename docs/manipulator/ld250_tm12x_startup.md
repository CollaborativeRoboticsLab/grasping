# LD250 TM12X Manipulator

## Installation

We have configured the devcontainer to install the drivers required by the TM robots during the build process.
    
If you are setting this up yourself (outside the devcontainer) and using a Techman Robot Manipulator, follow the official instructions [here](https://github.com/CollaborativeRoboticsLab/tm2_ros2)

## Calibration

We use an `ld250_tm12x` mobile manipulator with the TM12X arm mounted on the LD250 base and the soft two-finger gripper attached to the tool side.

## Start the Manipulator MoveIt Stack

Use the following command to start the combined LD250 + TM12X + gripper MoveIt stack:

```bash
source install/setup.bash
ros2 launch ld250_tm12x_soft_two_fingers_moveit_config hardware_with_moveit.launch.py tm_robot_ip:=<robot_controller_ip>
```

This launch starts the TM driver, robot state publisher, MoveIt, RViz, ros2_control, and the combined arm and gripper controllers for the `ld250_tm12x_soft_two_fingers` model.

The launch file defines these primary arguments:

- `tm_robot_ip:=<robot_controller_ip>`
- `tm_use_simulation:=false`
- `no_logging:=false`
- `launch_servo:=false`

The combined MoveIt configuration uses:

- `planning_group:=tmr_arm_with_base`
- `planning_frame:=base_link`
- `end_effector_link:=tool_tip`

Enable Servo if you want jogging through the forward position controller:

```bash
source install/setup.bash
ros2 launch ld250_tm12x_soft_two_fingers_moveit_config hardware_with_moveit.launch.py \
	tm_robot_ip:=<robot_controller_ip> \
	launch_servo:=true
```

For demo mode without hardware, use:

```bash
source install/setup.bash
ros2 launch ld250_tm12x_soft_two_fingers_moveit_config demo.launch.py
```

## Start the Full Mobile Manipulator Stack

If you want the LD250 base, TM12X arm, MoveIt, Nav2, and RViz together, use the top-level platform bringup:

```bash
source install/setup.bash
ros2 launch moma_ros ld250_tm12x.launch.py tm_robot_ip:=<robot_controller_ip>
```

This launch defines these top-level defaults:

- `tm_use_simulation:=false`
- `tm_robot_ip:=192.168.1.2`
- `use_rviz:=auto`
- `use_moveit:=true`
- `use_nav2:=true`

The included hardware launch also supports these lower-level options when you launch it directly:

- `use_arm:=true`
- `use_base:=true`
- `robot_description_override:=true`

## Notes

The manipulator-only launch above is the correct entrypoint for the new `ld250_tm12x_soft_two_fingers_moveit_config` package.

If controller activation fails during physical bringup, check the active controllers in `/controller_manager` and switch the required combined arm controller explicitly.

```bash
source install/setup.bash
ros2 control list_controllers
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['tmr_arm_with_base_controller'], deactivate_controllers: ['forward_position_controller'], strictness: 1, activate_asap: true, timeout: {sec: 5, nanosec: 0}}"
```