# LD250 TM12X Manipulator

## Installation

We have configured the devcontainer to install the drivers required by the TM robots during the build process.
    
If you are setting this up yourself (outside the devcontainer) and using a Techman Robot Manipulator, follow the official instructions [here](https://github.com/CollaborativeRoboticsLab/tm2_ros2)

## Calibration

We use an `ld250_tm12x` mobile manipulator with the TM12X arm mounted on the LD250 base and the soft two-finger gripper attached to the tool side.

## Start the Manipulator MoveIt Stack

Use the following command to start the combined LD250 + TM12X + gripper MoveIt stack through the `grasping_control` wrapper:

```bash
source install/setup.bash
ros2 launch grasping_control ld250_tm12x_soft_two_fingers.launch.py tm_robot_ip:=<robot_controller_ip>
```

This wrapper includes `ld250_tm12x_soft_two_fingers_moveit_config/hardware_with_moveit.launch.py` when `use_demo:=false`, or the package demo launch when `use_demo:=true`. It also starts the `motion_execution_node` from `grasping_control`.

The included hardware launch starts the shared soft two-finger gripper action server on `/gripper_command`. Do not start `gripper_ros gripper_soft_two_fingers.launch.py` separately with this stack unless you intentionally want an independent gripper-only session.

It can also include the LD250 base hardware and Nav2 stack from `moma_ros` when requested, while still keeping the arm + gripper + MoveIt path as the default behavior.

The launch file defines these primary arguments:

- `tm_robot_ip:=<robot_controller_ip>`
- `tm_use_simulation:=false`
- `no_logging:=false`
- `launch_servo:=false`
- `use_base:=false`
- `use_nav2:=false`

The combined MoveIt configuration uses:

- `planning_group:=tmr_arm_with_base`
- `planning_frame:=base_link`
- `end_effector_link:=tool_tip`

Enable Servo if you want jogging through the forward position controller:

```bash
source install/setup.bash
ros2 launch grasping_control ld250_tm12x_soft_two_fingers.launch.py \
	tm_robot_ip:=<robot_controller_ip> \
	launch_servo:=true
```

For demo mode without hardware, use:

```bash
source install/setup.bash
ros2 launch grasping_control ld250_tm12x_soft_two_fingers.launch.py use_demo:=true
```

To start the full mobile manipulator from the same high-level launch, enable the base and Nav2 explicitly:

```bash
source install/setup.bash
ros2 launch grasping_control ld250_tm12x_soft_two_fingers.launch.py \
	tm_robot_ip:=<robot_controller_ip> \
	use_base:=true \
	use_nav2:=true
```

With `use_base:=true`, the wrapper reuses the LD250 base hardware launch from `moma_ros` and keeps `use_arm:=false` on that included launch so the TM arm hardware is still owned only by the grasping stack.

## Notes

If controller activation fails during physical bringup, check the active controllers in `/controller_manager` and switch the required combined arm controller explicitly.

```bash
source install/setup.bash
ros2 control list_controllers
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['tmr_arm_with_base_controller'], deactivate_controllers: ['forward_position_controller'], strictness: 1, activate_asap: true, timeout: {sec: 5, nanosec: 0}}"
```