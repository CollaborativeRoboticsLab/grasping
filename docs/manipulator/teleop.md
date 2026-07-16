# Manipulator Teleoperation

This document covers manual UR10 jogging with the keyboard teleop node that publishes MoveIt Servo commands.

## Prerequisites

Start the robot in Servo mode first:

```bash
source install/setup.bash
ros2 launch ur10_soft_two_fingers_moveit_config hardware_with_moveit.launch.py launch_servo:=true
```

```bash
source install/setup.bash
ros2 launch tm12s_soft_two_fingers_moveit_config hardware_with_moveit.launch.py launch_servo:=true
```

That launch:

- starts the UR driver and MoveIt
- launches the `moveit_servo` node
- activates `forward_position_controller` for arm jogging instead of the trajectory controller

In this mode, keyboard teleop is the intended way to move the arm. Normal RViz trajectory execution is not the active arm command path in the same session.

## Start Teleop

Run the keyboard teleop in a separate terminal:

```bash
source install/setup.bash
ros2 run grasping_control servo_teleop
```

The node automatically calls `/servo_node/start_servo` on startup.

## Key Bindings

Linear motion in the `tool_tip` frame:

- `w/s`: `+x / -x`
- `a/d`: `+y / -y`
- `q/e`: `+z / -z`

Angular motion in the `tool_tip` frame:

- `u/o`: `+roll / -roll`
- `i/k`: `+pitch / -pitch`
- `j/l`: `+yaw / -yaw`

Servo control:

- `space`: stop motion
- `v`: start Servo
- `b`: stop Servo
- `h`: print help
- `x`: stop Servo and quit teleop

## Motion Behavior

This teleop is hold-to-move, not latch-to-move.

- hold a key to keep sending jog commands
- release the key and the command times out shortly after
- press `space` for an immediate stop

The teleop publishes `geometry_msgs/msg/TwistStamped` messages to `/servo_node/delta_twist_cmds`.

## Parameters

Useful runtime parameters:

- `topic`: Servo twist input topic. Default `/servo_node/delta_twist_cmds`
- `start_service`: Servo start service. Default `/servo_node/start_servo`
- `stop_service`: Servo stop service. Default `/servo_node/stop_servo`
- `frame_id`: command frame. Default `tool_tip`
- `linear_speed`: linear jogging speed in m/s. Default `0.50`
- `angular_speed`: angular jogging speed in rad/s. Default `0.75`
- `publish_rate_hz`: publish rate. Default `30.0`
- `command_timeout`: stop delay after key release. Default `0.18`
- `enable_smoothing`: whether commands ramp instead of step. Default `true`
- `smoothing_alpha`: smoothing blend factor in `[0, 1]`. Default `0.25`

Example with gentler motion:

```bash
source install/setup.bash
ros2 run grasping_control servo_teleop --ros-args \
	-p linear_speed:=0.15 \
	-p angular_speed:=0.30 \
	-p enable_smoothing:=true \
	-p smoothing_alpha:=0.15
```

## Smoothing

When `enable_smoothing` is `true`, the teleop ramps the published command toward the requested key command instead of jumping instantly from zero to full value.

- lower `smoothing_alpha` gives softer motion but slower response
- higher `smoothing_alpha` gives faster response but can feel more abrupt
- `enable_smoothing:=false` restores direct step changes

## Troubleshooting

If the arm does not move:

1. Make sure the robot was launched with `launch_servo:=true`.
2. Make sure the teleop terminal sourced `install/setup.bash`.
3. Press `v` to explicitly start Servo again.
4. Confirm the robot is not stopped by safety, protective stop, or program state on the UR side.

If RViz execution fails while Servo mode is active, that is expected. Servo mode activates `forward_position_controller`, while RViz plan execution normally targets the trajectory controller.
