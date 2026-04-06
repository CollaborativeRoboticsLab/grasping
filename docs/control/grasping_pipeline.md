# Grasping Pipeline

This document covers the pipeline controller implemented by the `grasping` package.

## Node

- Package: `grasping`
- Executable: `grasping_node`
- Default node name: `grasping_node`

When the node runs in server mode, it exposes the Trigger service `/grasping_node/run_grasp` when started through `grip.launch.py`.

## Responsibilities

The grasping node only orchestrates the grasp cycle. It does not perform TF transformations or call MoveIt directly.

Its runtime flow is:

1. Call `anygrasp_msgs/srv/GetGrasps` with `count = 1`.
2. Take the first returned `geometry_msgs/PoseStamped`.
3. Send that pose to the arm-control action server.
4. Close the gripper after a successful move.
5. Optionally send a configured post-grasp pose through the same arm-control action.

This keeps grasp generation, arm motion, and gripper control loosely coupled.

## Running Modes

The node supports two operating modes through `server_mode`.

- `true` (default): wait for external requests through the Trigger service.
- `false`: run one grasp cycle immediately after startup and then exit.

### Trigger the service in server mode

Start the node in server mode with:

```bash
ros2 run grasping grasping_node
```

Then on a separate terminal, call the service with:
```bash
ros2 service call /grasping_node/run_grasp std_srvs/srv/Trigger {}
```

## Trigger the service in one-shot mode

Start the node in one-shot mode with:
```bash
ros2 run grasping grasping_node --ros-args -p server_mode:=false
```

## AnyGrasp Interaction

The node expects an [AnyGrasp service](https://github.com/CollaborativeRoboticsLab/anygrasp_ros) that returns a list of `geometry_msgs/PoseStamped` values.

- Default service name: `detection`
- Alternate service name used in this repo: `tracking`

The returned pose keeps the original point-cloud header. The grasping node forwards that pose as-is and relies on `arm_control_node` to transform it into the planning frame.

## Gripper Interaction

The node uses the common gripper actions from `gripper_msgs` in [grippers](https://github.com/CollaborativeRoboticsLab/grippers).

- Open action: `gripper_msgs/action/OpenGripper`
- Close action: `gripper_msgs/action/CloseGripper`

The default grasp cycle uses position-based closing by sending `use_torque_mode=false`.

Available helpers in the node:

- `close_gripper_position()`
- `close_gripper_torque(torque)`

The numeric meaning of `torque` depends on the gripper driver.

## Post-Grasp Move

If `do_post_grasp_move` is enabled, the node builds a `PoseStamped` from `post_grasp_frame` and `post_grasp_pose`, then sends it through the same `MoveToPose` action path used for the grasp target.

Expected pose format:

- `post_grasp_pose = [x, y, z, qx, qy, qz, qw]`

## Parameters

### Pipeline Behavior

- `server_mode`: expose the Trigger service when true, or run once and exit when false
- `anygrasp_service`: AnyGrasp service name, default `detection`
- `arm_action_name`: arm-control action name, default `move_arm_to_pose`

### Post-Grasp Behavior

- `do_post_grasp_move`: enable post-grasp motion, default `true`
- `post_grasp_frame`: frame for the post-grasp target pose, default `base_link`
- `post_grasp_pose`: `[x, y, z, qx, qy, qz, qw]`, default identity pose at the origin

### Gripper Actions

- `open_action_name`: default `/open_gripper`
- `close_action_name`: default `/close_gripper`

## Required Services and Actions

For a successful cycle, the node depends on:

1. An AnyGrasp service configured by `anygrasp_service`.
2. A `grasping_msgs/action/MoveToPose` server configured by `arm_action_name`.
3. Gripper action servers for `open_action_name` and `close_action_name`.

## Failure Cases

The pipeline returns failure when any of these stages fails:

- AnyGrasp service is unavailable or returns no pose.
- The arm-control action server is unavailable.
- The arm-control goal is rejected or aborts.
- The close-gripper action server is unavailable or the close action fails.
- The optional post-grasp move fails.

## Troubleshooting

- AnyGrasp call fails or returns no pose: check the configured service name with `ros2 service list` and inspect the AnyGrasp node logs.
- Arm-control action not available: check the configured action name with `ros2 action list`.
- Gripper action not available: verify that the gripper driver is running and that `/close_gripper` exists.