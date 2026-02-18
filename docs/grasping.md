# ur_grasping

`ur_grasping` is the ROS 2 Python package that contains the “pipeline controller” node which coordinates:

1. AnyGrasp grasp pose estimation
2. MoveIt motion planning + execution to the grasp pose
3. Gripper close
4. Optional post-grasp motion

The goal is a single entry point that controls the full grasp process.

## Modes (`server_mode`)

The node supports two operating modes:

- **Server mode** (`server_mode: true`, default): exposes the Trigger service and waits for external requests.
- **One-shot mode** (`server_mode: false`): runs the pipeline once at startup and exits (useful for scripting).


### Triggering the pipeline

When `server_mode: true`, the node exposes a Trigger service:

- Service: `/ur_grasping_node/run_grasp`
- Type: `std_srvs/srv/Trigger`

Example:

```bash
ros2 service call /ur_grasping_node/run_grasp std_srvs/srv/Trigger {}
```

In one-shot mode (`server_mode: false`), you run the node and it will execute immediately:

```bash
ros2 run ur_grasping ur_grasping_node --ros-args -p server_mode:=false
```

## Pipeline (what happens)

### 1) Request a grasp pose from AnyGrasp

The node calls `anygrasp_msgs/srv/GetGrasps` with `count = 1`.

In this repo, AnyGrasp nodes expose services:

- `detection` (single-shot detection)
- `tracking` (tracking-based output)

Important detail: the service response contains a list of `geometry_msgs/Pose` with **no header**. This node assigns a frame using the `anygrasp_frame` parameter.

### 2) Transform to the planning frame

The pipeline transforms the pose from `anygrasp_frame` into `planning_frame` using TF.

This requires a connected TF tree that relates the robot base/planning frame to the camera frame. The `ur_launch/grip.launch.py` file publishes typical wrist-mounted static transforms (end-effector → camera and end-effector → gripper).

### 3) Move to the grasp pose via MoveIt

The node uses a MoveIt action client:

- Action type: `moveit_msgs/action/MoveGroup`
- Action name: configured by `move_group_action_name`

The goal is expressed as pose constraints for the configured `end_effector_link`.

### 4) Close the gripper

The node uses the common gripper actions from `gripper_msgs`:

- `/open_gripper` (`gripper_msgs/action/OpenGripper`)
- `/close_gripper` (`gripper_msgs/action/CloseGripper`)

It currently uses **position-based close** (i.e. `use_torque_mode=false`).

Two helper functions exist:

- `close_gripper_position()` (used by default)
- `close_gripper_torque(torque)` (available, but not used by default)

The numeric meaning of `torque` is driver-specific.

### 5) Post-grasp move

If `do_post_grasp_move` is true, the node moves to `post_grasp_pose` (a pose in `post_grasp_frame`).

## Parameters (most important)

### AnyGrasp

- `anygrasp_service` (string, default `detection`): service name to call
- `anygrasp_frame` (string, default `camera_color_optical_frame`): frame the returned pose is assumed to be in

### TF / planning frames

- `planning_frame` (string, default `base_link`)
- `end_effector_link` (string, default `tool0`)
- `planning_group` (string, default `manipulator`)

### MoveIt execution

- `move_group_action_name` (string, default `move_action`)
- `allowed_planning_time` (float)
- `num_planning_attempts` (int)
- `max_velocity_scaling` (float)
- `max_acceleration_scaling` (float)
- `position_tolerance_m` (float)
- `orientation_tolerance_rad` (float)
- `planning_pipeline_id` (string, optional)
- `planner_id` (string, optional)

### Gripper action names

- `open_action_name` (string, default `/open_gripper`)
- `close_action_name` (string, default `/close_gripper`)

### Post-grasp

- `do_post_grasp_move` (bool, default true)
- `post_grasp_frame` (string, default `base_link`)
- `post_grasp_pose` (list[7], default identity at origin): `[x,y,z,qx,qy,qz,qw]`

## Required external components

For a full run, you must have:

1. AnyGrasp node running (providing `detection` or `tracking` service)
2. A TF tree connecting `planning_frame` and `anygrasp_frame`
3. MoveIt MoveGroup action server running (`move_group_action_name`)
4. A gripper action server providing `/close_gripper` (Dynamixel or Feetech)

## Troubleshooting

- **AnyGrasp call fails / returns no pose**: check the AnyGrasp node logs and confirm the service name (`ros2 service list`).
- **TF lookup fails**: verify that `planning_frame` and `anygrasp_frame` exist and are connected (`tf2_echo`).
- **MoveIt action not available**: list actions (`ros2 action list`) and set `move_group_action_name` accordingly.
- **Gripper action not available**: start one of the gripper drivers and verify `/close_gripper` exists.

## Related docs

- `docs/launch.md` (what `ur_launch/grip.launch.py` starts)
- `../grippers/docs/action_interface.md` (gripper action fields + CLI examples)
