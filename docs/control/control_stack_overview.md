# Control Stack Overview

This document covers the shared architecture and bringup flow for the grasping control stack.

Use the topic-specific documents for implementation details:

- [grasping_pipeline.md](./grasping_pipeline.md) for `grasping_node`
- [arm_control.md](./arm_control.md) for `arm_control_node`
- [workspace_creation.md](./workspace_creation.md) for `workspace_calibration`

## Architecture

The motion stack is split into two ROS 2 nodes.

- `grasping_node` orchestrates the grasp pipeline.
- `arm_control_node` executes robot motion through MoveIt.

At runtime the stack usually depends on four subsystems:

1. AnyGrasp for grasp generation.
2. The control stack in this package.
3. MoveIt and the robot driver.
4. A gripper action server.

## Shared Motion Interface

The shared action interface lives in the `grasping_msgs` package.

Action:

- `grasping_msgs/action/MoveToPose`
- Goal: `geometry_msgs/PoseStamped target_pose`
- Result: `bool success`, `string message`
- Feedback: `string state`

The goal stays intentionally minimal. Clients send only a target pose, and `arm_control_node` resolves transforms, workspace configuration, and MoveIt planning from its own parameters.

## Bringup Launch File

`grasping/launch/grip.launch.py` starts the shared bringup.

It launches:

- a static TF publisher from end effector to gripper
- a static TF publisher from end effector to camera
- `arm_control_node`
- `grasping_node`

The launch file passes planning-related parameters only to `arm_control_node`. It passes pipeline parameters and the arm action name to `grasping_node`.

## Static TF Assumptions

The launch file publishes static transforms because AnyGrasp returns `PoseStamped` results in the source point-cloud frame, and the arm-control node later transforms those poses into its planning frame.

Use the static camera transform only when the camera is rigidly mounted to the end effector. If the camera is mounted elsewhere, provide the correct TF in your robot setup instead of relying on the static transform in `grip.launch.py`.

## Launch Arguments

### Shared Pipeline Arguments

- `server_mode`
- `anygrasp_service`
- `arm_action_name`
- `open_action_name`
- `close_action_name`
- `do_post_grasp_move`
- `post_grasp_frame`
- `post_grasp_x`
- `post_grasp_y`
- `post_grasp_z`
- `post_grasp_qx`
- `post_grasp_qy`
- `post_grasp_qz`
- `post_grasp_qw`

### Arm-Control Arguments

- `move_group_action_name`
- `planning_group`
- `planning_frame`
- `end_effector_link`
- `allowed_planning_time`
- `num_planning_attempts`
- `max_velocity_scaling`
- `max_acceleration_scaling`
- `position_tolerance_m`
- `orientation_tolerance_rad`
- `planning_pipeline_id`
- `planner_id`
- `workspace_config_path`

### Frame Arguments

- `ee_frame`
- `gripper_frame`
- `camera_frame`

### Static TF Arguments

For end effector to gripper:

- `ee_to_gripper_x`
- `ee_to_gripper_y`
- `ee_to_gripper_z`
- `ee_to_gripper_qx`
- `ee_to_gripper_qy`
- `ee_to_gripper_qz`
- `ee_to_gripper_qw`

For end effector to camera:

- `ee_to_camera_x`
- `ee_to_camera_y`
- `ee_to_camera_z`
- `ee_to_camera_qx`
- `ee_to_camera_qy`
- `ee_to_camera_qz`
- `ee_to_camera_qw`

## Typical Run

1. Start MoveIt and the robot driver.
2. Start the gripper action server.
3. Start AnyGrasp.
4. Calibrate or update `workspace.yaml` with `workspace_calibration` if needed.
5. Launch `grip.launch.py`.
6. Trigger `/grasping_node/run_grasp`, or run the launch file with `server_mode:=false`.

Example bringup:

```bash
ros2 launch grasping grip.launch.py
```

Example one-shot run:

```bash
ros2 launch grasping grip.launch.py server_mode:=false
```

## Runtime Requirements

For the full stack to work, you need:

1. AnyGrasp running and exposing the configured service.
2. `arm_control_node` running and exposing the configured `MoveToPose` action.
3. A TF tree connecting the planning frame and the frame attached to the returned grasp pose.
4. MoveIt running with the configured `MoveGroup` action.
5. A gripper action server exposing the configured open and close actions.

## TF Checks

List the TF tree:

```bash
ros2 run tf2_tools view_frames
```

Inspect a transform:

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

## Cross-Component Troubleshooting

- AnyGrasp service unavailable: confirm the configured service name with `ros2 service list`.
- MoveToPose action unavailable: confirm the configured action name with `ros2 action list`.
- TF lookup failures: confirm that the planning frame and grasp-pose frame are connected.
- MoveIt planning failures: verify that MoveIt is running and the workspace configuration matches the real scene.