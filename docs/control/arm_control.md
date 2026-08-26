# Arm Control

This document covers motion execution in the `grasping_control` package.

For calibration of the workspace file consumed by this node, see [creation.md](../workspace/creation.md).
For runtime scene ownership, activation, and active-scene APIs, see [scene_manager.md](../workspace/scene_manager.md).

## Runtime Split

The grasping runtime is now split into three responsibilities:

- `scene_manager_node` owns workspace document resolution, MoveIt planning-scene updates, allowed-collision updates, and publication of the singleton active scene
- `motion_execution_node` owns arm execution actions and consumes active workspace-area state from the scene manager
- `feasibility_service_node` owns arm-only feasibility checks and consumes the same active workspace-area state from the scene manager

This means scene activation is explicit runtime state, separate from both feasibility queries and arm execution requests.

## Features

`motion_execution_node` owns all robot-motion details after a client submits a grasp-pose, named-pose, or joint-pose action goal.

Its major features are:

- Transforming the incoming pose into the configured planning frame
- Validating that the target lies inside the calibrated workspace area, when configured
- Seeding MoveIt's IK with the current arm joint state and preferring a nearby joint-space solution
- Querying `GetActiveScene` at startup and staying synchronized with `/active_scene`
- Loading named motion poses from ROS parameters provided by `motion_config.yaml`
- Publishing the calibrated workspace area as an RViz marker
- Building MoveIt joint-goal or pose-goal constraints depending on the nearby-IK result
- Submitting the final motion request to `moveit_msgs/action/MoveGroup`

This keeps arm execution focused on motion while scene ownership stays centralized in `scene_manager_node`.

## Grasping Interfaces

The grasping control layer now exposes both execution actions and feasibility services.

Execution actions:

- `grasping_msgs/action/MoveToPose`
- `grasping_msgs/action/MoveToNamedPose`
- `grasping_msgs/action/MoveToJointPose`

Scene-management services and actions:

- `grasping_msgs/srv/GetActiveScene`
- `grasping_msgs/srv/ValidateWorkspaceDocument`
- `grasping_msgs/action/ActivateScene`
- `grasping_msgs/action/LoadSceneFromContent`

Arm-only feasibility services:

- `grasping_msgs/srv/CheckCartesianPoseFeasibility`
- `grasping_msgs/srv/CheckJointPoseFeasibility`

## Interfaces

`motion_execution_node` exposes three action interfaces:

- `grasping_msgs/action/MoveToPose` for arbitrary target poses
- `grasping_msgs/action/MoveToNamedPose` for configured named poses
- `grasping_msgs/action/MoveToJointPose` for explicit joint-space targets executed against the grasping-owned planning scene

`feasibility_service_node` exposes two service interfaces:

- `CheckCartesianPoseFeasibility`, with modes `arm_only_ik` and `arm_only_plan`
- `CheckJointPoseFeasibility`, with modes `state_validity` and `plan`

Both services return structured fields including `feasible`, `failure_reason`, `suggested_fallback`, `message`, and request-specific resolved outputs when available.

## Feasibility vs Execution vs Scene Activation

These three operations are intentionally separate:

- scene activation changes the grasping-side planning scene and active workspace-area state
- feasibility checks answer whether the current active scene admits an arm-only solution, but they do not execute motion or mutate the scene
- execution actions plan and optionally execute against the current active scene, but they do not implicitly switch scenes

That separation matters for higher-level planners. A task runner should activate the correct scene first, then call feasibility, then call execution only after it accepts the chosen plan or fallback.

## IK Success Is Not Planning Success

The grasping stack now exposes this distinction explicitly.

- `arm_only_ik` means: can MoveIt produce a collision-aware IK solution for the target pose in the current scene?
- `arm_only_plan` means: can MoveIt produce a planning-only motion request to that target in the current scene?
- `state_validity` means: is a supplied joint state collision-free and kinematically valid in the current scene?
- `plan` for joint feasibility means: can MoveIt produce a planning-only joint-space path to that joint state?

An IK success does not guarantee planning success. The target may have a valid end state while still failing trajectory planning because of path collisions, constraints, joint limits along the path, or planner failure. That is why the mobile-manipulator layer first asks the grasping layer for a feasibility mode that matches the policy decision it needs.

## Cartesian Feasibility Flow

For each `CheckCartesianPoseFeasibility` request, the node performs the following sequence:

1. Validate that `frame_id` is present.
2. Transform the request pose into `planning_frame`.
3. Reject the request with `failure_reason=workspace_area_violation` if the transformed pose lies outside the active workspace area.
4. Run nearby IK seeded from the current planning-joint state.
5. If mode is `arm_only_ik`, succeed only when IK succeeds.
6. If mode is `arm_only_plan`, prefer a joint-goal plan from the IK result and fall back to pose-constrained planning when configured.
7. When arm-only planning fails but a base move could help, return `suggested_fallback=move_base_then_arm`.

## Joint Feasibility Flow

For each `CheckJointPoseFeasibility` request, the node performs the following sequence:

1. Validate that at least one joint name is supplied.
2. Validate that `joint_names` and `joint_positions` have matching lengths.
3. For `state_validity`, call MoveIt state validity against the current active planning scene.
4. For `plan`, build a planning-only joint-goal request from the current state to the requested state.
5. Return structured `failure_reason` values for invalid request shape, missing joints, collision, constraint violation, or planner failure.

## Grasp-Pose Flow

For each `MoveToPose` goal, the node performs the following sequence:

1. Publish feedback state `transforming_target_pose`.
2. Reject the request if `target_pose.header.frame_id` is empty.
3. Transform the requested pose into `planning_frame`.
4. Publish feedback state `validating_workspace_area`.
5. Reject the goal if the target is outside the calibrated workspace area.
6. Publish feedback state `planning_and_executing`.
7. Read the latest configured planning-joint state from `joint_state_topic`.
8. Call `compute_ik_service` with the current arm state as the IK seed.
9. If IK succeeds, unwrap the returned joint angles toward the current branch and send a joint-space `MotionPlanRequest`.
10. If IK fails and fallback is enabled, log the IK reason and fall back to the original pose-constrained `MotionPlanRequest`.

If the goal succeeds, the action returns `success=true`. If it fails, the action aborts with a status message describing the cause.

## Named-Pose Flow

For each `MoveToNamedPose` goal, the node:

1. Looks up `pose_name` in the `poses_names` ROS parameter.
2. Reads the matching pose data from `poses_values.<name>`.
3. Converts the configured `[x, y, z, roll, pitch, yaw]` values into a `PoseStamped`.
4. Sends the target directly to MoveIt without applying the workspace-area filter.

The `workspace_center` name is a manually configured pose. Other named poses, including `pre_grasp` and `post_grasp`, use their configured position and orientation directly.

To move the arm to any configured named pose from the ROS 2 CLI:

```bash
source install/setup.bash
ros2 action send_goal /move_arm_to_named_pose grasping_msgs/action/MoveToNamedPose "{pose_name: workspace_center}"
```

Replace `workspace_center` with any configured entry from `poses_names`, for example `pre_grasp` or `post_grasp`.

## Joint-Pose Flow

For each `MoveToJointPose` goal, the node:

1. Validates that at least one joint name is supplied.
2. Validates that `target_joint_state.name` and `target_joint_state.position` have matching lengths.
3. Reuses the planning configuration already loaded by `motion_config.yaml`.
4. Builds a joint-constrained `MoveGroup` request against the same planning scene used by the other grasping actions.
5. Sends the request to MoveIt and returns the final status message.

To move the arm to a joint-space target from the ROS 2 CLI:

```bash
source install/setup.bash
ros2 action send_goal /move_arm_to_joint_pose grasping_msgs/action/MoveToJointPose "{target_joint_state: {name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], position: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]}}"
```

## Reading the Current Pose of a Link/Joint and making it a named pose

Use the following command to read the current pose of a link/joint.

```bash
source install/setup.bash
ros2 run grasping_teleop read_pose_node --ros-args -p from:=base_link -p to:=camera_link
```

To read current joint positions instead, use:

```bash
source install/setup.bash
ros2 run grasping_teleop read_pose_node --ros-args -p mode:=joint
```

Then update `motion_config.yaml` and restart `motion_execution_node`.

```YAML
poses_names: ["workspace_center", "pre_grasp", "post_grasp", "<new_named_pose>"]
poses_values:
  workspace_center:
    pose: [0.0, 0.0, 0.30, 0.0, 0.0, 0.0]
    target_frame: camera_link
  pre_grasp:
    pose: [0.0, 0.0, 0.30, 0.0, 0.0, 0.0]
    target_frame: tcp
  post_grasp:
    pose: [0.0, 0.0, 0.30, 0.0, 0.0, 0.0]
    target_frame: tcp
  <new_named_pose>:
    pose: [0.0, 0.0, 0.30, 0.0, 0.0, 0.0]
    target_frame: tcp
```

## Motion Configuration

The robot launch files load `motion_config.yaml` as a ROS parameter file for `motion_execution_node`.

That file contains:

- `poses_names`, which controls which pose names the named-pose action accepts
- `poses_values.<name>`, which stores each named pose as `[x, y, z, roll, pitch, yaw]` plus its `target_frame`
- planning settings such as `planning_group`, tolerances, planner selection, IK settings, and joint-goal behavior used by both pose and joint-space requests

## Workspace Integration

`motion_execution_node` and `feasibility_service_node` no longer own workspace document parsing or planning-scene application as their primary runtime behavior.

Instead they consume active-scene state from `scene_manager_node`:

- on startup, each node queries `GetActiveScene`
- during runtime, each node subscribes to `/active_scene`
- both nodes rebuild their local workspace-area filter cache from the published active scene metadata

The planning scene itself is updated by `scene_manager_node` through `ActivateScene` or `LoadSceneFromContent`.

## Workspace-Area Filtering

If `workspace_area` is not configured, the node accepts targets anywhere in the planning frame.

If `workspace_area` is configured, the node:

- checks the transformed target position against the saved four-corner polygon
- aborts `MoveToPose` goals with `Target pose lies outside the calibrated workspace area.` when the pose is outside
- treats the check as planar, using the XY polygon only

Named poses bypass this filter.

The current filter does not enforce a Z band.

## RViz Marker

When a valid workspace area is present, the node publishes it as a semi-transparent green marker on `workspace_area_marker_topic`.

Marker details:

- frame: workspace base frame from the workspace YAML
- type: triangle-list plane built from the four saved corner points
- color: green with partial transparency

If no workspace area exists, the node publishes a delete marker so stale visuals are cleared.

## MoveIt Planning Behavior

The node first tries to convert a target TCP pose into a nearby joint-space goal.

### Nearby IK Path

- The latest `planning_joint_names` state is read from `joint_state_topic` and used as the IK seed.
- `compute_ik_service` is called for the configured `planning_group` and `end_effector_link` or named-pose target frame.
- Returned joint angles are shifted by whole turns so each revolute joint stays as close as possible to the current arm configuration.
- When nearby IK succeeds, the final `MotionPlanRequest` uses `JointConstraint`s instead of TCP pose constraints.
- When nearby IK fails and fallback is enabled, the node logs the IK reason and falls back to a pose-constrained request.

### Pose-Constrained Fallback

- During pose-constrained fallback, position is represented as a spherical tolerance region around the requested pose.
- During pose-constrained fallback, orientation is normalized before building the orientation constraint.
- The request uses the configured planning group, planner, pipeline, planning time, and scaling factors.

The node sends the request to the configured `MoveGroup` action and reports any non-success MoveIt error code back to the caller.

## Parameters

### Action and Frames

- `action_name`: action server name, default `move_arm_to_pose`
- `named_pose_action_name`: named-pose action server name, default `move_arm_to_named_pose`
- `joint_pose_action_name`: joint-pose action server name, default `move_arm_to_joint_pose`
- `move_group_action_name`: MoveIt action name, default `move_action`
- `planning_group`: MoveIt group, default `manipulator`
- `planning_frame`: planning frame, default `base_link`
- `end_effector_link`: constrained link, default `tool0` in the node and `tcp` in soft-gripper launch files

### Named Poses

- `poses_names`: accepted named-pose identifiers
- `poses_values.<name>.pose`: configured pose as `[x, y, z, roll, pitch, yaw]`
- `poses_values.<name>.target_frame`: link expected to reach the configured pose

### Planning Tuning

- `allowed_planning_time`: default `5.0`
- `num_planning_attempts`: default `5`
- `max_velocity_scaling`: default `0.2`
- `max_acceleration_scaling`: default `0.2`
- `position_tolerance_m`: default `0.005`
- `orientation_tolerance_rad`: default `0.1`
- `planning_pipeline_id`: optional planner pipeline override
- `planner_id`: optional planner override
- `compute_ik_service`: default `/compute_ik`
- `joint_state_topic`: default `/joint_states`
- `planning_joint_names`: ordered arm joints used to seed IK and build the final joint goal
- `prefer_nearby_ik`: when true, compute a nearby IK solution before sending a MoveIt request
- `fallback_to_pose_planning_on_ik_failure`: when true, use the old pose-constrained planning path if nearby IK fails
- `joint_state_timeout_sec`: maximum age for cached planning joints before nearby IK is skipped
- `ik_timeout_sec`: timeout passed to MoveIt's IK request
- `joint_goal_tolerance_rad`: tolerance applied to each joint when a joint-goal request is built
- `log_joint_goal_deltas`: when true, log per-joint deltas between current state and the selected nearby IK goal

### Workspace Integration

- `apply_planning_scene_service`: default `/apply_planning_scene`
- `get_planning_scene_service`: default `/get_planning_scene`, used to preserve the existing MoveIt allowed-collision matrix before appending workspace object-link allowances
- `workspace_area_marker_topic`: default `/workspace_area_marker`

## Startup Behavior

On startup the node:

1. reads configured named poses from ROS parameters loaded by the launch file
2. queries `GetActiveScene` to initialize the local workspace-area cache when a scene is already active
3. subscribes to `/active_scene` for later scene updates
4. publishes the current workspace marker state
5. starts the `MoveToPose`, `MoveToNamedPose`, and `MoveToJointPose` action servers

If no active scene exists yet, the node keeps running and waits for scene activation.

## Typical Runtime Order

The intended runtime order is:

1. activate a scene with `ActivateScene` or indirectly through mobile-manipulator `ActivateSceneByName`
2. wait for the grasping scene manager to publish the new active scene
3. call `CheckCartesianPoseFeasibility` or `CheckJointPoseFeasibility` if the task needs an arm-only answer
4. call `MoveToPose`, `MoveToNamedPose`, or `MoveToJointPose` only after the correct scene is active and the task policy accepts execution

## Failure Cases

Common failure sources are:

- incoming pose cannot be transformed into `planning_frame`
- named pose is not listed in `motion_config.yaml`
- workspace area is configured but invalid
- `MoveToPose` target lies outside the calibrated workspace area
- no fresh `joint_state_topic` sample is available for `planning_joint_names`
- `compute_ik_service` is unavailable, times out, or returns a non-success MoveIt error code
- `MoveGroup` action server is unavailable
- MoveIt rejects or fails the motion request

## Runtime Notes Without Hardware

The nearby-IK path depends on live `/joint_states` and a running `/compute_ik` service from MoveIt. Without a robot or demo stack running, the new code can still be validated statically, but the runtime path will naturally fall back or abort depending on `fallback_to_pose_planning_on_ik_failure`.

For offline verification, temporarily set:

```yaml
prefer_nearby_ik: true
fallback_to_pose_planning_on_ik_failure: true
log_joint_goal_deltas: true
```

Then inspect the node logs while running against either the MoveIt demo launch or hardware bringup.
