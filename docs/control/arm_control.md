# Arm Control

This document covers motion execution in the `grasping_control` package.

For calibration of the workspace file consumed by this node, see [creation.md](../workspace/creation.md).

## Features

`motion_execution_node` owns all robot-motion details after a client submits a grasp-pose or named-pose action goal.

Its major features are:

- Transforming the incoming pose into the configured planning frame
- Validating that the target lies inside the calibrated workspace area, when configured
- Seeding MoveIt's IK with the current arm joint state and preferring a nearby joint-space solution
- Loading collision objects from workspace ROS parameters at startup
- Applying those objects to MoveIt through `ApplyPlanningScene`
- Loading named motion poses from ROS parameters provided by `motion_config.yaml`
- Publishing the calibrated workspace area as an RViz marker
- Building MoveIt joint-goal or pose-goal constraints depending on the nearby-IK result
- Submitting the final motion request to `moveit_msgs/action/MoveGroup`

This keeps MoveIt, TF, and workspace handling centralized in one server.

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

For each `MoveToNamedPose` goal, the node looks up `pose_name` in the `poses_names` ROS parameter, reads the matching pose parameter from `poses_values.<name>`, converts the configured `[x, y, z, roll, pitch, yaw]` pose into a `PoseStamped`, and then sends it directly to MoveIt without applying the workspace-area filter.

The `workspace_center` name is a manually configured pose. Other named poses, including `pre_grasp` and `post_grasp`, use their configured position and orientation directly.

To move the arm to the configured workspace-center pose from the ROS 2 CLI:

```bash
source install/setup.bash
ros2 action send_goal /move_arm_to_named_pose grasping_msgs/action/MoveToNamedPose "{pose_name: workspace_center}"
```

To move the arm to the configured pre-grasp pose from the ROS 2 CLI:

```bash
source install/setup.bash
ros2 action send_goal /move_arm_to_named_pose grasping_msgs/action/MoveToNamedPose "{pose_name: pre_grasp}"
```

To move the arm to the configured post-grasp pose:

```bash
source install/setup.bash
ros2 action send_goal /move_arm_to_named_pose grasping_msgs/action/MoveToNamedPose "{pose_name: post_grasp}"
```

## Reading the Current Pose of a Link/Joint and making it a named pose

Use the following command to read the current pose of a link/joint.

```bash
source install/setup.bash
ros2 run grasping_control read_pose --ros-args -p from:=base_link -p to:=camera_link
```

To read current joint positions instead, use:

```bash
source install/setup.bash
ros2 run grasping_control read_pose --ros-args -p mode:=joint
```

Then update the motion_config.yaml file with the new pose and restart the motion_execution_node to use it as a named pose.

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

## Workspace Loading

At startup, the node reads workspace configuration from ROS parameters. The robot launch files load the selected workspace YAML, such as `crlab_table.yaml`, as a ROS parameter file.

From the workspace configuration it reads:

- `workspace_objects` and `workspace_object`, which are converted into MoveIt collision objects
- optional `workspace_object.<name>.allowed_collision_links`, which allows configured object-link collision pairs in MoveIt's allowed collision matrix
- `workspace_area`, which is used as an acceptance filter for incoming goals
- `base_frame`, which is used as the workspace-area reference frame when needed

The robot launch files load `motion_config.yaml` as a ROS parameter file for `motion_execution_node`. That file contains:

- `poses_names`, which controls which pose names the named-pose action accepts
- `poses_values.<name>`, which stores each named pose as `[x, y, z, roll, pitch, yaw]` plus its `target_frame`

Workspace objects may allow collision with specific robot links when a fixed obstacle touches robot mounting hardware. For example:

```yaml
workspace_object:
  table:
    allowed_collision_links: [ur10_base_link]
```

This still keeps `table` as a collision object for every other robot link.

Unsupported geometry types are skipped with a warning. Supported runtime collision geometry types are:

- `box`
- `cylinder`

## Workspace-Area Filtering

If `workspace_area` is not configured, the node accepts targets anywhere in the planning frame.

If `workspace_area` is configured, the node:

- checks the transformed target position against the saved four-corner polygon
- aborts the goal with `Target pose lies outside the calibrated workspace area.` when the pose is outside
- treats the check as planar, using the XY polygon only

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

- The latest `planning_joint_names` state is read from `joint_state_topic` and used as the IK seed.
- `compute_ik_service` is called for the configured `planning_group` and `end_effector_link` or named-pose target frame.
- Returned joint angles are shifted by whole turns so each revolute joint stays as close as possible to the current arm configuration.
- When nearby IK succeeds, the final `MotionPlanRequest` uses `JointConstraint`s instead of TCP pose constraints.
- When nearby IK fails and fallback is enabled, the node logs the IK reason and falls back to a pose-constrained request.

- During pose-constrained fallback, position is represented as a spherical tolerance region around the requested pose.
- During pose-constrained fallback, orientation is normalized before building the orientation constraint.
- The request uses the configured planning group, planner, pipeline, planning time, and scaling factors.

The node sends the request to the configured `MoveGroup` action and reports any non-success MoveIt error code back to the caller.

## Parameters

### Action and Frames

- `action_name`: action server name, default `move_arm_to_pose`
- `named_pose_action_name`: named-pose action server name, default `move_arm_to_named_pose`
- `move_group_action_name`: MoveIt action name, default `move_action`
- `planning_group`: MoveIt group, default `manipulator`
- `planning_frame`: planning frame, default `base_link`
- `end_effector_link`: constrained link, default `tool0` in the node and `tcp` in soft-gripper launch files

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
2. reads workspace objects and optional workspace area from ROS parameters
3. publishes the workspace marker state
4. applies collision objects to the planning scene if `ApplyPlanningScene` is available, appending configured workspace object-link allowances to the existing MoveIt allowed-collision matrix
5. starts the `MoveToPose` and `MoveToNamedPose` action servers

If `ApplyPlanningScene` is unavailable, the node logs a warning and continues running without loading the planning scene.

## Failure Cases

Common failure sources are:

- incoming pose cannot be transformed into `planning_frame`
- named pose is not listed in `motion_config.yaml`
- workspace area is configured but invalid
- target pose lies outside the calibrated workspace area
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
