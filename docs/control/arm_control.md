# Arm Control

This document covers motion execution in the `grasping_control` package.

For calibration of the workspace file consumed by this node, see [creation.md](../workspace/creation.md).

## Features

`motion_execution_node` owns all robot-motion details after a client submits a `MoveToPose` goal.

Its major features are:

- Transforming the incoming pose into the configured planning frame
- Validating that the target lies inside the calibrated workspace area, when configured
- Loading collision objects from `workspace.yaml` at startup
- Applying those objects to MoveIt through `ApplyPlanningScene`
- Publishing the calibrated workspace area as an RViz marker
- Building MoveIt position and orientation constraints
- Submitting the final motion request to `moveit_msgs/action/MoveGroup`

This keeps MoveIt, TF, and workspace handling centralized in one server.

## Goal Execution Flow

For each `MoveToPose` goal, the node performs the following sequence:

1. Publish feedback state `transforming_target_pose`.
2. Transform the requested pose into `planning_frame`.
3. Publish feedback state `validating_workspace_area`.
4. Reject the goal if the target is outside the calibrated workspace area.
5. Publish feedback state `planning_and_executing`.
6. Build a `MotionPlanRequest` and send it to MoveIt.

If the goal succeeds, the action returns `success=true`. If it fails, the action aborts with a status message describing the cause.

## Workspace Loading

At startup, the node resolves a workspace configuration path and loads the YAML document written by `workspace_creation`.

In a colcon workspace, it prefers a root-level `workspace.yaml` when that file exists. Otherwise it falls back to the package config path, unless `workspace_config_path` explicitly points somewhere else.

From that file it reads:

- `objects`, which are converted into MoveIt collision objects
- `workspace_area`, which is used as an acceptance filter for incoming goals
- `base_frame`, which is used as the workspace-area reference frame when needed

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

The node converts a target TCP pose into MoveIt constraints.

- Position is represented as a spherical tolerance region around the requested pose.
- Orientation is normalized before building the orientation constraint.
- The request uses the configured planning group, planner, pipeline, planning time, and scaling factors.

The node sends the request to the configured `MoveGroup` action and reports any non-success MoveIt error code back to the caller.

## Parameters

### Action and Frames

- `action_name`: action server name, default `move_arm_to_pose`
- `move_group_action_name`: MoveIt action name, default `move_action`
- `planning_group`: MoveIt group, default `manipulator`
- `planning_frame`: planning frame, default `base_link`
- `end_effector_link`: constrained link, default `tool0`

### Planning Tuning

- `allowed_planning_time`: default `5.0`
- `num_planning_attempts`: default `5`
- `max_velocity_scaling`: default `0.2`
- `max_acceleration_scaling`: default `0.2`
- `position_tolerance_m`: default `0.005`
- `orientation_tolerance_rad`: default `0.1`
- `planning_pipeline_id`: optional planner pipeline override
- `planner_id`: optional planner override

### Workspace Integration

- `workspace_config_path`: optional override for the workspace YAML path
- `apply_planning_scene_service`: default `/apply_planning_scene`
- `workspace_area_marker_topic`: default `/workspace_area_marker`

## Startup Behavior

On startup the node:

1. resolves the workspace YAML path
2. loads workspace objects and optional workspace area
3. publishes the workspace marker state
4. applies collision objects to the planning scene if `ApplyPlanningScene` is available
5. starts the `MoveToPose` action server

If `ApplyPlanningScene` is unavailable, the node logs a warning and continues running without loading the planning scene.

## Failure Cases

Common failure sources are:

- incoming pose cannot be transformed into `planning_frame`
- workspace area is configured but invalid
- target pose lies outside the calibrated workspace area
- `MoveGroup` action server is unavailable
- MoveIt rejects or fails the motion request
