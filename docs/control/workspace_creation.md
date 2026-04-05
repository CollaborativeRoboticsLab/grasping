# Workspace Creation

The workspace calibration flow stores robot-referenced obstacles in `grasping_arm_control/config/workspace.yaml`.

## Purpose

The calibration node records manually sampled points around the robot and converts them into simple planning primitives. Those primitives are later loaded by `arm_control_node` into the MoveIt planning scene.

The current flow supports:

- a square workspace area derived from four captured work-area corners
- rectangular prisms derived from four top-face corners
- cylinders derived from one center point and four rim points

Shape requirements are defined in `grasping_arm_control/config/shape_definitions.yaml` so new primitive types can be added without rewriting the interaction loop.

## Calibration Node

Node:

- Package: `grasping_arm_control`
- Executable: `workspace_calibration`

What it does:

- subscribes to `/joint_states`
- looks up the current tool pose from TF using `base_frame` and `tool_frame`
- lets you calibrate a square workspace area used for runtime goal filtering
- lists existing objects from `workspace.yaml`
- lets you update an existing object or add a new one
- records both sampled Cartesian poses and the matching joint-state snapshot
- derives a primitive geometry block for each object
- writes the updated YAML back into the package config

## Stored YAML Structure

Top-level fields:

- `version`
- `updated_at`
- `base_frame`
- `tool_frame`
- `ground_plane_z`
- `workspace_area`
- `objects`

The `workspace_area` field is either `null` or a dictionary containing:

- `type`: currently `workspace_area`
- `created_at`
- `updated_at`
- `base_frame`
- `tool_frame`
- `ground_plane_z`
- `capture_samples`
- `geometry`

Each object contains:

- `name`
- `shape`
- `created_at`
- `updated_at`
- `capture_samples`
- `geometry`

Each sample contains:

- point label
- capture timestamp
- tool pose in the base frame
- joint state arrays at the moment of capture

The `geometry` block is what runtime planning uses. It contains a primitive type such as `box` or `cylinder`, its dimensions, and a pose.

For `workspace_area`, the `geometry` block contains:

- `type`: `square`
- `dimensions.side_length`: average side length from the four captured edges
- `dimensions.height_from_ground`: average sampled Z minus `ground_plane_z`
- `pose.position`: average center of the four corners
- `corner_points`: the four saved corners in capture order

The capture order matters. The area filter assumes the four corners are recorded in order around the workspace boundary.

## Workspace Area Convention

Use the `w` option in the calibration menu to record the robot's working area.

1. Move the tool to the first workspace corner and press Enter.
2. Continue around the square boundary in order for corners 2 through 4.
3. The node stores the four raw samples and derives a square-like area geometry.
4. `arm_control_node` later uses those saved corners to reject target poses outside the area.

This area is not added to the MoveIt planning scene as a collision object. It is used only as an acceptance filter for incoming grasp or motion poses.

When `arm_control_node` loads a calibrated area, it also publishes that area as a semi-transparent green plane marker on `/workspace_area_marker` by default so the zone is visible in RViz.

## Rectangle Convention

For a rectangle:

1. Move the tool to each top-face corner in order around the object.
2. Press Enter to capture each corner.
3. The node estimates the center and planar dimensions from those four points.
4. Height is computed as `top_z - ground_plane_z`.

The saved primitive becomes a box aligned with the planning frame.

## Cylinder Convention

For a cylinder:

1. Capture the top-face center.
2. Capture four rim points around the top face.
3. The node averages the XY distance from the center to the rim points to estimate radius.
4. Height is computed as `top_z - ground_plane_z`.

The saved primitive becomes a cylinder aligned with the planning frame.

## Usage

Example:

```bash
ros2 run grasping_arm_control workspace_calibration
```

Useful parameters:

- `joint_state_topic`
- `base_frame`
- `tool_frame`
- `ground_plane_z`
- `workspace_config_path`
- `shape_definitions_path`

If you need a custom config location, pass it as a node parameter and keep the runtime `arm_control_node` pointed at the same file.

## Typical Session

1. Run `workspace_calibration`.
2. Press `w` to calibrate the workspace area.
3. Capture the four workspace corners in order around the boundary.
4. Add or update object obstacles as needed.
5. Quit the session to persist both `workspace_area` and `objects` back to `workspace.yaml`.
