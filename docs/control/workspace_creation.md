# Workspace Creation

The workspace calibration flow stores robot-referenced obstacles in `grasping_arm_control/config/workspace.yaml`.

## Purpose

The calibration node records manually sampled points around the robot and converts them into simple planning primitives. Those primitives are later loaded by `arm_control_node` into the MoveIt planning scene.

The current flow supports:

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
- `objects`

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
