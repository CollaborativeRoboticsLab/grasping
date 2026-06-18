# Workspace Creation

This document covers the interactive calibration flow that starts from `grasping_control/config/workspace_empty.yaml` and saves named workspace YAML files on demand.

For how the runtime system consumes that file, see [arm_control.md](./arm_control.md).

## Purpose

The calibration workflow stores robot-referenced workspace data for two different runtime uses.

- `objects`: collision geometry that `motion_execution_node` loads into the MoveIt planning scene
- `workspace_area`: an optional square work zone used to reject out-of-bounds target poses before planning

The calibration node records manual samples around the robot, preserves the raw capture data, and derives simple planning primitives from those samples.

## Inputs and Dependencies

The node depends on:

- Manipulator's `joint_state_topic`, default `/joint_states`
- TF between `base_frame` and `tool_frame`
- `shape_definitions.yaml` for the supported object-capture layouts

During capture, each saved sample includes:

- a point label
- capture timestamp
- tool pose in the base frame
- a snapshot of the latest joint state

## Supported Calibration Targets

The current flow supports:

- a square workspace area derived from four captured corner points
- rectangular prisms derived from four top-face corners
- cylinders derived from one center point and four rim points

Shape requirements are defined in `grasping_control/config/shape_definitions.yaml`, so new capture patterns can be added without rewriting the interactive loop.

## Interactive Flow

When the node starts, it loads `workspace_empty.yaml` by default, unless `workspace_config_path` explicitly points somewhere else, and then enters a CLI session.

The top-level menu lets the operator:

- review the currently saved workspace area
- review existing objects
- add a new object
- update an existing object
- save the in-memory calibration to a named YAML file with the `s` option
- calibrate the workspace area with the `w` option
- quit without saving

The node keeps calibration changes in memory until you explicitly choose `save`.

## Workspace YAML Structure

Top-level fields written by the calibration flow:

- `version`
- `updated_at`
- `base_frame`
- `tool_frame`
- `ground_plane_z`
- `workspace_area`
- `objects`

### `workspace_area`

The `workspace_area` field is either `null` or a dictionary containing:

- `type`: currently `workspace_area`
- `created_at`
- `updated_at`
- `base_frame`
- `tool_frame`
- `ground_plane_z`
- `capture_samples`
- `geometry`

Its derived `geometry` block contains:

- `type`: `square`
- `dimensions.side_length`: average side length from the four captured edges
- `dimensions.height_from_ground`: average captured Z minus `ground_plane_z`
- `pose.position`: average center of the four corners
- `corner_points`: the four saved corner points in capture order

### `objects`

Each object entry contains:

- `name`
- `shape`
- `created_at`
- `updated_at`
- `base_frame`
- `tool_frame`
- `ground_plane_z`
- `capture_samples`
- `geometry`

The derived `geometry` block is what runtime planning uses.

Supported derived runtime geometry types are:

- `box`
- `cylinder`

## Workspace-Area Capture Convention

Use the `w` option in the calibration menu to record the robot working area.

1. Move the tool to the first workspace corner and press Enter.
2. Continue around the boundary in order for corners 2 through 4.
3. The node stores the raw samples and derives a square-like area geometry.
4. `motion_execution_node` later uses those saved corners for its planar inside/outside test.

The capture order matters. The runtime area filter assumes the four corners are recorded in order around the boundary.

You may start from any corner, but after that you must keep moving around the perimeter in a single direction.

- Clockwise is valid.
- Anticlockwise is also valid.
- Zigzagging across the square is invalid.

Example valid sequence:

- corner_1 = near-left
- corner_2 = far-left
- corner_3 = far-right
- corner_4 = near-right

Example invalid sequence:

- corner_1 = near-left
- corner_2 = far-right
- corner_3 = far-left
- corner_4 = near-right

There is no special requirement such as "top-left first". The important rule is that consecutive captured points must be neighboring corners on the workspace boundary.

The workspace area is not added to the MoveIt planning scene as a collision object.

## Rectangle Conventions

For `top_surface_rectangle`:

1. Move the tool to each top-face corner in order around the object.
2. Capture all four points.
3. The node estimates the center, in-plane rotation, and the two planar dimensions from the sampled edges.
4. The object height is computed as `top_z - ground_plane_z`.

The saved geometry becomes a box whose top face matches the captured rectangle.

For `side_face_rectangle`:

1. Move the tool to each side-face corner in order around the visible face.
2. Capture all four points.
3. Enter the obstacle depth measured inward from that captured face.
4. The node extrudes the captured face by the entered depth to build the box.

For `bottom_face_rectangle`:

1. Move the tool to each bottom-face corner in order around the hanging obstacle.
2. Capture all four points.
3. Enter the obstacle height above that captured bottom face.
4. The node extends the box upward from the captured face by the entered height.

## Cylinder Convention

For a cylinder:

1. Capture the top-face center.
2. Capture four rim points around the top face.
3. The node averages the XY distance from the center to the rim points to estimate radius.
4. The object height is computed as `top_z - ground_plane_z`.

The saved geometry becomes a cylinder aligned with the base frame.

## Parameters

- `joint_state_topic`
- `base_frame`
- `tool_frame`
- `ground_plane_z`
- `workspace_config_path`
- `shape_definitions_path`

If you override `workspace_config_path`, point `motion_execution_node` at the same file so both nodes use the same calibrated scene.

If you do not override `workspace_config_path`, `workspace_creation` starts from `workspace_empty.yaml`. When you choose `save`, it asks for a file name and writes that YAML into the colcon workspace root.



## Usage Commands

- Start the ur10 arm with gripper in servo mode. As an example,

```bash
source install/setup.bash
ros2 launch ur10_soft_two_fingers_moveit_config hardware_with_moveit.launch.py launch_servo:=true
```

- Start workspace calibration:

```bash
source install/setup.bash
ros2 run grasping_control workspace_creation
```

- Move the arm with following servo controller to relavent point in space

```bash
source install/setup.bash
ros2 run grasping_control servo_teleop
```

Follow the onscreen guidelines to add/edit workspace and objects.


### Typical Session

1. Run `workspace_creation`.
2. Press `w` to calibrate the workspace area if needed.
3. Capture the four workspace corners in order.
4. Add or update collision objects as needed.
5. Press `s` and enter a file name to save the updated `workspace_area` and `objects` into the workspace root.
