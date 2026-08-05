# Simple Grasping

This document covers the `simple_grasping` node in the `grasping_control` package.

## Purpose

`simple_grasping` is a small sequence runner that calls two existing action servers:

- `grasping_msgs/action/MoveToNamedPose` for arm motion
- `control_msgs/action/GripperCommand` for gripper open and close commands

It is intended for simple scripted grasp routines where the robot should step through a fixed ordered sequence instead of waiting for an external planner or grasp pipeline.

## Default Sequence

The default sequence is:

```text
pre_grasp -> workspace_center -> open -> grasp_pose -> close -> post_grasp
```

Internally this is configured as:

```text
[[pose, pre_grasp], [pose, workspace_center], [gripper, open], [pose, grasp_pose], [gripper, close], [pose, post_grasp]]
```

Each step has two fields:

- `pose`, `<named_pose>`: send a `MoveToNamedPose` goal
- `gripper`, `open|close`: send a `GripperCommand` goal using the configured open or close parameters

## Running the Node

Run the node directly with:

```bash
source install/setup.bash
ros2 run grasping_control simple_grasping
```

Run it through the shared launch file with:

```bash
source install/setup.bash
ros2 launch grasping_control simple_grasping.launch.py
```

Run it through the UR wrapper launch with:

```bash
source install/setup.bash
ros2 launch grasping_ur simple_grasping.launch.py
```

## Sequence Parameter

The sequence is configured through one parameter named `sequence`.

Example:

```bash
source install/setup.bash
ros2 launch grasping_ur simple_grasping.launch.py \
	sequence:="[[pose, pre_grasp], [pose, workspace_center], [gripper, open], [pose, grasp_pose], [gripper, close], [pose, post_grasp]]"
```

You can reorder, remove, or replace steps. For example, to skip `workspace_center`:

```bash
source install/setup.bash
ros2 launch grasping_ur simple_grasping.launch.py \
	sequence:="[[pose, pre_grasp], [gripper, open], [pose, grasp_pose], [gripper, close], [pose, post_grasp]]"
```

The parser also accepts a simpler list form such as:

```text
["pose:pre_grasp", "gripper:open", "pose:grasp_pose"]
```

## Parameters

### Action Names

- `arm_action_name`: default `/move_arm_to_named_pose`
- `gripper_action_name`: default `/gripper_command`

### Sequence

- `sequence`: ordered list of `[kind, value]` pairs

### Gripper Settings

- `open_position`: default `0.09`
- `open_max_effort`: default `0.0`
- `close_position`: default `0.0`
- `close_max_effort`: default `5.0`

### Timeouts

- `server_timeout_sec`: default `10.0`
- `result_timeout_sec`: default `120.0`

## Failure Behavior

The node stops at the first failed step.

- If a named pose is rejected or fails in MoveIt, the sequence aborts.
- If the gripper command does not reach its target, the sequence aborts.
- If either action server is unavailable, the sequence exits before sending commands.

This node does not retry failed steps or branch based on runtime conditions. It is a deterministic executor for one configured ordered sequence.
