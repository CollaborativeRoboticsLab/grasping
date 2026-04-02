# Arm Control

The motion stack is split into two ROS 2 nodes:

- `grasping_node` handles pipeline orchestration: request a grasp, close the gripper, and trigger post-grasp behavior.
- `arm_control_node` owns motion execution: transform poses, load the planning scene from `workspace.yaml`, build MoveIt constraints, and send the MoveGroup action.

## Interfaces

The shared action interface now lives in the `grasping_msgs` package.

Action:

- `grasping_msgs/action/MoveToPose`
- Goal: `geometry_msgs/PoseStamped target_pose`
- Result: `bool success`, `string message`
- Feedback: `string state`

This keeps the client side simple. A caller only sends a target pose and the arm-control node resolves the rest from its own parameters and workspace configuration.

## Arm Control Node

Node:

- Package: `grasping_arm_control`
- Executable: `arm_control_node`

Responsibilities:

- Accept `MoveToPose` goals.
- Transform incoming poses into the configured planning frame.
- Load collision objects from `grasping_arm_control/config/workspace.yaml` at startup.
- Push those objects into MoveIt through `ApplyPlanningScene`.
- Convert a target TCP pose into MoveIt position and orientation constraints.
- Execute motion through the configured `moveit_msgs/action/MoveGroup` server.

Important parameters:

- `action_name`: action server name, default `move_arm_to_pose`
- `move_group_action_name`: MoveIt action name, default `move_action`
- `planning_group`: MoveIt group, default `manipulator`
- `planning_frame`: planning frame, default `base_link`
- `end_effector_link`: constrained link, default `tool0`
- `workspace_config_path`: optional override for the workspace YAML path
- `apply_planning_scene_service`: default `/apply_planning_scene`

## Grasping Node

Node:

- Package: `grasping`
- Executable: `grasping_node`

Responsibilities:

- Call `anygrasp_msgs/srv/GetGrasps`
- Forward the returned `PoseStamped` to `MoveToPose`
- Close the gripper after a successful motion
- Optionally send a configured post-grasp pose through the same action

This keeps the grasping node independent from MoveIt-specific details.

## Launch Flow

`grasping/launch/grip.launch.py` now starts:

- static TF publishers
- `arm_control_node`
- `grasping_node`

The launch file passes planning parameters only to `arm_control_node` and passes the action name to `grasping_node`.

## Typical Run

1. Start MoveIt and the robot drivers.
2. Calibrate or update `workspace.yaml` with `workspace_calibration` if needed.
3. Launch `grip.launch.py`.
4. Call `/grasping_node/run_grasp`.

The grasping node will request a grasp pose, and the arm-control node will execute the motion while respecting the calibrated planning scene.
