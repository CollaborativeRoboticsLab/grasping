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
- Load an optional calibrated `workspace_area` from the same YAML file.
- Push those objects into MoveIt through `ApplyPlanningScene`.
- Publish the calibrated workspace area as a semi-transparent green RViz marker plane.
- Reject target poses that fall outside the calibrated workspace area before planning.
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
- `workspace_area_marker_topic`: RViz marker topic, default `/workspace_area_marker`

Runtime behavior notes:

- If `workspace_area` is not configured, the node accepts target poses anywhere in the planning frame.
- If `workspace_area` is configured, the node checks the transformed target pose against the saved square boundary before sending a MoveIt goal.
- If `workspace_area` is configured, the node also publishes a semi-transparent green plane marker so the work zone is visible in RViz.
- If a pose is outside that boundary, the action aborts with the message `Target pose lies outside the calibrated workspace area.`
- The current filter is planar: it checks the XY position against the saved corner polygon and does not enforce a Z band.

## Grasping Node

Node:

- Package: `grasping`
- Executable: `grasping_node`

When started by `grip.launch.py`, the node name is explicitly set to `grasping_node`, so its Trigger service is exposed as `/grasping_node/run_grasp`.

Responsibilities:

- Call `anygrasp_msgs/srv/GetGrasps`
- Forward the returned `PoseStamped` to `MoveToPose`
- Close the gripper after a successful motion
- Optionally send a configured post-grasp pose through the same action

Important parameters:

- `server_mode`: expose the Trigger service when true, or run once and exit when false
- `anygrasp_service`: AnyGrasp service name, default `detection`
- `arm_action_name`: `MoveToPose` action name, default `move_arm_to_pose`
- `do_post_grasp_move`: enable post-grasp motion, default `true`
- `post_grasp_frame`: frame for the post-grasp pose
- `post_grasp_pose`: `[x,y,z,qx,qy,qz,qw]`
- `open_action_name`: default `/open_gripper`
- `close_action_name`: default `/close_gripper`

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

The grasping node will request a grasp pose, and the arm-control node will execute the motion while respecting both the calibrated planning scene and the optional calibrated workspace area.
