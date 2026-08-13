# Control Stack Overview

This document covers the shared architecture and bringup flow for the grasping control stack.

Use the topic-specific documents for implementation details:

- [arm_control.md](./arm_control.md) for `motion_execution_node`
- [simple_grasping.md](./simple_grasping.md) for `simple_grasping`
- [creation.md](../workspace/creation.md) for `workspace_creation`

## Architecture

The motion stack is split into two ROS 2 nodes.

- `motion_execution_node` executes robot motion through MoveIt.

At runtime the stack usually depends on four subsystems:

1. An external stack for grasp detection such as [Anygrasp](https://github.com/CollaborativeRoboticsLab/anygrasp_grasping).
2. The control stack in this package.
3. MoveIt and the robot driver.
4. A gripper action server.

An optional fifth control surface is the `simple_grasping` node, which executes a fixed parameterized sequence of named-pose and gripper actions for simple scripted demos.

## Shared Motion Interface

The shared action interface lives in the `grasping_msgs` package.

Grasp-pose action:

- `grasping_msgs/action/MoveToPose`
- Goal: `geometry_msgs/PoseStamped target_pose`
- Result: `bool success`, `string message`
- Feedback: `string state`

Named-pose action:

- `grasping_msgs/action/MoveToNamedPose`
- Goal: `string pose_name`
- Result: `bool success`, `string message`
- Feedback: `string state`

The grasp-pose goal stays intentionally minimal. Clients send only a target pose, and `motion_execution_node` resolves transforms, workspace configuration, and MoveIt planning from its own parameters. Named-pose goals load preconfigured `[x, y, z, roll, pitch, yaw]` values from ROS parameters supplied by `motion_config.yaml`.


## Static TF Assumptions

AnyGrasp returns `PoseStamped` results in the source point-cloud frame, and `motion_execution_node` later transforms those poses into its planning frame.

Use the static camera transform only when the camera is rigidly mounted to the end effector. If the camera is mounted elsewhere, provide the correct TF in your robot setup instead of relying on an application-layer launch file.

## Motion Configuration

The shared `motion_execution.launch.py` launch file starts `motion_execution_node`, loads `motion_config.yaml`, resolves `workspace_file`, and loads the selected workspace YAML as a ROS parameter file.

The only shared motion-execution launch argument is:

| Argument | Default | Description |
| --- | --- | --- |
| `workspace_file` | `workspace_empty.yaml` | Workspace YAML selected from an absolute path, the colcon workspace root, or `grasping_control/config`; falls back to `workspace_empty.yaml`. |

Motion behavior is configured through `motion_config.yaml`:

| Argument | Default | Description |
| --- | --- | --- |
| `move_group_action_name` | `move_action` | Name of the MoveIt `MoveGroup` action server contacted by `motion_execution_node`. |
| `planning_group` | `manipulator` | MoveIt planning group used when building the motion request. |
| `planning_frame` | `base_link` | Target frame into which incoming poses are transformed before planning. |
| `end_effector_link` | `tcp` for soft-gripper launches | Link constrained to the requested pose in the generated goal constraints. |
| `allowed_planning_time` | `5.0` | Maximum planning time in seconds for each MoveIt request. |
| `num_planning_attempts` | `5` | Number of planning attempts MoveIt may use before reporting failure. |
| `max_velocity_scaling` | `0.2` | Velocity scaling factor applied to the generated motion plan. |
| `max_acceleration_scaling` | `0.2` | Acceleration scaling factor applied to the generated motion plan. |
| `position_tolerance_m` | `0.005` | Positional tolerance in meters used for the end-effector goal constraint. |
| `orientation_tolerance_rad` | `0.1` | Angular tolerance in radians used for the end-effector orientation constraint. |
| `planning_pipeline_id` | `''` | Optional MoveIt planning pipeline override. Leave empty to use the MoveIt default. |
| `planner_id` | `''` | Optional planner override within the selected planning pipeline. Leave empty to use the default planner. |
| `compute_ik_service` | `/compute_ik` | MoveIt IK service used to compute a nearby seeded joint solution before planning. |
| `joint_state_topic` | `/joint_states` | Joint-state source cached by `motion_execution_node` for nearby-IK seeding. |
| `planning_joint_names` | UR10 arm joints | Ordered joints extracted from `joint_state_topic` and constrained in the joint-goal request. |
| `prefer_nearby_ik` | `true` | When enabled, the node tries nearby seeded IK before sending the MoveIt request. |
| `fallback_to_pose_planning_on_ik_failure` | `true` | When enabled, IK failures fall back to the old pose-constrained planning path. |
| `joint_state_timeout_sec` | `0.5` | Maximum age of the cached planning-joint state before nearby IK is skipped. |
| `ik_timeout_sec` | `0.2` | IK timeout forwarded to `compute_ik_service`. |
| `joint_goal_tolerance_rad` | `0.001` | Symmetric tolerance used when MoveIt is given a joint-space goal. |
| `log_joint_goal_deltas` | `false` | Log the selected nearby IK joint deltas for runtime debugging. |

Named poses are triggered through `MoveToNamedPose`, for example with `pose_name=workspace_center`, `pose_name=pre_grasp`, or `pose_name=post_grasp`.

With nearby IK enabled, the stack no longer asks MoveIt to solve only "reach this pose somehow". Instead it seeds IK from the current arm joints, unwraps the result toward the current branch, and then plans in joint space. This reduces wrist and elbow branch flips on UR-style arms while preserving a pose-based external API.

