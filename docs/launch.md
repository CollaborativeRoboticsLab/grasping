# ur_launch

`ur_launch` is the “bringup” package for the UR10 grasping stack. It contains launch files that start the grasping pipeline nodes and the TF wiring required to relate:

- the robot end-effector frame
- the gripper frame
- the camera frame

## Launch files

### `camera.launch.py`

Launches `realsense2_camera` with a large set of configurable launch arguments.

This is a vendor-provided style launch file and is not specific to grasping logic. Use it if you want to bring up the RealSense camera within this repo.

### `grip.launch.py`

Launches the full grasping orchestrator and two static TF publishers:

- `tf2_ros/static_transform_publisher`: end-effector → gripper
- `tf2_ros/static_transform_publisher`: end-effector → camera
- `ur_grasping/ur_grasping_node`: the pipeline controller

## Concepts

### Why static transforms?

AnyGrasp produces a grasp pose in a camera frame (a `geometry_msgs/Pose` with no header/frame info). The pipeline needs a TF tree so it can transform:

`anygrasp_frame` → `planning_frame`

This launch file publishes the fixed transforms that are commonly constant for a wrist-mounted camera + gripper.

If your camera is not rigidly mounted to the end-effector, do not use static transforms.

## Usage

Start the pipeline:

```bash
ros2 launch ur_launch grip.launch.py
```

Then trigger a grasp cycle:

```bash
ros2 service call /ur_grasping_node/run_grasp std_srvs/srv/Trigger {}
```

Run a single grasp cycle and exit (one-shot mode):

```bash
ros2 launch ur_launch grip.launch.py server_mode:=false
```

## Launch arguments (grip.launch.py)

### Pipeline configuration

- `server_mode` (default: `true`)
	- If true, exposes `/ur_grasping_node/run_grasp`.
	- If false, runs once and exits.
- `anygrasp_service` (default: `detection`)
	- AnyGrasp service name to call (`detection` or `tracking` in this repo).
- `move_group_action_name` (default: `move_action`)
	- Name of the MoveIt `moveit_msgs/action/MoveGroup` action server.
	- Common values are `move_action` or `move_group` depending on your MoveIt setup.
- `planning_group` (default: `manipulator`)
- `planning_frame` (default: `base_link`)
- `end_effector_link` (default: `tool0`)

### Gripper actions

- `open_action_name` (default: `/open_gripper`)
- `close_action_name` (default: `/close_gripper`)

### Post-grasp motion

- `do_post_grasp_move` (default: `true`)
- `post_grasp_frame` (default: `base_link`)
- `post_grasp_x`, `post_grasp_y`, `post_grasp_z` (defaults: `0.0`)
- `post_grasp_qx`, `post_grasp_qy`, `post_grasp_qz`, `post_grasp_qw` (defaults: identity quaternion)

### Frames

- `ee_frame` (default: `tool0`)
- `gripper_frame` (default: `gripper`)
- `camera_frame` (default: `camera_color_optical_frame`)

### Static TF: end-effector → gripper

Quaternion transform arguments:

- `ee_to_gripper_x`, `ee_to_gripper_y`, `ee_to_gripper_z`
- `ee_to_gripper_qx`, `ee_to_gripper_qy`, `ee_to_gripper_qz`, `ee_to_gripper_qw`

The TF published is:

`parent = ee_frame`, `child = gripper_frame`

### Static TF: end-effector → camera

Quaternion transform arguments:

- `ee_to_camera_x`, `ee_to_camera_y`, `ee_to_camera_z`
- `ee_to_camera_qx`, `ee_to_camera_qy`, `ee_to_camera_qz`, `ee_to_camera_qw`

The TF published is:

`parent = ee_frame`, `child = camera_frame`

## Checking transforms

List frames:

```bash
ros2 run tf2_tools view_frames
```

Inspect a transform:

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

## Related docs

- `docs/grasping.md` (pipeline behavior and node parameters)
