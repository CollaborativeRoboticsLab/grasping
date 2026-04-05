# grasping stack

This repo’s grasping stack centers around the `grasping` ROS 2 Python package, which provides:

- A single “pipeline controller” node (`grasping/grasping_node`) that coordinates:
	1. AnyGrasp grasp pose estimation
	2. Forwarding the selected pose to the arm-control action server
	3. Gripper close
	4. Optional post-grasp motion
- A bringup launch file (`grasping/launch/grip.launch.py`) that wires up the TF frames needed to transform AnyGrasp outputs into the robot planning frame.

If you are using an Intel RealSense (D435 in our setup), see the “Camera (RealSense)” section below.

## Running the pipeline

### Modes (`server_mode`)

The node supports two operating modes:

- **Server mode** (`server_mode: true`, default): exposes the Trigger service and waits for external requests.
- **One-shot mode** (`server_mode: false`): runs the pipeline once at startup and exits (useful for scripting).
### Triggering the pipeline

When `server_mode: true`, the node exposes a Trigger service:

- Service: `/grasping_node/run_grasp` when started through `grip.launch.py`
- Type: `std_srvs/srv/Trigger`

Example:

```bash
ros2 service call /grasping_node/run_grasp std_srvs/srv/Trigger {}
```

In one-shot mode (`server_mode: false`), you run the node and it will execute immediately:

```bash
ros2 run grasping grasping_node --ros-args -p server_mode:=false
```

If you start the node directly without the launch file and keep the built-in node name, the Trigger service will be rooted under `/grasping_node` instead.

## What the pipeline does

### 1) Request a grasp pose from AnyGrasp

The node calls `anygrasp_msgs/srv/GetGrasps` with `count = 1`.

In this repo, AnyGrasp nodes expose services:

- `detection` (single-shot detection)
- `tracking` (tracking-based output)

Important detail: the service response contains a list of `geometry_msgs/PoseStamped`. Each pose keeps the source point cloud header, so the camera frame comes directly from AnyGrasp.

### 2) Send the grasp pose to arm control

The grasping node does not transform poses or call MoveIt directly anymore.

Instead it sends the returned `PoseStamped` to:

- Action type: `grasping_msgs/action/MoveToPose`
- Action name: configured by `arm_action_name`

The arm-control node then handles:

- TF transformation into its planning frame
- workspace collision objects
- optional workspace-area filtering
- MoveIt planning and execution

### 3) Close the gripper

The node uses the common gripper actions from `gripper_msgs`:

- `/open_gripper` (`gripper_msgs/action/OpenGripper`)
- `/close_gripper` (`gripper_msgs/action/CloseGripper`)

It currently uses **position-based close** (i.e. `use_torque_mode=false`).

Two helper functions exist:

- `close_gripper_position()` (used by default)
- `close_gripper_torque(torque)` (available, but not used by default)

The numeric meaning of `torque` is driver-specific.

### 4) Post-grasp move

If `do_post_grasp_move` is true, the node moves to `post_grasp_pose` (a pose in `post_grasp_frame`).

## Launch (bringup)

### `grip.launch.py`

This launch file starts the pipeline node and publishes two static transforms:

- `tf2_ros/static_transform_publisher`: end-effector → gripper
- `tf2_ros/static_transform_publisher`: end-effector → camera
- `grasping/grasping_node`: the pipeline controller

Why static transforms?

AnyGrasp returns `geometry_msgs/PoseStamped` with the original pointcloud header. The pipeline uses that frame directly and transforms:

`poses[0].header.frame_id` → `planning_frame`

That transform is performed inside `grasping_arm_control/arm_control_node`, not inside `grasping_node`.

If your camera is not rigidly mounted to the end-effector, do not use static transforms.

#### Usage

Start the pipeline:

```bash
ros2 launch grasping grip.launch.py
```

Trigger a grasp cycle (server mode):

```bash
ros2 service call /grasping_node/run_grasp std_srvs/srv/Trigger {}
```

Run once and exit:

```bash
ros2 launch grasping grip.launch.py server_mode:=false
```

#### Launch arguments (`grip.launch.py`)

Pipeline configuration:

- `server_mode` (default: `true`)
- `anygrasp_service` (default: `detection`)
	- Service name to call (`detection` or `tracking` in this repo).
- `arm_action_name` (default: `move_arm_to_pose`)
	- Name of the `grasping_msgs/action/MoveToPose` action server exposed by `arm_control_node`.

Gripper actions:

- `open_action_name` (default: `/open_gripper`)
- `close_action_name` (default: `/close_gripper`)

Post-grasp motion:

- `do_post_grasp_move` (default: `true`)
- `post_grasp_frame` (default: `base_link`)
- `post_grasp_x`, `post_grasp_y`, `post_grasp_z` (defaults: `0.0`)
- `post_grasp_qx`, `post_grasp_qy`, `post_grasp_qz`, `post_grasp_qw` (defaults: identity quaternion)

Frames:

- `ee_frame` (default: `tool0`)
- `gripper_frame` (default: `gripper`)
- `camera_frame` (default: `camera_color_optical_frame`)

Static TF: end-effector → gripper:

- `ee_to_gripper_x`, `ee_to_gripper_y`, `ee_to_gripper_z`
- `ee_to_gripper_qx`, `ee_to_gripper_qy`, `ee_to_gripper_qz`, `ee_to_gripper_qw`

Publishes: `parent = ee_frame`, `child = gripper_frame`

Static TF: end-effector → camera:

- `ee_to_camera_x`, `ee_to_camera_y`, `ee_to_camera_z`
- `ee_to_camera_qx`, `ee_to_camera_qy`, `ee_to_camera_qz`, `ee_to_camera_qw`

Publishes: `parent = ee_frame`, `child = camera_frame`

#### Checking transforms

List frames:

```bash
ros2 run tf2_tools view_frames
```

Inspect a transform:

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

## Parameters (pipeline node)

### AnyGrasp

- `anygrasp_service` (string, default `detection`): service name to call
- Returned grasp poses already include `header.frame_id` and `header.stamp` from the source pointcloud

### TF / planning frames

- The grasping node no longer owns planning-frame or MoveIt parameters.
- Those are configured on `grasping_arm_control/arm_control_node` or passed through `grip.launch.py` to that node.

### Arm-control action

- `arm_action_name` (string, default `move_arm_to_pose`)

### Gripper action names

- `open_action_name` (string, default `/open_gripper`)
- `close_action_name` (string, default `/close_gripper`)

### Post-grasp

- `do_post_grasp_move` (bool, default true)
- `post_grasp_frame` (string, default `base_link`)
- `post_grasp_pose` (list[7], default identity at origin): `[x,y,z,qx,qy,qz,qw]`

## Required external components

For a full run, you must have:

1. AnyGrasp node running (providing `detection` or `tracking` service)
2. `grasping_arm_control/arm_control_node` running and exposing the `MoveToPose` action configured by `arm_action_name`
3. A TF tree connecting the arm-control planning frame and the pointcloud frame reported in the returned grasp pose header
4. MoveIt and the arm-control node's downstream `MoveGroup` interface running
5. A gripper action server providing `/close_gripper` (Dynamixel or Feetech)

## Troubleshooting

- **AnyGrasp call fails / returns no pose**: check the AnyGrasp node logs and confirm the service name (`ros2 service list`).
- **Arm-control action not available**: list actions (`ros2 action list`) and confirm the `MoveToPose` action name configured by `arm_action_name`.
- **TF lookup fails**: verify that the arm-control planning frame and the grasp pose header frame exist and are connected (`tf2_echo`).
- **Gripper action not available**: start one of the gripper drivers and verify `/close_gripper` exists.