from __future__ import annotations

from typing import Any, Dict, List, Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from grasping_arm_control.common import (
	Quaternion,
	dict_to_pose,
	load_yaml_dict,
	normalize_quaternion,
	resolve_config_path,
	transform_pose_to_frame,
)
from grasping_msgs.action import MoveToPose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
	BoundingVolume,
	CollisionObject,
	Constraints,
	MotionPlanRequest,
	MoveItErrorCodes,
	OrientationConstraint,
	PlanningOptions,
	PlanningScene,
	PositionConstraint,
	RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
import tf2_ros


class ArmControlNode(Node):
	def __init__(self) -> None:
		super().__init__('arm_control_node')

		self.declare_parameter('action_name', 'move_arm_to_pose')
		self.declare_parameter('move_group_action_name', 'move_action')
		self.declare_parameter('planning_group', 'manipulator')
		self.declare_parameter('planning_frame', 'base_link')
		self.declare_parameter('end_effector_link', 'tool0')
		self.declare_parameter('allowed_planning_time', 5.0)
		self.declare_parameter('num_planning_attempts', 5)
		self.declare_parameter('max_velocity_scaling', 0.2)
		self.declare_parameter('max_acceleration_scaling', 0.2)
		self.declare_parameter('position_tolerance_m', 0.005)
		self.declare_parameter('orientation_tolerance_rad', 0.1)
		self.declare_parameter('planning_pipeline_id', '')
		self.declare_parameter('planner_id', '')
		self.declare_parameter('workspace_config_path', '')
		self.declare_parameter('apply_planning_scene_service', '/apply_planning_scene')

		self._planning_frame = str(self.get_parameter('planning_frame').value)
		self._workspace_config_path = resolve_config_path(
			'grasping_arm_control',
			str(self.get_parameter('workspace_config_path').value),
			'workspace.yaml',
		)

		# TF is only handled in this node so every incoming action goal is transformed into
		# the planning frame before MoveIt constraints are constructed.
		self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
		self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

		self._movegroup_client = ActionClient(
			self,
			MoveGroup,
			str(self.get_parameter('move_group_action_name').value),
		)
		self._planning_scene_client = self.create_client(
			ApplyPlanningScene,
			str(self.get_parameter('apply_planning_scene_service').value),
		)
		self._action_server = ActionServer(
			self,
			MoveToPose,
			str(self.get_parameter('action_name').value),
			execute_callback=self._execute_move_to_pose,
			goal_callback=self._goal_callback,
			cancel_callback=self._cancel_callback,
		)

		# Load static workspace obstacles once at startup so every later arm action is planned
		# against the calibrated scene written by workspace_calibration_node.py.
		self._load_workspace_into_planning_scene()
		self.get_logger().info(
			f"Arm control action server ready on {self.get_parameter('action_name').value}"
		)

	def destroy_node(self) -> bool:
		self._action_server.destroy()
		return super().destroy_node()

	def _goal_callback(self, _goal_request: MoveToPose.Goal) -> GoalResponse:
		return GoalResponse.ACCEPT

	def _cancel_callback(self, _goal_handle: Any) -> CancelResponse:
		return CancelResponse.ACCEPT

	def _execute_move_to_pose(self, goal_handle: Any) -> MoveToPose.Result:
		feedback = MoveToPose.Feedback()
		target_pose = goal_handle.request.target_pose

		try:
			# Clients can send poses in any connected frame. The server normalizes that first,
			# then uses a single planning pipeline for both grasp and post-grasp motion.
			feedback.state = 'transforming_target_pose'
			goal_handle.publish_feedback(feedback)
			target_pose = transform_pose_to_frame(
				self,
				self._tf_buffer,
				target_pose,
				self._planning_frame,
			)

			feedback.state = 'planning_and_executing'
			goal_handle.publish_feedback(feedback)
			ok, message = self._move_to_pose(target_pose)
		except Exception as exc:  # noqa: BLE001
			ok = False
			message = str(exc)

		result = MoveToPose.Result()
		result.success = bool(ok)
		result.message = message

		if ok:
			goal_handle.succeed()
		else:
			goal_handle.abort()
		return result

	def _load_workspace_into_planning_scene(self) -> None:
		# The workspace file is authored by the calibration node and already contains derived
		# primitive geometry, so startup only needs to translate it into CollisionObjects.
		workspace_config = load_yaml_dict(self._workspace_config_path, {'objects': []})
		collision_objects = self._collision_objects_from_workspace(workspace_config)
		object_names = [collision_object.id for collision_object in collision_objects]
		if object_names:
			self.get_logger().info('Workspace objects loaded: ' + ', '.join(object_names))
		else:
			self.get_logger().info('Workspace config contains no collision objects.')

		if not self._planning_scene_client.wait_for_service(timeout_sec=5.0):
			self.get_logger().warn('ApplyPlanningScene service not available; skipping workspace scene load.')
			return

		request = ApplyPlanningScene.Request()
		request.scene = PlanningScene()
		request.scene.is_diff = True
		request.scene.world.collision_objects = collision_objects

		future = self._planning_scene_client.call_async(request)
		rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
		if not future.done() or future.result() is None:
			self.get_logger().warn('ApplyPlanningScene request did not complete.')
			return

		if not future.result().success:
			self.get_logger().warn('MoveIt rejected the workspace planning scene update.')
			return

		self.get_logger().info(f'Applied {len(collision_objects)} workspace objects to the planning scene.')

	def _collision_objects_from_workspace(self, workspace_config: Dict[str, Any]) -> List[CollisionObject]:
		planning_frame = str(workspace_config.get('base_frame', self._planning_frame))
		objects: List[CollisionObject] = []

		for workspace_object in workspace_config.get('objects', []):
			# Keep the planning scene limited to primitives that MoveIt can consume directly from
			# the saved YAML without any mesh generation step.
			geometry = workspace_object.get('geometry', {})
			geometry_type = geometry.get('type')
			if geometry_type not in {'box', 'cylinder'}:
				self.get_logger().warn(
					f"Skipping {workspace_object.get('name', 'unnamed')} with unsupported geometry type {geometry_type}."
				)
				continue

			primitive = SolidPrimitive()
			dimensions = geometry.get('dimensions', {})
			if geometry_type == 'box':
				primitive.type = SolidPrimitive.BOX
				primitive.dimensions = [
					float(dimensions.get('x', 0.0)),
					float(dimensions.get('y', 0.0)),
					float(dimensions.get('z', 0.0)),
				]
			else:
				primitive.type = SolidPrimitive.CYLINDER
				primitive.dimensions = [
					float(dimensions.get('height', 0.0)),
					float(dimensions.get('radius', 0.0)),
				]

			pose = dict_to_pose(geometry.get('pose', {}))
			pose.orientation = self._normalized_orientation(pose.orientation)

			collision_object = CollisionObject()
			collision_object.id = str(workspace_object.get('name', f'object_{len(objects) + 1}'))
			collision_object.header.frame_id = planning_frame
			collision_object.primitives = [primitive]
			collision_object.primitive_poses = [pose]
			collision_object.operation = CollisionObject.ADD
			objects.append(collision_object)

		return objects

	def _normalized_orientation(self, orientation: Any) -> Any:
		normalized = normalize_quaternion(
			Quaternion(
				x=float(orientation.x),
				y=float(orientation.y),
				z=float(orientation.z),
				w=float(orientation.w),
			)
		)
		orientation.x = normalized.x
		orientation.y = normalized.y
		orientation.z = normalized.z
		orientation.w = normalized.w
		return orientation

	def _move_to_pose(self, target_pose: PoseStamped) -> tuple[bool, str]:
		action_name = str(self.get_parameter('move_group_action_name').value)
		if not self._movegroup_client.wait_for_server(timeout_sec=5.0):
			return False, f"MoveGroup action server '{action_name}' not available."

		# The custom action stays thin and delegates actual motion execution to MoveIt so the
		# rest of the system can talk to one stable arm-control interface.
		goal = MoveGroup.Goal()
		goal.request = self._build_motion_plan_request(target_pose)
		goal.planning_options = PlanningOptions()
		goal.planning_options.plan_only = False
		goal.planning_options.look_around = False
		goal.planning_options.replan = False
		goal.planning_options.replan_attempts = 0

		send_future = self._movegroup_client.send_goal_async(goal)
		rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
		if not send_future.done() or send_future.result() is None:
			return False, 'Failed to send MoveGroup goal.'

		goal_handle = send_future.result()
		if not goal_handle.accepted:
			return False, 'MoveGroup goal was rejected.'

		result_future = goal_handle.get_result_async()
		rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
		if not result_future.done() or result_future.result() is None:
			return False, 'MoveGroup result not received.'

		result = result_future.result().result
		if result.error_code.val != MoveItErrorCodes.SUCCESS:
			return False, f'MoveGroup failed with error code: {result.error_code.val}'

		return True, 'Arm motion completed successfully.'

	def _build_motion_plan_request(self, target_pose: PoseStamped) -> MotionPlanRequest:
		request = MotionPlanRequest()
		request.group_name = str(self.get_parameter('planning_group').value)
		request.allowed_planning_time = float(self.get_parameter('allowed_planning_time').value)
		request.num_planning_attempts = int(self.get_parameter('num_planning_attempts').value)
		request.max_velocity_scaling_factor = float(self.get_parameter('max_velocity_scaling').value)
		request.max_acceleration_scaling_factor = float(self.get_parameter('max_acceleration_scaling').value)

		pipeline_id = str(self.get_parameter('planning_pipeline_id').value)
		planner_id = str(self.get_parameter('planner_id').value)
		if pipeline_id:
			request.pipeline_id = pipeline_id
		if planner_id:
			request.planner_id = planner_id

		request.start_state = RobotState()
		request.goal_constraints = [self._pose_to_constraints(target_pose)]
		return request

	def _pose_to_constraints(self, target_pose: PoseStamped) -> Constraints:
		ee_link = str(self.get_parameter('end_effector_link').value)
		pos_tol = float(self.get_parameter('position_tolerance_m').value)
		ori_tol = float(self.get_parameter('orientation_tolerance_rad').value)

		# Position is represented as a sphere tolerance around the requested TCP target.
		sphere = SolidPrimitive()
		sphere.type = SolidPrimitive.SPHERE
		sphere.dimensions = [max(1e-4, pos_tol)]

		volume = BoundingVolume()
		volume.primitives = [sphere]
		volume.primitive_poses = [target_pose.pose]

		position_constraint = PositionConstraint()
		position_constraint.header.frame_id = self._planning_frame
		position_constraint.link_name = ee_link
		position_constraint.constraint_region = volume

		# Orientation is normalized before building constraints so invalid inputs do not leak
		# into the planner and cause confusing failures.
		normalized = normalize_quaternion(
			Quaternion(
				target_pose.pose.orientation.x,
				target_pose.pose.orientation.y,
				target_pose.pose.orientation.z,
				target_pose.pose.orientation.w,
			)
		)
		orientation_constraint = OrientationConstraint()
		orientation_constraint.header.frame_id = self._planning_frame
		orientation_constraint.link_name = ee_link
		orientation_constraint.orientation.x = normalized.x
		orientation_constraint.orientation.y = normalized.y
		orientation_constraint.orientation.z = normalized.z
		orientation_constraint.orientation.w = normalized.w
		orientation_constraint.absolute_x_axis_tolerance = ori_tol
		orientation_constraint.absolute_y_axis_tolerance = ori_tol
		orientation_constraint.absolute_z_axis_tolerance = ori_tol
		orientation_constraint.weight = 1.0

		constraints = Constraints()
		constraints.position_constraints = [position_constraint]
		constraints.orientation_constraints = [orientation_constraint]
		return constraints


def main(args: Optional[List[str]] = None) -> None:
	rclpy.init(args=args)
	node = ArmControlNode()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		rclpy.shutdown()
