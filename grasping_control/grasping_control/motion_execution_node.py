from __future__ import annotations

from typing import Any, Dict, List, Optional

from geometry_msgs.msg import Point, PoseStamped
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from grasping_control.common import (
	Quaternion,
	dict_to_pose,
	load_yaml_dict,
	normalize_quaternion,
	resolve_config_path,
	transform_pose_to_frame,
)
from grasping_control.workspace_utils import collision_objects_from_workspace, point_in_workspace_area
from grasping_msgs.action import MoveToPose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
	BoundingVolume,
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
from visualization_msgs.msg import Marker


class MotionExecutionNode(Node):
	"""
	@brief Action server that plans and executes arm motion requests with MoveIt.
	"""

	def __init__(self) -> None:
		"""
		@brief Initialize parameters, TF, MoveIt clients, and the action server.
		"""
		super().__init__('motion_execution_node')

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
		self.declare_parameter('workspace_area_marker_topic', '/workspace_area_marker')

		self._planning_frame = str(self.get_parameter('planning_frame').value)
		self._workspace_config_path = resolve_config_path(
			'grasping_control',
			str(self.get_parameter('workspace_config_path').value),
			'workspace.yaml',
		)
		self._workspace_area: Optional[Dict[str, Any]] = None
		self._workspace_area_frame = self._planning_frame
		self._post_grasp_pose: Optional[Dict[str, Any]] = None

		# TF is only handled in this node so every incoming action goal is transformed into
		# the planning frame before MoveIt constraints are constructed.
		self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
		self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
		marker_qos = QoSProfile(
			history=HistoryPolicy.KEEP_LAST,
			depth=1,
			durability=DurabilityPolicy.TRANSIENT_LOCAL,
		)
		self._workspace_area_marker_publisher = self.create_publisher(
			Marker,
			str(self.get_parameter('workspace_area_marker_topic').value),
			marker_qos,
		)

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
		# against the calibrated scene written by workspace_creation_node.py.
		self._load_workspace_into_planning_scene()

		self.get_logger().info(
			f"Motion execution action server ready on {self.get_parameter('action_name').value}"
		)

	def destroy_node(self) -> bool:
		"""
		@brief Destroy the action server before releasing the ROS node.

		@return Result from the base destroy_node implementation.
		"""
		self._action_server.destroy()
		return super().destroy_node()

	def _goal_callback(self, _goal_request: MoveToPose.Goal) -> GoalResponse:
		"""
		@brief Accept all incoming MoveToPose goals.

		@param _goal_request Requested goal payload.
		@return Goal acceptance decision.
		"""
		return GoalResponse.ACCEPT

	def _cancel_callback(self, _goal_handle: Any) -> CancelResponse:
		"""
		@brief Accept cancellation for active goals.

		@param _goal_handle Goal handle requesting cancellation.
		@return Cancel acceptance decision.
		"""
		return CancelResponse.ACCEPT

	def _execute_move_to_pose(self, goal_handle: Any) -> MoveToPose.Result:
		"""
		@brief Transform, plan, and execute an incoming pose goal.

		@param goal_handle Active action goal handle.
		@return Action result describing the outcome.
		"""
		feedback = MoveToPose.Feedback()
		target_pose = goal_handle.request.target_pose

		try:
			# Clients can send poses in any connected frame. The server normalizes that first,
			# then uses a single planning pipeline for both grasp and post-grasp motion.
			feedback.state = 'transforming_target_pose'
			goal_handle.publish_feedback(feedback)
			if bool(goal_handle.request.move_to_post_grasp_pose):
				configured_post_grasp_pose = self._get_post_grasp_pose_stamped()
				if configured_post_grasp_pose is None:
					raise RuntimeError('No post-grasp pose is configured in the workspace file.')
				target_pose = configured_post_grasp_pose
			target_pose = transform_pose_to_frame(
				self,
				self._tf_buffer,
				target_pose,
				self._planning_frame,
			)

			feedback.state = 'validating_workspace_area'
			goal_handle.publish_feedback(feedback)
			if not self._target_pose_in_workspace_area(target_pose):
				ok = False
				message = 'Target pose lies outside the calibrated workspace area.'
				result = MoveToPose.Result()
				result.success = False
				result.message = message
				goal_handle.abort()
				return result

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
		"""
		@brief Load persisted workspace obstacles into the MoveIt planning scene.
		"""
		if not self._workspace_config_path.exists():
			self.get_logger().warn(
				f'Workspace config not found at {self._workspace_config_path}; starting with an empty scene.'
			)
		else:
			self.get_logger().info(f'Loading workspace config from {self._workspace_config_path}')

		# The workspace file is authored by the calibration node and already contains derived
		# primitive geometry, so startup only needs to translate it into CollisionObjects.
		workspace_config = load_yaml_dict(self._workspace_config_path, {'workspace_area': None, 'objects': []})
		self._workspace_area_frame = str(workspace_config.get('base_frame', self._planning_frame))
		post_grasp_pose = workspace_config.get('post_grasp_pose')
		if isinstance(post_grasp_pose, dict):
			self._post_grasp_pose = post_grasp_pose
		else:
			self._post_grasp_pose = None
			if post_grasp_pose is not None:
				self.get_logger().warn('Ignoring invalid post_grasp_pose value; expected a mapping.')
		workspace_area = workspace_config.get('workspace_area')
		if isinstance(workspace_area, dict):
			self._workspace_area = workspace_area
		else:
			self._workspace_area = None
			if workspace_area is not None:
				self.get_logger().warn('Ignoring invalid workspace_area value; expected a mapping.')

		collision_objects = collision_objects_from_workspace(
			workspace_config,
			self._planning_frame,
			warn=self.get_logger().warn,
		)

		object_names = [collision_object.id for collision_object in collision_objects]

		if object_names:
			self.get_logger().info('Workspace objects loaded: ' + ', '.join(object_names))
		else:
			self.get_logger().info('Workspace config contains no collision objects.')

		if self._workspace_area is not None:
			self.get_logger().info('Workspace area filtering is enabled.')
		else:
			self.get_logger().info('Workspace area filtering is disabled.')
		self._publish_workspace_area_marker()

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

	def _get_post_grasp_pose_stamped(self) -> Optional[PoseStamped]:
		"""
		@brief Return the workspace-configured post-grasp pose as a PoseStamped.

		@return PoseStamped when configured and valid, otherwise None.
		"""
		if not isinstance(self._post_grasp_pose, dict):
			return None

		frame = str(self._post_grasp_pose.get('frame', '')).strip()
		pose_dict = self._post_grasp_pose.get('pose')
		if not frame or not isinstance(pose_dict, dict):
			self.get_logger().warn('Ignoring invalid post_grasp_pose entry in workspace config.')
			return None

		pose_stamped = PoseStamped()
		pose_stamped.header.stamp = self.get_clock().now().to_msg()
		pose_stamped.header.frame_id = frame
		pose_stamped.pose = dict_to_pose(pose_dict)
		return pose_stamped

	def _target_pose_in_workspace_area(self, target_pose: PoseStamped) -> bool:
		"""
		@brief Check whether a transformed target pose lies inside the calibrated work area.

		@param target_pose Goal pose expressed in the planning frame.
		@return True when no area is configured or the pose lies inside it.
		"""
		if self._workspace_area is None:
			return True

		geometry = self._workspace_area.get('geometry', {})
		if not geometry:
			self.get_logger().warn('Workspace area is configured but missing geometry; rejecting goal.')
			return False

		pose_for_check = target_pose
		if target_pose.header.frame_id != self._workspace_area_frame:
			pose_for_check = transform_pose_to_frame(
				self,
				self._tf_buffer,
				target_pose,
				self._workspace_area_frame,
			)

		return point_in_workspace_area(
			geometry,
			{
				'x': float(pose_for_check.pose.position.x),
				'y': float(pose_for_check.pose.position.y),
				'z': float(pose_for_check.pose.position.z),
			},
		)

	def _publish_workspace_area_marker(self) -> None:
		"""
		@brief Publish the calibrated workspace area as a semi-transparent RViz plane.
		"""
		marker = Marker()
		marker.header.stamp = self.get_clock().now().to_msg()
		marker.header.frame_id = self._workspace_area_frame
		marker.ns = 'workspace_area'
		marker.id = 0
		marker.action = Marker.DELETE
		if self._workspace_area is None:
			self._workspace_area_marker_publisher.publish(marker)
			return

		geometry = self._workspace_area.get('geometry', {})
		corner_points = geometry.get('corner_points', [])
		if len(corner_points) != 4:
			self.get_logger().warn('Workspace area marker was not published because four corners are required.')
			self._workspace_area_marker_publisher.publish(marker)
			return

		marker.action = Marker.ADD
		marker.type = Marker.TRIANGLE_LIST
		marker.pose.orientation.w = 1.0
		marker.scale.x = 1.0
		marker.scale.y = 1.0
		marker.scale.z = 1.0
		marker.color.r = 0.1
		marker.color.g = 0.8
		marker.color.b = 0.2
		marker.color.a = 0.25

		for index in [0, 1, 2, 0, 2, 3]:
			point = corner_points[index]
			marker_point = Point()
			marker_point.x = float(point['x'])
			marker_point.y = float(point['y'])
			marker_point.z = float(point['z']) + 0.002
			marker.points.append(marker_point)

		self._workspace_area_marker_publisher.publish(marker)

	def _move_to_pose(self, target_pose: PoseStamped) -> tuple[bool, str]:
		"""
		@brief Send a MoveGroup action goal for the requested target pose.

		@param target_pose Goal pose already expressed in the planning frame.
		@return Tuple of success flag and status message.
		"""
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
		"""
		@brief Construct a MoveIt motion planning request for a pose goal.

		@param target_pose Goal pose expressed in the planning frame.
		@return Configured MotionPlanRequest instance.
		"""
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
		"""
		@brief Convert a target pose into MoveIt position and orientation constraints.

		@param target_pose Goal pose expressed in the planning frame.
		@return Constraints object for the planner.
		"""
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
	"""
	@brief Run the arm control node until shutdown.

	@param args Optional ROS command-line arguments.
	"""
	rclpy.init(args=args)
	node = MotionExecutionNode()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		rclpy.shutdown()
