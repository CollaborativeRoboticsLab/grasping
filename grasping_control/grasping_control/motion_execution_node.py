from __future__ import annotations

from copy import deepcopy
import threading
from typing import Any, Dict, List, Optional

from geometry_msgs.msg import Point, PoseStamped
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from sensor_msgs.msg import JointState

from grasping_control.common import (
	Quaternion,
	coerce_float_sequence,
	coerce_string_sequence,
	nearest_equivalent_angle,
	quaternion_from_rpy,
	quaternion_to_rpy,
	rotate_vector_by_quaternion,
	transform_pose_to_frame,
)
from grasping_control.motion_utils import (
	MotionPlanningConfig,
	allowed_collision_pairs_from_workspace,
	append_allowed_collision_pairs,
	build_joint_move_group_goal,
	build_move_group_goal,
	planning_config_from_node,
	robot_state_from_joint_state,
)
from grasping_control.workspace_utils import (
	collision_objects_from_workspace,
	default_workspace_config,
	point_in_workspace_area,
	workspace_config_from_node_parameters,
)
from grasping_msgs.msg import ActiveScene, NamedPoseDescriptor
from grasping_msgs.action import MoveToJointPose, MoveToNamedPose, MoveToPose
from grasping_msgs.srv import GetActiveScene, ListNamedPoses
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
	AllowedCollisionMatrix,
	MoveItErrorCodes,
	PlanningScene,
	PlanningSceneComponents,
	RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionIK
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
		self.declare_parameter('planning_frame', 'world')
		self.declare_parameter('planning_pipeline_id', '')
		self.declare_parameter('planner_id', '')
		self.declare_parameter('allowed_planning_time', 5.0)
		self.declare_parameter('num_planning_attempts', 5)
		self.declare_parameter('max_velocity_scaling', 0.2)
		self.declare_parameter('max_acceleration_scaling', 0.2)
		self.declare_parameter('position_tolerance_m', 0.005)
		self.declare_parameter('orientation_tolerance_rad', 0.1)
		self.declare_parameter('end_effector_link', 'tool0')
		self.declare_parameter('named_pose_action_name', 'move_arm_to_named_pose')
		self.declare_parameter('joint_pose_action_name', 'move_arm_to_joint_pose')
		self.declare_parameter('poses_names', ['workspace_center', 'pre_grasp', 'post_grasp'])
		self.declare_parameter('poses_list', [])
		self.declare_parameter('apply_planning_scene_service', '/apply_planning_scene')
		self.declare_parameter('get_planning_scene_service', '/get_planning_scene')
		self.declare_parameter('compute_ik_service', '/compute_ik')
		self.declare_parameter('joint_state_topic', '/joint_states')
		self.declare_parameter('planning_joint_state_topic', '/manipulator_joint_states')
		self.declare_parameter(
			'planning_joint_names',
			[
				'shoulder_pan_joint',
				'shoulder_lift_joint',
				'elbow_joint',
				'wrist_1_joint',
				'wrist_2_joint',
				'wrist_3_joint',
			],
		)
		self.declare_parameter('prefer_nearby_ik', True)
		self.declare_parameter('fallback_to_pose_planning_on_ik_failure', True)
		self.declare_parameter('grasp_pose_recovery_enabled', True)
		self.declare_parameter('grasp_pose_recovery_tool_frame', 'tool_tip')
		self.declare_parameter('grasp_pose_recovery_recalculate_attempts', 3)
		self.declare_parameter('pose_relax_search_enabled', True)
		self.declare_parameter('pose_relax_limits', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
		self.declare_parameter('joint_state_timeout_sec', 0.5)
		self.declare_parameter('ik_timeout_sec', 0.2)
		self.declare_parameter('joint_goal_tolerance_rad', 0.001)
		self.declare_parameter('log_joint_goal_deltas', False)
		self.declare_parameter('workspace_area_marker_topic', '/workspace_area_marker')
		self.declare_parameter('active_scene_topic', '/active_scene')
		self.declare_parameter('get_active_scene_service_name', 'get_active_scene')
		self.declare_parameter('list_named_poses_service_name', 'list_named_poses')

		self._planning_frame = str(self.get_parameter('planning_frame').value)
		self._latest_joint_state: Optional[JointState] = None
		self._latest_joint_state_received_at: Optional[Time] = None
		self._latest_joint_positions_by_name: Dict[str, float] = {}
		self._latest_joint_position_received_at: Dict[str, Time] = {}
		self._declare_workspace_parameters()
		self._workspace_area: Optional[Dict[str, Any]] = None
		self._workspace_area_frame = self._planning_frame
		self._declare_configured_pose_parameters()

		# TF is only handled in this node so every incoming action goal is transformed into
		# the planning frame before MoveIt constraints are constructed.
		self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
		self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
		self._runtime_callback_group = ReentrantCallbackGroup()
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
		self._active_scene_subscription = self.create_subscription(
			ActiveScene,
			str(self.get_parameter('active_scene_topic').value),
			self._active_scene_callback,
			marker_qos,
			callback_group=self._runtime_callback_group,
		)

		self._movegroup_client = ActionClient(
			self,
			MoveGroup,
			str(self.get_parameter('move_group_action_name').value),
			callback_group=self._runtime_callback_group,
		)
		self._planning_scene_client = self.create_client(
			ApplyPlanningScene,
			str(self.get_parameter('apply_planning_scene_service').value),
			callback_group=self._runtime_callback_group,
		)
		self._get_planning_scene_client = self.create_client(
			GetPlanningScene,
			str(self.get_parameter('get_planning_scene_service').value),
			callback_group=self._runtime_callback_group,
		)
		self._compute_ik_client = self.create_client(
			GetPositionIK,
			str(self.get_parameter('compute_ik_service').value),
			callback_group=self._runtime_callback_group,
		)
		self._get_active_scene_client = self.create_client(
			GetActiveScene,
			str(self.get_parameter('get_active_scene_service_name').value),
			callback_group=self._runtime_callback_group,
		)
		self._joint_state_subscription = self.create_subscription(
			JointState,
			str(self.get_parameter('joint_state_topic').value),
			self._joint_state_callback,
			10,
			callback_group=self._runtime_callback_group,
		)
		self._list_named_poses_service = self.create_service(
			ListNamedPoses,
			str(self.get_parameter('list_named_poses_service_name').value),
			self._handle_list_named_poses,
			callback_group=self._runtime_callback_group,
		)
		self._planning_joint_state_publisher = self.create_publisher(
			JointState,
			str(self.get_parameter('planning_joint_state_topic').value),
			10,
		)
		self._grasp_pose_action_server = ActionServer(
			self,
			MoveToPose,
			str(self.get_parameter('action_name').value),
			execute_callback=self._execute_move_to_pose,
			goal_callback=self._goal_callback,
			cancel_callback=self._cancel_callback,
		)
		self._named_pose_action_server = ActionServer(
			self,
			MoveToNamedPose,
			str(self.get_parameter('named_pose_action_name').value),
			execute_callback=self._execute_move_to_named_pose,
			goal_callback=self._named_pose_goal_callback,
			cancel_callback=self._cancel_callback,
		)
		self._joint_pose_action_server = ActionServer(
			self,
			MoveToJointPose,
			str(self.get_parameter('joint_pose_action_name').value),
			execute_callback=self._execute_move_to_joint_pose,
			goal_callback=self._joint_pose_goal_callback,
			cancel_callback=self._cancel_callback,
		)

		self._sync_workspace_area_from_scene_manager()

		self.get_logger().info(
			f"Motion execution action server ready on {self.get_parameter('action_name').value}"
		)
		self.get_logger().info(
			f"Named-pose action server ready on {self.get_parameter('named_pose_action_name').value}"
		)
		self.get_logger().info(
			f"Joint-pose action server ready on {self.get_parameter('joint_pose_action_name').value}"
		)
		self.get_logger().info(
			f"Named-pose listing service ready on {self.get_parameter('list_named_poses_service_name').value}"
		)
		self.get_logger().info(
			'Nearby IK preference is '
			+ ('enabled' if self._get_bool_parameter('prefer_nearby_ik') else 'disabled')
		)

	def _active_scene_callback(self, message: ActiveScene) -> None:
		"""
		@brief Update the local workspace-area cache whenever the active scene changes.
		"""
		self._update_workspace_area_from_active_scene(message)

	def _handle_list_named_poses(
		self,
		_request: ListNamedPoses.Request,
		response: ListNamedPoses.Response,
	) -> ListNamedPoses.Response:
		"""
		@brief Return configured named poses with semantic descriptions.

		@param _request Empty request.
		@param response Service response to populate.
		@return Response containing configured pose descriptors.
		"""
		response.named_poses = self._configured_named_pose_descriptors()
		return response

	def _sync_workspace_area_from_scene_manager(self) -> None:
		"""
		@brief Bootstrap local workspace-area state from the scene-manager service.
		"""
		if not self._get_active_scene_client.wait_for_service(timeout_sec=2.0):
			self.get_logger().warn('GetActiveScene service not available during startup; waiting for /active_scene updates.')
			self._publish_workspace_area_marker()
			return
		future = self._get_active_scene_client.call_async(GetActiveScene.Request())
		if not self._wait_for_future(future, timeout_sec=5.0) or future.result() is None:
			self.get_logger().warn('GetActiveScene request did not complete during startup; waiting for /active_scene updates.')
			self._publish_workspace_area_marker()
			return
		response = future.result()
		if response.active:
			self._update_workspace_area_from_active_scene(response.active_scene)
		else:
			self.get_logger().info('Scene manager reports no active scene yet; workspace-area filtering is idle.')
			self._publish_workspace_area_marker()

	def _update_workspace_area_from_active_scene(self, active_scene: ActiveScene) -> None:
		"""
		@brief Rebuild the workspace-area geometry cache from an ActiveScene message.
		"""
		self._workspace_area_frame = str(active_scene.workspace_base_frame).strip() or self._planning_frame
		if not bool(active_scene.workspace_area_enabled):
			self._workspace_area = None
			self._publish_workspace_area_marker()
			return

		corner_points = []
		for point in active_scene.workspace_area_corner_points:
			corner_points.append(
				{
					'x': float(point.x),
					'y': float(point.y),
					'z': float(point.z),
				}
			)

		if len(corner_points) != 4:
			self.get_logger().warn('Ignoring active scene workspace area because it does not contain four corner points.')
			self._workspace_area = None
			self._publish_workspace_area_marker()
			return

		self._workspace_area = {
			'geometry': {
				'corner_points': corner_points,
			},
		}
		self._publish_workspace_area_marker()

	def destroy_node(self) -> bool:
		"""
		@brief Destroy the action server before releasing the ROS node.

		@return Result from the base destroy_node implementation.
		"""
		self._grasp_pose_action_server.destroy()
		self._named_pose_action_server.destroy()
		self._joint_pose_action_server.destroy()
		return super().destroy_node()

	def _goal_callback(self, _goal_request: MoveToPose.Goal) -> GoalResponse:
		"""
		@brief Accept all incoming MoveToPose goals.

		@param _goal_request Requested goal payload.
		@return Goal acceptance decision.
		"""
		return GoalResponse.ACCEPT

	def _named_pose_goal_callback(self, goal_request: MoveToNamedPose.Goal) -> GoalResponse:
		"""
		@brief Accept configured-pose goals only when the pose name is known.

		@param goal_request Requested named-pose payload.
		@return Goal acceptance decision.
		"""
		pose_name = str(goal_request.pose_name).strip()
		if not self._configured_pose_exists(pose_name):
			self.get_logger().warn(f"Rejecting unknown configured pose '{pose_name}'.")
			return GoalResponse.REJECT
		return GoalResponse.ACCEPT

	def _joint_pose_goal_callback(self, goal_request: MoveToJointPose.Goal) -> GoalResponse:
		"""
		@brief Accept joint-pose goals only when the joint target is well-formed.

		@param goal_request Requested joint-pose payload.
		@return Goal acceptance decision.
		"""
		joint_state = goal_request.target_joint_state
		joint_names = [str(name).strip() for name in joint_state.name if str(name).strip()]
		if not joint_names:
			self.get_logger().warn('Rejecting joint-pose goal with no joint names.')
			return GoalResponse.REJECT
		if len(joint_names) != len(joint_state.position):
			self.get_logger().warn(
				'Rejecting joint-pose goal because joint_names and joint_positions lengths differ.'
			)
			return GoalResponse.REJECT
		return GoalResponse.ACCEPT

	def _cancel_callback(self, _goal_handle: Any) -> CancelResponse:
		"""
		@brief Accept cancellation for active goals.

		@param _goal_handle Goal handle requesting cancellation.
		@return Cancel acceptance decision.
		"""
		return CancelResponse.ACCEPT

	def _joint_state_callback(self, msg: JointState) -> None:
		"""
		@brief Merge the latest robot joint state into the internal cache for nearby-IK seeding.

		@param msg Latest joint state message.
		"""
		now = self.get_clock().now()
		positions_by_name = self._joint_positions_by_name(msg)
		for joint_name, position in positions_by_name.items():
			self._latest_joint_positions_by_name[joint_name] = float(position)
			self._latest_joint_position_received_at[joint_name] = now

		joint_names = list(self._latest_joint_positions_by_name.keys())
		self._latest_joint_state = self._joint_state_from_positions(
			joint_names,
			self._latest_joint_positions_by_name,
			stamp=now,
		)
		self._latest_joint_state_received_at = now

		planning_joint_names = self._planning_joint_names()
		if all(joint_name in positions_by_name for joint_name in planning_joint_names):
			self._planning_joint_state_publisher.publish(
				self._joint_state_from_positions(
					planning_joint_names,
					positions_by_name,
					stamp=now,
				)
			)

	def _execute_move_to_pose(self, goal_handle: Any) -> MoveToPose.Result:
		"""
		@brief Transform, plan, and execute an incoming pose goal.

		@param goal_handle Active action goal handle.
		@return Action result describing the outcome.
		"""
		feedback = MoveToPose.Feedback()
		request = goal_handle.request
		planning_frame = str(request.planning_frame).strip() or self._planning_frame
		target_frame = str(request.target_frame).strip() or str(self.get_parameter('end_effector_link').value)
		target_pose = PoseStamped()
		target_pose.header.stamp = self.get_clock().now().to_msg()
		target_pose.header.frame_id = planning_frame
		target_pose.pose = request.pose

		try:
			# Clients can send poses in any connected frame. The server normalizes that first,
			# then uses one planning pipeline for supplied grasp poses.
			feedback.state = 'transforming_target_pose'
			goal_handle.publish_feedback(feedback)
			if not str(target_pose.header.frame_id).strip():
				raise RuntimeError('Grasp pose planning_frame must be set or configured in motion_config.')
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
			ok, message = self._move_to_pose(target_pose, target_frame)

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

	def _execute_move_to_named_pose(self, goal_handle: Any) -> MoveToNamedPose.Result:
		"""
		@brief Plan and execute a preconfigured named pose.

		@param goal_handle Active named-pose action goal handle.
		@return Action result describing the outcome.
		"""
		feedback = MoveToNamedPose.Feedback()
		pose_name = str(goal_handle.request.pose_name).strip()

		try:
			feedback.state = 'loading_named_pose'
			goal_handle.publish_feedback(feedback)
			target_pose, target_frame, relax_limits = self._get_named_pose_target(pose_name)

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
			ok, message = self._move_to_pose(target_pose, target_frame, relax_limits)

		except Exception as exc:  # noqa: BLE001
			ok = False
			message = str(exc)

		result = MoveToNamedPose.Result()
		result.success = bool(ok)
		result.message = message

		if ok:
			goal_handle.succeed()
		else:
			goal_handle.abort()
		return result

	def _execute_move_to_joint_pose(self, goal_handle: Any) -> MoveToJointPose.Result:
		"""
		@brief Plan and execute a joint-space arm target through the configured MoveIt scene.

		@param goal_handle Active joint-pose action goal handle.
		@return Action result describing the outcome.
		"""
		feedback = MoveToJointPose.Feedback()
		joint_state = deepcopy(goal_handle.request.target_joint_state)

		try:
			feedback.state = 'validating_joint_target'
			goal_handle.publish_feedback(feedback)
			joint_names = [str(name).strip() for name in joint_state.name if str(name).strip()]
			joint_positions = [float(position) for position in joint_state.position]
			if not joint_names:
				raise RuntimeError('Joint target must include at least one joint name.')
			if len(joint_names) != len(joint_positions):
				raise RuntimeError('Joint target names and positions must have the same length.')

			joint_state.name = joint_names
			joint_state.position = joint_positions
			joint_state.velocity = []
			joint_state.effort = []
			joint_state.header.stamp = self.get_clock().now().to_msg()

			feedback.state = 'planning_and_executing'
			goal_handle.publish_feedback(feedback)
			ok, message = self._move_to_joint_state(
				joint_state,
				self._motion_planning_config(),
			)

		except Exception as exc:  # noqa: BLE001
			ok = False
			message = str(exc)

		result = MoveToJointPose.Result()
		result.success = bool(ok)
		result.message = message

		if ok:
			goal_handle.succeed()
		else:
			goal_handle.abort()
		return result

	def _declare_configured_pose_parameters(self) -> None:
		"""
		@brief Declare pose parameters listed by poses_names or the legacy poses_list list.
		"""
		pose_names = self._configured_pose_names()
		for pose_name in pose_names:
			for parameter_key in self._configured_pose_parameter_keys(pose_name):
				if not self.has_parameter(f'{parameter_key}.pose'):
					self.declare_parameter(f'{parameter_key}.pose', [0.0, 0.0, 0.30, 0.0, 0.0, 0.0])
				if not self.has_parameter(f'{parameter_key}.target_frame'):
					self.declare_parameter(f'{parameter_key}.target_frame', '')
				if not self.has_parameter(f'{parameter_key}.relax_limits'):
					self.declare_parameter(f'{parameter_key}.relax_limits', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
				if not self.has_parameter(f'{parameter_key}.description'):
					self.declare_parameter(f'{parameter_key}.description', '')

		if pose_names:
			self.get_logger().info('Configured motion pose parameters: ' + ', '.join(pose_names))
		else:
			self.get_logger().warn('No configured motion poses listed in poses_names or legacy poses_list.')

	def _declare_workspace_parameters(self) -> None:
		"""
		@brief Declare ROS parameters used to describe the calibrated workspace.
		"""
		self.declare_parameter('workspace.version', 1)
		self.declare_parameter('workspace.updated_at', '')
		self.declare_parameter('workspace.base_frame', self._planning_frame)
		self.declare_parameter('workspace.tool_frame', '')
		self.declare_parameter('workspace.ground_plane_z', 0.0)
		self.declare_parameter('workspace_area.enabled', False)
		self.declare_parameter('workspace_area.geometry.type', '')
		self.declare_parameter('workspace_area.geometry.dimensions', [0.0, 0.0])
		self.declare_parameter('workspace_area.geometry.pose.position', [0.0, 0.0, 0.0])
		self.declare_parameter('workspace_area.geometry.pose.orientation', [0.0, 0.0, 0.0, 1.0])
		self.declare_parameter('workspace_area.geometry.corner_points.x', [0.0, 0.0, 0.0, 0.0])
		self.declare_parameter('workspace_area.geometry.corner_points.y', [0.0, 0.0, 0.0, 0.0])
		self.declare_parameter('workspace_area.geometry.corner_points.z', [0.0, 0.0, 0.0, 0.0])
		self.declare_parameter('workspace_objects', [''])

		for object_name in self._workspace_object_names():
			prefix = f'workspace_object.{object_name}'
			self.declare_parameter(f'{prefix}.geometry.type', '')
			self.declare_parameter(f'{prefix}.geometry.dimensions', [0.0, 0.0, 0.0])
			self.declare_parameter(f'{prefix}.geometry.pose.position', [0.0, 0.0, 0.0])
			self.declare_parameter(f'{prefix}.geometry.pose.orientation', [0.0, 0.0, 0.0, 1.0])
			self.declare_parameter(f'{prefix}.shape', '')
			self.declare_parameter(f'{prefix}.allowed_collision_links', [''])

	def _load_workspace_into_planning_scene(self) -> None:
		"""
		@brief Load persisted workspace obstacles into the MoveIt planning scene.
		"""
		if self._has_workspace_parameter_config():
			self.get_logger().info('Loading workspace config from ROS parameters.')
			workspace_config = self._workspace_config_from_parameters()
		else:
			self.get_logger().warn(
				'No workspace ROS parameters configured; starting with an empty scene.'
			)
			workspace_config = {'workspace_area': None, 'objects': [], 'base_frame': self._planning_frame}

		# The workspace config already contains derived primitive geometry, so startup only needs
		# to translate it into CollisionObjects.
		self._workspace_area_frame = str(workspace_config.get('base_frame', self._planning_frame))
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
		allowed_collision_matrix = self._allowed_collision_matrix_from_workspace(
			workspace_config,
			object_names,
		)
		if allowed_collision_matrix is not None:
			request.scene.allowed_collision_matrix = allowed_collision_matrix

		future = self._planning_scene_client.call_async(request)

		if not self._wait_for_future(future, timeout_sec=10.0) or future.result() is None:
			self.get_logger().warn('ApplyPlanningScene request did not complete.')
			return

		if not future.result().success:
			self.get_logger().warn('MoveIt rejected the workspace planning scene update.')
			return

		self.get_logger().info(f'Applied {len(collision_objects)} workspace objects to the planning scene.')

	def _has_workspace_parameter_config(self) -> bool:
		"""
		@brief Return whether workspace config was loaded as ROS parameters.
		"""
		return self._get_bool_parameter('workspace_area.enabled') or bool(self._workspace_object_names())

	def _workspace_config_from_parameters(self) -> Dict[str, Any]:
		"""
		@brief Reconstruct the runtime workspace config dictionary from ROS parameters.

		@return Workspace configuration in the runtime collision-object shape.
		"""
		workspace_config = workspace_config_from_node_parameters(
			self,
			default_workspace_config(
				self._planning_frame,
				str(self.get_parameter('workspace.tool_frame').value),
				float(self.get_parameter('workspace.ground_plane_z').value),
			),
		)

		valid_object_names = set(self._workspace_object_names())
		workspace_config['objects'] = [
			workspace_object
			for workspace_object in workspace_config.get('objects', [])
			if str(workspace_object.get('name', '')).strip() in valid_object_names
		]
		for object_name in valid_object_names:
			if not any(str(obj.get('name', '')).strip() == object_name for obj in workspace_config['objects']):
				self.get_logger().warn(
					f"Skipping workspace object '{object_name}' because geometry type is empty."
				)

		return workspace_config

	def _allowed_collision_matrix_from_workspace(
		self,
		workspace_config: Dict[str, Any],
		collision_object_names: List[str],
	) -> Optional[AllowedCollisionMatrix]:
		"""
		@brief Append workspace object-link allowances to MoveIt's current collision matrix.

		@param workspace_config Workspace configuration loaded from ROS parameters.
		@param collision_object_names Object ids that were added to the planning scene.
		@return Merged allowed collision matrix, or None when no pairs are configured.
		"""
		pairs = allowed_collision_pairs_from_workspace(workspace_config, collision_object_names)
		if not pairs:
			return None

		matrix = self._current_allowed_collision_matrix()
		if matrix is None:
			self.get_logger().warn(
				'GetPlanningScene service not available; workspace allowed collisions were not applied '
				'to avoid replacing the existing MoveIt allowed-collision matrix.'
			)
			return None

		matrix = append_allowed_collision_pairs(matrix, pairs)

		formatted_pairs = [f'{object_name}<->{link_name}' for object_name, link_name in pairs]
		self.get_logger().info('Appended workspace allowed collisions: ' + ', '.join(formatted_pairs))
		return matrix

	def _current_allowed_collision_matrix(self) -> Optional[AllowedCollisionMatrix]:
		"""
		@brief Fetch MoveIt's current allowed collision matrix.

		@return Current allowed collision matrix, or None when it cannot be fetched.
		"""
		if not self._get_planning_scene_client.wait_for_service(timeout_sec=5.0):
			return None

		request = GetPlanningScene.Request()
		request.components = PlanningSceneComponents()
		request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX

		future = self._get_planning_scene_client.call_async(request)
		if not self._wait_for_future(future, timeout_sec=10.0) or future.result() is None:
			return None
		return future.result().scene.allowed_collision_matrix

	def _workspace_object_names(self) -> List[str]:
		"""
		@brief Return configured workspace object names.
		"""
		return coerce_string_sequence(self.get_parameter('workspace_objects').value)

	def _configured_pose_names(self) -> List[str]:
		"""
		@brief Return the list of configured pose names.

		@return Pose names from the poses_names or legacy poses_list ROS parameter.
		"""
		poses_names = self.get_parameter('poses_names').value
		if isinstance(poses_names, str):
			poses_names = coerce_string_sequence(poses_names)
		if isinstance(poses_names, list):
			configured_names = [str(name).strip() for name in poses_names if str(name).strip()]
			if configured_names:
				return configured_names

		poses_list = self.get_parameter('poses_list').value
		if isinstance(poses_list, str):
			poses_list = coerce_string_sequence(poses_list)
		if not isinstance(poses_list, list):
			return []
		return [str(name).strip() for name in poses_list if str(name).strip()]

	def _configured_pose_exists(self, pose_name: str) -> bool:
		"""
		@brief Check whether a configured pose exists.

		@param pose_name Requested pose name.
		@return True when the pose name is allowed and has pose values.
		"""
		return pose_name in self._configured_pose_names()

	def _configured_named_pose_descriptors(self) -> List[NamedPoseDescriptor]:
		"""
		@brief Build descriptor messages for every configured named pose.

		@return Ordered list of pose descriptors.
		"""
		descriptors: List[NamedPoseDescriptor] = []
		for pose_name in self._configured_pose_names():
			parameter_key = self._configured_pose_parameter_key(pose_name)
			descriptor = NamedPoseDescriptor()
			descriptor.pose_name = pose_name
			descriptor.description = str(
				self.get_parameter(f'{parameter_key}.description').value
			).strip()
			descriptors.append(descriptor)
		return descriptors

	def _configured_pose_parameter_keys(self, pose_name: str) -> List[str]:
		"""
		@brief Return parameter key variants for a configured pose name.

		@param pose_name Name from poses_names or the legacy poses_list list.
		@return Candidate parameter prefixes in the poses_values layout plus legacy forms.
		"""
		source_names = [pose_name]
		if pose_name.endswith('_pose'):
			source_names.append(pose_name.removesuffix('_pose'))

		parameter_keys: List[str] = []
		for source_name in source_names:
			normalized_name = str(source_name).strip()
			if not normalized_name:
				continue
			parameter_keys.append(f'poses_values.{normalized_name}')
			parameter_keys.append(f'poses_list.{normalized_name}')
			parameter_keys.append(normalized_name)

		unique_parameter_keys: List[str] = []
		for parameter_key in parameter_keys:
			if parameter_key not in unique_parameter_keys:
				unique_parameter_keys.append(parameter_key)
		return unique_parameter_keys

	def _configured_pose_parameter_key(self, pose_name: str) -> str:
		"""
		@brief Return the parameter prefix containing the configured pose data.

		@param pose_name Name from poses_names or the legacy poses_list list.
		@return Structured parameter prefix for pose data.
		"""
		default_pose = [0.0, 0.0, 0.30, 0.0, 0.0, 0.0]
		for parameter_key in self._configured_pose_parameter_keys(pose_name):
			target_frame = str(self.get_parameter(f'{parameter_key}.target_frame').value).strip()
			pose_values = coerce_float_sequence(
				self.get_parameter(f'{parameter_key}.pose').value,
				6,
				f'{parameter_key}.pose',
			)
			if target_frame or pose_values != default_pose:
				return parameter_key
		return self._configured_pose_parameter_keys(pose_name)[0]

	def _get_named_pose_target(self, pose_name: str) -> tuple[PoseStamped, str, List[float]]:
		"""
		@brief Return a configured named pose and the frame that should reach it.

		@param pose_name Name from motion_config.yaml.
		@return PoseStamped in the planning frame, plus target frame and relax limits.
		"""
		if not self._configured_pose_exists(pose_name):
			raise RuntimeError(
				f"Unknown configured pose '{pose_name}'. Available poses: {', '.join(self._configured_pose_names())}"
			)

		parameter_key = self._configured_pose_parameter_key(pose_name)
		pose_values = coerce_float_sequence(
			self.get_parameter(f'{parameter_key}.pose').value,
			6,
			f'{parameter_key}.pose',
		)
		target_frame = str(self.get_parameter(f'{parameter_key}.target_frame').value).strip()
		if not target_frame:
			target_frame = str(self.get_parameter('end_effector_link').value)
		relax_limits = coerce_float_sequence(
			self.get_parameter(f'{parameter_key}.relax_limits').value,
			6,
			f'{parameter_key}.relax_limits',
		)

		return self._pose_stamped_from_values(self._planning_frame, pose_values), target_frame, relax_limits

	def _pose_stamped_from_values(self, frame: str, pose_values: List[float]) -> PoseStamped:
		"""
		@brief Convert [x, y, z, roll, pitch, yaw] values into a PoseStamped.

		@param frame Frame id for the output pose.
		@param pose_values Six pose values.
		@return PoseStamped in the requested frame.
		"""
		pose_stamped = PoseStamped()
		pose_stamped.header.stamp = self.get_clock().now().to_msg()
		pose_stamped.header.frame_id = frame
		pose_stamped.pose.position.x = pose_values[0]
		pose_stamped.pose.position.y = pose_values[1]
		pose_stamped.pose.position.z = pose_values[2]
		orientation = quaternion_from_rpy(pose_values[3], pose_values[4], pose_values[5])
		pose_stamped.pose.orientation.x = orientation.x
		pose_stamped.pose.orientation.y = orientation.y
		pose_stamped.pose.orientation.z = orientation.z
		pose_stamped.pose.orientation.w = orientation.w
		return pose_stamped

	def _default_pose_relax_limits(self) -> List[float]:
		"""
		@brief Return the default per-axis pose relax limits.

		@return [x, y, z, roll, pitch, yaw] search limits.
		"""
		return coerce_float_sequence(
			self.get_parameter('pose_relax_limits').value,
			6,
			'pose_relax_limits',
		)

	def _get_bool_parameter(self, name: str) -> bool:
		"""
		@brief Read a boolean ROS parameter that may arrive as a string.

		@param name Parameter name.
		@return Boolean parameter value.
		"""
		value = self.get_parameter(name).value
		if isinstance(value, str):
			return value.strip().lower() in {'1', 'true', 'yes', 'on'}
		return bool(value)

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

	def _move_to_pose(
		self,
		target_pose: PoseStamped,
		target_frame: Optional[str] = None,
		relax_limits: Optional[List[float]] = None,
	) -> tuple[bool, str]:
		"""
		@brief Send a MoveGroup action goal for the requested target pose.

		@param target_pose Goal pose already expressed in the planning frame.
		@param target_frame Robot frame/link that should reach the target pose.
		@param relax_limits Optional [x, y, z, roll, pitch, yaw] search limits.
		@return Tuple of success flag and status message.
		"""
		action_name = str(self.get_parameter('move_group_action_name').value)
		if not self._movegroup_client.wait_for_server(timeout_sec=5.0):
			return False, f"MoveGroup action server '{action_name}' not available."

		# The custom action stays thin and delegates actual motion execution to MoveIt so the
		# rest of the system can talk to one stable arm-control interface.
		planning_config = self._motion_planning_config()
		ok, message = self._try_pose_goal(target_pose, target_frame, planning_config)
		if ok:
			return ok, message

		recovery_ok, recovery_message = self._try_grasp_pose_recovery(
			target_pose,
			target_frame,
			planning_config,
		)
		if recovery_ok:
			return True, recovery_message

		effective_relax_limits = list(relax_limits) if relax_limits is not None else self._default_pose_relax_limits()
		if not self._get_bool_parameter('pose_relax_search_enabled') or not any(limit > 0.0 for limit in effective_relax_limits):
			return ok, message

		relaxed_ok, relaxed_message = self._search_relaxed_pose_candidates(
			target_pose,
			target_frame,
			planning_config,
			effective_relax_limits,
		)
		if relaxed_ok:
			return True, relaxed_message
		return False, message + ' Relaxed pose search exhausted within configured relax limits.'

	def _try_pose_goal(
		self,
		target_pose: PoseStamped,
		target_frame: Optional[str],
		planning_config: MotionPlanningConfig,
	) -> tuple[bool, str]:
		return self._evaluate_pose_goal(target_pose, target_frame, planning_config, plan_only=False)

	def _plan_pose_goal(
		self,
		target_pose: PoseStamped,
		target_frame: Optional[str],
		planning_config: MotionPlanningConfig,
	) -> tuple[bool, str]:
		"""
		@brief Check whether one pose goal is plan-feasible without executing it.

		@param target_pose Goal pose already expressed in the planning frame.
		@param target_frame Robot frame/link that should reach the target pose.
		@param planning_config Planning settings snapshot.
		@return Tuple of success flag and status message.
		"""
		return self._evaluate_pose_goal(target_pose, target_frame, planning_config, plan_only=True)

	def _evaluate_pose_goal(
		self,
		target_pose: PoseStamped,
		target_frame: Optional[str],
		planning_config: MotionPlanningConfig,
		plan_only: bool,
	) -> tuple[bool, str]:
		"""
		@brief Try one pose goal via nearby IK then pose-constrained planning.

		@param target_pose Goal pose already expressed in the planning frame.
		@param target_frame Robot frame/link that should reach the target pose.
		@param planning_config Planning settings snapshot.
		@param plan_only When True, validate planning without executing the trajectory.
		@return Tuple of success flag and status message.
		"""
		fallback_context: Optional[str] = None

		goal: MoveGroup.Goal
		if self._get_bool_parameter('prefer_nearby_ik'):
			ik_ok, ik_payload, ik_message = self._joint_goal_from_nearby_ik(target_pose, target_frame)
			if ik_ok:
				goal = build_joint_move_group_goal(
					ik_payload['joint_state'],
					planning_config,
					target_frame,
					ik_payload['start_state'],
					plan_only=plan_only,
				)
			else:
				if not self._get_bool_parameter('fallback_to_pose_planning_on_ik_failure'):
					return False, ik_message
				fallback_context = ik_message
				self.get_logger().warn(
					ik_message + ' Falling back to pose-constrained planning request.'
				)
				goal = build_move_group_goal(
					target_pose,
					planning_config,
					target_frame,
					self._current_robot_state_or_none(),
					plan_only=plan_only,
				)
		else:
			goal = build_move_group_goal(
				target_pose,
				planning_config,
				target_frame,
				self._current_robot_state_or_none(),
				plan_only=plan_only,
			)

		ok, message = self._execute_move_group_goal(goal)
		if not ok and fallback_context is not None:
			return False, fallback_context + ' Fallback pose-constrained planning also failed: ' + message
		return ok, message

	def _try_grasp_pose_recovery(
		self,
		target_pose: PoseStamped,
		target_frame: Optional[str],
		planning_config: MotionPlanningConfig,
	) -> tuple[bool, str]:
		"""
		@brief Recover a failed TCP grasp by probing tool-tip and interpolated offsets.

		@param target_pose Original requested grasp pose.
		@param target_frame Robot frame/link that should reach the target pose.
		@param planning_config Planning settings snapshot.
		@return Tuple of success flag and status message.
		"""
		frames = self._grasp_pose_recovery_frames(target_frame)
		if frames is None:
			return False, ''

		primary_frame, recovery_frame = frames
		frame_offset = self._lookup_recovery_frame_offset(primary_frame, recovery_frame)
		if frame_offset is None:
			return False, ''

		tool_tip_ok, tool_tip_message = self._plan_pose_goal(target_pose, recovery_frame, planning_config)
		if not tool_tip_ok:
			return False, tool_tip_message

		attempts = max(0, int(self.get_parameter('grasp_pose_recovery_recalculate_attempts').value))
		fraction = 0.5
		last_message = tool_tip_message
		for attempt_index in range(attempts):
			candidate_pose = self._pose_with_recovery_offset(target_pose, frame_offset, fraction)
			ok, message = self._try_pose_goal(candidate_pose, primary_frame, planning_config)
			if ok:
				return True, (
					message
					+ f' using grasp recovery between {primary_frame} and {recovery_frame} '
					+ f'(attempt {attempt_index + 1}/{attempts}, fraction={fraction:.3f}).'
				)
			last_message = message
			fraction = 1.0 - ((1.0 - fraction) * 0.5)

		ok, message = self._try_pose_goal(target_pose, recovery_frame, planning_config)
		if ok:
			return True, message + f' using grasp recovery target frame {recovery_frame}.'
		return False, last_message or message

	def _grasp_pose_recovery_frames(self, target_frame: Optional[str]) -> Optional[tuple[str, str]]:
		"""
		@brief Resolve the primary and recovery grasp frames for TCP fallback.

		@param target_frame Requested constrained link.
		@return (primary_frame, recovery_frame) when recovery should run.
		"""
		if not self._get_bool_parameter('grasp_pose_recovery_enabled'):
			return None

		primary_frame = str(target_frame or self.get_parameter('end_effector_link').value).strip()
		recovery_frame = str(self.get_parameter('grasp_pose_recovery_tool_frame').value).strip()
		configured_primary = str(self.get_parameter('end_effector_link').value).strip()
		if not primary_frame or not recovery_frame or primary_frame == recovery_frame:
			return None
		if primary_frame != configured_primary:
			return None
		return primary_frame, recovery_frame

	def _lookup_recovery_frame_offset(
		self,
		primary_frame: str,
		recovery_frame: str,
	) -> Optional[tuple[float, float, float]]:
		"""
		@brief Look up the recovery-frame origin expressed in the primary-frame coordinates.

		@param primary_frame Link used for the original grasp target.
		@param recovery_frame Alternate tool frame for fallback.
		@return XYZ offset tuple in the primary-frame basis, or None when unavailable.
		"""
		try:
			transform = self._tf_buffer.lookup_transform(
				primary_frame,
				recovery_frame,
				rclpy.time.Time(),
				timeout=rclpy.duration.Duration(seconds=1.0),
			)
		except Exception as exc:  # noqa: BLE001
			self.get_logger().warn(
				f'Grasp pose recovery skipped because TF lookup from {primary_frame} to {recovery_frame} failed: {exc}'
			)
			return None

		offset = transform.transform.translation
		if abs(offset.x) < 1e-6 and abs(offset.y) < 1e-6 and abs(offset.z) < 1e-6:
			return None
		return float(offset.x), float(offset.y), float(offset.z)

	def _pose_with_recovery_offset(
		self,
		target_pose: PoseStamped,
		frame_offset: tuple[float, float, float],
		fraction: float,
	) -> PoseStamped:
		"""
		@brief Shift a TCP pose so an interpolated tool point reaches the original target.

		@param target_pose Original requested target pose.
		@param frame_offset Recovery-frame origin expressed in the primary frame.
		@param fraction Interpolation factor from the primary frame toward the recovery frame.
		@return Adjusted pose that keeps orientation and backs off along the tool axis.
		"""
		adjusted_pose = deepcopy(target_pose)
		rotated_offset = rotate_vector_by_quaternion(
			(
				frame_offset[0] * fraction,
				frame_offset[1] * fraction,
				frame_offset[2] * fraction,
			),
			Quaternion(
				target_pose.pose.orientation.x,
				target_pose.pose.orientation.y,
				target_pose.pose.orientation.z,
				target_pose.pose.orientation.w,
			),
		)
		adjusted_pose.pose.position.x -= rotated_offset[0]
		adjusted_pose.pose.position.y -= rotated_offset[1]
		adjusted_pose.pose.position.z -= rotated_offset[2]
		return adjusted_pose

	def _search_relaxed_pose_candidates(
		self,
		target_pose: PoseStamped,
		target_frame: Optional[str],
		planning_config: MotionPlanningConfig,
		relax_limits: List[float],
	) -> tuple[bool, str]:
		"""
		@brief Try nearby pose variants within configured XYZ/RPY relax limits.

		@param target_pose Original exact pose expressed in the planning frame.
		@param target_frame Robot frame/link that should reach the target pose.
		@param planning_config Planning settings snapshot.
		@param relax_limits [x, y, z, roll, pitch, yaw] search limits.
		@return Tuple of success flag and message.
		"""
		last_message = 'No relaxed pose candidate succeeded.'
		for candidate_pose, summary in self._iter_relaxed_pose_candidates(target_pose, relax_limits):
			if not self._target_pose_in_workspace_area(candidate_pose):
				continue
			ok, message = self._try_pose_goal(candidate_pose, target_frame, planning_config)
			if ok:
				return True, message + f' using relaxed pose offsets ({summary}).'
			last_message = message
		return False, last_message

	def _iter_relaxed_pose_candidates(
		self,
		target_pose: PoseStamped,
		relax_limits: List[float],
	) -> List[tuple[PoseStamped, str]]:
		"""
		@brief Generate deterministic nearby XYZ/RPY pose candidates.

		@param target_pose Original exact pose in the planning frame.
		@param relax_limits [x, y, z, roll, pitch, yaw] search limits.
		@return Ordered candidate poses plus a short offset summary.
		"""
		roll, pitch, yaw = quaternion_to_rpy(
			target_pose.pose.orientation.x,
			target_pose.pose.orientation.y,
			target_pose.pose.orientation.z,
			target_pose.pose.orientation.w,
		)
		base_values = [
			float(target_pose.pose.position.x),
			float(target_pose.pose.position.y),
			float(target_pose.pose.position.z),
			roll,
			pitch,
			yaw,
		]
		axis_names = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
		axis_order = (2, 0, 1, 3, 4, 5)
		seen: set[tuple[float, ...]] = set()
		candidates: List[tuple[PoseStamped, str]] = []

		for fraction in (0.5, 1.0):
			for axis in axis_order:
				limit = abs(float(relax_limits[axis]))
				if limit <= 0.0:
					continue
				step = limit * fraction
				for direction in (1.0, -1.0):
					candidate_values = list(base_values)
					candidate_values[axis] += direction * step
					key = tuple(round(value, 6) for value in candidate_values)
					if key in seen:
						continue
					seen.add(key)
					candidates.append(
						(
							self._pose_stamped_from_values(target_pose.header.frame_id, candidate_values),
							f'{axis_names[axis]}={direction * step:+.3f}',
						)
					)
		return candidates

	def _move_to_joint_state(
		self,
		target_joint_state: JointState,
		planning_config: MotionPlanningConfig,
	) -> tuple[bool, str]:
		"""
		@brief Send a MoveGroup action goal for the requested joint target.

		@param target_joint_state Joint target for the planning group.
		@param planning_config Request-specific planning configuration.
		@return Tuple of success flag and status message.
		"""
		action_name = str(self.get_parameter('move_group_action_name').value)
		if not self._movegroup_client.wait_for_server(timeout_sec=5.0):
			return False, f"MoveGroup action server '{action_name}' not available."

		goal = build_joint_move_group_goal(
			target_joint_state,
			planning_config,
			None,
			self._current_robot_state_or_none(),
		)
		return self._execute_move_group_goal(goal)

	def _execute_move_group_goal(self, goal: MoveGroup.Goal) -> tuple[bool, str]:
		"""
		@brief Send a prepared MoveGroup goal and wait for the final result.

		@param goal Fully configured MoveGroup goal.
		@return Tuple of success flag and status message.
		"""
		send_future = self._movegroup_client.send_goal_async(goal)
		if not self._wait_for_future(send_future, timeout_sec=10.0) or send_future.result() is None:
			return False, 'Failed to send MoveGroup goal.'

		goal_handle = send_future.result()
		if not goal_handle.accepted:
			return False, 'MoveGroup goal was rejected.'

		result_future = goal_handle.get_result_async()
		if not self._wait_for_future(result_future, timeout_sec=60.0) or result_future.result() is None:
			return False, 'MoveGroup result not received.'

		result = result_future.result().result
		if result.error_code.val != MoveItErrorCodes.SUCCESS:
			return (
				False,
				'MoveGroup failed with '
				+ self._describe_moveit_error_code(result.error_code.val),
			)

		return True, 'Arm motion completed successfully.'

	def _joint_goal_from_nearby_ik(
		self,
		target_pose: PoseStamped,
		target_frame: Optional[str],
	) -> tuple[bool, Dict[str, Any], str]:
		"""
		@brief Compute a nearby IK solution and convert it into a joint goal.

		@param target_pose Goal pose already expressed in the planning frame.
		@param target_frame Robot frame/link that should reach the target pose.
		@return Success flag, payload with current/joint target state, and status message.
		"""
		current_joint_state, state_message = self._current_planning_joint_state()
		if current_joint_state is None:
			return False, {}, 'Nearby IK unavailable: ' + state_message

		start_state = robot_state_from_joint_state(current_joint_state)
		ik_solution, ik_message = self._compute_nearby_ik_solution(target_pose, target_frame, start_state)
		if ik_solution is None:
			return False, {}, ik_message

		target_joint_state = self._planning_joint_state_from_robot_state(ik_solution)
		if target_joint_state is None:
			return (
				False,
				{},
				'Nearby IK returned an incomplete joint solution for planning joints '
				+ ', '.join(self._planning_joint_names())
			)

		target_joint_state = self._unwrap_joint_state_near_current(target_joint_state, current_joint_state)
		self._log_joint_goal_deltas(current_joint_state, target_joint_state)
		return True, {
			'joint_state': target_joint_state,
			'start_state': start_state,
		}, 'Nearby IK selected a joint-space goal.'

	def _current_planning_joint_state(self) -> tuple[Optional[JointState], str]:
		"""
		@brief Return the latest fresh JointState restricted to the configured planning joints.

		@return Planning-joint JointState plus a status message.
		"""
		if not self._latest_joint_positions_by_name:
			return None, 'no /joint_states message has been received yet.'

		timeout_sec = float(self.get_parameter('joint_state_timeout_sec').value)
		now = self.get_clock().now()
		positions_by_name = self._latest_joint_positions_by_name
		planning_joint_names = self._planning_joint_names()
		missing_joint_names = [name for name in planning_joint_names if name not in positions_by_name]
		if missing_joint_names:
			return None, 'latest /joint_states message is missing planning joints: ' + ', '.join(missing_joint_names)

		if timeout_sec > 0.0:
			stale_joint_names = []
			oldest_age_sec = 0.0
			for joint_name in planning_joint_names:
				received_at = self._latest_joint_position_received_at.get(joint_name)
				if received_at is None:
					stale_joint_names.append(joint_name)
					continue
				age_sec = (now - received_at).nanoseconds / 1e9
				oldest_age_sec = max(oldest_age_sec, age_sec)
				if age_sec > timeout_sec:
					stale_joint_names.append(joint_name)
			if stale_joint_names:
				return None, (
					'latest planning joint state is stale '
					f'({oldest_age_sec:.3f}s oldest sample, timeout {timeout_sec:.3f}s) for joints: '
					+ ', '.join(stale_joint_names)
				)

		return self._joint_state_from_positions(planning_joint_names, positions_by_name), 'ok'

	def _current_robot_state_or_none(self) -> Optional[RobotState]:
		"""
		@brief Return the current planning-joint state wrapped as a RobotState when available.

		@return Current RobotState, or None when fresh joint data is unavailable.
		"""
		current_joint_state, _ = self._current_planning_joint_state()
		if current_joint_state is None:
			return None
		return robot_state_from_joint_state(current_joint_state)

	def _compute_nearby_ik_solution(
		self,
		target_pose: PoseStamped,
		target_frame: Optional[str],
		start_state: RobotState,
	) -> tuple[Optional[RobotState], str]:
		"""
		@brief Request one IK solution seeded with the current planning-joint state.

		@param target_pose Goal pose already expressed in the planning frame.
		@param target_frame Robot frame/link that should reach the target pose.
		@param start_state Current robot state used to seed IK.
		@return IK solution RobotState plus a status message.
		"""
		service_name = str(self.get_parameter('compute_ik_service').value)
		if not self._compute_ik_client.wait_for_service(timeout_sec=2.0):
			return None, f"GetPositionIK service '{service_name}' is not available."

		request = GetPositionIK.Request()
		request.ik_request.group_name = str(self.get_parameter('planning_group').value)
		request.ik_request.robot_state = start_state
		request.ik_request.avoid_collisions = True
		request.ik_request.ik_link_name = str(target_frame or self.get_parameter('end_effector_link').value)
		request.ik_request.pose_stamped = target_pose
		request.ik_request.timeout = rclpy.duration.Duration(
			seconds=float(self.get_parameter('ik_timeout_sec').value)
		).to_msg()

		future = self._compute_ik_client.call_async(request)
		if not self._wait_for_future(future, timeout_sec=5.0) or future.result() is None:
			return None, 'Nearby IK request did not complete before the client timeout.'

		response = future.result()
		if response.error_code.val != MoveItErrorCodes.SUCCESS:
			return (
				None,
				'Nearby IK failed with '
				+ self._describe_moveit_error_code(response.error_code.val)
				+ f" for group '{request.ik_request.group_name}' and link '{request.ik_request.ik_link_name}'.",
			)
		return response.solution, 'ok'

	@staticmethod
	def _wait_for_future(future: Any, timeout_sec: float) -> bool:
		if future.done():
			return True

		done_event = threading.Event()

		def _mark_done(_: Any) -> None:
			done_event.set()

		future.add_done_callback(_mark_done)
		done_event.wait(timeout_sec)
		return future.done()

	def _planning_joint_state_from_robot_state(self, robot_state: RobotState) -> Optional[JointState]:
		"""
		@brief Extract planning joints from a MoveIt RobotState.

		@param robot_state MoveIt RobotState returned by IK.
		@return JointState ordered by planning_joint_names, or None when incomplete.
		"""
		positions_by_name = self._joint_positions_by_name(robot_state.joint_state)
		planning_joint_names = self._planning_joint_names()
		missing_joint_names = [name for name in planning_joint_names if name not in positions_by_name]
		if missing_joint_names:
			self.get_logger().warn(
				'IK solution is missing planning joints: ' + ', '.join(missing_joint_names)
			)
			return None

		return self._joint_state_from_positions(planning_joint_names, positions_by_name)

	def _unwrap_joint_state_near_current(
		self,
		target_joint_state: JointState,
		current_joint_state: JointState,
	) -> JointState:
		"""
		@brief Shift target joint angles by whole turns to keep them near the current branch.

		@param target_joint_state IK result for the planning joints.
		@param current_joint_state Latest planning-joint state.
		@return Unwrapped joint target near the current configuration.
		"""
		current_positions_by_name = self._joint_positions_by_name(current_joint_state)
		unwrapped_positions: List[float] = []
		for joint_name, target_position in zip(target_joint_state.name, target_joint_state.position):
			reference_position = current_positions_by_name.get(str(joint_name), float(target_position))
			unwrapped_positions.append(
				nearest_equivalent_angle(float(target_position), reference_position)
			)

		unwrapped_joint_state = JointState()
		unwrapped_joint_state.header = target_joint_state.header
		unwrapped_joint_state.name = list(target_joint_state.name)
		unwrapped_joint_state.position = unwrapped_positions
		return unwrapped_joint_state

	def _log_joint_goal_deltas(self, current_joint_state: JointState, target_joint_state: JointState) -> None:
		"""
		@brief Optionally log per-joint deltas for nearby IK debugging.

		@param current_joint_state Latest planning-joint state.
		@param target_joint_state Unwrapped joint-space goal.
		"""
		if not self._get_bool_parameter('log_joint_goal_deltas'):
			return

		current_positions_by_name = self._joint_positions_by_name(current_joint_state)
		deltas = []
		for joint_name, target_position in zip(target_joint_state.name, target_joint_state.position):
			delta = float(target_position) - current_positions_by_name.get(str(joint_name), 0.0)
			deltas.append(f'{joint_name}={delta:.4f} rad')
		self.get_logger().info('Nearby IK joint deltas: ' + ', '.join(deltas))

	def _planning_joint_names(self) -> List[str]:
		"""
		@brief Return the ordered planning-joint list used for nearby IK and joint goals.

		@return Planning joint names.
		"""
		return coerce_string_sequence(self.get_parameter('planning_joint_names').value)

	def _joint_positions_by_name(self, joint_state: JointState) -> Dict[str, float]:
		"""
		@brief Convert a JointState message into a name-to-position mapping.

		@param joint_state Joint state message.
		@return Joint positions keyed by joint name.
		"""
		return {
			str(name): float(position)
			for name, position in zip(joint_state.name, joint_state.position)
		}

	def _joint_state_from_positions(
		self,
		joint_names: List[str],
		positions_by_name: Dict[str, float],
		stamp: Optional[Time] = None,
	) -> JointState:
		"""
		@brief Build a stamped JointState from ordered joint names and a position mapping.

		@param joint_names Ordered joint names.
		@param positions_by_name Joint positions keyed by joint name.
		@param stamp Optional timestamp to write into the message header.
		@return Stamped JointState in the requested order.
		"""
		joint_state = JointState()
		joint_state.header.stamp = (stamp or self.get_clock().now()).to_msg()
		joint_state.name = list(joint_names)
		joint_state.position = [positions_by_name[name] for name in joint_names]
		return joint_state

	@staticmethod
	def _describe_moveit_error_code(error_code: int) -> str:
		"""
		@brief Format a MoveIt error code as name plus numeric value.

		@param error_code Numeric MoveIt error code.
		@return Human-readable error code description.
		"""
		for attribute_name, attribute_value in MoveItErrorCodes.__dict__.items():
			if attribute_name.isupper() and attribute_value == error_code:
				return f'{attribute_name} ({error_code})'
		return f'error code {error_code}'

	def _motion_planning_config(self) -> MotionPlanningConfig:
		"""
		@brief Snapshot the ROS planning parameters used to build MoveIt requests.

		@return Immutable motion planning configuration.
		"""
		return planning_config_from_node(self, self._planning_frame)


def main(args: Optional[List[str]] = None) -> None:
	"""
	@brief Run the arm control node until shutdown.

	@param args Optional ROS command-line arguments.
	"""
	rclpy.init(args=args)
	node = MotionExecutionNode()
	executor = MultiThreadedExecutor(num_threads=2)
	executor.add_node(node)
	try:
		executor.spin()
	finally:
		executor.remove_node(node)
		executor.shutdown()
		node.destroy_node()
		rclpy.shutdown()
