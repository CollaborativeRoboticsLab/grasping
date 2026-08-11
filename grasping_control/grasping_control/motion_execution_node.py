from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
from typing import Any, Dict, List, Optional

from geometry_msgs.msg import Point, PoseStamped
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from sensor_msgs.msg import JointState

from grasping_control.common import (
	coerce_float_sequence,
	coerce_string_sequence,
	nearest_equivalent_angle,
	quaternion_from_rpy,
	transform_pose_to_frame,
)
from grasping_control.motion_utils import (
	MotionPlanningConfig,
	allowed_collision_pairs_from_workspace,
	append_allowed_collision_pairs,
	build_joint_move_group_goal,
	build_move_group_goal,
	robot_state_from_joint_state,
)
from grasping_control.workspace_utils import (
	collision_objects_from_workspace,
	default_workspace_config,
	point_in_workspace_area,
	workspace_config_from_node_parameters,
)
from grasping_msgs.action import MoveToNamedPose, MoveToPose
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
		self.declare_parameter('planning_frame', 'base_link')
		self.declare_parameter('planning_pipeline_id', '')
		self.declare_parameter('planner_id', '')
		self.declare_parameter('allowed_planning_time', 5.0)
		self.declare_parameter('num_planning_attempts', 5)
		self.declare_parameter('max_velocity_scaling', 0.2)
		self.declare_parameter('max_acceleration_scaling', 0.2)
		self.declare_parameter('enable_cartesian_vel_limit', False)
		self.declare_parameter('max_cartesian_velocity', [0.0, 0.0, 0.0])
		self.declare_parameter('position_tolerance_m', 0.005)
		self.declare_parameter('orientation_tolerance_rad', 0.1)
		self.declare_parameter('end_effector_link', 'tool0')
		self.declare_parameter('named_pose_action_name', 'move_arm_to_named_pose')
		self.declare_parameter('poses_names', ['workspace_center', 'pre_grasp', 'post_grasp'])
		self.declare_parameter('poses_list', [])
		self.declare_parameter('apply_planning_scene_service', '/apply_planning_scene')
		self.declare_parameter('get_planning_scene_service', '/get_planning_scene')
		self.declare_parameter('compute_ik_service', '/compute_ik')
		self.declare_parameter('joint_state_topic', '/joint_states')
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
		self.declare_parameter('joint_state_timeout_sec', 0.5)
		self.declare_parameter('ik_timeout_sec', 0.2)
		self.declare_parameter('joint_goal_tolerance_rad', 0.001)
		self.declare_parameter('log_joint_goal_deltas', False)
		self.declare_parameter('workspace_area_marker_topic', '/workspace_area_marker')

		self._planning_frame = str(self.get_parameter('planning_frame').value)
		self._latest_joint_state: Optional[JointState] = None
		self._latest_joint_state_received_at: Optional[Time] = None
		self._declare_workspace_parameters()
		self._workspace_area: Optional[Dict[str, Any]] = None
		self._workspace_area_frame = self._planning_frame
		self._declare_configured_pose_parameters()

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
		self._get_planning_scene_client = self.create_client(
			GetPlanningScene,
			str(self.get_parameter('get_planning_scene_service').value),
		)
		self._compute_ik_client = self.create_client(
			GetPositionIK,
			str(self.get_parameter('compute_ik_service').value),
		)
		self._joint_state_subscription = self.create_subscription(
			JointState,
			str(self.get_parameter('joint_state_topic').value),
			self._joint_state_callback,
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

		# Load static workspace obstacles once at startup so every later arm action is planned
		# against the calibrated scene written by workspace_creation_node.py.
		self._load_workspace_into_planning_scene()

		self.get_logger().info(
			f"Motion execution action server ready on {self.get_parameter('action_name').value}"
		)
		self.get_logger().info(
			f"Named-pose action server ready on {self.get_parameter('named_pose_action_name').value}"
		)
		self.get_logger().info(
			'Nearby IK preference is '
			+ ('enabled' if self._get_bool_parameter('prefer_nearby_ik') else 'disabled')
		)

	def destroy_node(self) -> bool:
		"""
		@brief Destroy the action server before releasing the ROS node.

		@return Result from the base destroy_node implementation.
		"""
		self._grasp_pose_action_server.destroy()
		self._named_pose_action_server.destroy()
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

	def _cancel_callback(self, _goal_handle: Any) -> CancelResponse:
		"""
		@brief Accept cancellation for active goals.

		@param _goal_handle Goal handle requesting cancellation.
		@return Cancel acceptance decision.
		"""
		return CancelResponse.ACCEPT

	def _joint_state_callback(self, msg: JointState) -> None:
		"""
		@brief Cache the latest robot joint state for nearby-IK seeding.

		@param msg Latest joint state message.
		"""
		self._latest_joint_state = deepcopy(msg)
		self._latest_joint_state_received_at = self.get_clock().now()

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
			# then uses one planning pipeline for supplied grasp poses.
			feedback.state = 'transforming_target_pose'
			goal_handle.publish_feedback(feedback)
			if not str(target_pose.header.frame_id).strip():
				raise RuntimeError('Grasp pose target_pose.header.frame_id must be set.')
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
			target_pose, target_frame = self._get_named_pose_target(pose_name)

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
			ok, message = self._move_to_pose(target_pose, target_frame)

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

		rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
		if not future.done() or future.result() is None:
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
		rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
		if not future.done() or future.result() is None:
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

	def _get_named_pose_target(self, pose_name: str) -> tuple[PoseStamped, str]:
		"""
		@brief Return a configured named pose and the frame that should reach it.

		@param pose_name Name from motion_config.yaml.
		@return PoseStamped in the workspace area frame or planning frame, plus target frame.
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

		return self._pose_stamped_from_values(self._planning_frame, pose_values), target_frame

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

	def _move_to_pose(self, target_pose: PoseStamped, target_frame: Optional[str] = None) -> tuple[bool, str]:
		"""
		@brief Send a MoveGroup action goal for the requested target pose.

		@param target_pose Goal pose already expressed in the planning frame.
		@param target_frame Robot frame/link that should reach the target pose.
		@return Tuple of success flag and status message.
		"""
		action_name = str(self.get_parameter('move_group_action_name').value)
		if not self._movegroup_client.wait_for_server(timeout_sec=5.0):
			return False, f"MoveGroup action server '{action_name}' not available."

		# The custom action stays thin and delegates actual motion execution to MoveIt so the
		# rest of the system can talk to one stable arm-control interface.
		planning_config, config_error = self._motion_planning_config_for_target(
			target_pose,
			target_frame,
		)
		if planning_config is None:
			return False, config_error

		goal: MoveGroup.Goal
		if self._get_bool_parameter('prefer_nearby_ik'):
			ik_ok, ik_payload, ik_message = self._joint_goal_from_nearby_ik(target_pose, target_frame)
			if ik_ok:
				goal = build_joint_move_group_goal(
					ik_payload['joint_state'],
					planning_config,
					target_frame,
					ik_payload['start_state'],
				)
			else:
				if not self._get_bool_parameter('fallback_to_pose_planning_on_ik_failure'):
					return False, ik_message
				self.get_logger().warn(
					ik_message + ' Falling back to pose-constrained planning request.'
				)
				goal = self._build_move_group_goal(
					target_pose,
					planning_config,
					target_frame,
					self._current_robot_state_or_none(),
				)
		else:
			goal = self._build_move_group_goal(
				target_pose,
				planning_config,
				target_frame,
				self._current_robot_state_or_none(),
			)

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
			return (
				False,
				'MoveGroup failed with '
				+ self._describe_moveit_error_code(result.error_code.val),
			)

		return True, 'Arm motion completed successfully.'

	def _build_move_group_goal(
		self,
		target_pose: PoseStamped,
		planning_config: MotionPlanningConfig,
		target_frame: Optional[str] = None,
		start_state: Optional[RobotState] = None,
	) -> MoveGroup.Goal:
		"""
		@brief Build a MoveGroup action goal for a target pose.

		@param target_pose Goal pose already expressed in the planning frame.
		@param planning_config Request-specific planning configuration.
		@param target_frame Robot frame/link that should reach the target pose.
		@param start_state Optional robot state used as the planner start state.
		@return Configured MoveGroup goal.
		"""
		return build_move_group_goal(
			target_pose,
			planning_config,
			target_frame,
			start_state,
		)

	def _motion_planning_config_for_target(
		self,
		target_pose: PoseStamped,
		target_frame: Optional[str],
	) -> tuple[Optional[MotionPlanningConfig], str]:
		"""
		@brief Build the planning config for one request, including directional Cartesian limits.

		@param target_pose Goal pose already expressed in the planning frame.
		@param target_frame Robot frame/link that should reach the target pose.
		@return Request config plus an error message when the request cannot satisfy the limit.
		"""
		config = self._motion_planning_config()
		if not config.enable_cartesian_vel_limit:
			return config, ''

		target_link = str(target_frame or config.end_effector_link)
		try:
			current_pose = self._current_link_pose_in_planning_frame(target_link)
			max_cartesian_speed = self._resolve_directional_cartesian_speed_limit(
				current_pose,
				target_pose,
				config,
				target_link,
			)
		except RuntimeError as exc:
			return None, str(exc)

		return replace(config, max_cartesian_speed=max_cartesian_speed), ''

	def _current_link_pose_in_planning_frame(self, link_name: str) -> PoseStamped:
		"""
		@brief Read the current pose of a link in the planning frame from TF.

		@param link_name Link whose pose should be read.
		@return Current link pose in the planning frame.
		"""
		try:
			transform = self._tf_buffer.lookup_transform(
				self._planning_frame,
				link_name,
				Time(),
				timeout=rclpy.duration.Duration(seconds=1.0),
			)
		except Exception as exc:  # noqa: BLE001
			raise RuntimeError(
				f"Unable to evaluate Cartesian velocity limit: could not read current pose of '{link_name}' in '{self._planning_frame}': {exc}"
			) from exc

		pose = PoseStamped()
		pose.header.stamp = self.get_clock().now().to_msg()
		pose.header.frame_id = self._planning_frame
		pose.pose.position.x = float(transform.transform.translation.x)
		pose.pose.position.y = float(transform.transform.translation.y)
		pose.pose.position.z = float(transform.transform.translation.z)
		pose.pose.orientation = transform.transform.rotation
		return pose

	def _resolve_directional_cartesian_speed_limit(
		self,
		current_pose: PoseStamped,
		target_pose: PoseStamped,
		config: MotionPlanningConfig,
		target_link: str,
	) -> float:
		"""
		@brief Convert per-axis Cartesian velocity limits into one scalar MoveIt speed cap.

		@param current_pose Current link pose in the planning frame.
		@param target_pose Requested goal pose in the planning frame.
		@param config Planning configuration.
		@param target_link Link constrained by the request.
		@return Scalar Cartesian speed cap for this motion.
		"""
		limits = config.max_cartesian_velocity
		movement_epsilon = max(1e-6, config.position_tolerance_m * 0.1)
		delta_x = float(target_pose.pose.position.x) - float(current_pose.pose.position.x)
		delta_y = float(target_pose.pose.position.y) - float(current_pose.pose.position.y)
		delta_z = float(target_pose.pose.position.z) - float(current_pose.pose.position.z)
		components = [
			('x', abs(delta_x), float(limits[0])),
			('y', abs(delta_y), float(limits[1])),
			('z', abs(delta_z), float(limits[2])),
		]
		active_components = [component for component in components if component[1] > movement_epsilon]
		if not active_components:
			return 0.0

		for axis_name, _, axis_limit in active_components:
			if axis_limit < 0.0:
				raise RuntimeError(
					f"Invalid Cartesian velocity limit for axis '{axis_name}': {axis_limit:.4f}. Limits must be non-negative."
				)
			if axis_limit == 0.0:
				raise RuntimeError(
					'Cartesian velocity limit blocks this motion: '
					+ f"link '{target_link}' must move along {axis_name} in frame '{self._planning_frame}', "
					+ f"but max_cartesian_velocity[{axis_name}] is 0.0. "
					+ 'Increase that axis limit or request a motion without movement on that axis.'
				)

		translation_norm = sum(component[1] ** 2 for component in active_components) ** 0.5
		if translation_norm <= 0.0:
			return 0.0

		return min(
			axis_limit / (axis_distance / translation_norm)
			for _, axis_distance, axis_limit in active_components
		)

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
		if self._latest_joint_state is None or self._latest_joint_state_received_at is None:
			return None, 'no /joint_states message has been received yet.'

		timeout_sec = float(self.get_parameter('joint_state_timeout_sec').value)
		if timeout_sec > 0.0:
			age_sec = (self.get_clock().now() - self._latest_joint_state_received_at).nanoseconds / 1e9
			if age_sec > timeout_sec:
				return None, (
					f'latest /joint_states sample is stale ({age_sec:.3f}s old, '
					f'timeout {timeout_sec:.3f}s).'
				)

		positions_by_name = self._joint_positions_by_name(self._latest_joint_state)
		planning_joint_names = self._planning_joint_names()
		missing_joint_names = [name for name in planning_joint_names if name not in positions_by_name]
		if missing_joint_names:
			return None, 'latest /joint_states message is missing planning joints: ' + ', '.join(missing_joint_names)

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
		rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
		if not future.done() or future.result() is None:
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
	) -> JointState:
		"""
		@brief Build a stamped JointState from ordered joint names and a position mapping.

		@param joint_names Ordered joint names.
		@param positions_by_name Joint positions keyed by joint name.
		@return Stamped JointState in the requested order.
		"""
		joint_state = JointState()
		joint_state.header.stamp = self.get_clock().now().to_msg()
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
		return MotionPlanningConfig(
			planning_frame=self._planning_frame,
			planning_group=str(self.get_parameter('planning_group').value),
			allowed_planning_time=float(self.get_parameter('allowed_planning_time').value),
			num_planning_attempts=int(self.get_parameter('num_planning_attempts').value),
			max_velocity_scaling=float(self.get_parameter('max_velocity_scaling').value),
			max_acceleration_scaling=float(self.get_parameter('max_acceleration_scaling').value),
			enable_cartesian_vel_limit=bool(self.get_parameter('enable_cartesian_vel_limit').value),
			max_cartesian_velocity=tuple(
				coerce_float_sequence(
					self.get_parameter('max_cartesian_velocity').value,
					3,
					'max_cartesian_velocity',
				)
			),
			max_cartesian_speed=0.0,
			position_tolerance_m=float(self.get_parameter('position_tolerance_m').value),
			orientation_tolerance_rad=float(self.get_parameter('orientation_tolerance_rad').value),
			end_effector_link=str(self.get_parameter('end_effector_link').value),
			joint_goal_tolerance_rad=float(self.get_parameter('joint_goal_tolerance_rad').value),
			planning_pipeline_id=str(self.get_parameter('planning_pipeline_id').value),
			planner_id=str(self.get_parameter('planner_id').value),
		)


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
