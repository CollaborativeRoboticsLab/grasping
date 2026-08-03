from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

from geometry_msgs.msg import Point, PoseStamped
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from grasping_control.common import (
	Quaternion,
	normalize_quaternion,
	transform_pose_to_frame,
)
from grasping_control.workspace_utils import (
	collision_objects_from_workspace,
	point_in_workspace_area,
)
from grasping_msgs.action import MoveToNamedPose, MoveToPose
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
		self.declare_parameter('workspace_to_end_effector_height', 0.0)
		self.declare_parameter('poses_list', ['workspace_center_pose', 'pre_grasp_pose', 'post_grasp_pose'])
		self.declare_parameter('apply_planning_scene_service', '/apply_planning_scene')
		self.declare_parameter('workspace_area_marker_topic', '/workspace_area_marker')

		self._planning_frame = str(self.get_parameter('planning_frame').value)
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
			target_pose = self._get_named_pose_stamped(pose_name)

			feedback.state = 'transforming_target_pose'
			goal_handle.publish_feedback(feedback)
			target_pose = transform_pose_to_frame(
				self,
				self._tf_buffer,
				target_pose,
				self._planning_frame,
			)

			feedback.state = 'validating_workspace_area'
			goal_handle.publish_feedback(feedback)
			if not self._target_pose_in_workspace_area(target_pose):
				result = MoveToNamedPose.Result()
				result.success = False
				result.message = f"Configured pose '{pose_name}' lies outside the calibrated workspace area."
				goal_handle.abort()
				return result

			feedback.state = 'planning_and_executing'
			goal_handle.publish_feedback(feedback)
			ok, message = self._move_to_pose(target_pose)

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
		@brief Declare pose parameters listed by poses_list.
		"""
		pose_names = self._configured_pose_names()
		for pose_name in pose_names:
			if not self.has_parameter(pose_name):
				self.declare_parameter(pose_name, [0.0, 0.0, 0.30, 0.0, 0.0, 0.0])

		if pose_names:
			self.get_logger().info('Configured motion pose parameters: ' + ', '.join(pose_names))
		else:
			self.get_logger().warn('No configured motion poses listed in poses_list.')

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
		workspace_config: Dict[str, Any] = {
			'version': int(self.get_parameter('workspace.version').value),
			'updated_at': str(self.get_parameter('workspace.updated_at').value),
			'base_frame': str(self.get_parameter('workspace.base_frame').value),
			'tool_frame': str(self.get_parameter('workspace.tool_frame').value),
			'ground_plane_z': float(self.get_parameter('workspace.ground_plane_z').value),
			'workspace_area': None,
			'objects': [],
		}

		if self._get_bool_parameter('workspace_area.enabled'):
			workspace_config['workspace_area'] = {
				'geometry': self._workspace_area_geometry_from_parameters(),
			}

		for object_name in self._workspace_object_names():
			workspace_object = self._workspace_object_from_parameters(object_name)
			if workspace_object is not None:
				workspace_config['objects'].append(workspace_object)

		return workspace_config

	def _workspace_area_geometry_from_parameters(self) -> Dict[str, Any]:
		"""
		@brief Build workspace-area geometry from ROS parameters.
		"""
		dimensions = self._coerce_float_sequence(
			self.get_parameter('workspace_area.geometry.dimensions').value,
			2,
			'workspace_area.geometry.dimensions',
		)
		corner_x = self._coerce_float_sequence(
			self.get_parameter('workspace_area.geometry.corner_points.x').value,
			4,
			'workspace_area.geometry.corner_points.x',
		)
		corner_y = self._coerce_float_sequence(
			self.get_parameter('workspace_area.geometry.corner_points.y').value,
			4,
			'workspace_area.geometry.corner_points.y',
		)
		corner_z = self._coerce_float_sequence(
			self.get_parameter('workspace_area.geometry.corner_points.z').value,
			4,
			'workspace_area.geometry.corner_points.z',
		)

		return {
			'type': str(self.get_parameter('workspace_area.geometry.type').value),
			'dimensions': {
				'side_length': dimensions[0],
				'height_from_ground': dimensions[1],
			},
			'pose': self._pose_dict_from_parameters('workspace_area.geometry.pose'),
			'corner_points': [
				{'x': corner_x[index], 'y': corner_y[index], 'z': corner_z[index]}
				for index in range(4)
			],
		}

	def _workspace_object_from_parameters(self, object_name: str) -> Optional[Dict[str, Any]]:
		"""
		@brief Build one workspace object dictionary from ROS parameters.
		"""
		prefix = f'workspace_object.{object_name}'
		geometry_type = str(self.get_parameter(f'{prefix}.geometry.type').value)
		if not geometry_type:
			self.get_logger().warn(f"Skipping workspace object '{object_name}' because geometry type is empty.")
			return None

		dimensions = self._coerce_float_sequence(
			self.get_parameter(f'{prefix}.geometry.dimensions').value,
			3 if geometry_type == 'box' else 2,
			f'{prefix}.geometry.dimensions',
		)
		if geometry_type == 'box':
			dimensions_dict = {'x': dimensions[0], 'y': dimensions[1], 'z': dimensions[2]}
		else:
			dimensions_dict = {'height': dimensions[0], 'radius': dimensions[1]}

		workspace_object: Dict[str, Any] = {
			'name': object_name,
			'geometry': {
				'type': geometry_type,
				'dimensions': dimensions_dict,
				'pose': self._pose_dict_from_parameters(f'{prefix}.geometry.pose'),
			},
		}
		shape = str(self.get_parameter(f'{prefix}.shape').value)
		if shape:
			workspace_object['shape'] = shape
		return workspace_object

	def _pose_dict_from_parameters(self, prefix: str) -> Dict[str, Any]:
		"""
		@brief Build a pose dictionary from position and orientation parameter arrays.
		"""
		position = self._coerce_float_sequence(
			self.get_parameter(f'{prefix}.position').value,
			3,
			f'{prefix}.position',
		)
		orientation = self._coerce_float_sequence(
			self.get_parameter(f'{prefix}.orientation').value,
			4,
			f'{prefix}.orientation',
		)
		return {
			'position': {'x': position[0], 'y': position[1], 'z': position[2]},
			'orientation': {
				'x': orientation[0],
				'y': orientation[1],
				'z': orientation[2],
				'w': orientation[3],
			},
		}

	def _workspace_object_names(self) -> List[str]:
		"""
		@brief Return configured workspace object names.
		"""
		workspace_objects = self.get_parameter('workspace_objects').value
		if isinstance(workspace_objects, str):
			workspace_objects = [item.strip() for item in workspace_objects.strip('[]()').split(',')]
		if not isinstance(workspace_objects, list):
			return []
		return [str(name).strip() for name in workspace_objects if str(name).strip()]

	def _configured_pose_names(self) -> List[str]:
		"""
		@brief Return the list of configured pose names.

		@return Pose names from the poses_list ROS parameter.
		"""
		poses_list = self.get_parameter('poses_list').value
		if isinstance(poses_list, str):
			poses_list = [item.strip() for item in poses_list.strip('[]()').split(',') if item.strip()]
		if not isinstance(poses_list, list):
			return []
		return [str(name) for name in poses_list]

	def _configured_pose_exists(self, pose_name: str) -> bool:
		"""
		@brief Check whether a configured pose exists.

		@param pose_name Requested pose name.
		@return True when the pose name is allowed and has pose values.
		"""
		return pose_name in self._configured_pose_names() and self.has_parameter(pose_name)

	def _get_named_pose_stamped(self, pose_name: str) -> PoseStamped:
		"""
		@brief Return a configured named pose as a PoseStamped.

		@param pose_name Name from motion_config.yaml.
		@return PoseStamped in the workspace area frame or planning frame.
		"""
		if not self._configured_pose_exists(pose_name):
			raise RuntimeError(
				f"Unknown configured pose '{pose_name}'. Available poses: {', '.join(self._configured_pose_names())}"
			)

		pose_values = self._coerce_float_sequence(
			self.get_parameter(pose_name).value,
			6,
			pose_name,
		)
		workspace_to_end_effector_height = float(
			self.get_parameter('workspace_to_end_effector_height').value
		)

		if pose_name == 'workspace_center_pose':
			workspace_center = self._workspace_area_center()
			if workspace_center is None:
				raise RuntimeError("Configured pose 'workspace_center_pose' requires a calibrated workspace area.")
			return self._pose_stamped_from_values(
				self._workspace_area_frame,
				[
					workspace_center['x'],
					workspace_center['y'],
					workspace_center['z'] + workspace_to_end_effector_height,
					pose_values[3],
					pose_values[4],
					pose_values[5],
				],
			)

		return self._pose_stamped_from_values(self._planning_frame, pose_values)

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
		orientation = self._quaternion_from_rpy(pose_values[3], pose_values[4], pose_values[5])
		pose_stamped.pose.orientation.x = orientation.x
		pose_stamped.pose.orientation.y = orientation.y
		pose_stamped.pose.orientation.z = orientation.z
		pose_stamped.pose.orientation.w = orientation.w
		return pose_stamped

	def _workspace_area_center(self) -> Optional[Dict[str, float]]:
		"""
		@brief Return the calibrated workspace-area center when available.

		@return Center point from workspace-area geometry, otherwise None.
		"""
		if self._workspace_area is None:
			return None

		geometry = self._workspace_area.get('geometry', {})
		pose = geometry.get('pose', {})
		position = pose.get('position', {}) if isinstance(pose, dict) else {}
		if isinstance(position, dict) and all(axis in position for axis in ('x', 'y', 'z')):
			return {
				'x': float(position['x']),
				'y': float(position['y']),
				'z': float(position['z']),
			}

		corner_points = geometry.get('corner_points', [])
		if len(corner_points) != 4:
			return None

		return {
			'x': sum(float(point['x']) for point in corner_points) / 4.0,
			'y': sum(float(point['y']) for point in corner_points) / 4.0,
			'z': sum(float(point['z']) for point in corner_points) / 4.0,
		}

	def _coerce_float_sequence(self, value: Any, expected_length: int, name: str) -> List[float]:
		"""
		@brief Convert a fixed-length sequence-like value into floats.

		@param value Sequence value from YAML or parameters.
		@param expected_length Required number of values.
		@param name Human-readable value name for error messages.
		@return Parameter values as floats.
		"""
		if isinstance(value, str):
			items = [item.strip() for item in value.strip('[]()').split(',') if item.strip()]
		else:
			items = list(value)

		if len(items) != expected_length:
			raise RuntimeError(f"'{name}' must contain {expected_length} values.")
		return [float(item) for item in items]

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

	@staticmethod
	def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
		"""
		@brief Convert roll-pitch-yaw Euler angles to a quaternion.

		@param roll Roll angle in radians.
		@param pitch Pitch angle in radians.
		@param yaw Yaw angle in radians.
		@return Equivalent normalized quaternion.
		"""
		cy = math.cos(yaw * 0.5)
		sy = math.sin(yaw * 0.5)
		cp = math.cos(pitch * 0.5)
		sp = math.sin(pitch * 0.5)
		cr = math.cos(roll * 0.5)
		sr = math.sin(roll * 0.5)

		return normalize_quaternion(
			Quaternion(
				sr * cp * cy - cr * sp * sy,
				cr * sp * cy + sr * cp * sy,
				cr * cp * sy - sr * sp * cy,
				cr * cp * cy + sr * sp * sy,
			)
		)

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
		goal = self._build_move_group_goal(target_pose)

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

	def _build_move_group_goal(self, target_pose: PoseStamped) -> MoveGroup.Goal:
		"""
		@brief Build a MoveGroup action goal for a target pose.

		@param target_pose Goal pose already expressed in the planning frame.
		@return Configured MoveGroup goal.
		"""
		goal = MoveGroup.Goal()
		goal.request = self._build_motion_plan_request(target_pose)
		goal.planning_options = PlanningOptions()
		goal.planning_options.plan_only = False
		goal.planning_options.look_around = False
		goal.planning_options.replan = False
		goal.planning_options.replan_attempts = 0
		return goal

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
