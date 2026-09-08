from __future__ import annotations

from threading import Lock
from typing import Optional

from geometry_msgs.msg import Point
from grasping_msgs.action import ActivateScene, LoadSceneFromContent
from grasping_msgs.msg import ActiveScene as ActiveSceneMsg
from grasping_msgs.msg import SceneReference as SceneReferenceMsg
from grasping_msgs.srv import GetActiveScene, ValidateWorkspaceDocument
from moveit_msgs.msg import AllowedCollisionMatrix, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from grasping_control.motion_utils import append_allowed_collision_pairs
from grasping_control.scene_manager_core import (
	SceneActivationData,
	SceneManagerError,
	SceneReference,
	load_scene_from_reference,
	load_workspace_document_from_content,
	scene_references_match,
	validate_workspace_document,
)


class SceneManagerNode(Node):
	"""
	@brief Runtime node that owns the active grasping scene.

	This first draft defines the public APIs and state shape for the future scene
	manager. It validates documents and tracks active-scene metadata while the
	full MoveIt application flow is completed in later refactor steps.
	"""

	def __init__(self) -> None:
		super().__init__('scene_manager_node')

		self.declare_parameter('get_active_scene_service_name', 'get_active_scene')
		self.declare_parameter('validate_workspace_document_service_name', 'validate_workspace_document')
		self.declare_parameter('activate_scene_action_name', 'activate_scene')
		self.declare_parameter('load_scene_from_content_action_name', 'load_scene_from_content')
		self.declare_parameter('apply_planning_scene_service', '/apply_planning_scene')
		self.declare_parameter('get_planning_scene_service', '/get_planning_scene')
		self.declare_parameter('active_scene_topic', '/active_scene')
		self.declare_parameter('default_base_frame', 'world')
		self.declare_parameter('default_tool_frame', 'tool_tip')
		self.declare_parameter('ground_plane_z', 0.0)
		self.declare_parameter('startup_activate_default_scene', False)
		self.declare_parameter('default_scene_name', '')
		self.declare_parameter('default_scene_package', '')
		self.declare_parameter('default_workspace_file', '')
		self.declare_parameter('default_scene_revision', '')

		self._active_scene_lock = Lock()
		self._scene_activation_lock = Lock()
		self._active_scene: Optional[SceneActivationData] = None

		marker_qos = QoSProfile(
			history=HistoryPolicy.KEEP_LAST,
			depth=1,
			durability=DurabilityPolicy.TRANSIENT_LOCAL,
		)
		self._active_scene_publisher = self.create_publisher(
			ActiveSceneMsg,
			str(self.get_parameter('active_scene_topic').value),
			marker_qos,
		)
		self._planning_scene_client = self.create_client(
			ApplyPlanningScene,
			str(self.get_parameter('apply_planning_scene_service').value),
		)
		self._get_planning_scene_client = self.create_client(
			GetPlanningScene,
			str(self.get_parameter('get_planning_scene_service').value),
		)
		self._get_active_scene_service = self.create_service(
			GetActiveScene,
			str(self.get_parameter('get_active_scene_service_name').value),
			self._handle_get_active_scene,
		)
		self._validate_workspace_document_service = self.create_service(
			ValidateWorkspaceDocument,
			str(self.get_parameter('validate_workspace_document_service_name').value),
			self._handle_validate_workspace_document,
		)
		self._activate_scene_action = ActionServer(
			self,
			ActivateScene,
			str(self.get_parameter('activate_scene_action_name').value),
			execute_callback=self._execute_activate_scene,
			goal_callback=self._activate_scene_goal_callback,
			cancel_callback=self._cancel_callback,
		)
		self._load_scene_from_content_action = ActionServer(
			self,
			LoadSceneFromContent,
			str(self.get_parameter('load_scene_from_content_action_name').value),
			execute_callback=self._execute_load_scene_from_content,
			goal_callback=self._load_scene_from_content_goal_callback,
			cancel_callback=self._cancel_callback,
		)

		self._maybe_activate_default_scene()

	def _handle_get_active_scene(
		self,
		_request: GetActiveScene.Request,
		response: GetActiveScene.Response,
	) -> GetActiveScene.Response:
		"""
		@brief Return the currently active scene metadata.
		"""
		with self._active_scene_lock:
			if self._active_scene is None:
				response.active = False
				response.message = 'No active scene has been loaded yet.'
				return response
			response.active = True
			response.active_scene = self._to_active_scene_msg(self._active_scene)
			response.message = 'ok'
			return response

	def _handle_validate_workspace_document(
		self,
		request: ValidateWorkspaceDocument.Request,
		response: ValidateWorkspaceDocument.Response,
	) -> ValidateWorkspaceDocument.Response:
		"""
		@brief Validate a workspace document without mutating the active scene.
		"""
		try:
			reference = SceneReference(
				scene_name=str(request.scene_name),
				package_name=str(request.package_name),
				workspace_file=str(request.workspace_file),
			)
			document = (
				load_workspace_document_from_content(str(request.workspace_document))
				if str(request.workspace_document).strip()
				else load_scene_from_reference(
					reference,
					self._default_base_frame(),
					self._default_tool_frame(),
					self._ground_plane_z(),
				).workspace_config
			)
			activation = validate_workspace_document(
				reference,
				document,
				self._default_base_frame(),
				self._default_tool_frame(),
				self._ground_plane_z(),
			)
			response.valid = True
			response.failure_reason = ''
			response.message = 'Workspace document is valid.'
			response.normalized_reference = self._to_scene_reference_msg(activation.reference)
			response.workspace_base_frame = activation.workspace_base_frame
			response.workspace_area_enabled = activation.workspace_area_enabled
			response.object_names = activation.object_names
		except SceneManagerError as exc:
			response.valid = False
			response.failure_reason = exc.failure_reason
			response.message = str(exc)
		except Exception as exc:  # noqa: BLE001
			response.valid = False
			response.failure_reason = 'scene_manager_internal_error'
			response.message = str(exc)
		return response

	def _activate_scene_goal_callback(self, _goal: ActivateScene.Goal) -> GoalResponse:
		"""
		@brief Accept all explicit scene activation goals.
		"""
		return GoalResponse.ACCEPT

	def _load_scene_from_content_goal_callback(self, _goal: LoadSceneFromContent.Goal) -> GoalResponse:
		"""
		@brief Accept all scene-from-content activation goals.
		"""
		return GoalResponse.ACCEPT

	def _cancel_callback(self, _goal_handle: object) -> CancelResponse:
		"""
		@brief Accept cancellation for active scene-manager goals.
		"""
		return CancelResponse.ACCEPT

	def _execute_activate_scene(self, goal_handle) -> ActivateScene.Result:
		"""
		@brief Resolve, validate, apply, and publish a scene reference.
		"""
		result = ActivateScene.Result()
		try:
			reference = self._scene_reference_from_message(goal_handle.request.reference)
			reusable = self._reuse_active_scene(reference, bool(goal_handle.request.force_reload))
			if reusable is not None:
				result.success = True
				result.failure_reason = ''
				result.message = 'Requested scene is already active.'
				result.active_scene = self._to_active_scene_msg(reusable)
				goal_handle.succeed()
				return result

			with self._scene_activation_lock:
				self._publish_action_feedback(goal_handle, ActivateScene.Feedback, 'resolving_scene', 'Resolving workspace scene reference.')
				activation = load_scene_from_reference(
					reference,
					self._default_base_frame(),
					self._default_tool_frame(),
					self._ground_plane_z(),
				)
				self._publish_action_feedback(goal_handle, ActivateScene.Feedback, 'applying_scene', 'Applying scene to MoveIt and active-scene state.')
				self._apply_scene_activation(activation)
			result.success = True
			result.failure_reason = ''
			result.message = 'Scene activated.'
			result.active_scene = self._to_active_scene_msg(activation)
			goal_handle.succeed()
		except SceneManagerError as exc:
			result.success = False
			result.failure_reason = exc.failure_reason
			result.message = str(exc)
			goal_handle.abort()
		except Exception as exc:  # noqa: BLE001
			result.success = False
			result.failure_reason = 'scene_manager_internal_error'
			result.message = str(exc)
			goal_handle.abort()
		return result

	def _execute_load_scene_from_content(self, goal_handle) -> LoadSceneFromContent.Result:
		"""
		@brief Validate, apply, and publish a supplied workspace document.
		"""
		result = LoadSceneFromContent.Result()
		try:
			with self._scene_activation_lock:
				self._publish_action_feedback(goal_handle, LoadSceneFromContent.Feedback, 'validating_document', 'Validating workspace document content.')
				activation = validate_workspace_document(
					SceneReference(
						scene_name=str(goal_handle.request.scene_name),
						package_name=str(goal_handle.request.package_name),
						workspace_file=str(goal_handle.request.workspace_file),
					),
					load_workspace_document_from_content(str(goal_handle.request.workspace_document)),
					self._default_base_frame(),
					self._default_tool_frame(),
					self._ground_plane_z(),
				)
				self._publish_action_feedback(goal_handle, LoadSceneFromContent.Feedback, 'applying_scene', 'Applying supplied scene to MoveIt and active-scene state.')
				self._apply_scene_activation(activation)
			result.success = True
			result.failure_reason = ''
			result.message = 'Scene activated from content.'
			result.active_scene = self._to_active_scene_msg(activation)
			goal_handle.succeed()
		except SceneManagerError as exc:
			result.success = False
			result.failure_reason = exc.failure_reason
			result.message = str(exc)
			goal_handle.abort()
		except Exception as exc:  # noqa: BLE001
			result.success = False
			result.failure_reason = 'scene_manager_internal_error'
			result.message = str(exc)
			goal_handle.abort()
		return result

	def _apply_scene_activation(self, activation: SceneActivationData) -> None:
		"""
		@brief Apply one validated scene to MoveIt and publish it as active.

		This first refactor step centralizes MoveIt scene-diff assembly and active
		state publication behind the new core data model.
		"""
		request = PlanningScene()
		request.is_diff = True
		request.world.collision_objects = activation.collision_objects
		allowed_collision_matrix = self._allowed_collision_matrix_for_activation(activation)
		if allowed_collision_matrix is not None:
			request.allowed_collision_matrix = allowed_collision_matrix

		if activation.collision_objects or allowed_collision_matrix is not None:
			if not self._planning_scene_client.wait_for_service(timeout_sec=5.0):
				raise SceneManagerError(
					'planning_scene_unavailable',
					'ApplyPlanningScene service is not available for scene activation.',
				)
			service_request = ApplyPlanningScene.Request()
			service_request.scene = request
			future = self._planning_scene_client.call_async(service_request)
			rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
			if not future.done() or future.result() is None:
				raise SceneManagerError(
					'planning_scene_apply_timeout',
					'ApplyPlanningScene request did not complete before the timeout.',
				)
			if not future.result().success:
				raise SceneManagerError(
					'planning_scene_rejected',
					'MoveIt rejected the active-scene planning-scene update.',
				)

		with self._active_scene_lock:
			self._active_scene = activation
		self._active_scene_publisher.publish(self._to_active_scene_msg(activation))

	def _allowed_collision_matrix_for_activation(
		self,
		activation: SceneActivationData,
	) -> Optional[AllowedCollisionMatrix]:
		"""
		@brief Fetch and extend MoveIt's current allowed-collision matrix for a scene.
		"""
		if not activation.allowed_collision_pairs:
			return None
		if not self._get_planning_scene_client.wait_for_service(timeout_sec=5.0):
			raise SceneManagerError(
				'allowed_collision_matrix_unavailable',
				'GetPlanningScene service is not available to extend the allowed collision matrix.',
			)
		request = GetPlanningScene.Request()
		response_future = self._get_planning_scene_client.call_async(request)
		rclpy.spin_until_future_complete(self, response_future, timeout_sec=10.0)
		if not response_future.done() or response_future.result() is None:
			raise SceneManagerError(
				'allowed_collision_matrix_unavailable',
				'GetPlanningScene did not return the current allowed collision matrix.',
			)
		matrix = response_future.result().scene.allowed_collision_matrix
		return append_allowed_collision_pairs(matrix, activation.allowed_collision_pairs)

	def _maybe_activate_default_scene(self) -> None:
		"""
		@brief Optionally activate a configured default scene during node startup.
		"""
		if not bool(self.get_parameter('startup_activate_default_scene').value):
			return
		workspace_file = str(self.get_parameter('default_workspace_file').value).strip()
		if not workspace_file:
			self.get_logger().warn('startup_activate_default_scene is true but default_workspace_file is empty.')
			return
		try:
			activation = load_scene_from_reference(
				SceneReference(
					scene_name=str(self.get_parameter('default_scene_name').value).strip(),
					package_name=str(self.get_parameter('default_scene_package').value).strip(),
					workspace_file=workspace_file,
					scene_revision=str(self.get_parameter('default_scene_revision').value).strip(),
				),
				self._default_base_frame(),
				self._default_tool_frame(),
				self._ground_plane_z(),
			)
			self._apply_scene_activation(activation)
			self.get_logger().info('Activated default scene during startup: ' + activation.scene_handle)
		except Exception as exc:  # noqa: BLE001
			self.get_logger().error('Failed to activate default scene during startup: ' + str(exc))

	def _scene_reference_from_message(self, reference: SceneReferenceMsg) -> SceneReference:
		"""
		@brief Convert a SceneReference ROS message into the Python-side dataclass.
		"""
		return SceneReference(
			scene_name=str(reference.scene_name),
			package_name=str(reference.package_name),
			workspace_file=str(reference.workspace_file),
			scene_revision=str(reference.scene_revision),
		)

	def _reuse_active_scene(self, reference: SceneReference, force_reload: bool) -> Optional[SceneActivationData]:
		"""
		@brief Return the current active scene when it already matches the requested reference.
		"""
		if force_reload:
			return None
		with self._active_scene_lock:
			if self._active_scene is None:
				return None
			return self._active_scene if scene_references_match(self._active_scene.reference, reference) else None

	def _to_scene_reference_msg(self, reference: SceneReference) -> SceneReferenceMsg:
		"""
		@brief Convert the Python-side scene reference dataclass into a ROS message.
		"""
		message = SceneReferenceMsg()
		message.scene_name = reference.scene_name
		message.package_name = reference.package_name
		message.workspace_file = reference.workspace_file
		message.scene_revision = reference.scene_revision
		return message

	def _to_active_scene_msg(self, activation: SceneActivationData) -> ActiveSceneMsg:
		"""
		@brief Convert validated activation data into the public ActiveScene message.
		"""
		message = ActiveSceneMsg()
		message.reference = self._to_scene_reference_msg(activation.reference)
		message.scene_handle = activation.scene_handle
		message.workspace_base_frame = activation.workspace_base_frame
		message.workspace_area_enabled = activation.workspace_area_enabled
		message.object_names = activation.object_names
		for point_dict in self._workspace_area_corner_points(activation):
			point = Point()
			point.x = float(point_dict['x'])
			point.y = float(point_dict['y'])
			point.z = float(point_dict['z'])
			message.workspace_area_corner_points.append(point)
		message.loaded_at = self.get_clock().now().to_msg()
		return message

	def _workspace_area_corner_points(self, activation: SceneActivationData) -> list[dict[str, float]]:
		"""
		@brief Return workspace-area corner points from a validated scene, when present.
		"""
		workspace_area = activation.workspace_config.get('workspace_area')
		if not isinstance(workspace_area, dict):
			return []
		geometry = workspace_area.get('geometry', {})
		corner_points = geometry.get('corner_points', [])
		if isinstance(corner_points, list):
			return [
				{
					'x': float(point.get('x', 0.0)),
					'y': float(point.get('y', 0.0)),
					'z': float(point.get('z', 0.0)),
				}
				for point in corner_points
				if isinstance(point, dict)
			]
		if not isinstance(corner_points, dict):
			return []
		x_values = corner_points.get('x', [])
		y_values = corner_points.get('y', [])
		z_values = corner_points.get('z', [])
		return [
			{'x': float(x), 'y': float(y), 'z': float(z)}
			for x, y, z in zip(x_values, y_values, z_values)
		]

	def _publish_action_feedback(self, goal_handle, feedback_type, stage: str, message: str) -> None:
		"""
		@brief Publish action feedback for either scene-manager action type.
		"""
		feedback_message = feedback_type()
		feedback_message.stage = stage
		feedback_message.message = message
		goal_handle.publish_feedback(feedback_message)

	def _default_base_frame(self) -> str:
		return str(self.get_parameter('default_base_frame').value)

	def _default_tool_frame(self) -> str:
		return str(self.get_parameter('default_tool_frame').value)

	def _ground_plane_z(self) -> float:
		return float(self.get_parameter('ground_plane_z').value)


def main(args: Optional[list[str]] = None) -> None:
	"""
	@brief Run the scene manager node until shutdown.

	@param args Optional ROS command-line arguments.
	"""
	rclpy.init(args=args)
	node = SceneManagerNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()