from __future__ import annotations

import threading
from typing import Any, Dict, List, Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState

from copy import deepcopy

from grasping_control.common import Quaternion, coerce_string_sequence, rotate_vector_by_quaternion, transform_pose_to_frame
from grasping_control.motion_utils import (
	MotionPlanningConfig,
	build_joint_move_group_goal,
	build_move_group_goal,
	planning_config_from_node,
	robot_state_from_joint_state,
)
from grasping_control.workspace_utils import (
	default_workspace_config,
	point_in_workspace_area,
	workspace_config_from_node_parameters,
)
from grasping_msgs.msg import ActiveScene
from grasping_msgs.srv import CheckCartesianPoseFeasibility, CheckJointPoseFeasibility
from grasping_msgs.srv import GetActiveScene
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
import tf2_ros


class FeasibilityServiceNode(Node):
	"""
	@brief Provide arm-only feasibility checks for Cartesian and joint targets.
	"""

	def __init__(self) -> None:
		super().__init__('feasibility_service_node')

		self.declare_parameter('cartesian_feasibility_service_name', 'check_cartesian_pose_feasibility')
		self.declare_parameter('joint_feasibility_service_name', 'check_joint_pose_feasibility')
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
		self.declare_parameter('compute_ik_service', '/compute_ik')
		self.declare_parameter('check_state_validity_service', '/check_state_validity')
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
		self.declare_parameter('fallback_to_pose_planning_on_ik_failure', True)
		self.declare_parameter('grasp_pose_recovery_enabled', True)
		self.declare_parameter('grasp_pose_recovery_tool_frame', 'tool_tip')
		self.declare_parameter('grasp_pose_recovery_recalculate_attempts', 3)
		self.declare_parameter('joint_state_timeout_sec', 0.5)
		self.declare_parameter('ik_timeout_sec', 0.2)
		self.declare_parameter('joint_goal_tolerance_rad', 0.001)
		self.declare_parameter('active_scene_topic', '/active_scene')
		self.declare_parameter('get_active_scene_service_name', 'get_active_scene')

		self._planning_frame = str(self.get_parameter('planning_frame').value)
		self._latest_joint_positions_by_name: Dict[str, float] = {}
		self._latest_joint_position_received_at: Dict[str, Time] = {}
		self._declare_workspace_parameters()
		self._workspace_area: Optional[Dict[str, Any]] = None
		self._workspace_area_frame = self._planning_frame

		self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
		self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
		self._runtime_callback_group = ReentrantCallbackGroup()
		active_scene_qos = QoSProfile(
			history=HistoryPolicy.KEEP_LAST,
			depth=1,
			durability=DurabilityPolicy.TRANSIENT_LOCAL,
		)
		self._movegroup_client = ActionClient(
			self,
			MoveGroup,
			str(self.get_parameter('move_group_action_name').value),
			callback_group=self._runtime_callback_group,
		)
		self._compute_ik_client = self.create_client(
			GetPositionIK,
			str(self.get_parameter('compute_ik_service').value),
			callback_group=self._runtime_callback_group,
		)
		self._state_validity_client = self.create_client(
			GetStateValidity,
			str(self.get_parameter('check_state_validity_service').value),
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
		self._active_scene_subscription = self.create_subscription(
			ActiveScene,
			str(self.get_parameter('active_scene_topic').value),
			self._active_scene_callback,
			active_scene_qos,
			callback_group=self._runtime_callback_group,
		)
		self._cartesian_service = self.create_service(
			CheckCartesianPoseFeasibility,
			str(self.get_parameter('cartesian_feasibility_service_name').value),
			self._handle_cartesian_feasibility,
			callback_group=self._runtime_callback_group,
		)
		self._joint_service = self.create_service(
			CheckJointPoseFeasibility,
			str(self.get_parameter('joint_feasibility_service_name').value),
			self._handle_joint_feasibility,
			callback_group=self._runtime_callback_group,
		)

		self._sync_workspace_area_from_scene_manager()

	def _active_scene_callback(self, message: ActiveScene) -> None:
		"""
		@brief Update the workspace-area filter from the latest active-scene topic state.
		"""
		self._update_workspace_area_from_active_scene(message)

	def _sync_workspace_area_from_scene_manager(self) -> None:
		"""
		@brief Bootstrap the workspace-area cache from the scene-manager service.
		"""
		if not self._get_active_scene_client.wait_for_service(timeout_sec=2.0):
			self.get_logger().warn('GetActiveScene service not available during startup; waiting for /active_scene updates.')
			return
		future = self._get_active_scene_client.call_async(GetActiveScene.Request())
		rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
		if not future.done() or future.result() is None:
			self.get_logger().warn('GetActiveScene request did not complete during startup; waiting for /active_scene updates.')
			return
		response = future.result()
		if response.active:
			self._update_workspace_area_from_active_scene(response.active_scene)
		else:
			self.get_logger().info('Scene manager reports no active scene yet; workspace-area filtering is idle.')

	def _update_workspace_area_from_active_scene(self, active_scene: ActiveScene) -> None:
		"""
		@brief Rebuild workspace-area geometry from the scene-manager active-scene message.
		"""
		self._workspace_area_frame = str(active_scene.workspace_base_frame).strip() or self._planning_frame
		if not bool(active_scene.workspace_area_enabled):
			self._workspace_area = None
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
			return

		self._workspace_area = {
			'geometry': {
				'corner_points': corner_points,
			},
		}

	def _handle_cartesian_feasibility(
		self,
		request: CheckCartesianPoseFeasibility.Request,
		response: CheckCartesianPoseFeasibility.Response,
	) -> CheckCartesianPoseFeasibility.Response:
		response.mode_used = str(request.mode).strip()
		response.planning_frame = self._planning_frame
		mode = response.mode_used
		if mode not in {'arm_only_ik', 'arm_only_plan'}:
			return self._fail_cartesian(response, 'unsupported_mode', '', f"Unsupported Cartesian feasibility mode '{mode}'.")

		planning_frame = str(request.planning_frame).strip() or self._planning_frame
		target_frame = str(request.target_frame).strip() or str(self.get_parameter('end_effector_link').value)
		if not planning_frame:
			return self._fail_cartesian(response, 'invalid_request', '', 'Cartesian feasibility request must include planning_frame or rely on configured motion_config.')

		target_pose = PoseStamped()
		target_pose.header.stamp = self.get_clock().now().to_msg()
		target_pose.header.frame_id = planning_frame
		target_pose.pose = request.pose

		try:
			target_pose = transform_pose_to_frame(self, self._tf_buffer, target_pose, self._planning_frame)
		except Exception as exc:  # noqa: BLE001
			return self._fail_cartesian(response, 'transform_failed', '', str(exc))

		response.planning_pose = target_pose.pose
		response.planning_pose_valid = True

		if not self._target_pose_in_workspace_area(target_pose):
			return self._fail_cartesian(
				response,
				'workspace_area_violation',
				'move_base_then_arm',
				'Target pose lies outside the calibrated workspace area.',
			)

		planning_config = self._motion_planning_config()
		result = self._evaluate_cartesian_target(target_pose, target_frame, planning_config, mode)
		if result['feasible']:
			response.feasible = True
			response.failure_reason = ''
			response.suggested_fallback = ''
			response.message = result['message']
			response.planning_pose = result['planning_pose'].pose
			response.planning_pose_valid = True
			if result['joint_state'] is not None:
				response.joint_state_solution = result['joint_state']
				response.joint_state_solution_valid = True
			return response

		recovery_result = self._evaluate_grasp_pose_recovery(target_pose, target_frame, planning_config, mode)
		if recovery_result['feasible']:
			response.feasible = True
			response.failure_reason = ''
			response.suggested_fallback = ''
			response.message = recovery_result['message']
			response.planning_pose = recovery_result['planning_pose'].pose
			response.planning_pose_valid = True
			if recovery_result['joint_state'] is not None:
				response.joint_state_solution = recovery_result['joint_state']
				response.joint_state_solution_valid = True
			return response

		return self._fail_cartesian(
			response,
			str(result['failure_reason']),
			str(result['suggested_fallback']),
			str(result['message']),
		)

	def _evaluate_cartesian_target(
		self,
		target_pose: PoseStamped,
		target_frame: Optional[str],
		planning_config: MotionPlanningConfig,
		mode: str,
	) -> Dict[str, Any]:
		"""
		@brief Evaluate one Cartesian target for IK-only or plan-only feasibility.

		@param target_pose Goal pose in the planning frame.
		@param target_frame Robot frame/link that should reach the target pose.
		@param planning_config Planning settings snapshot.
		@param mode Either arm_only_ik or arm_only_plan.
		@return Result mapping with feasibility, diagnostics, and optional joint solution.
		"""
		ik_ok, ik_payload, ik_message = self._joint_goal_from_nearby_ik(target_pose, target_frame)
		joint_state = ik_payload.get('joint_state') if ik_ok else None
		if mode == 'arm_only_ik':
			if ik_ok:
				return {
					'feasible': True,
					'failure_reason': '',
					'suggested_fallback': '',
					'message': 'Cartesian IK feasibility succeeded.',
					'joint_state': joint_state,
					'planning_pose': target_pose,
				}
			return {
				'feasible': False,
				'failure_reason': 'ik_failed',
				'suggested_fallback': 'move_base_then_arm',
				'message': ik_message,
				'joint_state': None,
				'planning_pose': target_pose,
			}

		if ik_ok:
			goal = build_joint_move_group_goal(
				ik_payload['joint_state'],
				planning_config,
				target_frame,
				ik_payload['start_state'],
				plan_only=True,
			)
			plan_ok, plan_message = self._execute_move_group_goal(goal)
			if plan_ok:
				return {
					'feasible': True,
					'failure_reason': '',
					'suggested_fallback': '',
					'message': 'Cartesian arm-only planning feasibility succeeded via nearby IK.',
					'joint_state': joint_state,
					'planning_pose': target_pose,
				}
			return {
				'feasible': False,
				'failure_reason': 'planning_failed',
				'suggested_fallback': 'move_base_then_arm',
				'message': plan_message,
				'joint_state': joint_state,
				'planning_pose': target_pose,
			}

		if not bool(self.get_parameter('fallback_to_pose_planning_on_ik_failure').value):
			return {
				'feasible': False,
				'failure_reason': 'ik_failed',
				'suggested_fallback': 'move_base_then_arm',
				'message': ik_message,
				'joint_state': None,
				'planning_pose': target_pose,
			}

		goal = build_move_group_goal(
			target_pose,
			planning_config,
			target_frame,
			self._current_robot_state_or_none(),
			plan_only=True,
		)
		plan_ok, plan_message = self._execute_move_group_goal(goal)
		if plan_ok:
			return {
				'feasible': True,
				'failure_reason': '',
				'suggested_fallback': '',
				'message': 'Cartesian arm-only planning feasibility succeeded via pose-constrained planning.',
				'joint_state': None,
				'planning_pose': target_pose,
			}
		return {
			'feasible': False,
			'failure_reason': 'planning_failed',
			'suggested_fallback': 'move_base_then_arm',
			'message': ik_message + ' ' + plan_message,
			'joint_state': None,
			'planning_pose': target_pose,
		}

	def _evaluate_grasp_pose_recovery(
		self,
		target_pose: PoseStamped,
		target_frame: Optional[str],
		planning_config: MotionPlanningConfig,
		mode: str,
	) -> Dict[str, Any]:
		"""
		@brief Evaluate tool-tip and interpolated grasp fallback targets.

		@param target_pose Original requested grasp pose.
		@param target_frame Requested constrained link.
		@param planning_config Planning settings snapshot.
		@param mode Either arm_only_ik or arm_only_plan.
		@return Result mapping matching _evaluate_cartesian_target.
		"""
		frames = self._grasp_pose_recovery_frames(target_frame)
		if frames is None:
			return {'feasible': False}

		primary_frame, recovery_frame = frames
		frame_offset = self._lookup_recovery_frame_offset(primary_frame, recovery_frame)
		if frame_offset is None:
			return {'feasible': False}

		tool_tip_result = self._evaluate_cartesian_target(target_pose, recovery_frame, planning_config, mode)
		if not tool_tip_result['feasible']:
			return {'feasible': False}

		attempts = max(0, int(self.get_parameter('grasp_pose_recovery_recalculate_attempts').value))
		fraction = 0.5
		for attempt_index in range(attempts):
			candidate_pose = self._pose_with_recovery_offset(target_pose, frame_offset, fraction)
			candidate_result = self._evaluate_cartesian_target(candidate_pose, primary_frame, planning_config, mode)
			if candidate_result['feasible']:
				candidate_result['message'] = (
					str(candidate_result['message'])
					+ f' using grasp recovery between {primary_frame} and {recovery_frame} '
					+ f'(attempt {attempt_index + 1}/{attempts}, fraction={fraction:.3f}).'
				)
				return candidate_result
			fraction = 1.0 - ((1.0 - fraction) * 0.5)

		tool_tip_result = dict(tool_tip_result)
		tool_tip_result['message'] = str(tool_tip_result['message']) + f' using grasp recovery target frame {recovery_frame}.'
		return tool_tip_result

	def _grasp_pose_recovery_frames(self, target_frame: Optional[str]) -> Optional[tuple[str, str]]:
		"""
		@brief Resolve the primary and recovery grasp frames for TCP fallback.

		@param target_frame Requested constrained link.
		@return (primary_frame, recovery_frame) when recovery should run.
		"""
		if not bool(self.get_parameter('grasp_pose_recovery_enabled').value):
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
		except Exception:
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

	def _handle_joint_feasibility(
		self,
		request: CheckJointPoseFeasibility.Request,
		response: CheckJointPoseFeasibility.Response,
	) -> CheckJointPoseFeasibility.Response:
		response.mode_used = str(request.mode).strip()
		response.planning_frame = self._planning_frame
		mode = response.mode_used
		if mode not in {'state_validity', 'plan'}:
			return self._fail_joint(response, 'unsupported_mode', '', f"Unsupported joint feasibility mode '{mode}'.")

		joint_names = [str(name).strip() for name in request.joint_names if str(name).strip()]
		joint_positions = [float(position) for position in request.joint_positions]
		if not joint_names:
			return self._fail_joint(response, 'invalid_request', '', 'Joint feasibility request must include at least one joint name.')
		if len(joint_names) != len(joint_positions):
			return self._fail_joint(
				response,
				'invalid_request',
				'',
				'joint_names and joint_positions must contain the same number of values.',
			)

		target_joint_state = self._joint_state_from_positions(joint_names, dict(zip(joint_names, joint_positions)))
		response.checked_joint_state = target_joint_state
		response.checked_joint_state_valid = True

		if mode == 'state_validity':
			valid, message, contacts = self._check_joint_state_validity(target_joint_state)
			response.collision_contacts = contacts
			if valid:
				response.feasible = True
				response.failure_reason = ''
				response.suggested_fallback = ''
				response.message = 'Joint state is valid in the current planning scene.'
				return response
			return self._fail_joint(response, 'state_invalid', '', message)

		goal = build_joint_move_group_goal(
			target_joint_state,
			self._motion_planning_config(),
			None,
			self._current_robot_state_or_none(),
			plan_only=True,
		)
		plan_ok, plan_message = self._execute_move_group_goal(goal)
		if plan_ok:
			response.feasible = True
			response.failure_reason = ''
			response.suggested_fallback = ''
			response.message = 'Joint arm-only planning feasibility succeeded.'
			return response
		return self._fail_joint(response, 'planning_failed', '', plan_message)

	def _fail_cartesian(
		self,
		response: CheckCartesianPoseFeasibility.Response,
		failure_reason: str,
		suggested_fallback: str,
		message: str,
	) -> CheckCartesianPoseFeasibility.Response:
		response.feasible = False
		response.failure_reason = failure_reason
		response.suggested_fallback = suggested_fallback
		response.message = message
		return response

	def _fail_joint(
		self,
		response: CheckJointPoseFeasibility.Response,
		failure_reason: str,
		suggested_fallback: str,
		message: str,
	) -> CheckJointPoseFeasibility.Response:
		response.feasible = False
		response.failure_reason = failure_reason
		response.suggested_fallback = suggested_fallback
		response.message = message
		return response

	def _joint_state_callback(self, msg: JointState) -> None:
		now = self.get_clock().now()
		positions_by_name = self._joint_positions_by_name(msg)
		for joint_name, position in positions_by_name.items():
			self._latest_joint_positions_by_name[joint_name] = float(position)
			self._latest_joint_position_received_at[joint_name] = now

	def _check_joint_state_validity(self, joint_state: JointState) -> tuple[bool, str, List[str]]:
		service_name = str(self.get_parameter('check_state_validity_service').value)
		if not self._state_validity_client.wait_for_service(timeout_sec=2.0):
			return False, f"GetStateValidity service '{service_name}' is not available.", []

		request = GetStateValidity.Request()
		request.robot_state = RobotState()
		request.robot_state.joint_state = joint_state
		request.group_name = str(self.get_parameter('planning_group').value)

		future = self._state_validity_client.call_async(request)
		if not self._wait_for_future(future, timeout_sec=5.0) or future.result() is None:
			return False, 'State validity request did not complete before the client timeout.', []

		result = future.result()
		contacts: List[str] = []
		for contact in result.contacts:
			for body_name in [str(contact.contact_body_1), str(contact.contact_body_2)]:
				if body_name and body_name not in contacts:
					contacts.append(body_name)
		if bool(result.valid):
			return True, 'ok', contacts
		if contacts:
			return False, 'Joint state is invalid due to current-scene collision contacts: ' + ', '.join(contacts), contacts
		return False, 'Joint state is invalid in the current planning scene.', contacts

	def _execute_move_group_goal(self, goal: MoveGroup.Goal) -> tuple[bool, str]:
		action_name = str(self.get_parameter('move_group_action_name').value)
		if not self._movegroup_client.wait_for_server(timeout_sec=5.0):
			return False, f"MoveGroup action server '{action_name}' not available."

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
			return False, 'MoveGroup failed with ' + self._describe_moveit_error_code(result.error_code.val)
		return True, 'MoveGroup planning feasibility succeeded.'

	def _joint_goal_from_nearby_ik(self, target_pose: PoseStamped, target_frame: Optional[str] = None) -> tuple[bool, Dict[str, Any], str]:
		current_joint_state, state_message = self._current_planning_joint_state()
		if current_joint_state is None:
			return False, {}, 'Nearby IK unavailable: ' + state_message

		start_state = robot_state_from_joint_state(current_joint_state)
		ik_solution, ik_message = self._compute_nearby_ik_solution(target_pose, start_state, target_frame)
		if ik_solution is None:
			return False, {}, ik_message

		target_joint_state = self._planning_joint_state_from_robot_state(ik_solution)
		if target_joint_state is None:
			return False, {}, 'Nearby IK returned an incomplete joint solution for planning joints ' + ', '.join(self._planning_joint_names())

		return True, {
			'joint_state': target_joint_state,
			'start_state': start_state,
		}, 'Nearby IK selected a joint-space goal.'

	def _current_planning_joint_state(self) -> tuple[Optional[JointState], str]:
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
		current_joint_state, _ = self._current_planning_joint_state()
		if current_joint_state is None:
			return None
		return robot_state_from_joint_state(current_joint_state)

	def _compute_nearby_ik_solution(
		self,
		target_pose: PoseStamped,
		start_state: RobotState,
		target_frame: Optional[str] = None,
	) -> tuple[Optional[RobotState], str]:
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
			return None, 'Nearby IK failed with ' + self._describe_moveit_error_code(response.error_code.val)
		return response.solution, 'ok'

	def _planning_joint_state_from_robot_state(self, robot_state: RobotState) -> Optional[JointState]:
		positions_by_name = self._joint_positions_by_name(robot_state.joint_state)
		planning_joint_names = self._planning_joint_names()
		missing_joint_names = [name for name in planning_joint_names if name not in positions_by_name]
		if missing_joint_names:
			return None
		return self._joint_state_from_positions(planning_joint_names, positions_by_name)

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

	def _target_pose_in_workspace_area(self, target_pose: PoseStamped) -> bool:
		if self._workspace_area is None:
			return True
		geometry = self._workspace_area.get('geometry', {})
		if target_pose.header.frame_id != self._workspace_area_frame:
			return False
		return point_in_workspace_area(
			geometry,
			{
				'x': float(target_pose.pose.position.x),
				'y': float(target_pose.pose.position.y),
				'z': float(target_pose.pose.position.z),
			},
		)

	def _load_workspace_from_parameters(self) -> None:
		workspace_config = workspace_config_from_node_parameters(
			self,
			default_workspace_config(
				self._planning_frame,
				str(self.get_parameter('workspace.tool_frame').value),
				float(self.get_parameter('workspace.ground_plane_z').value),
			),
		)
		self._workspace_area_frame = str(workspace_config.get('base_frame', self._planning_frame))
		workspace_area = workspace_config.get('workspace_area')
		self._workspace_area = workspace_area if isinstance(workspace_area, dict) else None

	def _declare_workspace_parameters(self) -> None:
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

	def _workspace_object_names(self) -> List[str]:
		return coerce_string_sequence(self.get_parameter('workspace_objects').value)

	def _motion_planning_config(self) -> MotionPlanningConfig:
		return planning_config_from_node(self, self._planning_frame)

	def _planning_joint_names(self) -> List[str]:
		return coerce_string_sequence(self.get_parameter('planning_joint_names').value)

	@staticmethod
	def _joint_positions_by_name(joint_state: JointState) -> Dict[str, float]:
		return {
			str(joint_name): float(position)
			for joint_name, position in zip(joint_state.name, joint_state.position)
			if str(joint_name).strip()
		}

	def _joint_state_from_positions(
		self,
		joint_names: List[str],
		positions_by_name: Dict[str, float],
		stamp: Optional[Time] = None,
	) -> JointState:
		joint_state = JointState()
		joint_state.header.stamp = (stamp or self.get_clock().now()).to_msg()
		joint_state.name = list(joint_names)
		joint_state.position = [float(positions_by_name[joint_name]) for joint_name in joint_names]
		joint_state.velocity = []
		joint_state.effort = []
		return joint_state

	@staticmethod
	def _describe_moveit_error_code(error_code: int) -> str:
		labels = {
			MoveItErrorCodes.SUCCESS: 'SUCCESS',
			MoveItErrorCodes.FAILURE: 'FAILURE',
			MoveItErrorCodes.PLANNING_FAILED: 'PLANNING_FAILED',
			MoveItErrorCodes.INVALID_MOTION_PLAN: 'INVALID_MOTION_PLAN',
			MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
			MoveItErrorCodes.CONTROL_FAILED: 'CONTROL_FAILED',
			MoveItErrorCodes.TIMED_OUT: 'TIMED_OUT',
			MoveItErrorCodes.PREEMPTED: 'PREEMPTED',
			MoveItErrorCodes.START_STATE_IN_COLLISION: 'START_STATE_IN_COLLISION',
			MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
			MoveItErrorCodes.GOAL_IN_COLLISION: 'GOAL_IN_COLLISION',
			MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
			MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: 'GOAL_CONSTRAINTS_VIOLATED',
			MoveItErrorCodes.INVALID_GROUP_NAME: 'INVALID_GROUP_NAME',
			MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: 'INVALID_GOAL_CONSTRAINTS',
			MoveItErrorCodes.INVALID_ROBOT_STATE: 'INVALID_ROBOT_STATE',
			MoveItErrorCodes.INVALID_LINK_NAME: 'INVALID_LINK_NAME',
			MoveItErrorCodes.INVALID_OBJECT_NAME: 'INVALID_OBJECT_NAME',
			MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: 'FRAME_TRANSFORM_FAILURE',
			MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: 'COLLISION_CHECKING_UNAVAILABLE',
			MoveItErrorCodes.ROBOT_STATE_STALE: 'ROBOT_STATE_STALE',
			MoveItErrorCodes.SENSOR_INFO_STALE: 'SENSOR_INFO_STALE',
			MoveItErrorCodes.NO_IK_SOLUTION: 'NO_IK_SOLUTION',
		}
		return labels.get(error_code, f'UNKNOWN_ERROR_CODE_{error_code}')


def main(args: Optional[List[str]] = None) -> None:
	rclpy.init(args=args)
	node = FeasibilityServiceNode()
	executor = MultiThreadedExecutor(num_threads=2)
	executor.add_node(node)
	try:
		executor.spin()
	finally:
		executor.remove_node(node)
		executor.shutdown()
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()