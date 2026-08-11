from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Optional

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
	AllowedCollisionEntry,
	AllowedCollisionMatrix,
	BoundingVolume,
	Constraints,
	JointConstraint,
	MotionPlanRequest,
	OrientationConstraint,
	PlanningOptions,
	PositionConstraint,
	RobotState,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from grasping_control.common import Quaternion, coerce_string_sequence, normalize_quaternion


@dataclass(frozen=True)
class MotionPlanningConfig:
	"""
	@brief Immutable planning settings used to build MoveIt requests.
	"""

	planning_frame: str
	planning_group: str
	allowed_planning_time: float
	num_planning_attempts: int
	max_velocity_scaling: float
	max_acceleration_scaling: float
	enable_cartesian_vel_limit: bool
	max_cartesian_velocity: tuple[float, float, float]
	max_cartesian_speed: float
	position_tolerance_m: float
	orientation_tolerance_rad: float
	end_effector_link: str
	joint_goal_tolerance_rad: float = 1e-3
	planning_pipeline_id: str = ''
	planner_id: str = ''


def build_move_group_goal(
	target_pose: PoseStamped,
	config: MotionPlanningConfig,
	target_frame: Optional[str] = None,
	start_state: Optional[RobotState] = None,
) -> MoveGroup.Goal:
	"""
	@brief Build a MoveGroup goal for a target pose.

	@param target_pose Goal pose already expressed in the planning frame.
	@param config Planning configuration.
	@param target_frame Robot frame/link that should reach the target pose.
	@return Configured MoveGroup goal.
	"""
	goal = MoveGroup.Goal()
	goal.request = build_motion_plan_request(target_pose, config, target_frame, start_state)
	goal.planning_options = PlanningOptions()
	goal.planning_options.plan_only = False
	goal.planning_options.look_around = False
	goal.planning_options.replan = False
	goal.planning_options.replan_attempts = 0
	return goal


def build_joint_move_group_goal(
	target_joint_state: JointState,
	config: MotionPlanningConfig,
	target_frame: Optional[str] = None,
	start_state: Optional[RobotState] = None,
) -> MoveGroup.Goal:
	"""
	@brief Build a MoveGroup goal for a joint-space target.

	@param target_joint_state Joint target for the planning group.
	@param config Planning configuration.
	@param target_frame Robot frame/link that should satisfy the Cartesian speed cap.
	@param start_state Robot state used as the planner start state.
	@return Configured MoveGroup goal.
	"""
	goal = MoveGroup.Goal()
	goal.request = build_joint_motion_plan_request(target_joint_state, config, target_frame, start_state)
	goal.planning_options = PlanningOptions()
	goal.planning_options.plan_only = False
	goal.planning_options.look_around = False
	goal.planning_options.replan = False
	goal.planning_options.replan_attempts = 0
	return goal


def build_motion_plan_request(
	target_pose: PoseStamped,
	config: MotionPlanningConfig,
	target_frame: Optional[str] = None,
	start_state: Optional[RobotState] = None,
) -> MotionPlanRequest:
	"""
	@brief Construct a MoveIt motion planning request for a pose goal.

	@param target_pose Goal pose expressed in the planning frame.
	@param config Planning configuration.
	@param target_frame Robot frame/link that should reach the target pose.
	@return Configured MotionPlanRequest instance.
	"""
	request = MotionPlanRequest()
	request.group_name = config.planning_group
	request.allowed_planning_time = config.allowed_planning_time
	request.num_planning_attempts = config.num_planning_attempts
	request.max_velocity_scaling_factor = config.max_velocity_scaling
	request.max_acceleration_scaling_factor = config.max_acceleration_scaling
	_apply_cartesian_speed_limit(request, config, target_frame)

	if config.planning_pipeline_id:
		request.pipeline_id = config.planning_pipeline_id
	if config.planner_id:
		request.planner_id = config.planner_id

	request.start_state = start_state if start_state is not None else RobotState()
	request.goal_constraints = [pose_to_constraints(target_pose, config, target_frame)]
	return request


def build_joint_motion_plan_request(
	target_joint_state: JointState,
	config: MotionPlanningConfig,
	target_frame: Optional[str] = None,
	start_state: Optional[RobotState] = None,
) -> MotionPlanRequest:
	"""
	@brief Construct a MoveIt motion planning request for a joint goal.

	@param target_joint_state Joint target for the planning group.
	@param config Planning configuration.
	@param target_frame Robot frame/link that should satisfy the Cartesian speed cap.
	@param start_state Robot state used as the planner start state.
	@return Configured MotionPlanRequest instance.
	"""
	request = MotionPlanRequest()
	request.group_name = config.planning_group
	request.allowed_planning_time = config.allowed_planning_time
	request.num_planning_attempts = config.num_planning_attempts
	request.max_velocity_scaling_factor = config.max_velocity_scaling
	request.max_acceleration_scaling_factor = config.max_acceleration_scaling
	_apply_cartesian_speed_limit(request, config, target_frame)

	if config.planning_pipeline_id:
		request.pipeline_id = config.planning_pipeline_id
	if config.planner_id:
		request.planner_id = config.planner_id

	request.start_state = start_state if start_state is not None else RobotState()
	request.goal_constraints = [joint_state_to_constraints(target_joint_state, config.joint_goal_tolerance_rad)]
	return request


def _apply_cartesian_speed_limit(
	request: MotionPlanRequest,
	config: MotionPlanningConfig,
	target_frame: Optional[str],
) -> None:
	"""
	@brief Apply an optional end-effector Cartesian speed cap to the request.

	@param request Request being populated.
	@param config Planning configuration.
	@param target_frame Optional target link override.
	"""
	if not config.enable_cartesian_vel_limit:
		return
	if config.max_cartesian_speed <= 0.0:
		return

	request.cartesian_speed_end_effector_link = str(target_frame or config.end_effector_link)
	request.max_cartesian_speed = config.max_cartesian_speed


def pose_to_constraints(
	target_pose: PoseStamped,
	config: MotionPlanningConfig,
	target_frame: Optional[str] = None,
) -> Constraints:
	"""
	@brief Convert a target pose into MoveIt position and orientation constraints.

	@param target_pose Goal pose expressed in the planning frame.
	@param config Planning configuration.
	@param target_frame Robot frame/link that should reach the target pose.
	@return Constraints object for the planner.
	"""
	ee_link = str(target_frame or config.end_effector_link)

	sphere = SolidPrimitive()
	sphere.type = SolidPrimitive.SPHERE
	sphere.dimensions = [max(1e-4, config.position_tolerance_m)]

	volume = BoundingVolume()
	volume.primitives = [sphere]
	volume.primitive_poses = [target_pose.pose]

	position_constraint = PositionConstraint()
	position_constraint.header.frame_id = config.planning_frame
	position_constraint.link_name = ee_link
	position_constraint.constraint_region = volume

	normalized = normalize_quaternion(
		Quaternion(
			target_pose.pose.orientation.x,
			target_pose.pose.orientation.y,
			target_pose.pose.orientation.z,
			target_pose.pose.orientation.w,
		)
	)
	orientation_constraint = OrientationConstraint()
	orientation_constraint.header.frame_id = config.planning_frame
	orientation_constraint.link_name = ee_link
	orientation_constraint.orientation.x = normalized.x
	orientation_constraint.orientation.y = normalized.y
	orientation_constraint.orientation.z = normalized.z
	orientation_constraint.orientation.w = normalized.w
	orientation_constraint.absolute_x_axis_tolerance = config.orientation_tolerance_rad
	orientation_constraint.absolute_y_axis_tolerance = config.orientation_tolerance_rad
	orientation_constraint.absolute_z_axis_tolerance = config.orientation_tolerance_rad
	orientation_constraint.weight = 1.0

	constraints = Constraints()
	constraints.position_constraints = [position_constraint]
	constraints.orientation_constraints = [orientation_constraint]
	return constraints


def joint_state_to_constraints(target_joint_state: JointState, tolerance_rad: float) -> Constraints:
	"""
	@brief Convert a joint target into MoveIt joint constraints.

	@param target_joint_state Joint target for the planner.
	@param tolerance_rad Symmetric tolerance applied to each joint target.
	@return Constraints object for the planner.
	"""
	constraints = Constraints()
	constraints.joint_constraints = []
	for joint_name, position in zip(target_joint_state.name, target_joint_state.position):
		joint_constraint = JointConstraint()
		joint_constraint.joint_name = str(joint_name)
		joint_constraint.position = float(position)
		joint_constraint.tolerance_above = tolerance_rad
		joint_constraint.tolerance_below = tolerance_rad
		joint_constraint.weight = 1.0
		constraints.joint_constraints.append(joint_constraint)
	return constraints


def robot_state_from_joint_state(joint_state: JointState) -> RobotState:
	"""
	@brief Wrap a JointState inside a MoveIt RobotState.

	@param joint_state Joint state to wrap.
	@return RobotState populated with the supplied joint state.
	"""
	robot_state = RobotState()
	robot_state.joint_state = joint_state
	return robot_state


def allowed_collision_pairs_from_workspace(
	workspace_config: dict[str, Any],
	collision_object_names: list[str],
) -> list[tuple[str, str]]:
	"""
	@brief Collect configured allowed-collision pairs for loaded workspace objects.

	@param workspace_config Workspace configuration loaded from ROS parameters.
	@param collision_object_names Object ids that were added to the planning scene.
	@return Unique object-link allowance pairs.
	"""
	valid_objects = set(collision_object_names)
	pairs: list[tuple[str, str]] = []

	for workspace_object in workspace_config.get('objects', []):
		object_name = str(workspace_object.get('name', '')).strip()
		if object_name not in valid_objects:
			continue
		for link_name in coerce_string_sequence(workspace_object.get('allowed_collision_links', [])):
			pair = (object_name, link_name)
			if pair not in pairs:
				pairs.append(pair)

	return pairs


def append_allowed_collision_pairs(
	matrix: AllowedCollisionMatrix,
	pairs: list[tuple[str, str]],
) -> AllowedCollisionMatrix:
	"""
	@brief Copy a collision matrix and mark the requested pairs as allowed.

	@param matrix Existing allowed collision matrix.
	@param pairs Object-link or link-link pairs to allow.
	@return Copied and updated collision matrix.
	"""
	updated_matrix = deepcopy(matrix)
	for first_name, second_name in pairs:
		set_allowed_collision(updated_matrix, first_name, second_name)
	return updated_matrix


def set_allowed_collision(
	matrix: AllowedCollisionMatrix,
	first_name: str,
	second_name: str,
) -> None:
	"""
	@brief Mark one collision pair as allowed in an existing collision matrix.

	@param matrix Matrix to mutate.
	@param first_name First link/object name.
	@param second_name Second link/object name.
	"""
	first_index = ensure_allowed_collision_entry(matrix, first_name)
	second_index = ensure_allowed_collision_entry(matrix, second_name)
	matrix.entry_values[first_index].enabled[second_index] = True
	matrix.entry_values[second_index].enabled[first_index] = True


def ensure_allowed_collision_entry(matrix: AllowedCollisionMatrix, entry_name: str) -> int:
	"""
	@brief Ensure a link/object has a row and column in the allowed collision matrix.

	@param matrix Matrix to mutate.
	@param entry_name Link or object name.
	@return Index of the entry.
	"""
	normalize_allowed_collision_matrix(matrix)
	if entry_name in matrix.entry_names:
		return matrix.entry_names.index(entry_name)

	matrix.entry_names.append(entry_name)
	for entry in matrix.entry_values:
		entry.enabled.append(False)

	new_entry = AllowedCollisionEntry()
	new_entry.enabled = [False] * len(matrix.entry_names)
	matrix.entry_values.append(new_entry)
	return len(matrix.entry_names) - 1


def normalize_allowed_collision_matrix(matrix: AllowedCollisionMatrix) -> None:
	"""
	@brief Make sure the matrix dimensions match its entry names.

	@param matrix Matrix to normalize in place.
	"""
	size = len(matrix.entry_names)
	while len(matrix.entry_values) < size:
		entry = AllowedCollisionEntry()
		entry.enabled = [False] * size
		matrix.entry_values.append(entry)
	for entry in matrix.entry_values:
		if len(entry.enabled) < size:
			entry.enabled.extend([False] * (size - len(entry.enabled)))