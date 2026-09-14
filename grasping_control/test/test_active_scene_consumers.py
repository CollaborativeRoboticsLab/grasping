from geometry_msgs.msg import Point, PoseStamped

import grasping_control.motion_execution_node as motion_execution_module

from grasping_control.feasibility_service_node import FeasibilityServiceNode
from grasping_control.motion_execution_node import MotionExecutionNode
from grasping_msgs.msg import ActiveScene
from grasping_msgs.srv import ListNamedPoses


class _Logger:
    def warn(self, _message: str) -> None:
        pass


def _active_scene_message(enabled: bool = True) -> ActiveScene:
    message = ActiveScene()
    message.workspace_base_frame = 'world'
    message.workspace_area_enabled = enabled
    if enabled:
        for x_value, y_value in ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)):
            point = Point()
            point.x = x_value
            point.y = y_value
            point.z = 0.0
            message.workspace_area_corner_points.append(point)
    return message


def test_motion_execution_updates_workspace_area_from_active_scene():
    node = MotionExecutionNode.__new__(MotionExecutionNode)
    node._planning_frame = 'base_link'
    node._workspace_area = None
    node._workspace_area_frame = 'base_link'
    publish_calls = []
    node._publish_workspace_area_marker = lambda: publish_calls.append(True)
    node.get_logger = lambda: _Logger()

    node._update_workspace_area_from_active_scene(_active_scene_message())

    assert node._workspace_area_frame == 'world'
    assert node._workspace_area is not None
    assert len(node._workspace_area['geometry']['corner_points']) == 4
    assert publish_calls == [True]


def test_feasibility_updates_workspace_area_from_active_scene_and_filters_pose():
    node = FeasibilityServiceNode.__new__(FeasibilityServiceNode)
    node._planning_frame = 'base_link'
    node._workspace_area = None
    node._workspace_area_frame = 'base_link'
    node.get_logger = lambda: _Logger()

    node._update_workspace_area_from_active_scene(_active_scene_message())

    inside_pose = PoseStamped()
    inside_pose.header.frame_id = 'world'
    inside_pose.pose.position.x = 0.5
    inside_pose.pose.position.y = 0.5
    inside_pose.pose.position.z = 0.0

    outside_pose = PoseStamped()
    outside_pose.header.frame_id = 'world'
    outside_pose.pose.position.x = 1.5
    outside_pose.pose.position.y = 0.5
    outside_pose.pose.position.z = 0.0

    assert node._target_pose_in_workspace_area(inside_pose) is True
    assert node._target_pose_in_workspace_area(outside_pose) is False


class _Parameter:
    def __init__(self, value):
        self.value = value


def test_motion_execution_lists_named_pose_descriptions():
    values = {
        'poses_names': ['workspace_center', 'pre_grasp'],
        'poses_values.workspace_center.target_frame': 'camera_link',
        'poses_values.workspace_center.pose': [0.0, 0.0, 0.30, 0.0, 0.0, 0.0],
        'poses_values.workspace_center.description': 'Observation pose over the workspace.',
        'poses_values.pre_grasp.target_frame': 'tcp',
        'poses_values.pre_grasp.pose': [0.0, 0.0, 0.30, 0.0, 0.0, 0.0],
        'poses_values.pre_grasp.description': 'Approach pose before grasping.',
    }

    node = MotionExecutionNode.__new__(MotionExecutionNode)
    node.get_parameter = lambda name: _Parameter(values.get(name, ''))

    response = node._handle_list_named_poses(ListNamedPoses.Request(), ListNamedPoses.Response())

    assert [descriptor.pose_name for descriptor in response.named_poses] == ['workspace_center', 'pre_grasp']
    assert [descriptor.description for descriptor in response.named_poses] == [
        'Observation pose over the workspace.',
        'Approach pose before grasping.',
    ]


def test_motion_execution_reports_ik_failure_when_fallback_plan_also_fails(monkeypatch):
    values = {
        'move_group_action_name': 'move_action',
        'prefer_nearby_ik': True,
        'fallback_to_pose_planning_on_ik_failure': True,
        'grasp_pose_recovery_enabled': False,
        'pose_relax_search_enabled': False,
        'pose_relax_limits': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }

    node = MotionExecutionNode.__new__(MotionExecutionNode)
    node.get_parameter = lambda name: _Parameter(values.get(name, ''))
    node._get_bool_parameter = lambda name: bool(values[name])
    node.get_logger = lambda: _Logger()
    node._movegroup_client = type(
        '_MoveGroupClient',
        (),
        {'wait_for_server': staticmethod(lambda timeout_sec: True)},
    )()
    node._motion_planning_config = lambda: object()
    node._current_robot_state_or_none = lambda: None
    node._joint_goal_from_nearby_ik = lambda target_pose, target_frame: (
        False,
        {},
        "Nearby IK failed with NO_IK_SOLUTION (-31) for group 'tm12s_arm' and link 'tcp'.",
    )
    node._execute_move_group_goal = lambda goal: (False, 'MoveGroup failed with FAILURE (99999)')

    monkeypatch.setattr(motion_execution_module, 'build_move_group_goal', lambda *args, **kwargs: object())

    target_pose = PoseStamped()
    target_pose.header.frame_id = 'world'

    ok, message = node._move_to_pose(target_pose)

    assert ok is False
    assert message == (
        "Nearby IK failed with NO_IK_SOLUTION (-31) for group 'tm12s_arm' and link 'tcp'. "
        'Fallback pose-constrained planning also failed: MoveGroup failed with FAILURE (99999)'
    )


def test_motion_execution_searches_relaxed_pose_candidates_when_exact_goal_fails():
    values = {
        'move_group_action_name': 'move_action',
        'prefer_nearby_ik': True,
        'fallback_to_pose_planning_on_ik_failure': True,
        'grasp_pose_recovery_enabled': False,
        'pose_relax_search_enabled': True,
        'pose_relax_limits': [0.0, 0.0, 0.10, 0.0, 0.0, 0.0],
    }

    node = MotionExecutionNode.__new__(MotionExecutionNode)
    node.get_parameter = lambda name: _Parameter(values.get(name, ''))
    node._get_bool_parameter = lambda name: bool(values[name])
    node.get_logger = lambda: _Logger()
    node._movegroup_client = type(
        '_MoveGroupClient',
        (),
        {'wait_for_server': staticmethod(lambda timeout_sec: True)},
    )()
    node._motion_planning_config = lambda: object()
    node._target_pose_in_workspace_area = lambda pose: True
    node._pose_stamped_from_values = lambda frame, pose_values: _pose_from_values(frame, pose_values)

    attempted_z_values = []

    def _try_pose_goal(pose, target_frame, planning_config):
        del target_frame, planning_config
        attempted_z_values.append(round(pose.pose.position.z, 3))
        if pose.pose.position.z > 1.0:
            return True, 'Arm motion completed successfully.'
        return False, 'MoveGroup failed with FAILURE (99999)'

    node._try_pose_goal = _try_pose_goal

    target_pose = PoseStamped()
    target_pose.header.frame_id = 'world'
    target_pose.pose.position.z = 1.0
    target_pose.pose.orientation.w = 1.0

    ok, message = node._move_to_pose(target_pose)

    assert ok is True
    assert attempted_z_values[:2] == [1.0, 1.05]
    assert 'using relaxed pose offsets (z=+0.050)' in message


def test_motion_execution_uses_grasp_recovery_between_tcp_and_tool_tip():
    values = {
        'move_group_action_name': 'move_action',
        'prefer_nearby_ik': True,
        'fallback_to_pose_planning_on_ik_failure': True,
        'grasp_pose_recovery_enabled': True,
        'grasp_pose_recovery_tool_frame': 'tool_tip',
        'grasp_pose_recovery_recalculate_attempts': 2,
        'end_effector_link': 'tcp',
        'pose_relax_search_enabled': False,
        'pose_relax_limits': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }

    node = MotionExecutionNode.__new__(MotionExecutionNode)
    node.get_parameter = lambda name: _Parameter(values.get(name, ''))
    node._get_bool_parameter = lambda name: bool(values[name])
    node.get_logger = lambda: _Logger()
    node._movegroup_client = type(
        '_MoveGroupClient',
        (),
        {'wait_for_server': staticmethod(lambda timeout_sec: True)},
    )()
    node._motion_planning_config = lambda: object()
    node._lookup_recovery_frame_offset = lambda primary_frame, recovery_frame: (0.0, 0.0, 0.2)
    node._plan_pose_goal = lambda pose, target_frame, planning_config: (target_frame == 'tool_tip', 'tool_tip feasible')

    attempts = []

    def _try_pose_goal(pose, target_frame, planning_config):
        del planning_config
        attempts.append((target_frame, round(pose.pose.position.z, 3)))
        if target_frame == 'tcp' and round(pose.pose.position.z, 3) == 0.9:
            return True, 'Arm motion completed successfully.'
        return False, 'MoveGroup failed with FAILURE (99999)'

    node._try_pose_goal = _try_pose_goal

    target_pose = PoseStamped()
    target_pose.header.frame_id = 'world'
    target_pose.pose.position.z = 1.0
    target_pose.pose.orientation.w = 1.0

    ok, message = node._try_grasp_pose_recovery(target_pose, 'tcp', object())

    assert ok is True
    assert attempts == [('tcp', 0.9)]
    assert 'using grasp recovery between tcp and tool_tip' in message


def _pose_from_values(frame: str, pose_values):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = pose_values[0]
    pose.pose.position.y = pose_values[1]
    pose.pose.position.z = pose_values[2]
    pose.pose.orientation.w = 1.0
    return pose