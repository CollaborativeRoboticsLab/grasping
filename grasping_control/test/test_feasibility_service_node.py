import pytest
import rclpy

from grasping_control.feasibility_service_node import FeasibilityServiceNode
from grasping_msgs.srv import CheckCartesianPoseFeasibility, CheckJointPoseFeasibility
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK


@pytest.fixture(scope='module', autouse=True)
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def node():
    created_node = FeasibilityServiceNode()
    yield created_node
    created_node.destroy_node()


def test_cartesian_rejects_unsupported_mode(node):
    request = CheckCartesianPoseFeasibility.Request()
    request.mode = 'invalid_mode'
    response = CheckCartesianPoseFeasibility.Response()

    result = node._handle_cartesian_feasibility(request, response)

    assert result.feasible is False
    assert result.failure_reason == 'unsupported_mode'


def test_cartesian_requires_frame_id(node):
    node._planning_frame = ''
    request = CheckCartesianPoseFeasibility.Request()
    request.mode = 'arm_only_ik'
    request.planning_frame = ''
    response = CheckCartesianPoseFeasibility.Response()

    result = node._handle_cartesian_feasibility(request, response)

    assert result.feasible is False
    assert result.failure_reason == 'invalid_request'


def test_joint_request_requires_matching_lengths(node):
    request = CheckJointPoseFeasibility.Request()
    request.mode = 'plan'
    request.joint_names = ['joint_a']
    request.joint_positions = [0.0, 1.0]
    response = CheckJointPoseFeasibility.Response()

    result = node._handle_joint_feasibility(request, response)

    assert result.feasible is False
    assert result.failure_reason == 'invalid_request'


def test_joint_request_requires_names(node):
    request = CheckJointPoseFeasibility.Request()
    request.mode = 'state_validity'
    response = CheckJointPoseFeasibility.Response()

    result = node._handle_joint_feasibility(request, response)

    assert result.feasible is False
    assert result.failure_reason == 'invalid_request'


def test_cartesian_workspace_rejection_suggests_mobile_fallback(node):
    node._workspace_area_frame = node._planning_frame
    node._workspace_area = {
        'geometry': {
            'corner_points': [
                {'x': 0.0, 'y': 0.0, 'z': 0.0},
                {'x': 1.0, 'y': 0.0, 'z': 0.0},
                {'x': 1.0, 'y': 1.0, 'z': 0.0},
                {'x': 0.0, 'y': 1.0, 'z': 0.0},
            ]
        }
    }

    request = CheckCartesianPoseFeasibility.Request()
    request.mode = 'arm_only_ik'
    request.planning_frame = node._planning_frame
    request.pose.position.x = 2.0
    request.pose.position.y = 0.5
    request.pose.position.z = 0.0
    response = CheckCartesianPoseFeasibility.Response()

    result = node._handle_cartesian_feasibility(request, response)

    assert result.feasible is False
    assert result.failure_reason == 'workspace_area_violation'
    assert result.suggested_fallback == 'move_base_then_arm'


class _FakeFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _FakeComputeIkClient:
    def __init__(self, response):
        self._response = response

    def wait_for_service(self, timeout_sec):
        del timeout_sec
        return True

    def call_async(self, request):
        del request
        return _FakeFuture(self._response)


def test_cartesian_ik_failure_maps_moveit_error_to_response(node, monkeypatch):
    received_at = node.get_clock().now()
    for index, joint_name in enumerate(node._planning_joint_names()):
        node._latest_joint_positions_by_name[joint_name] = float(index)
        node._latest_joint_position_received_at[joint_name] = received_at

    ik_response = GetPositionIK.Response()
    ik_response.error_code.val = MoveItErrorCodes.NO_IK_SOLUTION
    node._compute_ik_client = _FakeComputeIkClient(ik_response)
    monkeypatch.setattr(rclpy, 'spin_until_future_complete', lambda *args, **kwargs: None)

    request = CheckCartesianPoseFeasibility.Request()
    request.mode = 'arm_only_ik'
    request.planning_frame = node._planning_frame
    request.pose.orientation.w = 1.0
    response = CheckCartesianPoseFeasibility.Response()

    result = node._handle_cartesian_feasibility(request, response)

    assert result.feasible is False
    assert result.failure_reason == 'ik_failed'
    assert result.suggested_fallback == 'move_base_then_arm'
    assert result.message == 'Nearby IK failed with NO_IK_SOLUTION'


def test_cartesian_feasibility_uses_grasp_recovery_when_tool_tip_is_feasible(node, monkeypatch):
    request = CheckCartesianPoseFeasibility.Request()
    request.mode = 'arm_only_plan'
    request.planning_frame = node._planning_frame
    request.target_frame = 'tcp'
    request.pose.orientation.w = 1.0
    response = CheckCartesianPoseFeasibility.Response()

    node._target_pose_in_workspace_area = lambda pose: True
    node._grasp_pose_recovery_frames = lambda target_frame: ('tcp', 'tool_tip') if target_frame == 'tcp' else None
    node._lookup_recovery_frame_offset = lambda primary_frame, recovery_frame: (0.0, 0.0, 0.2)

    def _evaluate(target_pose, target_frame, planning_config, mode):
        del planning_config, mode
        if target_frame == 'tool_tip':
            return {
                'feasible': True,
                'failure_reason': '',
                'suggested_fallback': '',
                'message': 'Cartesian arm-only planning feasibility succeeded via nearby IK.',
                'joint_state': None,
                'planning_pose': target_pose,
            }
        if round(target_pose.pose.position.z, 3) == -0.1:
            return {
                'feasible': True,
                'failure_reason': '',
                'suggested_fallback': '',
                'message': 'Cartesian arm-only planning feasibility succeeded via nearby IK.',
                'joint_state': None,
                'planning_pose': target_pose,
            }
        return {
            'feasible': False,
            'failure_reason': 'planning_failed',
            'suggested_fallback': 'move_base_then_arm',
            'message': 'MoveGroup failed with FAILURE (99999)',
            'joint_state': None,
            'planning_pose': target_pose,
        }

    node._evaluate_cartesian_target = _evaluate
    node._motion_planning_config = lambda: object()
    monkeypatch.setattr('grasping_control.feasibility_service_node.transform_pose_to_frame', lambda *args, **kwargs: args[2])

    result = node._handle_cartesian_feasibility(request, response)

    assert result.feasible is True
    assert result.message.endswith('using grasp recovery between tcp and tool_tip (attempt 1/3, fraction=0.500).')