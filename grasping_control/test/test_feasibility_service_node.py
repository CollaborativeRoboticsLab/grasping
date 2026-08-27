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
    request = CheckCartesianPoseFeasibility.Request()
    request.mode = 'arm_only_ik'
    request.frame_id = ''
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
    request.frame_id = node._planning_frame
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
    request.frame_id = node._planning_frame
    request.pose.orientation.w = 1.0
    response = CheckCartesianPoseFeasibility.Response()

    result = node._handle_cartesian_feasibility(request, response)

    assert result.feasible is False
    assert result.failure_reason == 'ik_failed'
    assert result.suggested_fallback == 'move_base_then_arm'
    assert result.message == 'Nearby IK failed with NO_IK_SOLUTION'