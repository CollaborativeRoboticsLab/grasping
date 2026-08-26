import pytest
import rclpy

from grasping_control.feasibility_service_node import FeasibilityServiceNode
from grasping_msgs.srv import CheckCartesianPoseFeasibility, CheckJointPoseFeasibility


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