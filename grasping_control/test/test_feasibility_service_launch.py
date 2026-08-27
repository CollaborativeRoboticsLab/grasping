import time
import threading
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState

from grasping_msgs.srv import CheckCartesianPoseFeasibility, CheckJointPoseFeasibility


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='grasping_control',
        executable='feasibility_service_node',
        name='feasibility_service_node',
        output='screen',
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {'service_node': node}


class MockMoveItServices:
    def __init__(self):
        self.node = rclpy.create_node('mock_moveit_services')
        self._planning_joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        self._planning_joint_positions = [0.0] * len(self._planning_joint_names)
        self._ik_joint_positions = [0.1] * len(self._planning_joint_names)
        self._ik_should_succeed = True
        self._state_should_be_valid = True
        self.publish_count = 0
        self.node.create_service(GetPositionIK, '/compute_ik', self._handle_compute_ik)
        self.node.create_service(GetStateValidity, '/check_state_validity', self._handle_state_validity)
        self._joint_state_publisher = self.node.create_publisher(JointState, '/joint_states', 10)
        self._joint_state_timer = self.node.create_timer(0.1, self._publish_joint_state)

    def destroy_node(self):
        self.node.destroy_node()

    def set_ik_result(self, should_succeed):
        self._ik_should_succeed = should_succeed

    def set_state_validity(self, is_valid):
        self._state_should_be_valid = is_valid

    def _publish_joint_state(self):
        message = JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name = list(self._planning_joint_names)
        message.position = list(self._planning_joint_positions)
        self._joint_state_publisher.publish(message)
        self.publish_count += 1

    def _handle_compute_ik(self, request, response):
        del request
        if self._ik_should_succeed:
            response.error_code.val = MoveItErrorCodes.SUCCESS
            response.solution = RobotState()
            response.solution.joint_state = JointState()
            response.solution.joint_state.header.stamp = self.node.get_clock().now().to_msg()
            response.solution.joint_state.name = list(self._planning_joint_names)
            response.solution.joint_state.position = list(self._ik_joint_positions)
            return response

        response.error_code.val = MoveItErrorCodes.NO_IK_SOLUTION
        return response

    def _handle_state_validity(self, request, response):
        del request
        response.valid = self._state_should_be_valid
        return response


class TestFeasibilityServiceLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.client_node = rclpy.create_node('feasibility_service_launch_test')
        self.mock_moveit = MockMoveItServices()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.client_node)
        self.executor.add_node(self.mock_moveit.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def tearDown(self):
        self.executor.shutdown()
        self.executor_thread.join(timeout=2.0)
        self.mock_moveit.destroy_node()
        self.client_node.destroy_node()

    def _wait_until(self, predicate, timeout_sec=10.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        return predicate()

    def _call_service(self, client, request, timeout_sec=5.0):
        future = client.call_async(request)
        assert self._wait_until(future.done, timeout_sec=timeout_sec)
        return future.result()

    def test_services_become_available(self, service_node):
        del service_node
        cartesian_client = self.client_node.create_client(
            CheckCartesianPoseFeasibility, 'check_cartesian_pose_feasibility'
        )
        joint_client = self.client_node.create_client(
            CheckJointPoseFeasibility, 'check_joint_pose_feasibility'
        )

        deadline = time.time() + 10.0
        while time.time() < deadline:
            if cartesian_client.wait_for_service(timeout_sec=0.2) and joint_client.wait_for_service(timeout_sec=0.2):
                break
        assert cartesian_client.wait_for_service(timeout_sec=0.1)
        assert joint_client.wait_for_service(timeout_sec=0.1)

    def test_cartesian_ik_success_and_failure(self, service_node):
        del service_node
        cartesian_client = self.client_node.create_client(
            CheckCartesianPoseFeasibility, 'check_cartesian_pose_feasibility'
        )
        assert cartesian_client.wait_for_service(timeout_sec=10.0)
        assert self._wait_until(lambda: self.mock_moveit.publish_count > 0, timeout_sec=2.0)

        request = CheckCartesianPoseFeasibility.Request()
        request.mode = 'arm_only_ik'
        request.frame_id = 'base_link'
        request.pose.orientation.w = 1.0

        self.mock_moveit.set_ik_result(True)
        success_response = self._call_service(cartesian_client, request)
        assert success_response is not None
        assert success_response.feasible is True
        assert success_response.failure_reason == ''
        assert success_response.suggested_fallback == ''
        assert success_response.joint_state_solution_valid is True
        assert success_response.message == 'Cartesian IK feasibility succeeded.'

        self.mock_moveit.set_ik_result(False)
        failure_response = self._call_service(cartesian_client, request)
        assert failure_response is not None
        assert failure_response.feasible is False
        assert failure_response.failure_reason == 'ik_failed'
        assert failure_response.suggested_fallback == 'move_base_then_arm'
        assert failure_response.message == 'Nearby IK failed with NO_IK_SOLUTION'

    def test_joint_state_validity_success_and_failure(self, service_node):
        del service_node
        joint_client = self.client_node.create_client(
            CheckJointPoseFeasibility, 'check_joint_pose_feasibility'
        )
        assert joint_client.wait_for_service(timeout_sec=10.0)

        request = CheckJointPoseFeasibility.Request()
        request.mode = 'state_validity'
        request.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        request.joint_positions = [0.0] * len(request.joint_names)

        self.mock_moveit.set_state_validity(True)
        valid_response = self._call_service(joint_client, request)
        assert valid_response is not None
        assert valid_response.feasible is True
        assert valid_response.failure_reason == ''
        assert valid_response.suggested_fallback == ''
        assert valid_response.checked_joint_state_valid is True
        assert valid_response.message == 'Joint state is valid in the current planning scene.'

        self.mock_moveit.set_state_validity(False)
        invalid_response = self._call_service(joint_client, request)
        assert invalid_response is not None
        assert invalid_response.feasible is False
        assert invalid_response.failure_reason == 'state_invalid'
        assert invalid_response.suggested_fallback == ''
        assert invalid_response.checked_joint_state_valid is True
        assert invalid_response.message == 'Joint state is invalid in the current planning scene.'