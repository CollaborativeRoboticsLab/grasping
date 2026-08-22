import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy

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


class TestFeasibilityServiceLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_services_become_available(self, service_node):
        del service_node
        client_node = rclpy.create_node('feasibility_service_launch_test')
        try:
            cartesian_client = client_node.create_client(
                CheckCartesianPoseFeasibility, 'check_cartesian_pose_feasibility'
            )
            joint_client = client_node.create_client(
                CheckJointPoseFeasibility, 'check_joint_pose_feasibility'
            )

            deadline = time.time() + 10.0
            while time.time() < deadline:
                if cartesian_client.wait_for_service(timeout_sec=0.2) and joint_client.wait_for_service(timeout_sec=0.2):
                    break
            assert cartesian_client.wait_for_service(timeout_sec=0.1)
            assert joint_client.wait_for_service(timeout_sec=0.1)
        finally:
            client_node.destroy_node()