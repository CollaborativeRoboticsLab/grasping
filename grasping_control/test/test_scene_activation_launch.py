import time
import unittest

from geometry_msgs.msg import Pose
from grasping_msgs.action import LoadSceneFromContent
from grasping_msgs.msg import ActiveScene
from grasping_msgs.srv import CheckCartesianPoseFeasibility, GetActiveScene
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from visualization_msgs.msg import Marker


WORKSPACE_DOCUMENT = """
motion_execution_node:
  ros__parameters:
    workspace:
      version: 1
      updated_at: '2026-08-26T00:00:00+00:00'
      base_frame: world
      tool_frame: tool_tip
      ground_plane_z: 0.0
    workspace_area:
      enabled: true
      geometry:
        type: square
        dimensions: [1.0, 1.0]
        pose:
          position: [0.5, 0.5, 0.0]
          orientation: [0.0, 0.0, 0.0, 1.0]
        corner_points:
          x: [0.0, 1.0, 1.0, 0.0]
          y: [0.0, 0.0, 1.0, 1.0]
          z: [0.0, 0.0, 0.0, 0.0]
    workspace_objects: []
"""


@pytest.mark.launch_test
def generate_test_description():
    scene_manager = launch_ros.actions.Node(
        package='grasping_control',
        executable='scene_manager_node',
        name='scene_manager_node',
        output='screen',
    )
    motion_execution = launch_ros.actions.Node(
        package='grasping_control',
        executable='motion_execution_node',
        name='motion_execution_node',
        output='screen',
    )
    feasibility_service = launch_ros.actions.Node(
        package='grasping_control',
        executable='feasibility_service_node',
        name='feasibility_service_node',
        output='screen',
    )
    return launch.LaunchDescription(
        [scene_manager, motion_execution, feasibility_service, launch_testing.actions.ReadyToTest()]
    ), {}


class TestSceneActivationLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('scene_activation_launch_test')
        self.active_scene_messages = []
        self.marker_messages = []
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(
            ActiveScene,
            '/active_scene',
            lambda message: self.active_scene_messages.append(message),
            transient_qos,
        )
        self.node.create_subscription(
            Marker,
            '/workspace_area_marker',
            lambda message: self.marker_messages.append(message),
            transient_qos,
        )

    def tearDown(self):
        self.node.destroy_node()

    def _spin_until(self, predicate, timeout_sec=10.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return predicate()

    def test_scene_activation_updates_active_scene_and_consumers(self):
        action_client = ActionClient(self.node, LoadSceneFromContent, 'load_scene_from_content')
        assert action_client.wait_for_server(timeout_sec=10.0)

        goal = LoadSceneFromContent.Goal()
        goal.scene_name = 'test_scene'
        goal.package_name = ''
        goal.workspace_file = 'inline.yaml'
        goal.workspace_document = WORKSPACE_DOCUMENT
        goal.force_reload = True

        send_future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        assert goal_handle is not None
        assert goal_handle.accepted

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=10.0)
        wrapped_result = result_future.result()
        assert wrapped_result is not None
        assert wrapped_result.result.success is True
        assert wrapped_result.result.active_scene.reference.scene_name == 'test_scene'

        assert self._spin_until(lambda: len(self.active_scene_messages) > 0)
        assert self._spin_until(lambda: len(self.marker_messages) > 0)

        active_scene_client = self.node.create_client(GetActiveScene, 'get_active_scene')
        assert active_scene_client.wait_for_service(timeout_sec=5.0)
        active_scene_future = active_scene_client.call_async(GetActiveScene.Request())
        rclpy.spin_until_future_complete(self.node, active_scene_future, timeout_sec=5.0)
        active_scene_response = active_scene_future.result()
        assert active_scene_response is not None
        assert active_scene_response.active is True
        assert active_scene_response.active_scene.workspace_base_frame == 'world'

        latest_marker = self.marker_messages[-1]
        assert latest_marker.action == Marker.ADD
        assert latest_marker.header.frame_id == 'world'
        assert len(latest_marker.points) == 6

        feasibility_client = self.node.create_client(
            CheckCartesianPoseFeasibility,
            'check_cartesian_pose_feasibility',
        )
        assert feasibility_client.wait_for_service(timeout_sec=5.0)
        feasibility_request = CheckCartesianPoseFeasibility.Request()
        feasibility_request.mode = 'arm_only_ik'
        feasibility_request.frame_id = 'world'
        feasibility_request.pose = Pose()
        feasibility_request.pose.position.x = 2.0
        feasibility_request.pose.position.y = 0.5
        feasibility_request.pose.position.z = 0.0
        feasibility_request.pose.orientation.w = 1.0

        feasibility_future = feasibility_client.call_async(feasibility_request)
        rclpy.spin_until_future_complete(self.node, feasibility_future, timeout_sec=5.0)
        feasibility_response = feasibility_future.result()
        assert feasibility_response is not None
        assert feasibility_response.feasible is False
        assert feasibility_response.failure_reason == 'workspace_area_violation'
        assert feasibility_response.suggested_fallback == 'move_base_then_arm'