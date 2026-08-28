from geometry_msgs.msg import Point, PoseStamped

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