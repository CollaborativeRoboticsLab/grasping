from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
	grasping_node = Node(
		package='grasping',
		executable='grasping_node',
		name='grasping_node',
		output='screen',
		parameters=[
			{
				'server_mode': LaunchConfiguration('server_mode'),
				'anygrasp_service': LaunchConfiguration('anygrasp_service'),
				'arm_action_name': LaunchConfiguration('arm_action_name'),
				'do_post_grasp_move': LaunchConfiguration('do_post_grasp_move'),
				'open_action_name': LaunchConfiguration('open_action_name'),
				'close_action_name': LaunchConfiguration('close_action_name'),
			}
		],
	)

	return LaunchDescription(
		[
			DeclareLaunchArgument('server_mode', default_value='true'),
			DeclareLaunchArgument('anygrasp_service', default_value='detection'),
			DeclareLaunchArgument('arm_action_name', default_value='move_arm_to_pose'),
			DeclareLaunchArgument('open_action_name', default_value='/open_gripper'),
			DeclareLaunchArgument('close_action_name', default_value='/close_gripper'),
			DeclareLaunchArgument('do_post_grasp_move', default_value='true'),
			grasping_node,
		]
	)