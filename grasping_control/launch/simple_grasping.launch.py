from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
	return LaunchDescription(
		[
			DeclareLaunchArgument('arm_action_name', default_value='/move_arm_to_named_pose'),
			DeclareLaunchArgument('gripper_action_name', default_value='/gripper_command'),
			DeclareLaunchArgument(
				'sequence',
				default_value='[[pose, pre_grasp], [pose, workspace_center], [gripper, open], [pose, grasp_pose], [gripper, close], [pose, post_grasp]]',
			),
			DeclareLaunchArgument('open_position', default_value='0.09'),
			DeclareLaunchArgument('open_max_effort', default_value='0.0'),
			DeclareLaunchArgument('close_position', default_value='0.0'),
			DeclareLaunchArgument('close_max_effort', default_value='5.0'),
			DeclareLaunchArgument('server_timeout_sec', default_value='10.0'),
			DeclareLaunchArgument('result_timeout_sec', default_value='120.0'),
			Node(
				package='grasping_control',
				executable='simple_grasping',
				name='simple_grasping',
				output='screen',
				parameters=[
					{
						'arm_action_name': LaunchConfiguration('arm_action_name'),
						'gripper_action_name': LaunchConfiguration('gripper_action_name'),
						'sequence': LaunchConfiguration('sequence'),
						'open_position': LaunchConfiguration('open_position'),
						'open_max_effort': LaunchConfiguration('open_max_effort'),
						'close_position': LaunchConfiguration('close_position'),
						'close_max_effort': LaunchConfiguration('close_max_effort'),
						'server_timeout_sec': LaunchConfiguration('server_timeout_sec'),
						'result_timeout_sec': LaunchConfiguration('result_timeout_sec'),
					}
				],
			),
		]
	)
