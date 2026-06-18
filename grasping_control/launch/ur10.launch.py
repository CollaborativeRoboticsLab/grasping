from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
	ur10_moveit_config_share = get_package_share_directory('ur10_moveit_config')

	hardware_with_moveit = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			str(Path(ur10_moveit_config_share) / 'launch' / 'hardware_with_moveit.launch.py')
		),
		launch_arguments={
			'ur_type': LaunchConfiguration('ur_type'),
			'robot_ip': LaunchConfiguration('robot_ip'),
			'reverse_ip': LaunchConfiguration('reverse_ip'),
			'use_sim': LaunchConfiguration('use_sim'),
			'launch_rviz': LaunchConfiguration('launch_rviz'),
			'initial_joint_controller': LaunchConfiguration('initial_joint_controller'),
			'activate_joint_controller': LaunchConfiguration('activate_joint_controller'),
		}.items(),
	)

	motion_execution_node = Node(
		package='grasping_control',
		executable='motion_execution_node',
		name='motion_execution_node',
		output='screen',
		parameters=[
			{
				'action_name': LaunchConfiguration('arm_action_name'),
				'move_group_action_name': LaunchConfiguration('move_group_action_name'),
				'planning_group': LaunchConfiguration('planning_group'),
				'planning_frame': LaunchConfiguration('planning_frame'),
				'end_effector_link': LaunchConfiguration('end_effector_link'),
				'allowed_planning_time': LaunchConfiguration('allowed_planning_time'),
				'num_planning_attempts': LaunchConfiguration('num_planning_attempts'),
				'max_velocity_scaling': LaunchConfiguration('max_velocity_scaling'),
				'max_acceleration_scaling': LaunchConfiguration('max_acceleration_scaling'),
				'position_tolerance_m': LaunchConfiguration('position_tolerance_m'),
				'orientation_tolerance_rad': LaunchConfiguration('orientation_tolerance_rad'),
				'planning_pipeline_id': LaunchConfiguration('planning_pipeline_id'),
				'planner_id': LaunchConfiguration('planner_id'),
				'workspace_config_path': LaunchConfiguration('workspace_config_path'),
			}
		],
	)

	return LaunchDescription(
		[
			DeclareLaunchArgument('ur_type', default_value='ur10'),
			DeclareLaunchArgument('robot_ip', default_value='192.168.10.156'),
			DeclareLaunchArgument('reverse_ip', default_value='192.168.10.130'),
			DeclareLaunchArgument('use_sim', default_value='false'),
			DeclareLaunchArgument('launch_rviz', default_value='true'),
			DeclareLaunchArgument('initial_joint_controller', default_value='scaled_joint_trajectory_controller'),
			DeclareLaunchArgument('activate_joint_controller', default_value='true'),
			DeclareLaunchArgument('arm_action_name', default_value='move_arm_to_pose'),
			DeclareLaunchArgument('move_group_action_name', default_value='move_action'),
			DeclareLaunchArgument('planning_group', default_value='manipulator'),
			DeclareLaunchArgument('planning_frame', default_value='base_link'),
			DeclareLaunchArgument('end_effector_link', default_value='tool0'),
			DeclareLaunchArgument('allowed_planning_time', default_value='5.0'),
			DeclareLaunchArgument('num_planning_attempts', default_value='5'),
			DeclareLaunchArgument('max_velocity_scaling', default_value='0.2'),
			DeclareLaunchArgument('max_acceleration_scaling', default_value='0.2'),
			DeclareLaunchArgument('position_tolerance_m', default_value='0.005'),
			DeclareLaunchArgument('orientation_tolerance_rad', default_value='0.1'),
			DeclareLaunchArgument('planning_pipeline_id', default_value=''),
			DeclareLaunchArgument('planner_id', default_value=''),
			DeclareLaunchArgument('workspace_config_path', default_value=''),
			hardware_with_moveit,
			motion_execution_node,
		]
	)