from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
	grasping_control_share = get_package_share_directory('grasping_control')
	ld250_tm12x_moveit_config_share = get_package_share_directory(
		'ld250_tm12x_soft_two_fingers_moveit_config'
	)
	moma_ros_share = get_package_share_directory('moma_ros')

	base_hardware = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			str(Path(moma_ros_share) / 'launch' / 'ld250_tm12x' / 'ld250_tm12x.hardware.launch.py')
		),
		launch_arguments={
			'tm_robot_ip': LaunchConfiguration('tm_robot_ip'),
			'tm_use_simulation': LaunchConfiguration('tm_use_simulation'),
			'use_arm': 'false',
			'use_base': 'true',
			'robot_description_override': 'false',
		}.items(),
		condition=IfCondition(
			AndSubstitution(
				LaunchConfiguration('use_base'),
				NotSubstitution(LaunchConfiguration('use_demo')),
			)
		),
	)

	hardware_with_moveit = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			str(
				Path(ld250_tm12x_moveit_config_share) / 'launch' / 'hardware_with_moveit.launch.py'
			)
		),
		launch_arguments={
			'tm_robot_ip': LaunchConfiguration('tm_robot_ip'),
			'tm_use_simulation': LaunchConfiguration('tm_use_simulation'),
			'no_logging': LaunchConfiguration('no_logging'),
			'launch_servo': LaunchConfiguration('launch_servo'),
		}.items(),
		condition=UnlessCondition(LaunchConfiguration('use_demo')),
	)

	demo = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			str(Path(ld250_tm12x_moveit_config_share) / 'launch' / 'demo.launch.py')
		),
		condition=IfCondition(LaunchConfiguration('use_demo')),
	)

	nav2 = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			str(Path(moma_ros_share) / 'launch' / 'ld250_tm12x' / 'ld250_tm12x.nav2.launch.py')
		),
		condition=IfCondition(
			AndSubstitution(
				LaunchConfiguration('use_nav2'),
				AndSubstitution(
					LaunchConfiguration('use_base'),
					NotSubstitution(LaunchConfiguration('use_demo')),
				),
			)
		),
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
			DeclareLaunchArgument('use_demo', default_value='false'),
			DeclareLaunchArgument('use_base', default_value='false'),
			DeclareLaunchArgument('use_nav2', default_value='false'),
			DeclareLaunchArgument('tm_robot_ip', default_value=''),
			DeclareLaunchArgument('tm_use_simulation', default_value='false'),
			DeclareLaunchArgument('no_logging', default_value='false'),
			DeclareLaunchArgument('launch_servo', default_value='false'),
			DeclareLaunchArgument('arm_action_name', default_value='move_arm_to_pose'),
			DeclareLaunchArgument('move_group_action_name', default_value='move_action'),
			DeclareLaunchArgument('planning_group', default_value='tmr_arm_with_base'),
			DeclareLaunchArgument('planning_frame', default_value='base_link'),
			DeclareLaunchArgument('end_effector_link', default_value='tool_tip'),
			DeclareLaunchArgument('allowed_planning_time', default_value='5.0'),
			DeclareLaunchArgument('num_planning_attempts', default_value='5'),
			DeclareLaunchArgument('max_velocity_scaling', default_value='0.2'),
			DeclareLaunchArgument('max_acceleration_scaling', default_value='0.2'),
			DeclareLaunchArgument('position_tolerance_m', default_value='0.005'),
			DeclareLaunchArgument('orientation_tolerance_rad', default_value='0.1'),
			DeclareLaunchArgument('planning_pipeline_id', default_value=''),
			DeclareLaunchArgument('planner_id', default_value=''),
			DeclareLaunchArgument(
				'workspace_config_path',
				default_value=str(Path(grasping_control_share) / 'config' / 'crlab_table.yaml'),
			),
			base_hardware,
			demo,
			hardware_with_moveit,
			nav2,
			motion_execution_node,
		]
	)