from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution


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

	motion_execution = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			str(Path(grasping_control_share) / 'launch' / 'motion_execution.launch.py')
		),
		launch_arguments={
			'workspace_file': LaunchConfiguration('workspace_file'),
		}.items(),
	)

	return LaunchDescription(
		[
			DeclareLaunchArgument('use_demo', default_value='false'),
			DeclareLaunchArgument('workspace_file', default_value='crlab_table.yaml'),
			DeclareLaunchArgument('use_base', default_value='false'),
			DeclareLaunchArgument('use_nav2', default_value='false'),
			DeclareLaunchArgument('tm_robot_ip', default_value=''),
			DeclareLaunchArgument('tm_use_simulation', default_value='false'),
			DeclareLaunchArgument('no_logging', default_value='false'),
			DeclareLaunchArgument('launch_servo', default_value='false'),
			base_hardware,
			demo,
			hardware_with_moveit,
			nav2,
			motion_execution,
		]
	)