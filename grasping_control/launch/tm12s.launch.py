from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
	grasping_control_share = get_package_share_directory('grasping_control')
	tm12s_moveit_config_share = get_package_share_directory('tm12s_moveit_config')

	hardware_with_moveit = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			str(Path(tm12s_moveit_config_share) / 'launch' / 'hardware_with_moveit.launch.py')
		),
		condition=UnlessCondition(LaunchConfiguration('use_demo')),
	)

	demo = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			str(Path(tm12s_moveit_config_share) / 'launch' / 'demo.launch.py')
		),
		condition=IfCondition(LaunchConfiguration('use_demo')),
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
			demo,
			hardware_with_moveit,
			motion_execution,
		]
	)