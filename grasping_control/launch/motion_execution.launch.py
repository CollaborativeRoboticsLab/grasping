from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _motion_execution_node(context, package_share: str):
	motion_config_file = LaunchConfiguration('motion_config_file').perform(context)
	motion_config_path = Path(motion_config_file).expanduser()
	motion_config = (
		str(motion_config_path)
		if motion_config_path.is_absolute()
		else str(Path(package_share) / 'config' / motion_config_path)
	)
	return [
		Node(
			package='grasping_control',
			executable='scene_manager_node',
			name='scene_manager_node',
			output='screen',
			parameters=[
				{
					'active_scene_topic': LaunchConfiguration('active_scene_topic'),
					'get_active_scene_service_name': LaunchConfiguration('get_active_scene_service_name'),
					'validate_workspace_document_service_name': LaunchConfiguration('validate_workspace_document_service_name'),
					'activate_scene_action_name': LaunchConfiguration('activate_scene_action_name'),
					'load_scene_from_content_action_name': LaunchConfiguration('load_scene_from_content_action_name'),
					'startup_activate_default_scene': LaunchConfiguration('startup_activate_default_scene'),
					'default_scene_name': LaunchConfiguration('default_scene_name'),
					'default_scene_package': LaunchConfiguration('default_scene_package'),
					'default_workspace_file': LaunchConfiguration('workspace_file'),
					'default_scene_revision': LaunchConfiguration('default_scene_revision'),
				}
			],
		),
		Node(
			package='grasping_control',
			executable='motion_execution_node',
			name='motion_execution_node',
			output='screen',
			parameters=[
				motion_config,
				{
					'active_scene_topic': LaunchConfiguration('active_scene_topic'),
					'get_active_scene_service_name': LaunchConfiguration('get_active_scene_service_name'),
				},
			],
		),
		Node(
			package='grasping_control',
			executable='feasibility_service_node',
			name='feasibility_service_node',
			output='screen',
			parameters=[
				motion_config,
				{
					'active_scene_topic': LaunchConfiguration('active_scene_topic'),
					'get_active_scene_service_name': LaunchConfiguration('get_active_scene_service_name'),
				},
			],
			condition=IfCondition(LaunchConfiguration('launch_feasiblity_service')),
		),
	]


def generate_launch_description() -> LaunchDescription:
	grasping_control_share = get_package_share_directory('grasping_control')

	return LaunchDescription(
		[
			DeclareLaunchArgument('motion_config_file', default_value='motion_config.yaml'),
			DeclareLaunchArgument('workspace_file', default_value='config/crlab_table.yaml'),
			DeclareLaunchArgument('default_scene_name', default_value='crlab_table'),
			DeclareLaunchArgument('default_scene_package', default_value='grasping_control'),
			DeclareLaunchArgument('default_scene_revision', default_value=''),
			DeclareLaunchArgument('startup_activate_default_scene', default_value='true'),
			DeclareLaunchArgument('active_scene_topic', default_value='/active_scene'),
			DeclareLaunchArgument('get_active_scene_service_name', default_value='get_active_scene'),
			DeclareLaunchArgument('validate_workspace_document_service_name', default_value='validate_workspace_document'),
			DeclareLaunchArgument('activate_scene_action_name', default_value='activate_scene'),
			DeclareLaunchArgument('load_scene_from_content_action_name', default_value='load_scene_from_content'),
			DeclareLaunchArgument('launch_feasiblity_service', default_value='false'),
			OpaqueFunction(function=_motion_execution_node, args=[grasping_control_share]),
		]
	)