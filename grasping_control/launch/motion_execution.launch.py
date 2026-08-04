from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _workspace_root_from_share(package_share: str) -> Path:
	share_path = Path(package_share)
	for parent in share_path.parents:
		if parent.name == 'install':
			return parent.parent
	return Path.cwd()


def _resolve_workspace_config(context, package_share: str) -> str:
	workspace_file = LaunchConfiguration('workspace_file').perform(context)
	workspace_path = Path(workspace_file).expanduser()

	if workspace_path.is_absolute() and workspace_path.exists():
		return str(workspace_path)

	package_config_path = Path(package_share) / 'config' / workspace_path
	if package_config_path.exists():
		return str(package_config_path)

	workspace_root_path = _workspace_root_from_share(package_share) / workspace_path
	if workspace_root_path.exists():
		return str(workspace_root_path)

	return str(Path(package_share) / 'config' / 'workspace_empty.yaml')


def _motion_execution_node(context, package_share: str):
	motion_config_file = LaunchConfiguration('motion_config_file').perform(context)
	motion_config_path = Path(motion_config_file).expanduser()
	motion_config = (
		str(motion_config_path)
		if motion_config_path.is_absolute()
		else str(Path(package_share) / 'config' / motion_config_path)
	)
	workspace_config = _resolve_workspace_config(context, package_share)
	return [
		Node(
			package='grasping_control',
			executable='motion_execution_node',
			name='motion_execution_node',
			output='screen',
			parameters=[
				motion_config,
				workspace_config,
			],
		)
	]


def generate_launch_description() -> LaunchDescription:
	grasping_control_share = get_package_share_directory('grasping_control')

	return LaunchDescription(
		[
			DeclareLaunchArgument('motion_config_file', default_value='motion_config.yaml'),
			DeclareLaunchArgument('workspace_file', default_value='crlab_table.yaml'),
			OpaqueFunction(function=_motion_execution_node, args=[grasping_control_share]),
		]
	)