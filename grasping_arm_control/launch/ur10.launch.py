from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
	ur_robot_driver_share = get_package_share_directory('ur_robot_driver')
	ur_moveit_config_share = get_package_share_directory('ur_moveit_config')
	grasping_arm_control_share = get_package_share_directory('grasping_arm_control')

	ur_type = LaunchConfiguration('ur_type')
	robot_ip = LaunchConfiguration('robot_ip')
	reverse_ip = LaunchConfiguration('reverse_ip')
	kinematics_params_file = LaunchConfiguration('kinematics_params_file')
	launch_rviz = LaunchConfiguration('launch_rviz')
	initial_joint_controller = LaunchConfiguration('initial_joint_controller')
	activate_joint_controller = LaunchConfiguration('activate_joint_controller')

	ur_control_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			str(Path(ur_robot_driver_share) / 'launch' / 'ur_control.launch.py')
		),
		launch_arguments={
			'ur_type': ur_type,
			'robot_ip': robot_ip,
			'reverse_ip': reverse_ip,
			'kinematics_params_file': kinematics_params_file,
			'initial_joint_controller': initial_joint_controller,
			'activate_joint_controller': activate_joint_controller,
		}.items(),
	)

	ur_moveit_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			str(Path(ur_moveit_config_share) / 'launch' / 'ur_moveit.launch.py')
		),
		launch_arguments={
			'ur_type': ur_type,
			'launch_rviz': launch_rviz,
		}.items(),
	)

	return LaunchDescription([
		DeclareLaunchArgument('ur_type', default_value='ur10'),
		DeclareLaunchArgument('robot_ip', default_value='10.0.0.89'),
		DeclareLaunchArgument('reverse_ip', default_value='10.0.0.224'),
		DeclareLaunchArgument(
			'kinematics_params_file',
			default_value=str(
				Path(grasping_arm_control_share) / 'config' / 'ur_kinematics.yaml'
			),
		),
		DeclareLaunchArgument('launch_rviz', default_value='true'),
		DeclareLaunchArgument(
			'initial_joint_controller',
			default_value='scaled_joint_trajectory_controller',
		),
		DeclareLaunchArgument('activate_joint_controller', default_value='true'),
		ur_control_launch,
		ur_moveit_launch,
	])
