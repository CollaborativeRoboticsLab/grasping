from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
	ur_robot_driver_share = get_package_share_directory('ur_robot_driver')
	ur_moveit_config_share = get_package_share_directory('ur_moveit_config')
	grasping_arm_control_share = get_package_share_directory('grasping_arm_control')
	gripper_ros_share = get_package_share_directory('gripper_ros')

	ur_type = LaunchConfiguration('ur_type')
	robot_ip = LaunchConfiguration('robot_ip')
	reverse_ip = LaunchConfiguration('reverse_ip')
	use_sim = LaunchConfiguration('use_sim')
	kinematics_params_file = LaunchConfiguration('kinematics_params_file')
	launch_rviz = LaunchConfiguration('launch_rviz')
	rviz_config_file = LaunchConfiguration('rviz_config_file')
	initial_joint_controller = LaunchConfiguration('initial_joint_controller')
	activate_joint_controller = LaunchConfiguration('activate_joint_controller')
	motor_params_file = LaunchConfiguration('motor_params_file')
	gripper_params_file = LaunchConfiguration('gripper_params_file')

	ur_control_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			str(Path(ur_robot_driver_share) / 'launch' / 'ur_control.launch.py')
		),
		launch_arguments={
			'ur_type': ur_type,
			'robot_ip': robot_ip,
			'reverse_ip': reverse_ip,
			'description_package': 'grasping_description',
			'description_file': 'ur10-two-finger.urdf.xacro',
			'sim_gazebo': use_sim,
			'launch_rviz': 'false',
			'kinematics_params_file': kinematics_params_file,
			'initial_joint_controller': initial_joint_controller,
			'activate_joint_controller': activate_joint_controller,
		}.items(),
	)

	ur_control_group = GroupAction(
		scoped=True,
		actions=[ur_control_launch],
	)

	ur_moveit_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			str(Path(ur_moveit_config_share) / 'launch' / 'ur_moveit.launch.py')
		),
		launch_arguments={
			'ur_type': ur_type,
			'description_package': 'grasping_description',
			'description_file': 'ur10-two-finger.urdf.xacro',
			'sim_gazebo': use_sim,
			'launch_rviz': 'false',
		}.items(),
	)

	ur_moveit_group = GroupAction(
		scoped=True,
		actions=[ur_moveit_launch],
	)

	gripper_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			str(Path(gripper_ros_share) / 'launch' / 'gripper_soft_two_fingers.launch.py')
		),
		launch_arguments={
			'motor_params_file': motor_params_file,
			'gripper_params_file': gripper_params_file,
		}.items(),
	)

	gripper_group = GroupAction(
		scoped=True,
		actions=[gripper_launch],
	)

	rviz_node = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2_moveit',
		output='log',
		condition=IfCondition(launch_rviz),
		arguments=['-d', rviz_config_file],
	)

	return LaunchDescription([
		DeclareLaunchArgument('ur_type', default_value='ur10'),
		DeclareLaunchArgument('robot_ip', default_value='192.168.10.156'),
		DeclareLaunchArgument('reverse_ip', default_value='192.168.10.130'),
		DeclareLaunchArgument('use_sim', default_value='false'),
		DeclareLaunchArgument(
			'kinematics_params_file',
			default_value=str(
				Path(grasping_arm_control_share) / 'config' / 'ur_kinematics.yaml'
			),
		),
		DeclareLaunchArgument('launch_rviz', default_value='true'),
		DeclareLaunchArgument(
			'rviz_config_file',
			default_value=str(
				Path(grasping_arm_control_share) / 'rviz' / 'view_tobot.rviz'
			),
		),
		DeclareLaunchArgument(
			'initial_joint_controller',
			default_value='scaled_joint_trajectory_controller',
		),
		DeclareLaunchArgument('activate_joint_controller', default_value='true'),
		DeclareLaunchArgument(
			'motor_params_file',
			default_value=str(
				Path(gripper_ros_share) / 'config' / 'servos' / 'dynamixel.yaml'
			),
		),
		DeclareLaunchArgument(
			'gripper_params_file',
			default_value=str(
				Path(gripper_ros_share) / 'config' / 'grippers' / 'soft_two_finger_dynamixel.yaml'
			),
		),
		ur_control_group,
		ur_moveit_group,
		gripper_group,
		rviz_node,
	])