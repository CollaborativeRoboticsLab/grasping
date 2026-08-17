from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
	return LaunchDescription(
		[
			DeclareLaunchArgument('arm_action_name', default_value='/move_arm_to_named_pose'),
			DeclareLaunchArgument('gripper_action_name', default_value='/gripper_command'),
			DeclareLaunchArgument(
				'sequence',
				default_value='[[pose, post_grasp], [gripper, open], [pose, grasp_pose], [gripper, close], [pose, post_grasp]]',
			),
			DeclareLaunchArgument('open_position', default_value='0.09'),
			DeclareLaunchArgument('open_max_effort', default_value='0.0'),
			DeclareLaunchArgument('close_position', default_value='0.0'),
			DeclareLaunchArgument('close_max_effort', default_value='40.0'),
			DeclareLaunchArgument('server_timeout_sec', default_value='10.0'),
			DeclareLaunchArgument('result_timeout_sec', default_value='120.0'),
			DeclareLaunchArgument('startup_delay_sec', default_value='5.0'),

			DeclareLaunchArgument('interactive_mode', default_value='false'),
			DeclareLaunchArgument('tare_on_startup', default_value='true'),
			DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
			DeclareLaunchArgument('object_name', default_value='test_object'),
			DeclareLaunchArgument('trial_number', default_value='1'),
			DeclareLaunchArgument('test_read_count', default_value='5'),
			DeclareLaunchArgument('trial_duration', default_value='60.0'),
			DeclareLaunchArgument('record_data', default_value='true'),

			Node(
				package='grasping_control',
				executable='simple_grasping_node',
				name='simple_grasping_node',
				output='screen',
				parameters=[
					{
						'arm_action_name': LaunchConfiguration('arm_action_name'),
						'gripper_action_name': LaunchConfiguration('gripper_action_name'),
						'sequence': ParameterValue(LaunchConfiguration('sequence'), value_type=str),
						'open_position': LaunchConfiguration('open_position'),
						'open_max_effort': LaunchConfiguration('open_max_effort'),
						'close_position': LaunchConfiguration('close_position'),
						'close_max_effort': LaunchConfiguration('close_max_effort'),
						'server_timeout_sec': LaunchConfiguration('server_timeout_sec'),
						'result_timeout_sec': LaunchConfiguration('result_timeout_sec'),
						'startup_delay_sec': LaunchConfiguration('startup_delay_sec'),
						'trigger_retention_recording_on_close': True,
						'trigger_retention_recording_service': '/trigger_retention_recording',
					}
				],
			),
			Node(
				package='gripper_estimator',
				executable='retention_force_estimate',
				name='retention_force_estimate',
				output='screen',
				parameters=[
					{
						'interactive_mode': ParameterValue(LaunchConfiguration('interactive_mode'), value_type=bool),
						'tare_on_startup': ParameterValue(LaunchConfiguration('tare_on_startup'), value_type=bool),
						'test_read_count': ParameterValue(LaunchConfiguration('test_read_count'), value_type=int),
						'record_data': ParameterValue(LaunchConfiguration('record_data'), value_type=bool),
						'wait_for_start_trigger': True,
						'default_object_name': LaunchConfiguration('object_name'),
						'default_trial_number': ParameterValue(LaunchConfiguration('trial_number'), value_type=str),
						'startup_delay_sec': ParameterValue(LaunchConfiguration('startup_delay_sec'), value_type=float),
						'default_duration_sec': ParameterValue(LaunchConfiguration('trial_duration'), value_type=float),
						'port': LaunchConfiguration('port'),
					}
				],
			),


			
		]
	)
