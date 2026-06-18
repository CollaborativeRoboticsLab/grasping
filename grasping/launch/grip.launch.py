from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

	motion_execution_node = Node(
		package="grasping_control",
		executable="motion_execution_node",
		name="motion_execution_node",
		output="screen",
		parameters=[
			{
				"action_name": LaunchConfiguration("arm_action_name"),
				"move_group_action_name": LaunchConfiguration("move_group_action_name"),
				"planning_group": LaunchConfiguration("planning_group"),
				"planning_frame": LaunchConfiguration("planning_frame"),
				"end_effector_link": LaunchConfiguration("end_effector_link"),
				"allowed_planning_time": LaunchConfiguration("allowed_planning_time"),
				"num_planning_attempts": LaunchConfiguration("num_planning_attempts"),
				"max_velocity_scaling": LaunchConfiguration("max_velocity_scaling"),
				"max_acceleration_scaling": LaunchConfiguration("max_acceleration_scaling"),
				"position_tolerance_m": LaunchConfiguration("position_tolerance_m"),
				"orientation_tolerance_rad": LaunchConfiguration("orientation_tolerance_rad"),
				"planning_pipeline_id": LaunchConfiguration("planning_pipeline_id"),
				"planner_id": LaunchConfiguration("planner_id"),
				"workspace_config_path": LaunchConfiguration("workspace_config_path"),
			}
		],
	)

	grasping_node = Node(
		package="grasping",
		executable="grasping_node",
		name="grasping_node",
		output="screen",
		parameters=[
			{
				"server_mode": LaunchConfiguration("server_mode"),
				"anygrasp_service": LaunchConfiguration("anygrasp_service"),
				"arm_action_name": LaunchConfiguration("arm_action_name"),
				"do_post_grasp_move": LaunchConfiguration("do_post_grasp_move"),
				"open_action_name": LaunchConfiguration("open_action_name"),
				"close_action_name": LaunchConfiguration("close_action_name"),
			}
		],
	)

	return LaunchDescription(
		[
			DeclareLaunchArgument("server_mode", default_value="true"),
			DeclareLaunchArgument("anygrasp_service", default_value="detection"),
			DeclareLaunchArgument("arm_action_name", default_value="move_arm_to_pose"),
			DeclareLaunchArgument("move_group_action_name", default_value="move_action"),
			DeclareLaunchArgument("planning_group", default_value="manipulator"),
			DeclareLaunchArgument("planning_frame", default_value="base_link"),
			DeclareLaunchArgument("end_effector_link", default_value="tool0"),
			DeclareLaunchArgument("allowed_planning_time", default_value="5.0"),
			DeclareLaunchArgument("num_planning_attempts", default_value="5"),
			DeclareLaunchArgument("max_velocity_scaling", default_value="0.2"),
			DeclareLaunchArgument("max_acceleration_scaling", default_value="0.2"),
			DeclareLaunchArgument("position_tolerance_m", default_value="0.005"),
			DeclareLaunchArgument("orientation_tolerance_rad", default_value="0.1"),
			DeclareLaunchArgument("planning_pipeline_id", default_value=""),
			DeclareLaunchArgument("planner_id", default_value=""),
			DeclareLaunchArgument("workspace_config_path", default_value=""),
			DeclareLaunchArgument("open_action_name", default_value="/open_gripper"),
			DeclareLaunchArgument("close_action_name", default_value="/close_gripper"),
			DeclareLaunchArgument("do_post_grasp_move", default_value="true"),
			motion_execution_node,
			grasping_node,
		]
	)
