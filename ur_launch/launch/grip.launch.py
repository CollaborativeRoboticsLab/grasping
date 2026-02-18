from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
	# Frames
	ee_frame = LaunchConfiguration("ee_frame")
	gripper_frame = LaunchConfiguration("gripper_frame")
	camera_frame = LaunchConfiguration("camera_frame")

	# Static TFs are provided as quaternion transforms, to avoid roll/pitch/yaw ordering ambiguity.
	# Args: x y z qx qy qz qw parent child
	ee_to_gripper_tf = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="ee_to_gripper_static_tf",
		output="screen",
		arguments=[
			LaunchConfiguration("ee_to_gripper_x"),
			LaunchConfiguration("ee_to_gripper_y"),
			LaunchConfiguration("ee_to_gripper_z"),
			LaunchConfiguration("ee_to_gripper_qx"),
			LaunchConfiguration("ee_to_gripper_qy"),
			LaunchConfiguration("ee_to_gripper_qz"),
			LaunchConfiguration("ee_to_gripper_qw"),
			ee_frame,
			gripper_frame,
		],
	)

	ee_to_camera_tf = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="ee_to_camera_static_tf",
		output="screen",
		arguments=[
			LaunchConfiguration("ee_to_camera_x"),
			LaunchConfiguration("ee_to_camera_y"),
			LaunchConfiguration("ee_to_camera_z"),
			LaunchConfiguration("ee_to_camera_qx"),
			LaunchConfiguration("ee_to_camera_qy"),
			LaunchConfiguration("ee_to_camera_qz"),
			LaunchConfiguration("ee_to_camera_qw"),
			ee_frame,
			camera_frame,
		],
	)

	grasping_node = Node(
		package="ur_grasping",
		executable="ur_grasping_node",
		name="ur_grasping_node",
		output="screen",
		parameters=[
			{
				"server_mode": LaunchConfiguration("server_mode"),
				"anygrasp_service": LaunchConfiguration("anygrasp_service"),
				"anygrasp_frame": camera_frame,
				"move_group_action_name": LaunchConfiguration("move_group_action_name"),
				"planning_group": LaunchConfiguration("planning_group"),
				"planning_frame": LaunchConfiguration("planning_frame"),
				"end_effector_link": LaunchConfiguration("end_effector_link"),
				"do_post_grasp_move": LaunchConfiguration("do_post_grasp_move"),
				"post_grasp_frame": LaunchConfiguration("post_grasp_frame"),
				"post_grasp_pose": [
					LaunchConfiguration("post_grasp_x"),
					LaunchConfiguration("post_grasp_y"),
					LaunchConfiguration("post_grasp_z"),
					LaunchConfiguration("post_grasp_qx"),
					LaunchConfiguration("post_grasp_qy"),
					LaunchConfiguration("post_grasp_qz"),
					LaunchConfiguration("post_grasp_qw"),
				],
				"open_action_name": LaunchConfiguration("open_action_name"),
				"close_action_name": LaunchConfiguration("close_action_name"),
			}
		],
	)

	return LaunchDescription(
		[
			DeclareLaunchArgument("server_mode", default_value="true"),
			DeclareLaunchArgument("anygrasp_service", default_value="detection"),
			DeclareLaunchArgument("move_group_action_name", default_value="move_action"),
			DeclareLaunchArgument("planning_group", default_value="manipulator"),
			DeclareLaunchArgument("planning_frame", default_value="base_link"),
			DeclareLaunchArgument("end_effector_link", default_value="tool0"),
			DeclareLaunchArgument("open_action_name", default_value="/open_gripper"),
			DeclareLaunchArgument("close_action_name", default_value="/close_gripper"),
			DeclareLaunchArgument("do_post_grasp_move", default_value="true"),
			DeclareLaunchArgument("post_grasp_frame", default_value="base_link"),
			DeclareLaunchArgument("post_grasp_x", default_value="0.0"),
			DeclareLaunchArgument("post_grasp_y", default_value="0.0"),
			DeclareLaunchArgument("post_grasp_z", default_value="0.0"),
			DeclareLaunchArgument("post_grasp_qx", default_value="0.0"),
			DeclareLaunchArgument("post_grasp_qy", default_value="0.0"),
			DeclareLaunchArgument("post_grasp_qz", default_value="0.0"),
			DeclareLaunchArgument("post_grasp_qw", default_value="1.0"),
			DeclareLaunchArgument("ee_frame", default_value="tool0"),
			DeclareLaunchArgument("gripper_frame", default_value="gripper"),
			DeclareLaunchArgument("camera_frame", default_value="camera_color_optical_frame"),
			DeclareLaunchArgument("ee_to_gripper_x", default_value="0.0"),
			DeclareLaunchArgument("ee_to_gripper_y", default_value="0.0"),
			DeclareLaunchArgument("ee_to_gripper_z", default_value="0.0"),
			DeclareLaunchArgument("ee_to_gripper_qx", default_value="0.0"),
			DeclareLaunchArgument("ee_to_gripper_qy", default_value="0.0"),
			DeclareLaunchArgument("ee_to_gripper_qz", default_value="0.0"),
			DeclareLaunchArgument("ee_to_gripper_qw", default_value="1.0"),
			DeclareLaunchArgument("ee_to_camera_x", default_value="0.0"),
			DeclareLaunchArgument("ee_to_camera_y", default_value="0.0"),
			DeclareLaunchArgument("ee_to_camera_z", default_value="0.0"),
			DeclareLaunchArgument("ee_to_camera_qx", default_value="0.0"),
			DeclareLaunchArgument("ee_to_camera_qy", default_value="0.0"),
			DeclareLaunchArgument("ee_to_camera_qz", default_value="0.0"),
			DeclareLaunchArgument("ee_to_camera_qw", default_value="1.0"),
			ee_to_gripper_tf,
			ee_to_camera_tf,
			grasping_node,
		]
	)
