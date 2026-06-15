from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "ur", package_name="ur10_soft_two_fingers_moveit_config"
    ).to_moveit_configs()

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }
    octomap_configuration = {
        "octomap_resolution": 0.1,
        "octomap_frame": "world",
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("debug", default_value="false"),
            DeclareLaunchArgument("allow_trajectory_execution", default_value="true"),
            DeclareLaunchArgument(
                "publish_monitored_planning_scene", default_value="true"
            ),
            DeclareLaunchArgument("capabilities", default_value=""),
            DeclareLaunchArgument("disable_capabilities", default_value=""),
            DeclareLaunchArgument("monitor_dynamics", default_value="false"),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    move_group_configuration,
                    octomap_configuration,
                ],
            ),
        ]
    )
