import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

import yaml


def load_yaml(package_share_directory, relative_file_path):
    with open(os.path.join(package_share_directory, relative_file_path), "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    tm_robot_ip = LaunchConfiguration("tm_robot_ip")
    tm_use_simulation = LaunchConfiguration("tm_use_simulation")
    no_logging = LaunchConfiguration("no_logging")
    launch_servo = LaunchConfiguration("launch_servo")

    moveit_config = MoveItConfigsBuilder(
        "tm12s_soft_two_fingers",
        package_name="tm12s_soft_two_fingers_moveit_config",
    ).to_moveit_configs()

    package_share = get_package_share_directory("tm12s_soft_two_fingers_moveit_config")
    tm_driver_share = get_package_share_directory("tm_driver")
    ros2_controllers_path = os.path.join(package_share, "config", "ros2_controllers.yaml")
    rviz_config_file = os.path.join(package_share, "config", "moveit.rviz")

    declare_robot_ip = DeclareLaunchArgument(
        "tm_robot_ip",
        default_value="",
        description="Target robot IP address for tm_driver.",
    )

    declare_use_simulation = DeclareLaunchArgument(
        "tm_use_simulation",
        default_value="false",
        description="Run tm_driver in simulation/fake mode.",
    )

    declare_no_logging = DeclareLaunchArgument(
        "no_logging",
        default_value="false",
        description="Use the driver's direct console print functions instead of ROS logging.",
    )

    declare_launch_servo = DeclareLaunchArgument(
        "launch_servo",
        default_value="false",
        description="Launch MoveIt Servo and activate the forward position controller for arm jogging.",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    servo_params = {
        "moveit_servo": load_yaml(package_share_directory=package_share, relative_file_path="config/tm12s_servo.yaml")
    }

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        output="screen",
        condition=IfCondition(launch_servo),
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    tm_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=UnlessCondition(launch_servo),
        arguments=[
            "tmr_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=IfCondition(launch_servo),
        arguments=[
            "forward_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "soft_two_fingers_gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    tm_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tm_driver_share, "launch", "tm_bringup.launch.py")
        ),
        launch_arguments={
            "tm_robot_ip": tm_robot_ip,
            "tm_use_simulation": tm_use_simulation,
            "no_logging": no_logging,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_robot_ip,
            declare_use_simulation,
            declare_no_logging,
            declare_launch_servo,
            tm_driver_launch,
            rviz_node,
            static_tf,
            robot_state_publisher,
            move_group_node,
            servo_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            tm_arm_controller_spawner,
            forward_position_controller_spawner,
            gripper_controller_spawner,
        ]
    )