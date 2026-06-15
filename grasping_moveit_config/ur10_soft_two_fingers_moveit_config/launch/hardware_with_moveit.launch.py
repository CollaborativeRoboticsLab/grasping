from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ur_robot_driver_share = get_package_share_directory("ur_robot_driver")
    grasping_description_share = get_package_share_directory("grasping_description")
    gripper_ros_share = get_package_share_directory("gripper_ros")
    moveit_config_share = get_package_share_directory("ur10_soft_two_fingers_moveit_config")

    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    reverse_ip = LaunchConfiguration("reverse_ip")
    use_sim = LaunchConfiguration("use_sim")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    motor_params_file = LaunchConfiguration("motor_params_file")
    gripper_params_file = LaunchConfiguration("gripper_params_file")

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(ur_robot_driver_share) / "launch" / "ur_control.launch.py")
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "reverse_ip": reverse_ip,
            "description_package": "grasping_description",
            "description_file": "ur10-soft-two-fingers.urdf.xacro",
            "sim_gazebo": use_sim,
            "launch_rviz": "false",
            "kinematics_params_file": kinematics_params_file,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
        }.items(),
    )

    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur10_soft_two_fingers_moveit_config")
        .robot_description(
            file_path=Path(grasping_description_share)
            / "xacro"
            / "ur10_soft_two_fingers"
            / "ur10-soft-two-fingers.urdf.xacro",
            mappings={
                "name": "ur",
                "ur_type": ur_type,
                "kinematics_params": kinematics_params_file,
                "robot_ip": robot_ip,
                "reverse_ip": reverse_ip,
                "sim_gazebo": use_sim,
                "script_filename": "ros_control.urscript",
                "input_recipe_filename": "rtde_input_recipe.txt",
                "output_recipe_filename": "rtde_output_recipe.txt",
            },
        )
        .robot_description_semantic(file_path=Path("config") / "ur.srdf")
        .robot_description_kinematics(file_path=Path("config") / "kinematics.yaml")
        .joint_limits(file_path=Path("config") / "joint_limits.yaml")
        .trajectory_execution(
            file_path=Path("config") / "moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=False,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines()
        .pilz_cartesian_limits(file_path=Path("config") / "pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": "",
        "disable_capabilities": "",
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "use_sim_time": use_sim,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_configuration],
    )

    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(gripper_ros_share) / "launch" / "gripper_soft_two_fingers.launch.py")
        ),
        launch_arguments={
            "motor_params_file": motor_params_file,
            "gripper_params_file": gripper_params_file,
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur10"),
            DeclareLaunchArgument("robot_ip", default_value="192.168.10.156"),
            DeclareLaunchArgument("reverse_ip", default_value="192.168.10.130"),
            DeclareLaunchArgument("use_sim", default_value="false"),
            DeclareLaunchArgument(
                "kinematics_params_file",
                default_value=str(Path(moveit_config_share) / "config" / "kinematics.yaml"),
            ),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=str(Path(moveit_config_share) / "config" / "moveit.rviz"),
            ),
            DeclareLaunchArgument(
                "initial_joint_controller",
                default_value="scaled_joint_trajectory_controller",
            ),
            DeclareLaunchArgument("activate_joint_controller", default_value="true"),
            DeclareLaunchArgument(
                "motor_params_file",
                default_value=str(
                    Path(gripper_ros_share) / "config" / "servos" / "dynamixel.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "gripper_params_file",
                default_value=str(
                    Path(gripper_ros_share)
                    / "config"
                    / "grippers"
                    / "soft_two_finger_dynamixel.yaml"
                ),
            ),
            GroupAction(scoped=True, actions=[ur_control_launch]),
            move_group_node,
            GroupAction(scoped=True, actions=[gripper_launch]),
            rviz_node,
        ]
    )