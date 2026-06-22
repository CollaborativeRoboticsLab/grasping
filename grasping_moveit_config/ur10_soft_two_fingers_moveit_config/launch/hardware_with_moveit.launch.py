from functools import partial
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_share_directory, relative_file_path):
    with open(Path(package_share_directory) / relative_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def launch_setup(
    context,
    *,
    grasping_description_share,
    ur_client_library_share,
    ur_client_library_prefix,
    ur_robot_driver_share,
):
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    reverse_ip = LaunchConfiguration("reverse_ip")
    use_sim = LaunchConfiguration("use_sim")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controller_config_package = LaunchConfiguration("controller_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    headless_mode = LaunchConfiguration("headless_mode")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    launch_servo = LaunchConfiguration("launch_servo")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    tool_voltage = LaunchConfiguration("tool_voltage")
    script_command_port = LaunchConfiguration("script_command_port")
    reverse_port = LaunchConfiguration("reverse_port")
    script_sender_port = LaunchConfiguration("script_sender_port")
    trajectory_port = LaunchConfiguration("trajectory_port")

    runtime_config_share = get_package_share_directory(runtime_config_package.perform(context))
    controller_config_share = get_package_share_directory(controller_config_package.perform(context))
    robot_description_file = (
        Path(grasping_description_share)
        / "xacro"
        / "ur10_soft_two_fingers"
        / "ur10-soft-two-fingers.urdf.xacro"
    )
    script_filename = Path(ur_client_library_share) / "resources" / "external_control.urscript"
    input_recipe_filename = Path(ur_robot_driver_share) / "resources" / "rtde_input_recipe.txt"
    output_recipe_filename = Path(ur_robot_driver_share) / "resources" / "rtde_output_recipe.txt"
    initial_joint_controllers = (
        Path(controller_config_share) / "config" / controllers_file.perform(context)
    )
    update_rate_config_file = (
        Path(runtime_config_share) / "config" / f"{ur_type.perform(context)}_update_rate.yaml"
    )
    selected_joint_controller = initial_joint_controller.perform(context)

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

    moveit_controller_config = {
        "moveit_simple_controller_manager": {
            "controller_names": [
                selected_joint_controller,
                "soft_two_fingers_gripper_controller",
            ],
            selected_joint_controller: {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
            },
            "soft_two_fingers_gripper_controller": {
                "type": "GripperCommand",
                "joints": ["gripper_planar_5", "gripper_planar_4"],
                "action_ns": "gripper_cmd",
                "default": True,
            },
        }
    }

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
        parameters=[
            moveit_config.to_dict(),
            moveit_controller_config,
            move_group_configuration,
        ],
    )

    servo_params = {
        "moveit_servo": load_yaml(controller_config_share, Path("config") / "ur_servo.yaml")
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

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            str(robot_description_file),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "kinematics_params:=",
            kinematics_params_file,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            ur_type,
            " ",
            "script_filename:=",
            str(script_filename),
            " ",
            "input_recipe_filename:=",
            str(input_recipe_filename),
            " ",
            "output_recipe_filename:=",
            str(output_recipe_filename),
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "headless_mode:=",
            headless_mode,
            " ",
            "use_tool_communication:=",
            use_tool_communication,
            " ",
            "tool_parity:=",
            tool_parity,
            " ",
            "tool_baud_rate:=",
            tool_baud_rate,
            " ",
            "tool_stop_bits:=",
            tool_stop_bits,
            " ",
            "tool_rx_idle_chars:=",
            tool_rx_idle_chars,
            " ",
            "tool_tx_idle_chars:=",
            tool_tx_idle_chars,
            " ",
            "tool_device_name:=",
            tool_device_name,
            " ",
            "tool_tcp_port:=",
            tool_tcp_port,
            " ",
            "tool_voltage:=",
            tool_voltage,
            " ",
            "reverse_ip:=",
            reverse_ip,
            " ",
            "script_command_port:=",
            script_command_port,
            " ",
            "reverse_port:=",
            reverse_port,
            " ",
            "script_sender_port:=",
            script_sender_port,
            " ",
            "trajectory_port:=",
            trajectory_port,
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            str(update_rate_config_file),
            ParameterFile(str(initial_joint_controllers), allow_substs=True),
        ],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            robot_description,
            str(update_rate_config_file),
            ParameterFile(str(initial_joint_controllers), allow_substs=True),
        ],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    dashboard_client_node = IncludeLaunchDescription(
        condition=IfCondition(
            AndSubstitution(launch_dashboard_client, NotSubstitution(use_fake_hardware))
        ),
        launch_description_source=AnyLaunchDescriptionSource(
            str(Path(ur_robot_driver_share) / "launch" / "ur_dashboard_client.launch.py")
        ),
        launch_arguments={"robot_ip": robot_ip}.items(),
    )

    robot_state_helper_node = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
        parameters=[{"headless_mode": headless_mode}, {"robot_ip": robot_ip}],
    )

    tool_communication_script = ExecuteProcess(
        name="ur_tool_comm",
        condition=IfCondition(use_tool_communication),
        cmd=[
            str(Path(ur_client_library_prefix) / "lib" / "ur_client_library" / "tool_communication.py"),
            robot_ip,
            "--tcp-port",
            tool_tcp_port,
            "--device-name",
            tool_device_name,
        ],
        output="screen",
    )

    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip}],
        output="screen",
    )

    controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": activate_joint_controller},
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                    "tcp_pose_broadcaster",
                    "ur_configuration_controller",
                ]
            },
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    trajectory_until_node = Node(
        package="ur_robot_driver",
        executable="trajectory_until_node",
        name="trajectory_until_node",
        output="screen",
        parameters=[
            {
                "motion_controller_uri": f"/{initial_joint_controller.perform(context)}/follow_joint_trajectory",
                "until_action_uri": "tool_contact_controller/detect_tool_contact",
            },
        ],
    )

    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "tcp_pose_broadcaster",
        "ur_configuration_controller",
        "friction_model_controller",
    ]
    controllers_inactive = [
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        "forward_effort_controller",
        "force_mode_controller",
        "passthrough_trajectory_controller",
        "freedrive_mode_controller",
        "tool_contact_controller",
    ]

    if launch_servo.perform(context) == "true":
        controllers_active.append("forward_position_controller")
        controllers_inactive.remove("forward_position_controller")

    elif activate_joint_controller.perform(context) == "true":
        controllers_active.append(initial_joint_controller.perform(context))
        controllers_inactive.remove(initial_joint_controller.perform(context))

    if use_fake_hardware.perform(context) == "true":
        controllers_active.remove("tcp_pose_broadcaster")

    controller_spawners = [
        controller_spawner(controllers_active),
        controller_spawner(controllers_inactive, active=False),
    ]

    return [
        control_node,
        ur_control_node,
        dashboard_client_node,
        robot_state_helper_node,
        tool_communication_script,
        controller_stopper_node,
        urscript_interface,
        robot_state_publisher_node,
        trajectory_until_node,
        move_group_node,
        servo_node,
        *controller_spawners,
    ]


def generate_launch_description():
    ur_robot_driver_share = get_package_share_directory("ur_robot_driver")
    ur_client_library_share = get_package_share_directory("ur_client_library")
    ur_client_library_prefix = get_package_prefix("ur_client_library")
    ur10_moveit_config_share = get_package_share_directory("ur10_moveit_config")
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
    motor_params_file = LaunchConfiguration("motor_params_file")
    gripper_params_file = LaunchConfiguration("gripper_params_file")
    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(gripper_ros_share) / "launch" / "gripper_soft_two_fingers.launch.py")
        ),
        launch_arguments={
            "motor_params_file": motor_params_file,
            "gripper_params_file": gripper_params_file,
        }.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(moveit_config_share) / "launch" / "moveit_rviz.launch.py")
        ),
        condition=IfCondition(launch_rviz),
        launch_arguments={"rviz_config": rviz_config_file}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur10"),
            DeclareLaunchArgument('robot_ip', default_value='10.0.0.89'),
            DeclareLaunchArgument('reverse_ip', default_value='10.0.0.179'),
            DeclareLaunchArgument("safety_limits", default_value="true"),
            DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
            DeclareLaunchArgument("safety_k_position", default_value="20"),
            DeclareLaunchArgument("runtime_config_package", default_value="ur_robot_driver"),
            DeclareLaunchArgument(
                "controller_config_package",
                default_value="ur10_soft_two_fingers_moveit_config",
            ),
            DeclareLaunchArgument("controllers_file", default_value="ur_controllers.yaml"),
            DeclareLaunchArgument("tf_prefix", default_value=""),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
            DeclareLaunchArgument("use_sim", default_value="false"),
            DeclareLaunchArgument("headless_mode", default_value="true"),
            DeclareLaunchArgument("controller_spawner_timeout", default_value="10"),
            DeclareLaunchArgument(
                "kinematics_params_file",
                default_value=str(
                    Path(ur10_moveit_config_share) / "config" / "ur_kinematics.yaml"
                ),
            ),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_servo", default_value="false"),
            DeclareLaunchArgument("launch_dashboard_client", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=str(Path(moveit_config_share) / "config" / "moveit.rviz"),
            ),
            DeclareLaunchArgument(
                "initial_joint_controller",
                default_value="scaled_joint_trajectory_controller",
            ),
            DeclareLaunchArgument("activate_joint_controller", default_value="true"),
            DeclareLaunchArgument("use_tool_communication", default_value="false"),
            DeclareLaunchArgument("tool_parity", default_value="0"),
            DeclareLaunchArgument("tool_baud_rate", default_value="115200"),
            DeclareLaunchArgument("tool_stop_bits", default_value="1"),
            DeclareLaunchArgument("tool_rx_idle_chars", default_value="1.5"),
            DeclareLaunchArgument("tool_tx_idle_chars", default_value="3.5"),
            DeclareLaunchArgument("tool_device_name", default_value="/tmp/ttyUR"),
            DeclareLaunchArgument("tool_tcp_port", default_value="54321"),
            DeclareLaunchArgument("tool_voltage", default_value="0"),
            DeclareLaunchArgument("script_command_port", default_value="50004"),
            DeclareLaunchArgument("reverse_port", default_value="50001"),
            DeclareLaunchArgument("script_sender_port", default_value="50002"),
            DeclareLaunchArgument("trajectory_port", default_value="50003"),
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
            OpaqueFunction(
                function=partial(
                    launch_setup,
                    grasping_description_share=grasping_description_share,
                    ur_client_library_share=ur_client_library_share,
                    ur_client_library_prefix=ur_client_library_prefix,
                    ur_robot_driver_share=ur_robot_driver_share,
                )
            ),
            GroupAction(scoped=True, actions=[gripper_launch]),
            rviz_launch,
        ]
    )