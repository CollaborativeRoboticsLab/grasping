import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_moveit_config():
    package_path = get_package_share_directory("ld250_tm12x_soft_two_fingers_moveit_config")
    tm12s_package_path = get_package_share_directory("tm12s_soft_two_fingers_moveit_config")

    return (
        MoveItConfigsBuilder(
            "ld250_tm12x_soft_two_fingers",
            package_name="ld250_tm12x_soft_two_fingers_moveit_config",
        )
        .robot_description(
            file_path=os.path.join(package_path, "config", "ld250_tm12x_soft_two_fingers.urdf.xacro")
        )
        .robot_description_semantic(
            file_path=os.path.join(package_path, "config", "ld250_tm12x_soft_two_fingers.srdf")
        )
        .robot_description_kinematics(file_path=os.path.join(package_path, "config", "kinematics.yaml"))
        .trajectory_execution(file_path=os.path.join(package_path, "config", "moveit_controllers.yaml"))
        .joint_limits(file_path=os.path.join(tm12s_package_path, "config", "joint_limits.yaml"))
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=False,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines()
        .pilz_cartesian_limits(
            file_path=os.path.join(tm12s_package_path, "config", "pilz_cartesian_limits.yaml")
        )
        .to_moveit_configs()
    )