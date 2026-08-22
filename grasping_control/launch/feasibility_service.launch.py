from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package='grasping_control',
                executable='feasibility_service_node',
                name='feasibility_service_node',
                output='screen',
            )
        ]
    )