import os
import sys

from moveit_configs_utils.launches import generate_moveit_rviz_launch

sys.path.append(os.path.dirname(__file__))

from _moveit_config import load_moveit_config


def generate_launch_description():
    return generate_moveit_rviz_launch(load_moveit_config())
