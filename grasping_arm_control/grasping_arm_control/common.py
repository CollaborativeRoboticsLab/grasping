from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Dict

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer
import yaml

try:
    from tf2_geometry_msgs import do_transform_pose  # type: ignore
except Exception:  # noqa: BLE001
    do_transform_pose = None


@dataclass
class Quaternion:
    x: float
    y: float
    z: float
    w: float


def normalize_quaternion(q: Quaternion) -> Quaternion:
    # MoveIt constraints are sensitive to invalid quaternions, so every externally
    # supplied orientation is normalized before being used in planning requests.
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm <= 0.0:
        return Quaternion(0.0, 0.0, 0.0, 1.0)
    return Quaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm)


def resolve_config_path(package_name: str, configured_path: str, default_name: str) -> Path:
    # Prefer an explicit path, otherwise resolve the config from the source tree during
    # development or the installed package share directory after colcon install/build.
    if configured_path:
        return Path(configured_path).expanduser().resolve()

    package_root = Path(__file__).resolve().parents[1]
    source_config_path = package_root / 'config' / default_name
    if source_config_path.exists() or package_root.name == package_name:
        return source_config_path

    share_dir = Path(get_package_share_directory(package_name))
    return share_dir / 'config' / default_name


def load_yaml_dict(path: Path, default_value: Dict[str, Any]) -> Dict[str, Any]:
    # Config files are optional during first run, so callers can provide a default schema
    # and still work before the YAML exists on disk.
    if not path.exists():
        return deepcopy(default_value)

    with path.open('r', encoding='utf-8') as config_file:
        loaded = yaml.safe_load(config_file) or {}

    if not isinstance(loaded, dict):
        raise RuntimeError(f'Expected a dictionary in {path}')
    return loaded


def write_yaml_dict(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as config_file:
        yaml.safe_dump(data, config_file, sort_keys=False)


def dict_to_pose(pose_dict: Dict[str, Any]) -> Pose:
    # Workspace geometry is stored as plain YAML dictionaries; this helper converts that
    # persisted representation back into a ROS pose message for planning-scene loading.
    pose = Pose()
    position = pose_dict.get('position', {})
    orientation = pose_dict.get('orientation', {})
    pose.position.x = float(position.get('x', 0.0))
    pose.position.y = float(position.get('y', 0.0))
    pose.position.z = float(position.get('z', 0.0))
    pose.orientation.x = float(orientation.get('x', 0.0))
    pose.orientation.y = float(orientation.get('y', 0.0))
    pose.orientation.z = float(orientation.get('z', 0.0))
    pose.orientation.w = float(orientation.get('w', 1.0))
    return pose


def transform_pose_to_frame(
    node: Node,
    tf_buffer: Buffer,
    pose: PoseStamped,
    target_frame: str,
) -> PoseStamped:
    # Action goals may arrive in camera, base, or any other connected frame. The arm-control
    # node always plans in one frame, so transforms are centralized here.
    if do_transform_pose is None:
        raise RuntimeError('tf2_geometry_msgs is required to transform PoseStamped')

    if pose.header.frame_id == target_frame:
        return pose

    transform = tf_buffer.lookup_transform(
        target_frame,
        pose.header.frame_id,
        rclpy.time.Time(),
        timeout=rclpy.duration.Duration(seconds=1.0),
    )
    out = do_transform_pose(pose, transform)
    out.header.stamp = node.get_clock().now().to_msg()
    out.header.frame_id = target_frame
    return out