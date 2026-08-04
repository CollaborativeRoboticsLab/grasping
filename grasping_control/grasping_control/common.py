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
    """
    @brief Simple quaternion container used for normalization helpers.

    @var x X component.
    @var y Y component.
    @var z Z component.
    @var w W component.
    """
    x: float
    y: float
    z: float
    w: float


def normalize_quaternion(q: Quaternion) -> Quaternion:
    """
    @brief Normalize a quaternion and fall back to identity when invalid.

    @param q Quaternion to normalize.
    @return Unit quaternion suitable for MoveIt requests.
    """
    # MoveIt constraints are sensitive to invalid quaternions, so every externally
    # supplied orientation is normalized before being used in planning requests.
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm <= 0.0:
        return Quaternion(0.0, 0.0, 0.0, 1.0)
    return Quaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm)


def coerce_float_sequence(value: Any, expected_length: int, name: str) -> list[float]:
    """
    @brief Convert a fixed-length sequence-like value into floats.

    @param value Sequence value from YAML or parameters.
    @param expected_length Required number of values.
    @param name Human-readable value name for error messages.
    @return Parameter values as floats.
    """
    if isinstance(value, str):
        items = [item.strip() for item in value.strip('[]()').split(',') if item.strip()]
    else:
        items = list(value)

    if len(items) != expected_length:
        raise RuntimeError(f"'{name}' must contain {expected_length} values.")
    return [float(item) for item in items]


def coerce_string_sequence(value: Any) -> list[str]:
    """
    @brief Convert a string or sequence-like value into clean string values.

    @param value String or sequence-like value.
    @return Non-empty strings.
    """
    if isinstance(value, str):
        items = [item.strip() for item in value.strip('[]()').split(',')]
    else:
        items = list(value)
    return [str(item).strip().strip('"\'') for item in items if str(item).strip()]


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """
    @brief Convert roll-pitch-yaw Euler angles to a quaternion.

    @param roll Roll angle in radians.
    @param pitch Pitch angle in radians.
    @param yaw Yaw angle in radians.
    @return Equivalent normalized quaternion.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return normalize_quaternion(
        Quaternion(
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """
    @brief Convert a quaternion to roll, pitch, yaw angles.

    @param x Quaternion x component.
    @param y Quaternion y component.
    @param z Quaternion z component.
    @param w Quaternion w component.
    @return Equivalent roll, pitch, yaw tuple in radians.
    """
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def nearest_equivalent_angle(target_angle: float, reference_angle: float) -> float:
    """
    @brief Shift an angular target by whole turns so it stays closest to a reference angle.

    @param target_angle Target angle in radians.
    @param reference_angle Reference angle in radians.
    @return Equivalent target angle nearest to the reference.
    """
    two_pi = 2.0 * math.pi
    return target_angle + two_pi * round((reference_angle - target_angle) / two_pi)


def resolve_config_path(package_name: str, configured_path: str, default_name: str) -> Path:
    """
    @brief Resolve a config file from an explicit path, source tree, or install share.

    @param package_name ROS package name that owns the config.
    @param configured_path User-supplied override path.
    @param default_name Default config filename.
    @return Absolute path to the resolved config file.
    """
    # Prefer an explicit path, otherwise resolve the config from the source tree during
    # development or the installed package share directory after colcon install/build.
    if configured_path:
        return Path(configured_path).expanduser().resolve()

    package_root = Path(__file__).resolve().parents[1]

    # Workspace runtime configs are commonly written to the colcon workspace root.
    if default_name == 'workspace.yaml':
        workspace_root = find_colcon_workspace_root(package_root)
        if workspace_root is not None:
            workspace_config_path = workspace_root / default_name
            if workspace_config_path.exists():
                return workspace_config_path

    source_config_path = package_root / 'config' / default_name
    if source_config_path.exists() or package_root.name == package_name:
        return source_config_path

    share_dir = Path(get_package_share_directory(package_name))
    return share_dir / 'config' / default_name


def find_colcon_workspace_root(start_path: Path) -> Path | None:
    """
    @brief Find the nearest colcon workspace root above a file or directory.

    @param start_path File or directory path inside a workspace.
    @return Workspace root when found, otherwise None.
    """
    current = start_path.resolve()
    search_roots = (current,) if current.is_dir() else current.parents
    for candidate in search_roots:
        has_src = (candidate / 'src').is_dir()
        has_colcon_artifacts = any((candidate / name).exists() for name in ('build', 'install', 'log'))
        if has_src and has_colcon_artifacts:
            return candidate
    return None


def load_yaml_dict(path: Path, default_value: Dict[str, Any]) -> Dict[str, Any]:
    """
    @brief Load a YAML mapping from disk or return a default copy when absent.

    @param path YAML file path.
    @param default_value Default mapping to return when the file does not exist.
    @return Loaded YAML mapping.
    @throws RuntimeError Raised when the YAML root is not a dictionary.
    """
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
    """
    @brief Persist a dictionary to YAML, creating parent directories as needed.

    @param path Destination YAML file path.
    @param data Mapping to serialize.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as config_file:
        yaml.safe_dump(data, config_file, sort_keys=False)


def dict_to_pose(pose_dict: Dict[str, Any]) -> Pose:
    """
    @brief Convert a persisted pose dictionary into a ROS Pose message.

    @param pose_dict YAML-compatible pose mapping.
    @return Converted Pose message.
    """
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
    """
    @brief Transform a stamped pose into the requested frame.

    @param node ROS node used for timestamps.
    @param tf_buffer TF buffer used to query transforms.
    @param pose Input pose to transform.
    @param target_frame Frame required by the caller.
    @return Pose expressed in the target frame.
    @throws RuntimeError Raised when tf2_geometry_msgs is unavailable.
    """
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
    transformed_pose = do_transform_pose(pose.pose, transform)
    out = PoseStamped()
    out.header.stamp = node.get_clock().now().to_msg()
    out.header.frame_id = target_frame
    out.pose = transformed_pose.pose if hasattr(transformed_pose, 'pose') else transformed_pose
    return out