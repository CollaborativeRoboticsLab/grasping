from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import math
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from grasping_arm_control.common import Quaternion, dict_to_pose, normalize_quaternion, write_yaml_dict


def iso_timestamp() -> str:
	"""!
	@brief Return the current UTC time as an ISO-8601 string.

	@return Timestamp string suitable for persisted workspace metadata.
	"""
	return datetime.now(timezone.utc).isoformat()


def default_workspace_config(
	base_frame: str,
	tool_frame: str,
	ground_plane_z: float,
) -> Dict[str, Any]:
	"""!
	@brief Build the default workspace calibration document.

	@param base_frame Frame used as the workspace reference.
	@param tool_frame Tool frame used when samples are captured.
	@param ground_plane_z Ground height in the base frame.
	@return Default workspace configuration dictionary.
	"""
	return {
		'version': 1,
		'updated_at': iso_timestamp(),
		'base_frame': base_frame,
		'tool_frame': tool_frame,
		'ground_plane_z': ground_plane_z,
		'workspace_area': None,
		'objects': [],
	}


def default_shape_definitions() -> Dict[str, Any]:
	"""!
	@brief Return the built-in calibration shape definitions.

	@return Shape definition dictionary used when no YAML exists yet.
	"""
	return {
		'version': 1,
		'shapes': {
			'rectangle': {
				'display_name': 'Rectangular prism',
				'geometry_type': 'box',
				'description': 'Capture the four top-face corners in order around the object.',
				'point_labels': ['corner_1', 'corner_2', 'corner_3', 'corner_4'],
			},
			'cylinder': {
				'display_name': 'Cylinder',
				'geometry_type': 'cylinder',
				'description': 'Capture the top-face center, then four rim points around the cylinder.',
				'point_labels': ['center', 'rim_1', 'rim_2', 'rim_3', 'rim_4'],
			},
		},
	}


def prepare_workspace_config(
	workspace_config: Dict[str, Any],
	base_frame: str,
	tool_frame: str,
	ground_plane_z: float,
) -> Dict[str, Any]:
	"""!
	@brief Normalize workspace metadata before it is persisted.

	@param workspace_config Existing workspace configuration.
	@param base_frame Frame used as the workspace reference.
	@param tool_frame Tool frame used when samples are captured.
	@param ground_plane_z Ground height in the base frame.
	@return A copied and normalized workspace configuration dictionary.
	"""
	prepared = deepcopy(workspace_config)
	prepared.setdefault('version', 1)
	prepared.setdefault('workspace_area', None)
	prepared.setdefault('objects', [])
	if not isinstance(prepared.get('workspace_area'), dict):
		prepared['workspace_area'] = None
	prepared['updated_at'] = iso_timestamp()
	prepared['base_frame'] = base_frame
	prepared['tool_frame'] = tool_frame
	prepared['ground_plane_z'] = ground_plane_z
	return prepared


def write_workspace_config(
	path: Path,
	workspace_config: Dict[str, Any],
	base_frame: str,
	tool_frame: str,
	ground_plane_z: float,
) -> Dict[str, Any]:
	"""!
	@brief Write a normalized workspace configuration to disk.

	@param path Destination YAML path.
	@param workspace_config Existing workspace configuration.
	@param base_frame Frame used as the workspace reference.
	@param tool_frame Tool frame used when samples are captured.
	@param ground_plane_z Ground height in the base frame.
	@return The normalized configuration that was written.
	"""
	prepared = prepare_workspace_config(workspace_config, base_frame, tool_frame, ground_plane_z)
	write_yaml_dict(path, prepared)
	return prepared


def build_geometry(
	shape_key: str,
	samples: List[Dict[str, Any]],
	shape_definition: Dict[str, Any],
	ground_plane_z: float,
) -> Dict[str, Any]:
	"""!
	@brief Derive MoveIt-friendly geometry from captured calibration samples.

	@param shape_key Logical shape name such as rectangle or cylinder.
	@param samples Raw capture samples collected from the robot pose.
	@param shape_definition Shape metadata that describes the expected capture layout.
	@param ground_plane_z Ground height in the base frame.
	@return Geometry dictionary ready to be written into the workspace YAML.
	"""
	points = [sample['pose']['position'] for sample in samples]
	geometry_type = shape_definition.get('geometry_type', 'generic')

	if shape_key == 'rectangle' and len(points) == 4:
		return _build_rectangle_geometry(points, geometry_type, ground_plane_z)
	if shape_key == 'cylinder' and len(points) >= 2:
		return _build_cylinder_geometry(points, geometry_type, ground_plane_z)

	average_z = sum(point['z'] for point in points) / len(points)
	return {
		'type': geometry_type,
		'height': max(0.0, average_z - ground_plane_z),
		'top_face_points': points,
	}


def build_workspace_area(
	samples: List[Dict[str, Any]],
	ground_plane_z: float,
) -> Dict[str, Any]:
	"""!
	@brief Derive a square-like workspace area from four captured corner samples.

	@param samples Raw capture samples collected from the robot pose.
	@param ground_plane_z Ground height in the base frame.
	@return Workspace area dictionary ready to be written into the workspace YAML.
	@throws RuntimeError Raised when the capture does not contain exactly four points.
	"""
	points = [sample['pose']['position'] for sample in samples]
	if len(points) != 4:
		raise RuntimeError('Workspace area capture requires exactly four corners.')

	center_x = sum(point['x'] for point in points) / len(points)
	center_y = sum(point['y'] for point in points) / len(points)
	center_z = sum(point['z'] for point in points) / len(points)
	side_length = sum(_distance_xy(points[index], points[(index + 1) % len(points)]) for index in range(4)) / 4.0
	height = max(0.0, center_z - ground_plane_z)

	return {
		'type': 'square',
		'dimensions': {
			'side_length': side_length,
			'height_from_ground': height,
		},
		'pose': {
			'position': {
				'x': center_x,
				'y': center_y,
				'z': center_z,
			},
			'orientation': {
				'x': 0.0,
				'y': 0.0,
				'z': 0.0,
				'w': 1.0,
			},
		},
		'corner_points': points,
	}


def point_in_workspace_area(
	workspace_area: Dict[str, Any],
	point: Dict[str, float],
	tolerance: float = 1e-6,
) -> bool:
	"""!
	@brief Check whether a 2D point lies inside the calibrated workspace polygon.

	@param workspace_area Workspace area dictionary loaded from YAML.
	@param point Point to test, typically a target pose position.
	@param tolerance Numerical tolerance used on polygon edges.
	@return True when the point lies inside or on the boundary of the area.
	"""
	corner_points = workspace_area.get('corner_points', [])
	if len(corner_points) != 4:
		return False

	cross_values: List[float] = []
	for index, start in enumerate(corner_points):
		end = corner_points[(index + 1) % len(corner_points)]
		edge_x = float(end['x']) - float(start['x'])
		edge_y = float(end['y']) - float(start['y'])
		point_x = float(point['x']) - float(start['x'])
		point_y = float(point['y']) - float(start['y'])
		cross_values.append((edge_x * point_y) - (edge_y * point_x))

	all_non_negative = all(value >= -tolerance for value in cross_values)
	all_non_positive = all(value <= tolerance for value in cross_values)
	return all_non_negative or all_non_positive


def collision_objects_from_workspace(
	workspace_config: Dict[str, Any],
	default_frame: str,
	warn: Optional[Callable[[str], None]] = None,
) -> List[CollisionObject]:
	"""!
	@brief Convert persisted workspace geometry into MoveIt collision objects.

	@param workspace_config Workspace configuration loaded from YAML.
	@param default_frame Fallback frame when the workspace file does not define one.
	@param warn Optional callback used for unsupported geometry warnings.
	@return Collision objects that can be applied to the planning scene.
	"""
	planning_frame = str(workspace_config.get('base_frame', default_frame))
	objects: List[CollisionObject] = []

	for workspace_object in workspace_config.get('objects', []):
		geometry = workspace_object.get('geometry', {})
		geometry_type = geometry.get('type')
		if geometry_type not in {'box', 'cylinder'}:
			if warn is not None:
				warn(
					f"Skipping {workspace_object.get('name', 'unnamed')} "
					f'with unsupported geometry type {geometry_type}.'
				)
			continue

		primitive = SolidPrimitive()
		dimensions = geometry.get('dimensions', {})
		if geometry_type == 'box':
			primitive.type = SolidPrimitive.BOX
			primitive.dimensions = [
				float(dimensions.get('x', 0.0)),
				float(dimensions.get('y', 0.0)),
				float(dimensions.get('z', 0.0)),
			]
		else:
			primitive.type = SolidPrimitive.CYLINDER
			primitive.dimensions = [
				float(dimensions.get('height', 0.0)),
				float(dimensions.get('radius', 0.0)),
			]

		pose = dict_to_pose(geometry.get('pose', {}))
		pose.orientation = _normalized_orientation(pose.orientation)

		collision_object = CollisionObject()
		collision_object.id = str(workspace_object.get('name', f'object_{len(objects) + 1}'))
		collision_object.header.frame_id = planning_frame
		collision_object.primitives = [primitive]
		collision_object.primitive_poses = [pose]
		collision_object.operation = CollisionObject.ADD
		objects.append(collision_object)

	return objects


def _build_rectangle_geometry(
	points: List[Dict[str, float]],
	geometry_type: str,
	ground_plane_z: float,
) -> Dict[str, Any]:
	"""
	@brief Build a box-like geometry model from four top-face corner samples.

	@param points Captured top-face corner positions.
	@param geometry_type Output primitive type name.
	@param ground_plane_z Ground height in the base frame.
	@return Geometry dictionary describing a rectangular prism.
	"""
	edge_a = _distance(points[0], points[1])
	edge_b = _distance(points[1], points[2])
	edge_c = _distance(points[2], points[3])
	edge_d = _distance(points[3], points[0])
	size_x = (edge_a + edge_c) / 2.0
	size_y = (edge_b + edge_d) / 2.0
	top_z = sum(point['z'] for point in points) / len(points)
	center_x = sum(point['x'] for point in points) / len(points)
	center_y = sum(point['y'] for point in points) / len(points)
	height = max(0.0, top_z - ground_plane_z)

	return {
		'type': geometry_type,
		'dimensions': {
			'x': size_x,
			'y': size_y,
			'z': height,
		},
		'pose': {
			'position': {
				'x': center_x,
				'y': center_y,
				'z': ground_plane_z + (height / 2.0),
			},
			'orientation': {
				'x': 0.0,
				'y': 0.0,
				'z': 0.0,
				'w': 1.0,
			},
		},
		'top_face_points': points,
	}


def _build_cylinder_geometry(
	points: List[Dict[str, float]],
	geometry_type: str,
	ground_plane_z: float,
) -> Dict[str, Any]:
	"""
	@brief Build a cylinder model from one center sample and rim samples.

	@param points Captured top-face points with the center first.
	@param geometry_type Output primitive type name.
	@param ground_plane_z Ground height in the base frame.
	@return Geometry dictionary describing a cylinder.
	"""
	center_point = points[0]
	rim_points = points[1:]
	top_z = sum(point['z'] for point in points) / len(points)
	height = max(0.0, top_z - ground_plane_z)
	radius = sum(_distance_xy(center_point, point) for point in rim_points) / len(rim_points)

	return {
		'type': geometry_type,
		'dimensions': {
			'height': height,
			'radius': radius,
		},
		'pose': {
			'position': {
				'x': center_point['x'],
				'y': center_point['y'],
				'z': ground_plane_z + (height / 2.0),
			},
			'orientation': {
				'x': 0.0,
				'y': 0.0,
				'z': 0.0,
				'w': 1.0,
			},
		},
		'top_face_points': points,
	}


def _normalized_orientation(orientation: Any) -> Any:
	"""
	@brief Normalize an in-place orientation object with x/y/z/w members.

	@param orientation Orientation-like object to normalize.
	@return The same orientation object after normalization.
	"""
	normalized = normalize_quaternion(
		Quaternion(
			x=float(orientation.x),
			y=float(orientation.y),
			z=float(orientation.z),
			w=float(orientation.w),
		)
	)
	orientation.x = normalized.x
	orientation.y = normalized.y
	orientation.z = normalized.z
	orientation.w = normalized.w
	return orientation


def _distance(start: Dict[str, float], end: Dict[str, float]) -> float:
	"""
	@brief Compute Euclidean distance between two 3D points.

	@param start Start point.
	@param end End point.
	@return 3D distance between the points.
	"""
	return math.sqrt(
		((end['x'] - start['x']) ** 2)
		+ ((end['y'] - start['y']) ** 2)
		+ ((end['z'] - start['z']) ** 2)
	)


def _distance_xy(start: Dict[str, float], end: Dict[str, float]) -> float:
	"""
	@brief Compute planar distance between two points using x and y only.

	@param start Start point.
	@param end End point.
	@return XY-plane distance between the points.
	"""
	return math.sqrt(((end['x'] - start['x']) ** 2) + ((end['y'] - start['y']) ** 2))
