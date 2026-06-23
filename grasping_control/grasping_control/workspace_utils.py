from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import math
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from grasping_control.common import Quaternion, dict_to_pose, normalize_quaternion, write_yaml_dict


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
			'top_surface_rectangle': {
				'shape_key': 'top_surface_rectangle',
				'display_name': 'Top-surface rectangle',
				'geometry_type': 'box',
				'description': 'Capture the four top-face corners in order around the object.',
				'point_labels': ['corner_1', 'corner_2', 'corner_3', 'corner_4'],
			},
			'right_side_face_rectangle': {
				'shape_key': 'right_side_face_rectangle',
				'display_name': 'Right-side face rectangle (robot on right)',
				'geometry_type': 'box',
				'description': 'Capture the four side-face corners in order, with robot positioned on the RIGHT side. Depth extends to the LEFT.',
				'point_labels': ['corner_1', 'corner_2', 'corner_3', 'corner_4'],
				'manual_parameters': {
					'depth': {
						'prompt': 'Enter obstacle depth extending LEFT from the captured face in meters',
						'min_value': 0.0,
					},
				},
			},
			'left_side_face_rectangle': {
				'shape_key': 'left_side_face_rectangle',
				'display_name': 'Left-side face rectangle (robot on left)',
				'geometry_type': 'box',
				'description': 'Capture the four side-face corners in order, with robot positioned on the LEFT side. Depth extends to the RIGHT.',
				'point_labels': ['corner_1', 'corner_2', 'corner_3', 'corner_4'],
				'manual_parameters': {
					'depth': {
						'prompt': 'Enter obstacle depth extending RIGHT from the captured face in meters',
						'min_value': 0.0,
					},
				},
			},
			'bottom_face_rectangle': {
				'shape_key': 'bottom_face_rectangle',
				'display_name': 'Bottom-face rectangle',
				'geometry_type': 'box',
				'description': 'Capture the four bottom-face corners in order around the hanging obstacle, then enter the obstacle height above that face.',
				'point_labels': ['corner_1', 'corner_2', 'corner_3', 'corner_4'],
				'manual_parameters': {
					'depth': {
						'prompt': 'Enter obstacle height above the captured bottom face in meters',
						'min_value': 0.0,
					},
				},
			},
			'cylinder': {
				'shape_key': 'cylinder',
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
	prepared = {
		'version': int(workspace_config.get('version', 1)),
		'updated_at': iso_timestamp(),
		'base_frame': base_frame,
		'tool_frame': tool_frame,
		'ground_plane_z': ground_plane_z,
		'workspace_area': None,
		'objects': [],
	}

	workspace_area = workspace_config.get('workspace_area')
	if isinstance(workspace_area, dict):
		prepared['workspace_area'] = _prepare_workspace_area(workspace_area)

	for workspace_object in workspace_config.get('objects', []):
		if isinstance(workspace_object, dict):
			prepared['objects'].append(_prepare_workspace_object(workspace_object))

	return prepared


def _prepare_workspace_area(workspace_area: Dict[str, Any]) -> Dict[str, Any]:
	"""!
	@brief Reduce a workspace-area entry to the persisted fields used after calibration.

	@param workspace_area Workspace area entry from the in-memory calibration model.
	@return Minimal persisted workspace area mapping.
	"""
	return {
		'geometry': _prepare_geometry(workspace_area.get('geometry', {})),
	}


def _prepare_workspace_object(workspace_object: Dict[str, Any]) -> Dict[str, Any]:
	"""!
	@brief Reduce a workspace object entry to the persisted fields used after calibration.

	@param workspace_object Object entry from the in-memory calibration model.
	@return Minimal persisted workspace object mapping.
	"""
	prepared = {
		'name': str(workspace_object.get('name', 'unnamed')),
		'geometry': _prepare_geometry(workspace_object.get('geometry', {})),
	}

	shape = workspace_object.get('shape')
	if shape is not None:
		prepared['shape'] = str(shape)

	return prepared


def _prepare_geometry(geometry: Dict[str, Any]) -> Dict[str, Any]:
	"""!
	@brief Keep only the geometry fields consumed by runtime code.

	@param geometry Geometry mapping from the in-memory calibration model.
	@return Minimal persisted geometry mapping.
	"""
	prepared: Dict[str, Any] = {}

	geometry_type = geometry.get('type')
	if geometry_type is not None:
		prepared['type'] = geometry_type

	dimensions = geometry.get('dimensions')
	if isinstance(dimensions, dict):
		prepared['dimensions'] = deepcopy(dimensions)

	pose = geometry.get('pose')
	if isinstance(pose, dict):
		prepared['pose'] = deepcopy(pose)

	corner_points = geometry.get('corner_points')
	if isinstance(corner_points, list):
		prepared['corner_points'] = deepcopy(corner_points)

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
	shape_parameters: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
	"""!
	@brief Derive MoveIt-friendly geometry from captured calibration samples.

	@param shape_key Logical shape name such as rectangle or cylinder.
	@param samples Raw capture samples collected from the robot pose.
	@param shape_definition Shape metadata that describes the expected capture layout.
	@param ground_plane_z Ground height in the base frame.
	@param shape_parameters Optional operator-supplied parameters for the selected shape.
	@return Geometry dictionary ready to be written into the workspace YAML.
	"""
	points = [sample['pose']['position'] for sample in samples]
	geometry_type = shape_definition.get('geometry_type', 'generic')
	shape_parameters = shape_parameters or {}
	horizontal_plane = bool(shape_parameters.get('parallel_to_ground', False))

	if shape_key in {'rectangle', 'top_surface_rectangle'} and len(points) == 4:
		return _build_top_surface_rectangle_geometry(points, geometry_type, ground_plane_z, horizontal_plane)
	if shape_key in {'side_face_rectangle', 'left_side_face_rectangle', 'right_side_face_rectangle'} and len(points) == 4:
		# Determine orientation: left=True for left-side, False for right-side or legacy side_face_rectangle
		is_left_side = shape_key == 'left_side_face_rectangle'
		return _build_side_face_rectangle_geometry(points, geometry_type, shape_parameters, is_left_side)
	if shape_key == 'bottom_face_rectangle' and len(points) == 4:
		return _build_bottom_face_rectangle_geometry(points, geometry_type, shape_parameters, horizontal_plane)
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


def _build_top_surface_rectangle_geometry(
	points: List[Dict[str, float]],
	geometry_type: str,
	ground_plane_z: float,
	horizontal_plane: bool = False,
) -> Dict[str, Any]:
	"""
	@brief Build a box-like geometry model from four top-face corner samples.

	@param points Captured top-face corner positions.
	@param geometry_type Output primitive type name.
	@param ground_plane_z Ground height in the base frame.
	@return Geometry dictionary describing a rectangular prism.
	"""
	frame = _rectangle_frame(_horizontalized_points(points) if horizontal_plane else points)
	top_z = frame['center']['z']
	height = max(0.0, top_z - ground_plane_z)
	center = _translate_point(frame['center'], frame['normal'], -(height / 2.0))
	orientation = _quaternion_from_axes(frame['axis_u'], frame['axis_v'], frame['normal'])

	return {
		'type': geometry_type,
		'dimensions': {
			'x': frame['size_u'],
			'y': frame['size_v'],
			'z': height,
		},
		'pose': {
			'position': center,
			'orientation': orientation,
		},
		'top_face_points': points,
	}


def _build_side_face_rectangle_geometry(
	points: List[Dict[str, float]],
	geometry_type: str,
	shape_parameters: Dict[str, Any],
	is_left_side: bool = False,
) -> Dict[str, Any]:
	"""
	@brief Build a box model from a captured side face and operator-supplied depth.

	@param points Captured side-face corner positions.
	@param geometry_type Output primitive type name.
	@param shape_parameters Operator-supplied shape parameters.
	@param is_left_side When True, the robot is on the left side and depth extends right.
	                    When False, the robot is on the right side and depth extends left.
	@return Geometry dictionary describing a side-face anchored box.
	"""
	frame = _rectangle_frame(points)
	depth = max(0.0, float(shape_parameters.get('depth', 0.0)))
	
	# For left-side capture, we need to flip the normal direction so depth extends to the right
	normal = frame['normal']
	if is_left_side:
		normal = {
			'x': -normal['x'],
			'y': -normal['y'],
			'z': -normal['z'],
		}
	
	center = _translate_point(frame['center'], normal, depth / 2.0)
	orientation = _quaternion_from_axes(frame['axis_u'], frame['axis_v'], normal)

	return {
		'type': geometry_type,
		'dimensions': {
			'x': frame['size_u'],
			'y': frame['size_v'],
			'z': depth,
		},
		'pose': {
			'position': center,
			'orientation': orientation,
		},
		'face_points': points,
	}


def _build_bottom_face_rectangle_geometry(
	points: List[Dict[str, float]],
	geometry_type: str,
	shape_parameters: Dict[str, Any],
	horizontal_plane: bool = False,
) -> Dict[str, Any]:
	"""
	@brief Build a box model from a captured bottom face and operator-supplied height.

	@param points Captured bottom-face corner positions.
	@param geometry_type Output primitive type name.
	@param shape_parameters Operator-supplied shape parameters.
	@return Geometry dictionary describing a hanging rectangular obstacle.
	"""
	frame = _rectangle_frame(_horizontalized_points(points) if horizontal_plane else points)
	depth = max(0.0, float(shape_parameters.get('depth', 0.0)))
	if horizontal_plane:
		up_axis = {'x': 0.0, 'y': 0.0, 'z': 1.0}
		axis_u = _normalize_vector({'x': frame['axis_u']['x'], 'y': frame['axis_u']['y'], 'z': 0.0})
		axis_v = _normalize_vector(_cross_product(up_axis, axis_u))
		center = _translate_point(frame['center'], up_axis, depth / 2.0)
		orientation = _quaternion_from_axes(axis_u, axis_v, up_axis)
	else:
		center = _translate_point(frame['center'], frame['normal'], depth / 2.0)
		orientation = _quaternion_from_axes(frame['axis_u'], frame['axis_v'], frame['normal'])

	return {
		'type': geometry_type,
		'dimensions': {
			'x': frame['size_u'],
			'y': frame['size_v'],
			'z': depth,
		},
		'pose': {
			'position': center,
			'orientation': orientation,
		},
		'bottom_face_points': points,
	}


def _horizontalized_points(points: List[Dict[str, float]]) -> List[Dict[str, float]]:
	"""
	@brief Return copies of points with a shared averaged z value.

	@param points Captured face points.
	@return Copied points flattened onto a horizontal plane.
	"""
	if not points:
		return []

	average_z = sum(float(point['z']) for point in points) / len(points)
	return [
		{
			'x': float(point['x']),
			'y': float(point['y']),
			'z': average_z,
		}
		for point in points
	]


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


def _rectangle_frame(points: List[Dict[str, float]]) -> Dict[str, Any]:
	"""
	@brief Build an orthonormal frame and dimensions from four rectangle corner samples.

	@param points Captured rectangle corners in order around the face.
	@return Rectangle center, in-plane axes, outward normal, and edge lengths.
	"""
	if len(points) != 4:
		raise RuntimeError('Rectangle geometry requires exactly four corners.')

	center = {
		'x': sum(point['x'] for point in points) / len(points),
		'y': sum(point['y'] for point in points) / len(points),
		'z': sum(point['z'] for point in points) / len(points),
	}
	edge_u = _average_vector(_subtract_points(points[1], points[0]), _subtract_points(points[2], points[3]))
	edge_v = _average_vector(_subtract_points(points[2], points[1]), _subtract_points(points[3], points[0]))
	size_u = (_distance(points[0], points[1]) + _distance(points[2], points[3])) / 2.0
	size_v = (_distance(points[1], points[2]) + _distance(points[3], points[0])) / 2.0
	axis_u = _normalize_vector(edge_u)
	axis_v = _normalize_vector(edge_v)
	normal = _normalize_vector(_cross_product(axis_u, axis_v))

	if normal['z'] < 0.0:
		axis_v = _scale_vector(axis_v, -1.0)
		normal = _scale_vector(normal, -1.0)

	axis_v = _normalize_vector(_cross_product(normal, axis_u))
	return {
		'center': center,
		'axis_u': axis_u,
		'axis_v': axis_v,
		'normal': normal,
		'size_u': size_u,
		'size_v': size_v,
	}


def _quaternion_from_axes(
	axis_x: Dict[str, float],
	axis_y: Dict[str, float],
	axis_z: Dict[str, float],
) -> Dict[str, float]:
	"""
	@brief Convert an orthonormal basis into a quaternion dictionary.

	@param axis_x Local x axis.
	@param axis_y Local y axis.
	@param axis_z Local z axis.
	@return Quaternion dictionary.
	"""
	m00 = axis_x['x']
	m01 = axis_y['x']
	m02 = axis_z['x']
	m10 = axis_x['y']
	m11 = axis_y['y']
	m12 = axis_z['y']
	m20 = axis_x['z']
	m21 = axis_y['z']
	m22 = axis_z['z']
	trace = m00 + m11 + m22

	if trace > 0.0:
		s = math.sqrt(trace + 1.0) * 2.0
		quaternion = {
			'w': 0.25 * s,
			'x': (m21 - m12) / s,
			'y': (m02 - m20) / s,
			'z': (m10 - m01) / s,
		}
	elif m00 > m11 and m00 > m22:
		s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
		quaternion = {
			'w': (m21 - m12) / s,
			'x': 0.25 * s,
			'y': (m01 + m10) / s,
			'z': (m02 + m20) / s,
		}
	elif m11 > m22:
		s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
		quaternion = {
			'w': (m02 - m20) / s,
			'x': (m01 + m10) / s,
			'y': 0.25 * s,
			'z': (m12 + m21) / s,
		}
	else:
		s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
		quaternion = {
			'w': (m10 - m01) / s,
			'x': (m02 + m20) / s,
			'y': (m12 + m21) / s,
			'z': 0.25 * s,
		}

	normalized = normalize_quaternion(
		Quaternion(
			x=float(quaternion['x']),
			y=float(quaternion['y']),
			z=float(quaternion['z']),
			w=float(quaternion['w']),
		)
	)
	return {
		'x': normalized.x,
		'y': normalized.y,
		'z': normalized.z,
		'w': normalized.w,
	}


def _subtract_points(end: Dict[str, float], start: Dict[str, float]) -> Dict[str, float]:
	"""
	@brief Build a vector from start to end.

	@param end Vector end point.
	@param start Vector start point.
	@return Vector components.
	"""
	return {
		'x': float(end['x']) - float(start['x']),
		'y': float(end['y']) - float(start['y']),
		'z': float(end['z']) - float(start['z']),
	}


def _average_vector(first: Dict[str, float], second: Dict[str, float]) -> Dict[str, float]:
	"""
	@brief Average two vectors component-wise.

	@param first First vector.
	@param second Second vector.
	@return Average vector.
	"""
	return {
		'x': (float(first['x']) + float(second['x'])) / 2.0,
		'y': (float(first['y']) + float(second['y'])) / 2.0,
		'z': (float(first['z']) + float(second['z'])) / 2.0,
	}


def _cross_product(first: Dict[str, float], second: Dict[str, float]) -> Dict[str, float]:
	"""
	@brief Compute the cross product of two 3D vectors.

	@param first First vector.
	@param second Second vector.
	@return Cross product vector.
	"""
	return {
		'x': (float(first['y']) * float(second['z'])) - (float(first['z']) * float(second['y'])),
		'y': (float(first['z']) * float(second['x'])) - (float(first['x']) * float(second['z'])),
		'z': (float(first['x']) * float(second['y'])) - (float(first['y']) * float(second['x'])),
	}


def _normalize_vector(vector: Dict[str, float]) -> Dict[str, float]:
	"""
	@brief Normalize a 3D vector.

	@param vector Vector to normalize.
	@return Unit-length vector.
	"""
	length = math.sqrt(
		(float(vector['x']) ** 2) + (float(vector['y']) ** 2) + (float(vector['z']) ** 2)
	)
	if length <= 1e-9:
		raise RuntimeError('Captured rectangle points do not define a valid face.')
	return {
		'x': float(vector['x']) / length,
		'y': float(vector['y']) / length,
		'z': float(vector['z']) / length,
	}


def _scale_vector(vector: Dict[str, float], scale: float) -> Dict[str, float]:
	"""
	@brief Scale a vector by a scalar.

	@param vector Vector to scale.
	@param scale Scalar multiplier.
	@return Scaled vector.
	"""
	return {
		'x': float(vector['x']) * scale,
		'y': float(vector['y']) * scale,
		'z': float(vector['z']) * scale,
	}


def _translate_point(point: Dict[str, float], direction: Dict[str, float], distance: float) -> Dict[str, float]:
	"""
	@brief Translate a point along a direction vector by a scalar distance.

	@param point Point to translate.
	@param direction Translation direction.
	@param distance Translation distance.
	@return Translated point.
	"""
	return {
		'x': float(point['x']) + (float(direction['x']) * distance),
		'y': float(point['y']) + (float(direction['y']) * distance),
		'z': float(point['z']) + (float(direction['z']) * distance),
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
