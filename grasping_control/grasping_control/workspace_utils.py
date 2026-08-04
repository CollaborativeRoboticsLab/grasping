from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import math
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from grasping_control.common import (
	Quaternion,
	coerce_float_sequence,
	coerce_string_sequence,
	dict_to_pose,
	normalize_quaternion,
	write_yaml_dict,
)


WORKSPACE_PARAMETER_NODE = 'motion_execution_node'


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


def workspace_config_from_document(
	document: Dict[str, Any],
	default_config: Dict[str, Any],
) -> Dict[str, Any]:
	"""!
	@brief Convert a ROS parameter workspace YAML document into runtime config.

	@param document Parsed YAML document.
	@param default_config Default workspace configuration used for missing values.
	@return Runtime workspace configuration dictionary.
	"""
	if not isinstance(document, dict):
		return deepcopy(default_config)

	parameters = _workspace_parameter_mapping(document)
	if parameters is None:
		return deepcopy(default_config)

	return workspace_config_from_ros_parameters(parameters, default_config)


def workspace_config_from_ros_parameters(
	parameters: Dict[str, Any],
	default_config: Dict[str, Any],
) -> Dict[str, Any]:
	"""!
	@brief Convert motion_execution_node workspace parameters into runtime config.

	@param parameters Mapping from the `ros__parameters` section.
	@param default_config Default workspace configuration used for missing values.
	@return Runtime workspace configuration dictionary.
	"""
	workspace = _nested_mapping(parameters, 'workspace')
	workspace_config: Dict[str, Any] = {
		'version': int(_nested_value(parameters, 'workspace.version', workspace.get('version', default_config.get('version', 1)))),
		'updated_at': str(_nested_value(parameters, 'workspace.updated_at', workspace.get('updated_at', default_config.get('updated_at', iso_timestamp())))),
		'base_frame': str(_nested_value(parameters, 'workspace.base_frame', workspace.get('base_frame', default_config.get('base_frame', 'world')))),
		'tool_frame': str(_nested_value(parameters, 'workspace.tool_frame', workspace.get('tool_frame', default_config.get('tool_frame', 'tool_tip')))),
		'ground_plane_z': float(_nested_value(parameters, 'workspace.ground_plane_z', workspace.get('ground_plane_z', default_config.get('ground_plane_z', 0.0)))),
		'workspace_area': None,
		'objects': [],
	}

	workspace_area = _workspace_area_from_parameters(parameters)
	if workspace_area is not None:
		workspace_config['workspace_area'] = workspace_area

	workspace_config['objects'] = _workspace_objects_from_parameters(parameters)
	return workspace_config


def workspace_config_from_node_parameters(
	node: Node,
	default_config: Dict[str, Any],
) -> Dict[str, Any]:
	"""!
	@brief Reconstruct runtime workspace config from motion_execution_node ROS parameters.

	@param node ROS node exposing the workspace parameters.
	@param default_config Default workspace configuration used for missing values.
	@return Runtime workspace configuration dictionary.
	"""
	workspace_object_names = coerce_string_sequence(node.get_parameter('workspace_objects').value)
	parameters: Dict[str, Any] = {
		'workspace': {
			'version': int(node.get_parameter('workspace.version').value),
			'updated_at': str(node.get_parameter('workspace.updated_at').value),
			'base_frame': str(node.get_parameter('workspace.base_frame').value),
			'tool_frame': str(node.get_parameter('workspace.tool_frame').value),
			'ground_plane_z': float(node.get_parameter('workspace.ground_plane_z').value),
		},
		'workspace_area': {
			'enabled': _parameter_bool(node.get_parameter('workspace_area.enabled').value),
			'geometry': {
				'type': str(node.get_parameter('workspace_area.geometry.type').value),
				'dimensions': coerce_float_sequence(
					node.get_parameter('workspace_area.geometry.dimensions').value,
					2,
					'workspace_area.geometry.dimensions',
				),
				'pose': {
					'position': coerce_float_sequence(
						node.get_parameter('workspace_area.geometry.pose.position').value,
						3,
						'workspace_area.geometry.pose.position',
					),
					'orientation': coerce_float_sequence(
						node.get_parameter('workspace_area.geometry.pose.orientation').value,
						4,
						'workspace_area.geometry.pose.orientation',
					),
				},
				'corner_points': {
					'x': coerce_float_sequence(
						node.get_parameter('workspace_area.geometry.corner_points.x').value,
						4,
						'workspace_area.geometry.corner_points.x',
					),
					'y': coerce_float_sequence(
						node.get_parameter('workspace_area.geometry.corner_points.y').value,
						4,
						'workspace_area.geometry.corner_points.y',
					),
					'z': coerce_float_sequence(
						node.get_parameter('workspace_area.geometry.corner_points.z').value,
						4,
						'workspace_area.geometry.corner_points.z',
					),
				},
			},
		},
		'workspace_objects': workspace_object_names,
		'workspace_object': {},
	}

	workspace_object_parameters = parameters['workspace_object']
	for object_name in workspace_object_names:
		prefix = f'workspace_object.{object_name}'
		geometry_type = str(node.get_parameter(f'{prefix}.geometry.type').value)
		geometry_dimensions = coerce_float_sequence(
			node.get_parameter(f'{prefix}.geometry.dimensions').value,
			3 if geometry_type == 'box' else 2,
			f'{prefix}.geometry.dimensions',
		)
		workspace_object_parameters[object_name] = {
			'geometry': {
				'type': geometry_type,
				'dimensions': geometry_dimensions,
				'pose': {
					'position': coerce_float_sequence(
						node.get_parameter(f'{prefix}.geometry.pose.position').value,
						3,
						f'{prefix}.geometry.pose.position',
					),
					'orientation': coerce_float_sequence(
						node.get_parameter(f'{prefix}.geometry.pose.orientation').value,
						4,
						f'{prefix}.geometry.pose.orientation',
					),
				},
			},
			'shape': str(node.get_parameter(f'{prefix}.shape').value),
			'allowed_collision_links': coerce_string_sequence(
				node.get_parameter(f'{prefix}.allowed_collision_links').value
			),
		}

	return workspace_config_from_ros_parameters(parameters, default_config)


def workspace_config_to_ros_parameters_document(
	workspace_config: Dict[str, Any],
	node_name: str = WORKSPACE_PARAMETER_NODE,
) -> Dict[str, Any]:
	"""!
	@brief Convert runtime workspace config into ROS parameter YAML document shape.

	@param workspace_config Runtime workspace configuration dictionary.
	@param node_name Node name that should receive the parameters.
	@return ROS parameter YAML document.
	"""
	parameters: Dict[str, Any] = {
		'workspace': {
			'version': int(workspace_config.get('version', 1)),
			'updated_at': str(workspace_config.get('updated_at', iso_timestamp())),
			'base_frame': str(workspace_config.get('base_frame', 'world')),
			'tool_frame': str(workspace_config.get('tool_frame', 'tool_tip')),
			'ground_plane_z': float(workspace_config.get('ground_plane_z', 0.0)),
		},
		'workspace_area': None,
		'workspace_objects': [],
		'workspace_object': {},
	}

	workspace_area = workspace_config.get('workspace_area')
	if isinstance(workspace_area, dict):
		parameters['workspace_area'] = _workspace_area_to_parameters(workspace_area)

	for workspace_object in workspace_config.get('objects', []):
		if not isinstance(workspace_object, dict):
			continue
		object_name = str(workspace_object.get('name', 'unnamed'))
		parameters['workspace_objects'].append(object_name)
		parameters['workspace_object'][object_name] = _workspace_object_to_parameters(workspace_object)

	return {
		node_name: {
			'ros__parameters': parameters,
		}
	}


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
	allowed_collision_links = workspace_object.get('allowed_collision_links')
	if isinstance(allowed_collision_links, list):
		prepared['allowed_collision_links'] = [
			str(link_name).strip()
			for link_name in allowed_collision_links
			if str(link_name).strip()
		]

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
	write_yaml_dict(path, workspace_config_to_ros_parameters_document(prepared))
	return prepared


def _workspace_parameter_mapping(document: Dict[str, Any]) -> Optional[Dict[str, Any]]:
	"""Return the ros__parameters mapping when a workspace document uses ROS params."""
	node_config = document.get(WORKSPACE_PARAMETER_NODE)
	if isinstance(node_config, dict):
		parameters = node_config.get('ros__parameters')
		if isinstance(parameters, dict):
			return parameters

	parameters = document.get('ros__parameters')
	if isinstance(parameters, dict):
		return parameters
	return None


def _parameter_bool(value: Any) -> bool:
	"""Return a boolean ROS parameter value that may arrive as a string."""
	if isinstance(value, str):
		return value.strip().lower() in {'1', 'true', 'yes', 'on'}
	return bool(value)


def _workspace_area_from_parameters(parameters: Dict[str, Any]) -> Optional[Dict[str, Any]]:
	"""Build a workspace_area entry from ROS parameters when one is configured."""
	workspace_area = _nested_value(parameters, 'workspace_area', None)
	if workspace_area is None:
		return None
	if isinstance(workspace_area, dict) and not bool(workspace_area.get('enabled', True)):
		return None

	geometry = _nested_mapping(parameters, 'workspace_area.geometry')
	if not geometry and isinstance(workspace_area, dict):
		geometry = workspace_area.get('geometry', {}) if isinstance(workspace_area.get('geometry'), dict) else {}
	if not geometry:
		return None

	return {'geometry': _geometry_from_parameter_mapping(geometry, is_workspace_area=True)}


def _workspace_objects_from_parameters(parameters: Dict[str, Any]) -> List[Dict[str, Any]]:
	"""Build workspace object entries from ROS parameters."""
	object_names = _nested_value(parameters, 'workspace_objects', [])
	if not isinstance(object_names, list):
		return []

	objects: List[Dict[str, Any]] = []
	workspace_object_map = _nested_mapping(parameters, 'workspace_object')
	for object_name_value in object_names:
		object_name = str(object_name_value).strip()
		if not object_name:
			continue
		object_config = workspace_object_map.get(object_name, {})
		if not isinstance(object_config, dict):
			continue
		geometry_config = object_config.get('geometry', {})
		if not isinstance(geometry_config, dict):
			continue
		workspace_object = {
			'name': object_name,
			'geometry': _geometry_from_parameter_mapping(geometry_config, is_workspace_area=False),
		}
		shape = object_config.get('shape')
		if shape is not None:
			workspace_object['shape'] = str(shape)
		allowed_collision_links = object_config.get('allowed_collision_links')
		if isinstance(allowed_collision_links, list):
			workspace_object['allowed_collision_links'] = [
				str(link_name).strip()
				for link_name in allowed_collision_links
				if str(link_name).strip()
			]
		objects.append(workspace_object)
	return objects


def _geometry_from_parameter_mapping(geometry: Dict[str, Any], is_workspace_area: bool) -> Dict[str, Any]:
	"""Convert parameter geometry arrays into runtime geometry mappings."""
	geometry_type = str(geometry.get('type', ''))
	dimensions = _dimensions_from_parameter_value(geometry_type, geometry.get('dimensions', []), is_workspace_area)
	pose = _pose_from_parameter_mapping(geometry.get('pose', {}))
	converted: Dict[str, Any] = {
		'type': geometry_type,
		'dimensions': dimensions,
		'pose': pose,
	}

	corner_points = geometry.get('corner_points')
	if isinstance(corner_points, dict):
		converted['corner_points'] = _corner_points_from_axes(corner_points)
	return converted


def _dimensions_from_parameter_value(
	geometry_type: str,
	value: Any,
	is_workspace_area: bool,
) -> Dict[str, float]:
	"""Convert dimension arrays into named dimension mappings."""
	if isinstance(value, dict):
		return deepcopy(value)
	dimensions = [float(item) for item in list(value)] if isinstance(value, list) else []
	if is_workspace_area:
		return {
			'side_length': dimensions[0] if len(dimensions) > 0 else 0.0,
			'height_from_ground': dimensions[1] if len(dimensions) > 1 else 0.0,
		}
	if geometry_type == 'cylinder':
		return {
			'height': dimensions[0] if len(dimensions) > 0 else 0.0,
			'radius': dimensions[1] if len(dimensions) > 1 else 0.0,
		}
	return {
		'x': dimensions[0] if len(dimensions) > 0 else 0.0,
		'y': dimensions[1] if len(dimensions) > 1 else 0.0,
		'z': dimensions[2] if len(dimensions) > 2 else 0.0,
	}


def _pose_from_parameter_mapping(pose: Any) -> Dict[str, Any]:
	"""Convert pose position/orientation arrays into named mappings."""
	if not isinstance(pose, dict):
		pose = {}
	position = pose.get('position', [0.0, 0.0, 0.0])
	orientation = pose.get('orientation', [0.0, 0.0, 0.0, 1.0])
	if isinstance(position, dict) and isinstance(orientation, dict):
		return {'position': deepcopy(position), 'orientation': deepcopy(orientation)}
	position_values = [float(item) for item in list(position)]
	orientation_values = [float(item) for item in list(orientation)]
	return {
		'position': {
			'x': position_values[0] if len(position_values) > 0 else 0.0,
			'y': position_values[1] if len(position_values) > 1 else 0.0,
			'z': position_values[2] if len(position_values) > 2 else 0.0,
		},
		'orientation': {
			'x': orientation_values[0] if len(orientation_values) > 0 else 0.0,
			'y': orientation_values[1] if len(orientation_values) > 1 else 0.0,
			'z': orientation_values[2] if len(orientation_values) > 2 else 0.0,
			'w': orientation_values[3] if len(orientation_values) > 3 else 1.0,
		},
	}


def _corner_points_from_axes(corner_points: Dict[str, Any]) -> List[Dict[str, float]]:
	"""Convert x/y/z corner arrays into a list of point mappings."""
	x_values = [float(item) for item in list(corner_points.get('x', []))]
	y_values = [float(item) for item in list(corner_points.get('y', []))]
	z_values = [float(item) for item in list(corner_points.get('z', []))]
	count = min(len(x_values), len(y_values), len(z_values))
	return [
		{'x': x_values[index], 'y': y_values[index], 'z': z_values[index]}
		for index in range(count)
	]


def _workspace_area_to_parameters(workspace_area: Dict[str, Any]) -> Dict[str, Any]:
	"""Convert a runtime workspace_area entry into ROS parameter schema."""
	geometry = workspace_area.get('geometry', {})
	dimensions = geometry.get('dimensions', {}) if isinstance(geometry, dict) else {}
	return {
		'enabled': True,
		'geometry': {
			'type': str(geometry.get('type', 'square')),
			'dimensions': [
				float(dimensions.get('side_length', 0.0)),
				float(dimensions.get('height_from_ground', 0.0)),
			],
			'pose': _pose_to_parameters(geometry.get('pose', {})),
			'corner_points': _corner_points_to_axes(geometry.get('corner_points', [])),
		},
	}


def _workspace_object_to_parameters(workspace_object: Dict[str, Any]) -> Dict[str, Any]:
	"""Convert a runtime workspace object into ROS parameter schema."""
	geometry = workspace_object.get('geometry', {})
	geometry_type = str(geometry.get('type', ''))
	dimensions = geometry.get('dimensions', {})
	if geometry_type == 'cylinder':
		dimension_values = [float(dimensions.get('height', 0.0)), float(dimensions.get('radius', 0.0))]
	else:
		dimension_values = [
			float(dimensions.get('x', 0.0)),
			float(dimensions.get('y', 0.0)),
			float(dimensions.get('z', 0.0)),
		]
	parameters = {
		'geometry': {
			'type': geometry_type,
			'dimensions': dimension_values,
			'pose': _pose_to_parameters(geometry.get('pose', {})),
		},
		'shape': str(workspace_object.get('shape', '')),
	}
	allowed_collision_links = workspace_object.get('allowed_collision_links')
	if isinstance(allowed_collision_links, list):
		parameters['allowed_collision_links'] = [
			str(link_name).strip()
			for link_name in allowed_collision_links
			if str(link_name).strip()
		]
	return parameters


def _pose_to_parameters(pose: Any) -> Dict[str, List[float]]:
	"""Convert a runtime pose mapping into array-based ROS parameter schema."""
	if not isinstance(pose, dict):
		pose = {}
	position = pose.get('position', {}) if isinstance(pose.get('position'), dict) else {}
	orientation = pose.get('orientation', {}) if isinstance(pose.get('orientation'), dict) else {}
	return {
		'position': [
			float(position.get('x', 0.0)),
			float(position.get('y', 0.0)),
			float(position.get('z', 0.0)),
		],
		'orientation': [
			float(orientation.get('x', 0.0)),
			float(orientation.get('y', 0.0)),
			float(orientation.get('z', 0.0)),
			float(orientation.get('w', 1.0)),
		],
	}


def _corner_points_to_axes(corner_points: Any) -> Dict[str, List[float]]:
	"""Convert runtime corner point mappings into x/y/z arrays."""
	if not isinstance(corner_points, list):
		corner_points = []
	return {
		'x': [float(point.get('x', 0.0)) for point in corner_points if isinstance(point, dict)],
		'y': [float(point.get('y', 0.0)) for point in corner_points if isinstance(point, dict)],
		'z': [float(point.get('z', 0.0)) for point in corner_points if isinstance(point, dict)],
	}


def _nested_mapping(parameters: Dict[str, Any], path: str) -> Dict[str, Any]:
	"""Return a nested or dotted mapping value."""
	value = _nested_value(parameters, path, {})
	return value if isinstance(value, dict) else {}


def _nested_value(parameters: Dict[str, Any], path: str, default: Any) -> Any:
	"""Return a parameter value from either nested maps or dotted keys."""
	if path in parameters:
		return parameters[path]
	value: Any = parameters
	for part in path.split('.'):
		if not isinstance(value, dict) or part not in value:
			return default
		value = value[part]
	return value


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
