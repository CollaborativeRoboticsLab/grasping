from copy import deepcopy
from datetime import datetime, timezone
import math
import threading
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from grasping_arm_control.common import load_yaml_dict, resolve_config_path, write_yaml_dict


def _iso_timestamp() -> str:
	return datetime.now(timezone.utc).isoformat()


class WorkspaceCalibrationNode(Node):
	def __init__(self) -> None:
		super().__init__('workspace_calibration_node')

		self.declare_parameter('joint_state_topic', '/joint_states')
		self.declare_parameter('base_frame', 'base_link')
		self.declare_parameter('tool_frame', 'tool0')
		self.declare_parameter('ground_plane_z', 0.0)
		self.declare_parameter('workspace_config_path', '')
		self.declare_parameter('shape_definitions_path', '')

		self._joint_state_lock = threading.Lock()
		self._latest_joint_state: Optional[JointState] = None
		self._shutdown_requested = threading.Event()

		self._base_frame = str(self.get_parameter('base_frame').value)
		self._tool_frame = str(self.get_parameter('tool_frame').value)
		self._ground_plane_z = float(self.get_parameter('ground_plane_z').value)
		self._workspace_config_path = resolve_config_path(
			'grasping_arm_control',
			str(self.get_parameter('workspace_config_path').value),
			'workspace.yaml',
		)
		self._shape_definitions_path = resolve_config_path(
			'grasping_arm_control',
			str(self.get_parameter('shape_definitions_path').value),
			'shape_definitions.yaml',
		)

		self._tf_buffer = Buffer()
		self._tf_listener = TransformListener(self._tf_buffer, self)

		joint_state_topic = str(self.get_parameter('joint_state_topic').value)
		self._joint_subscriber = self.create_subscription(
			JointState,
			joint_state_topic,
			self._joint_state_callback,
			10,
		)

		self.create_timer(0.2, self._shutdown_if_requested)
		self._cli_thread = threading.Thread(target=self._run_cli, daemon=True)
		self._cli_thread.start()

		self.get_logger().info(
			f'Listening for joint states on {joint_state_topic} and TF {self._base_frame} -> {self._tool_frame}'
		)
		self.get_logger().info(f'Workspace config: {self._workspace_config_path}')
		self.get_logger().info(f'Shape definitions: {self._shape_definitions_path}')

	def _joint_state_callback(self, msg: JointState) -> None:
		with self._joint_state_lock:
			self._latest_joint_state = deepcopy(msg)

	def _shutdown_if_requested(self) -> None:
		if self._shutdown_requested.is_set():
			rclpy.shutdown()

	def _run_cli(self) -> None:
		try:
			# Shape requirements are defined separately from object instances so adding a new
			# primitive later only requires extending the shape definition YAML.
			shape_definitions = load_yaml_dict(self._shape_definitions_path, self._default_shape_definitions())
			workspace_config = load_yaml_dict(self._workspace_config_path, self._default_workspace_config())
			workspace_config.setdefault('objects', [])
			self._interactive_loop(workspace_config, shape_definitions)
		except Exception as exc:
			self.get_logger().error(f'Calibration session failed: {exc}')
		finally:
			self._shutdown_requested.set()

	def _interactive_loop(self, workspace_config: Dict[str, Any], shape_definitions: Dict[str, Any]) -> None:
		shapes = shape_definitions.get('shapes', {})
		if not shapes:
			raise RuntimeError('No shapes defined in shape_definitions.yaml')

		print('')
		print('Workspace calibration session started.')
		print(f'Base frame: {self._base_frame}')
		print(f'Tool frame: {self._tool_frame}')
		print(f'Ground plane z: {self._ground_plane_z:.4f}')

		while rclpy.ok():
			print('')
			objects = workspace_config.get('objects', [])
			if objects:
				print('Existing objects:')
				for index, obj in enumerate(objects, start=1):
					print(f'  {index}. {obj.get("name", "unnamed")} [{obj.get("shape", "unknown")}]')
			else:
				print('No objects recorded yet.')

			add_index = len(objects) + 1
			print(f'  {add_index}. Add new object')
			print('  q. Quit')

			selection = input('Select an object to update or choose add new: ').strip().lower()
			if selection in {'q', 'quit', 'exit'}:
				self._write_workspace_config(workspace_config)
				print('Calibration session ended.')
				return

			if not selection.isdigit():
				print('Enter a number or q to quit.')
				continue

			selected_index = int(selection)
			if selected_index == add_index:
				object_entry = self._create_new_object(shapes)
				if object_entry is None:
					continue
				objects.append(object_entry)
				workspace_config['updated_at'] = _iso_timestamp()
				self._write_workspace_config(workspace_config)
				print(f'Saved object {object_entry["name"]}.')
				continue

			if 1 <= selected_index <= len(objects):
				updated_entry = self._update_existing_object(objects[selected_index - 1], shapes)
				if updated_entry is None:
					continue
				objects[selected_index - 1] = updated_entry
				workspace_config['updated_at'] = _iso_timestamp()
				self._write_workspace_config(workspace_config)
				print(f'Updated object {updated_entry["name"]}.')
				continue

			print('Selection out of range.')

	def _create_new_object(self, shapes: Dict[str, Any]) -> Optional[Dict[str, Any]]:
		print('')
		name = input('Object name: ').strip()
		if not name:
			print('Object name is required.')
			return None

		shape_key = self._prompt_for_shape(shapes)
		if shape_key is None:
			return None

		object_entry = {
			'name': name,
			'shape': shape_key,
			'created_at': _iso_timestamp(),
		}
		return self._capture_object(object_entry, shapes[shape_key])

	def _update_existing_object(
		self,
		existing_object: Dict[str, Any],
		shapes: Dict[str, Any],
	) -> Optional[Dict[str, Any]]:
		print('')
		print(f'Updating {existing_object.get("name", "unnamed")}.')
		print(f'Current shape: {existing_object.get("shape", "unknown")}')

		shape_key = str(existing_object.get('shape', ''))
		if shape_key not in shapes:
			print('Existing shape is no longer defined. Select a new shape.')
			shape_key = self._prompt_for_shape(shapes)
			if shape_key is None:
				return None

		updated_object = deepcopy(existing_object)
		rename = input('Rename object? Leave empty to keep current name: ').strip()
		if rename:
			updated_object['name'] = rename
		updated_object['shape'] = shape_key
		return self._capture_object(updated_object, shapes[shape_key])

	def _prompt_for_shape(self, shapes: Dict[str, Any]) -> Optional[str]:
		print('')
		print('Available shapes:')
		shape_keys = list(shapes.keys())
		for index, shape_key in enumerate(shape_keys, start=1):
			definition = shapes[shape_key]
			display_name = definition.get('display_name', shape_key)
			description = definition.get('description', '')
			print(f'  {index}. {display_name} ({shape_key})')
			if description:
				print(f'     {description}')
		print('  q. Cancel')

		selection = input('Select shape: ').strip().lower()
		if selection in {'q', 'quit', 'exit'}:
			return None
		if not selection.isdigit():
			print('Enter a number or q to cancel.')
			return None

		selected_index = int(selection)
		if 1 <= selected_index <= len(shape_keys):
			return shape_keys[selected_index - 1]

		print('Selection out of range.')
		return None

	def _capture_object(
		self,
		object_entry: Dict[str, Any],
		shape_definition: Dict[str, Any],
	) -> Optional[Dict[str, Any]]:
		point_labels = list(shape_definition.get('point_labels', []))
		if not point_labels:
			print('Shape definition has no point labels.')
			return None

		print('')
		print(f'Capturing {object_entry["name"]} as {object_entry["shape"]}.')
		print('Move the robot in freedrive/manual mode to each requested point.')
		print('Press Enter to capture the current pose, or type cancel to stop this object.')

		# Each saved point stores both Cartesian pose and the current joint state so the raw
		# calibration record can be reused later if the geometry derivation needs refinement.
		samples: List[Dict[str, Any]] = []
		for point_label in point_labels:
			while rclpy.ok():
				response = input(f'Capture {point_label}: ').strip().lower()
				if response in {'cancel', 'c', 'q'}:
					print('Object capture cancelled.')
					return None

				sample = self._capture_current_sample(point_label)
				if sample is None:
					retry = input('Capture failed. Press Enter to retry or type cancel: ').strip().lower()
					if retry in {'cancel', 'c', 'q'}:
						return None
					continue

				position = sample['pose']['position']
				print(
					f"Recorded {point_label}: x={position['x']:.4f}, y={position['y']:.4f}, z={position['z']:.4f}"
				)
				samples.append(sample)
				break

		geometry = self._build_geometry(object_entry['shape'], samples, shape_definition)
		object_entry['updated_at'] = _iso_timestamp()
		object_entry['base_frame'] = self._base_frame
		object_entry['tool_frame'] = self._tool_frame
		object_entry['ground_plane_z'] = self._ground_plane_z
		object_entry['capture_samples'] = samples
		object_entry['geometry'] = geometry
		return object_entry

	def _capture_current_sample(self, point_label: str) -> Optional[Dict[str, Any]]:
		joint_state = self._get_latest_joint_state()
		if joint_state is None:
			print('No joint state received yet. Wait for /joint_states and try again.')
			return None

		try:
			transform = self._tf_buffer.lookup_transform(
				self._base_frame,
				self._tool_frame,
				rclpy.time.Time(),
			)
		except TransformException as exc:
			print(f'Unable to lookup transform {self._base_frame} -> {self._tool_frame}: {exc}')
			return None

		translation = transform.transform.translation
		rotation = transform.transform.rotation
		return {
			'label': point_label,
			'captured_at': _iso_timestamp(),
			'pose': {
				'position': {
					'x': float(translation.x),
					'y': float(translation.y),
					'z': float(translation.z),
				},
				'orientation': {
					'x': float(rotation.x),
					'y': float(rotation.y),
					'z': float(rotation.z),
					'w': float(rotation.w),
				},
			},
			'joint_state': joint_state,
		}

	def _get_latest_joint_state(self) -> Optional[Dict[str, Any]]:
		with self._joint_state_lock:
			if self._latest_joint_state is None:
				return None

			return {
				'name': list(self._latest_joint_state.name),
				'position': [float(value) for value in self._latest_joint_state.position],
				'velocity': [float(value) for value in self._latest_joint_state.velocity],
				'effort': [float(value) for value in self._latest_joint_state.effort],
			}

	def _build_geometry(
		self,
		shape_key: str,
		samples: List[Dict[str, Any]],
		shape_definition: Dict[str, Any],
	) -> Dict[str, Any]:
		# Geometry is derived into basic MoveIt-friendly primitives now so the runtime planning
		# scene loader does not need to reinterpret manual calibration points every startup.
		points = [sample['pose']['position'] for sample in samples]
		geometry_type = shape_definition.get('geometry_type', 'generic')

		if shape_key == 'rectangle' and len(points) == 4:
			return self._build_rectangle_geometry(points, geometry_type)
		if shape_key == 'cylinder' and len(points) >= 2:
			return self._build_cylinder_geometry(points, geometry_type)

		average_z = sum(point['z'] for point in points) / len(points)
		return {
			'type': geometry_type,
			'height': max(0.0, average_z - self._ground_plane_z),
			'top_face_points': points,
		}

	def _build_rectangle_geometry(self, points: List[Dict[str, float]], geometry_type: str) -> Dict[str, Any]:
		edge_a = self._distance(points[0], points[1])
		edge_b = self._distance(points[1], points[2])
		edge_c = self._distance(points[2], points[3])
		edge_d = self._distance(points[3], points[0])
		size_x = (edge_a + edge_c) / 2.0
		size_y = (edge_b + edge_d) / 2.0
		top_z = sum(point['z'] for point in points) / len(points)
		center_x = sum(point['x'] for point in points) / len(points)
		center_y = sum(point['y'] for point in points) / len(points)
		height = max(0.0, top_z - self._ground_plane_z)

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
					'z': self._ground_plane_z + (height / 2.0),
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

	def _build_cylinder_geometry(self, points: List[Dict[str, float]], geometry_type: str) -> Dict[str, Any]:
		center_point = points[0]
		rim_points = points[1:]
		top_z = sum(point['z'] for point in points) / len(points)
		height = max(0.0, top_z - self._ground_plane_z)
		radius = sum(self._distance_xy(center_point, point) for point in rim_points) / len(rim_points)

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
					'z': self._ground_plane_z + (height / 2.0),
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

	def _distance(self, start: Dict[str, float], end: Dict[str, float]) -> float:
		return math.sqrt(
			((end['x'] - start['x']) ** 2)
			+ ((end['y'] - start['y']) ** 2)
			+ ((end['z'] - start['z']) ** 2)
		)

	def _distance_xy(self, start: Dict[str, float], end: Dict[str, float]) -> float:
		return math.sqrt(((end['x'] - start['x']) ** 2) + ((end['y'] - start['y']) ** 2))

	def _write_workspace_config(self, workspace_config: Dict[str, Any]) -> None:
		workspace_config.setdefault('version', 1)
		workspace_config['updated_at'] = _iso_timestamp()
		workspace_config['base_frame'] = self._base_frame
		workspace_config['tool_frame'] = self._tool_frame
		workspace_config['ground_plane_z'] = self._ground_plane_z

		write_yaml_dict(self._workspace_config_path, workspace_config)

	def _default_workspace_config(self) -> Dict[str, Any]:
		return {
			'version': 1,
			'updated_at': _iso_timestamp(),
			'base_frame': self._base_frame,
			'tool_frame': self._tool_frame,
			'ground_plane_z': self._ground_plane_z,
			'objects': [],
		}

	def _default_shape_definitions(self) -> Dict[str, Any]:
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


def main(args: Optional[List[str]] = None) -> None:
	rclpy.init(args=args)
	node = WorkspaceCalibrationNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		if rclpy.ok():
			rclpy.shutdown()
		node.destroy_node()
