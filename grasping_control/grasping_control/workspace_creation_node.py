from copy import deepcopy
from pathlib import Path
import threading
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from grasping_control.common import (
	find_colcon_workspace_root,
	load_yaml_dict,
	resolve_config_path,
)
from grasping_control.workspace_utils import (
	build_geometry,
	build_workspace_area,
	default_shape_definitions,
	default_workspace_config,
	iso_timestamp,
	write_workspace_config,
)


class WorkspaceCreationNode(Node):
	"""
	@brief Interactive ROS node used to calibrate workspace collision objects.
	"""

	def __init__(self) -> None:
		"""
		@brief Initialize subscriptions, TF access, and the CLI worker thread.
		"""
		super().__init__('workspace_creation_node')

		self.declare_parameter('joint_state_topic', '/joint_states')
		self.declare_parameter('base_frame', 'world')
		self.declare_parameter('tool_frame', 'tool_tip')
		self.declare_parameter('ground_plane_z', 0.0)
		self.declare_parameter('workspace_config_path', '')
		self.declare_parameter('workspace_write_path', '')
		self.declare_parameter('get_package_share_directory', '')
		self.declare_parameter('shape_definitions_path', '')

		self._joint_state_lock = threading.Lock()
		self._latest_joint_state: Optional[JointState] = None
		self._shutdown_requested = threading.Event()

		self._base_frame = str(self.get_parameter('base_frame').value)
		self._tool_frame = str(self.get_parameter('tool_frame').value)
		self._ground_plane_z = float(self.get_parameter('ground_plane_z').value)
		self._workspace_config_path = resolve_config_path(
			'grasping_control',
			str(self.get_parameter('workspace_config_path').value),
			'workspace_empty.yaml',
		)
		configured_write_path = str(self.get_parameter('workspace_write_path').value)
		self._workspace_write_path = Path(configured_write_path).expanduser().resolve() if configured_write_path else None
		self._workspace_root = find_colcon_workspace_root(Path(__file__))
		self._shape_definitions_path = resolve_config_path(
			'grasping_control',
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
		self.get_logger().info(f'Workspace template: {self._workspace_config_path}')
		if self._workspace_write_path is not None:
			self.get_logger().info(f'Workspace save path override: {self._workspace_write_path}')
		elif self._workspace_root is not None:
			self.get_logger().info(f'Workspace save root: {self._workspace_root}')
		else:
			self.get_logger().warn('Colcon workspace root not detected; saves will fall back to the config directory.')
		self.get_logger().info(f'Shape definitions: {self._shape_definitions_path}')

	def _joint_state_callback(self, msg: JointState) -> None:
		"""
		@brief Cache the most recent joint state message.

		@param msg Joint state message from the robot.
		"""
		with self._joint_state_lock:
			self._latest_joint_state = deepcopy(msg)

	def _shutdown_if_requested(self) -> None:
		"""
		@brief Shut ROS down once the CLI thread has requested termination.
		"""
		if self._shutdown_requested.is_set():
			rclpy.shutdown()

	def _run_cli(self) -> None:
		"""
		@brief Load workspace data and run the interactive calibration loop.
		"""
		try:
			# Shape requirements are defined separately from object instances so adding a new
			# primitive later only requires extending the shape definition YAML.
			shape_definitions = load_yaml_dict(self._shape_definitions_path, default_shape_definitions())
			workspace_config = load_yaml_dict(
				self._workspace_config_path,
				default_workspace_config(self._base_frame, self._tool_frame, self._ground_plane_z),
			)
			workspace_config.setdefault('workspace_area', None)
			workspace_config.setdefault('objects', [])
			self._interactive_loop(workspace_config, shape_definitions)
		except Exception as exc:
			self.get_logger().error(f'Workspace creation session failed: {exc}')
		finally:
			self._shutdown_requested.set()

	def _interactive_loop(self, workspace_config: Dict[str, Any], shape_definitions: Dict[str, Any]) -> None:
		"""
		@brief Drive the top-level calibration menu.

		@param workspace_config Mutable workspace configuration.
		@param shape_definitions Available shape definitions.
		"""
		shapes = shape_definitions.get('shapes', {})
		if not shapes:
			raise RuntimeError('No shapes defined in shape_definitions.yaml')
		is_dirty = False

		print('')
		print('Workspace creation session started.')
		print(f'Base frame: {self._base_frame}')
		print(f'Tool frame: {self._tool_frame}')
		print(f'Ground plane z: {self._ground_plane_z:.4f}')

		while rclpy.ok():
			print('')
			print(f'Unsaved changes: {"yes" if is_dirty else "no"}')
			workspace_area = workspace_config.get('workspace_area')
			if workspace_area:
				area_size = workspace_area.get('geometry', {}).get('dimensions', {}).get('side_length', 0.0)
				print(f'Workspace area configured: square side={float(area_size):.4f} m')
			else:
				print('Workspace area not calibrated yet.')

			objects = workspace_config.get('objects', [])
			if objects:
				print('Existing objects:')
				for index, obj in enumerate(objects, start=1):
					print(f'  {index}. {obj.get("name", "unnamed")} [{obj.get("shape", "unknown")}]')
			else:
				print('No objects recorded yet.')

			add_index = len(objects) + 1
			print(f'  {add_index}. Add new object')
			print('  s. Save workspace file')
			print('  w. Calibrate workspace area')
			print('  q. Quit')

			selection = input('Select an object to update or choose add new: ').strip().lower()
			if selection in {'q', 'quit', 'exit'}:
				if is_dirty:
					confirm = input('Unsaved changes will be lost. Quit anyway? [y/N]: ').strip().lower()
					if confirm not in {'y', 'yes'}:
						continue
				print('Calibration session ended.')
				return

			if selection in {'s', 'save'}:
				saved_config = self._save_workspace_config(workspace_config)
				if saved_config is not None:
					workspace_config = saved_config
					is_dirty = False
				continue

			if selection in {'w', 'workspace'}:
				workspace_area_entry = self._capture_workspace_area()
				if workspace_area_entry is None:
					continue
				workspace_config['workspace_area'] = workspace_area_entry
				is_dirty = True
				print('Workspace area updated in memory. Use save to persist it.')
				continue

			if not selection.isdigit():
				print('Enter a number, s, w, or q to quit.')
				continue

			selected_index = int(selection)
			if selected_index == add_index:
				object_entry = self._create_new_object(shapes)
				if object_entry is None:
					continue
				objects.append(object_entry)
				is_dirty = True
				print(f'Updated object {object_entry["name"]} in memory. Use save to persist it.')
				continue

			if 1 <= selected_index <= len(objects):
				updated_entry = self._update_existing_object(objects[selected_index - 1], shapes)
				if updated_entry is None:
					continue
				objects[selected_index - 1] = updated_entry
				is_dirty = True
				print(f'Updated object {updated_entry["name"]} in memory. Use save to persist it.')
				continue

			print('Selection out of range.')

	def _create_new_object(self, shapes: Dict[str, Any]) -> Optional[Dict[str, Any]]:
		"""
		@brief Prompt the operator for a new object definition and capture it.

		@param shapes Available shape definitions.
		@return New object entry, or None when creation is cancelled.
		"""
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
			'created_at': iso_timestamp(),
		}
		return self._capture_object(object_entry, shapes[shape_key])

	def _update_existing_object(
		self,
		existing_object: Dict[str, Any],
		shapes: Dict[str, Any],
	) -> Optional[Dict[str, Any]]:
		"""
		@brief Update an existing workspace object.

		@param existing_object Existing object entry from the workspace file.
		@param shapes Available shape definitions.
		@return Updated object entry, or None when the operation is cancelled.
		"""
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
		"""
		@brief Prompt the operator to choose one of the configured shapes.

		@param shapes Available shape definitions.
		@return Selected shape key, or None when cancelled.
		"""
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
		"""
		@brief Capture all required samples for one workspace object.

		@param object_entry Object metadata being populated.
		@param shape_definition Selected shape definition.
		@return Completed object entry, or None when capture is cancelled.
		"""
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

		shape_parameters = self._prompt_for_shape_parameters(object_entry['shape'], shape_definition)
		if shape_parameters is None:
			return None

		geometry = build_geometry(
			object_entry['shape'],
			samples,
			shape_definition,
			self._ground_plane_z,
			shape_parameters,
		)
		object_entry['updated_at'] = iso_timestamp()
		object_entry['base_frame'] = self._base_frame
		object_entry['tool_frame'] = self._tool_frame
		object_entry['ground_plane_z'] = self._ground_plane_z
		object_entry['capture_samples'] = samples
		if shape_parameters:
			object_entry['shape_parameters'] = shape_parameters
		elif 'shape_parameters' in object_entry:
			del object_entry['shape_parameters']
		object_entry['geometry'] = geometry
		return object_entry

	def _prompt_for_shape_parameters(
		self,
		shape_key: str,
		shape_definition: Dict[str, Any],
	) -> Optional[Dict[str, Any]]:
		"""
		@brief Prompt the operator for any manual parameters required by the shape.

		@param shape_key Logical shape key for the capture.
		@param shape_definition Selected shape definition.
		@return Shape parameter mapping, or None when the operator cancels.
		"""
		captured_parameters: Dict[str, Any] = {}
		if shape_key in {'top_surface_rectangle', 'bottom_face_rectangle'}:
			horizontal_plane = self._prompt_yes_no(
				'Are the points in horizontal plane parallel to ground? If yes, z values will be averaged [Y/n]: ',
				default=True,
			)
			if horizontal_plane is None:
				print('Object capture cancelled.')
				return None
			captured_parameters['parallel_to_ground'] = horizontal_plane

		manual_parameters = shape_definition.get('manual_parameters', {})
		if not isinstance(manual_parameters, dict) or not manual_parameters:
			return captured_parameters

		for parameter_name, parameter_definition in manual_parameters.items():
			if not isinstance(parameter_definition, dict):
				parameter_definition = {}

			prompt = str(parameter_definition.get('prompt', f'Enter {parameter_name}: ')).strip()
			if not prompt.endswith(':'):
				prompt = f'{prompt}: '
			else:
				prompt = f'{prompt} '

			min_value = parameter_definition.get('min_value')
			max_value = parameter_definition.get('max_value')

			while rclpy.ok():
				response = input(prompt).strip().lower()
				if response in {'cancel', 'c', 'q'}:
					print('Object capture cancelled.')
					return None

				try:
					value = float(response)
				except ValueError:
					print('Enter a numeric value or type cancel.')
					continue

				if min_value is not None and value < float(min_value):
					print(f'Value must be at least {float(min_value):.4f}.')
					continue
				if max_value is not None and value > float(max_value):
					print(f'Value must be at most {float(max_value):.4f}.')
					continue

				captured_parameters[parameter_name] = value
				break

		return captured_parameters

	def _prompt_yes_no(self, prompt: str, default: bool = True) -> Optional[bool]:
		"""
		@brief Prompt the operator for a yes/no answer.

		@param prompt Prompt text shown to the operator.
		@param default Value returned when the operator presses Enter.
		@return Boolean answer, or None when cancelled.
		"""
		while rclpy.ok():
			response = input(prompt).strip().lower()
			if response in {'cancel', 'c', 'q'}:
				return None
			if not response:
				return default
			if response in {'y', 'yes'}:
				return True
			if response in {'n', 'no'}:
				return False
			print('Enter y, n, or type cancel.')

		return None

	def _capture_workspace_area(self) -> Optional[Dict[str, Any]]:
		"""
		@brief Capture the four corners of the robot work area.

		@return Workspace area entry, or None when capture is cancelled.
		"""
		print('')
		print('Capturing workspace area.')
		print('Move the robot to the four workspace corners in order around the square.')
		print('Press Enter to capture the current pose, or type cancel to stop.')

		samples: List[Dict[str, Any]] = []
		for point_label in ['corner_1', 'corner_2', 'corner_3', 'corner_4']:
			while rclpy.ok():
				response = input(f'Capture {point_label}: ').strip().lower()
				if response in {'cancel', 'c', 'q'}:
					print('Workspace area capture cancelled.')
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

		return {
			'type': 'workspace_area',
			'created_at': iso_timestamp(),
			'updated_at': iso_timestamp(),
			'base_frame': self._base_frame,
			'tool_frame': self._tool_frame,
			'ground_plane_z': self._ground_plane_z,
			'capture_samples': samples,
			'geometry': build_workspace_area(samples, self._ground_plane_z),
		}

	def _capture_current_sample(self, point_label: str) -> Optional[Dict[str, Any]]:
		"""
		@brief Capture the current TCP pose and joint state for a labeled point.

		@param point_label Human-readable label for the sample.
		@return Captured sample dictionary, or None when capture is unavailable.
		"""
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
			'captured_at': iso_timestamp(),
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
		"""
		@brief Return the latest cached joint state as plain Python data.

		@return Joint state mapping, or None when no message has arrived yet.
		"""
		with self._joint_state_lock:
			if self._latest_joint_state is None:
				return None

			return {
				'name': list(self._latest_joint_state.name),
				'position': [float(value) for value in self._latest_joint_state.position],
				'velocity': [float(value) for value in self._latest_joint_state.velocity],
				'effort': [float(value) for value in self._latest_joint_state.effort],
			}

	def _save_workspace_config(self, workspace_config: Dict[str, Any]) -> Optional[Dict[str, Any]]:
		"""
		@brief Persist the workspace configuration after prompting for a destination filename.

		@param workspace_config Workspace configuration to write.
		@return Normalized workspace configuration that was persisted, or None when cancelled.
		"""
		save_path = self._resolve_save_path()
		if save_path is None:
			return None

		saved_config = write_workspace_config(
			save_path,
			workspace_config,
			self._base_frame,
			self._tool_frame,
			self._ground_plane_z,
		)
		print(f'Saved workspace file to {save_path}')
		return saved_config

	def _resolve_save_path(self) -> Optional[Path]:
		"""
		@brief Resolve the destination path for an explicit save request.

		@return Output path, or None when the save is cancelled.
		"""
		if self._workspace_write_path is not None:
			return self._workspace_write_path

		save_root = self._workspace_root or self._workspace_config_path.parent
		while rclpy.ok():
			response = input(f'Save file name in {save_root} (without path): ').strip()
			if response.lower() in {'cancel', 'c', 'q'}:
				print('Save cancelled.')
				return None
			if not response:
				print('File name is required, or type cancel.')
				continue

			file_name = Path(response)
			if file_name.name != response:
				print('Enter a file name only; the file will be saved in the workspace root.')
				continue
			if file_name.suffix not in {'.yaml', '.yml'}:
				file_name = file_name.with_suffix('.yaml')

			save_path = save_root / file_name
			if save_path.exists():
				overwrite = input(f'{save_path.name} exists. Overwrite? [y/N]: ').strip().lower()
				if overwrite not in {'y', 'yes'}:
					continue
			return save_path


def main(args: Optional[List[str]] = None) -> None:
	"""
	@brief Run the workspace calibration node until shutdown.

	@param args Optional ROS command-line arguments.
	"""
	rclpy.init(args=args)
	node = WorkspaceCreationNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		if rclpy.ok():
			rclpy.shutdown()
		node.destroy_node()
