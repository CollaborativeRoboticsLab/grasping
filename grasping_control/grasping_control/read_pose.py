from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


class ReadPoseNode(Node):
	"""
	@brief Print the current TF pose of a link relative to a reference frame.
	"""

	def __init__(self) -> None:
		"""
		@brief Declare parameters and create the TF listener.
		"""
		super().__init__('read_pose')
		self.declare_parameter('mode', 'tf')
		self.declare_parameter('from', 'world')
		self.declare_parameter('to', 'tcp')
		self.declare_parameter('link_name', '')
		self.declare_parameter('reference_frame', '')
		self.declare_parameter('joint_state_topic', '/joint_states')
		self.declare_parameter('joint_names', [''])
		self.declare_parameter('timeout_sec', 2.0)

		self._tf_buffer = Buffer()
		self._tf_listener = TransformListener(self._tf_buffer, self)
		self._latest_joint_state: Optional[JointState] = None
		self._joint_positions_by_name: dict[str, float] = {}
		self.create_subscription(
			JointState,
			str(self.get_parameter('joint_state_topic').value),
			self._joint_state_callback,
			10,
		)

	def _joint_state_callback(self, msg: JointState) -> None:
		"""
		@brief Cache the most recent joint-state message.

		@param msg Latest joint-state message.
		"""
		self._latest_joint_state = msg
		for name, position in zip(msg.name, msg.position):
			self._joint_positions_by_name[str(name)] = float(position)

	def run(self) -> bool:
		"""
		@brief Wait for and print the requested TF pose or joint positions.

		@return True when the requested data was printed successfully.
		"""
		mode = str(self.get_parameter('mode').value).strip().lower()
		if mode == 'joint':
			return self._run_joint()
		if mode != 'tf':
			self.get_logger().error("Parameter mode must be either 'tf' or 'joint'.")
			return False
		return self._run_tf()

	def _run_tf(self) -> bool:
		"""
		@brief Wait for and print the requested TF pose.

		@return True when the pose was printed successfully.
		"""
		from_frame = str(self.get_parameter('from').value).strip()
		to_frame = str(self.get_parameter('to').value).strip()
		legacy_link_name = str(self.get_parameter('link_name').value).strip()
		legacy_reference_frame = str(self.get_parameter('reference_frame').value).strip()
		if legacy_link_name:
			to_frame = legacy_link_name
		if legacy_reference_frame:
			from_frame = legacy_reference_frame
		timeout_sec = float(self.get_parameter('timeout_sec').value)

		if not to_frame:
			self.get_logger().error('Parameter to must not be empty.')
			return False
		if not from_frame:
			self.get_logger().error('Parameter from must not be empty.')
			return False

		transform = None
		last_error = ''
		deadline = time.monotonic() + timeout_sec
		while rclpy.ok() and time.monotonic() < deadline:
			rclpy.spin_once(self, timeout_sec=0.1)
			try:
				transform = self._tf_buffer.lookup_transform(
					from_frame,
					to_frame,
					rclpy.time.Time(),
				)
				break
			except TransformException as exc:
				last_error = str(exc)

		if transform is None:
			self.get_logger().error(
				f'Unable to read pose from {from_frame} to {to_frame}: {last_error}'
			)
			return False

		translation = transform.transform.translation
		rotation = transform.transform.rotation
		roll, pitch, yaw = quaternion_to_rpy(rotation.x, rotation.y, rotation.z, rotation.w)

		print(f'Pose from {from_frame} to {to_frame}:')
		print(
			'  position: '
			f'x={translation.x:.6f}, y={translation.y:.6f}, z={translation.z:.6f}'
		)
		print(
			'  orientation_xyzw: '
			f'x={rotation.x:.6f}, y={rotation.y:.6f}, z={rotation.z:.6f}, w={rotation.w:.6f}'
		)
		print(f'  orientation_rpy_rad: roll={roll:.6f}, pitch={pitch:.6f}, yaw={yaw:.6f}')
		return True

	def _run_joint(self) -> bool:
		"""
		@brief Wait for and print the current joint positions.

		@return True when the joint positions were printed successfully.
		"""
		timeout_sec = float(self.get_parameter('timeout_sec').value)
		joint_names = self._coerce_string_sequence(self.get_parameter('joint_names').value)

		deadline = time.monotonic() + timeout_sec
		while rclpy.ok() and time.monotonic() < deadline:
			rclpy.spin_once(self, timeout_sec=0.1)

		if not self._joint_positions_by_name:
			self.get_logger().error('Unable to read joint positions: no /joint_states message received.')
			return False

		joint_positions = dict(self._joint_positions_by_name)

		if joint_names:
			missing_joint_names = [name for name in joint_names if name not in joint_positions]
			if missing_joint_names:
				available_joint_names = ', '.join(sorted(joint_positions)) or '<none>'
				self.get_logger().error(
					'Unable to read joint positions for: '
					+ ', '.join(missing_joint_names)
					+ '. Available joints: '
					+ available_joint_names
				)
				return False
			selected_joint_names = joint_names
		else:
			selected_joint_names = sorted(joint_positions)

		print('Joint positions:')
		for joint_name in selected_joint_names:
			position_rad = joint_positions[joint_name]
			position_deg = math.degrees(position_rad)
			print(f'  {joint_name}: {position_rad:.6f} rad ({position_deg:.3f} deg)')
		return True

	@staticmethod
	def _coerce_string_sequence(value: object) -> list[str]:
		"""
		@brief Convert a parameter value into a list of non-empty strings.

		@param value String or sequence-like parameter value.
		@return Clean string values.
		"""
		if isinstance(value, str):
			items = [item.strip() for item in value.strip('[]()').split(',')]
		else:
			items = list(value)
		return [str(item).strip().strip('"\'') for item in items if str(item).strip()]


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
	"""
	@brief Convert a quaternion to roll, pitch, yaw angles.
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


def main(args: Optional[list[str]] = None) -> None:
	"""
	@brief Run the one-shot pose reader.
	"""
	rclpy.init(args=args)
	node = ReadPoseNode()
	try:
		ok = node.run()
	finally:
		node.destroy_node()
		rclpy.shutdown()
	if not ok:
		raise SystemExit(1)
