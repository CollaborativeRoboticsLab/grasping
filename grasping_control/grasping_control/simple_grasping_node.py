from __future__ import annotations

import re
import time
from dataclasses import dataclass
from typing import Optional

from control_msgs.action import GripperCommand
from grasping_msgs.action import MoveToNamedPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger


@dataclass(frozen=True)
class SequenceStep:
	kind: str
	value: str


class SimpleGraspingNode(Node):
	"""
	@brief Run a fixed grasp sequence through the named-pose and gripper action servers.
	"""

	def __init__(self) -> None:
		"""
		@brief Declare parameters and create action clients.
		"""
		super().__init__('simple_grasping_node')
		self.declare_parameter('arm_action_name', '/move_arm_to_named_pose')
		self.declare_parameter('gripper_action_name', '/gripper_command')
		self.declare_parameter(
			'sequence',
			'[[pose, pre_grasp], [pose, workspace_center], [gripper, open], '
			'[pose, grasp_pose], [gripper, close], [pose, post_grasp]]',
		)
		self.declare_parameter('open_position', 0.09)
		self.declare_parameter('open_max_effort', 0.0)
		self.declare_parameter('close_position', 0.0)
		self.declare_parameter('close_max_effort', 5.0)
		self.declare_parameter('server_timeout_sec', 10.0)
		self.declare_parameter('result_timeout_sec', 120.0)
		self.declare_parameter('startup_delay_sec', 0.0)
		self.declare_parameter('trigger_retention_recording_on_close', False)
		self.declare_parameter('trigger_retention_recording_service', '/trigger_retention_recording')
		self.declare_parameter('retention_recording_service_timeout_sec', 5.0)

		self._arm_client = ActionClient(
			self,
			MoveToNamedPose,
			str(self.get_parameter('arm_action_name').value),
		)
		self._gripper_client = ActionClient(
			self,
			GripperCommand,
			str(self.get_parameter('gripper_action_name').value),
		)
		self._retention_start_client = self.create_client(
			Trigger,
			str(self.get_parameter('trigger_retention_recording_service').value),
		)
		self._retention_recording_started = False

	def run(self) -> bool:
		"""
		@brief Execute the fixed grasp sequence once.

		@return True when every step succeeds.
		"""
		server_timeout_sec = float(self.get_parameter('server_timeout_sec').value)
		if not self._arm_client.wait_for_server(timeout_sec=server_timeout_sec):
			self.get_logger().error('Named-pose action server is not available.')
			return False
		if not self._gripper_client.wait_for_server(timeout_sec=server_timeout_sec):
			self.get_logger().error('Gripper action server is not available.')
			return False

		startup_delay_sec = max(0.0, float(self.get_parameter('startup_delay_sec').value))
		if startup_delay_sec > 0.0:
			self.get_logger().info(
				f'Waiting {startup_delay_sec:.2f}s before starting the grasp sequence.'
			)
			time.sleep(startup_delay_sec)

		sequence = self._load_sequence()
		if not sequence:
			self.get_logger().error('Sequence is empty. Configure at least one step.')
			return False

		for step in sequence:
			self._maybe_trigger_retention_recording_for_step(step)
			if step.kind == 'pose':
				if not self._move_to_named_pose(step.value):
					return False
			else:
				if step.value == 'open':
					position = float(self.get_parameter('open_position').value)
					effort = float(self.get_parameter('open_max_effort').value)
				else:
					position = float(self.get_parameter('close_position').value)
					effort = float(self.get_parameter('close_max_effort').value)
				if not self._command_gripper(step.value, position, effort):
					return False

		self.get_logger().info('Simple grasp sequence completed successfully.')
		return True

	def _load_sequence(self) -> list[SequenceStep]:
		"""
		@brief Parse the configured grasp sequence parameter into executable steps.

		@return Ordered sequence steps.
		"""
		sequence_value = self.get_parameter('sequence').value
		if isinstance(sequence_value, list):
			steps = self._sequence_steps_from_list(sequence_value)
		else:
			steps = self._sequence_steps_from_string(str(sequence_value))

		for step in steps:
			if step.kind not in {'pose', 'gripper'}:
				raise RuntimeError(
					f"Unsupported sequence step kind '{step.kind}'. Expected 'pose' or 'gripper'."
				)
			if not step.value:
				raise RuntimeError('Sequence step values must not be empty.')
		return steps

	def _sequence_steps_from_list(self, sequence_value: list[object]) -> list[SequenceStep]:
		"""
		@brief Convert a flat ROS parameter list into sequence steps.

		@param sequence_value Parameter value.
		@return Ordered sequence steps.
		"""
		steps: list[SequenceStep] = []
		for item in sequence_value:
			text = str(item).strip()
			if not text:
				continue
			parts = [part.strip() for part in re.split(r'[:,]', text, maxsplit=1)]
			if len(parts) != 2:
				raise RuntimeError(
					"List-based sequence entries must look like 'pose:pre_grasp' or 'gripper:open'."
				)
			steps.append(SequenceStep(parts[0], parts[1]))
		return steps

	def _sequence_steps_from_string(self, sequence_value: str) -> list[SequenceStep]:
		"""
		@brief Parse a string sequence such as [[pose, pre_grasp], [gripper, open]].

		@param sequence_value Raw parameter string.
		@return Ordered sequence steps.
		"""
		matches = re.findall(r'\[\s*([^\[\],]+?)\s*,\s*([^\[\],]+?)\s*\]', sequence_value)
		steps = [
			SequenceStep(kind.strip().strip('"\''), value.strip().strip('"\''))
			for kind, value in matches
		]
		if steps:
			return steps

		fallback_parts = [part.strip() for part in sequence_value.split(',') if part.strip()]
		if len(fallback_parts) % 2 != 0:
			raise RuntimeError(
				"Sequence string must be a list of [kind, value] pairs, for example "
				"[[pose, pre_grasp], [gripper, open]]."
			)

		return [
			SequenceStep(
				fallback_parts[index].strip('[] "\''),
				fallback_parts[index + 1].strip('[] "\''),
			)
			for index in range(0, len(fallback_parts), 2)
		]

	def _move_to_named_pose(self, pose_name: str) -> bool:
		"""
		@brief Send one named-pose goal and wait for the result.

		@param pose_name Configured named pose.
		@return True when the goal succeeds.
		"""
		self.get_logger().info(f"Moving to named pose '{pose_name}'.")
		goal = MoveToNamedPose.Goal()
		goal.pose_name = pose_name

		goal_handle = self._send_goal(self._arm_client, goal)
		if goal_handle is None:
			return False

		result = self._wait_for_result(goal_handle)
		if result is None:
			return False
		if not result.success:
			self.get_logger().error(
				f"Named pose '{pose_name}' failed: {result.message}"
			)
			return False

		self.get_logger().info(f"Named pose '{pose_name}' completed.")
		return True

	def _command_gripper(self, label: str, position: float, max_effort: float) -> bool:
		"""
		@brief Send one gripper command and wait for the result.

		@param label Human-readable command label.
		@param position Requested gripper position.
		@param max_effort Requested maximum effort.
		@return True when the goal succeeds.
		"""
		self.get_logger().info(
			f"Sending gripper command '{label}' with position={position:.4f}, max_effort={max_effort:.4f}."
		)
		goal = GripperCommand.Goal()
		goal.command.position = position
		goal.command.max_effort = max_effort

		goal_handle = self._send_goal(self._gripper_client, goal)
		if goal_handle is None:
			return False

		result = self._wait_for_result(goal_handle)
		if result is None:
			return False
		allow_stall = label == 'close' and abs(max_effort) > 0.0
		if not bool(result.reached_goal or (allow_stall and result.stalled)):
			self.get_logger().error(
				f"Gripper command '{label}' did not reach the goal (reached_goal={bool(result.reached_goal)}, stalled={bool(result.stalled)})."
			)
			return False
		if bool(result.stalled):
			self.get_logger().info(
				f"Gripper command '{label}' completed by reaching the requested holding effort at position={result.position:.4f}, holding_effort={result.effort:.4f}, target_effort={max_effort:.4f}."
			)
			self._maybe_start_retention_recording(label)
			return True

		self.get_logger().info(
			f"Gripper command '{label}' completed at position={result.position:.4f}."
		)
		self._maybe_start_retention_recording(label)
		return True

	def _maybe_start_retention_recording(self, label: str) -> None:
		if label != 'close':
			return
		if not self._get_bool_parameter('trigger_retention_recording_on_close'):
			return
		if self._retention_recording_started:
			return

		timeout_sec = max(0.0, float(self.get_parameter('retention_recording_service_timeout_sec').value))
		service_name = str(self.get_parameter('trigger_retention_recording_service').value)
		if not self._retention_start_client.wait_for_service(timeout_sec=timeout_sec):
			self.get_logger().warning(
				f"Retention recording start service '{service_name}' is not available."
			)
			return

		future = self._retention_start_client.call_async(Trigger.Request())
		rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
		if not future.done() or future.result() is None:
			self.get_logger().warning('Retention recording start service did not return a result.')
			return

		response = future.result()
		if not response.success:
			self.get_logger().warning(
				'Retention recording did not start: ' + str(response.message)
			)
			return

		self._retention_recording_started = True
		self.get_logger().info(str(response.message))  

	def _maybe_trigger_retention_recording_for_step(self, step: SequenceStep) -> None:
		if self._retention_recording_started:
			return
		if step.kind != 'pose' or step.value != 'post_grasp':
			return
		if not self._get_bool_parameter('trigger_retention_recording_on_close'):
			return
		self._maybe_start_retention_recording('close')

	def _get_bool_parameter(self, name: str) -> bool:
		value = self.get_parameter(name).value
		if isinstance(value, str):
			return value.strip().lower() in {'1', 'true', 'yes', 'on'}
		return bool(value)

	def _send_goal(self, client: ActionClient, goal: object) -> Optional[object]:
		"""
		@brief Send an action goal and wait for acceptance.

		@param client Action client.
		@param goal Goal message.
		@return Accepted goal handle, or None on failure.
		"""
		send_future = client.send_goal_async(goal)
		rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
		if not send_future.done() or send_future.result() is None:
			self.get_logger().error('Failed to send action goal.')
			return None

		goal_handle = send_future.result()
		if not goal_handle.accepted:
			self.get_logger().error('Action goal was rejected.')
			return None
		return goal_handle

	def _wait_for_result(self, goal_handle: object) -> Optional[object]:
		"""
		@brief Wait for an accepted action goal to finish.

		@param goal_handle Accepted action goal handle.
		@return Result payload, or None on failure.
		"""
		result_future = goal_handle.get_result_async()
		rclpy.spin_until_future_complete(
			self,
			result_future,
			timeout_sec=float(self.get_parameter('result_timeout_sec').value),
		)
		if not result_future.done() or result_future.result() is None:
			self.get_logger().error('Action result was not received.')
			return None
		return result_future.result().result


def main(args: Optional[list[str]] = None) -> None:
	"""
	@brief Run the simple grasp sequence node once.
	"""
	rclpy.init(args=args)
	node = SimpleGraspingNode()
	try:
		ok = node.run()
	finally:
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()
	if not ok:
		raise SystemExit(1)
