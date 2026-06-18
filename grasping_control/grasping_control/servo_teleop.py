import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_srvs.srv import Trigger


HELP_TEXT = """
Keyboard Servo Teleop

Linear motion:
  w/s : +x / -x
  a/d : +y / -y
  q/e : +z / -z

Angular motion:
  u/o : +roll / -roll
  i/k : +pitch / -pitch
  j/l : +yaw / -yaw

Other:
  space : stop
  v     : start servo
  b     : stop servo
  h     : show this help
  x     : quit
""".strip()


class ServoTeleop(Node):
	def __init__(self) -> None:
		super().__init__("servo_teleop")
		self.declare_parameter("topic", "/servo_node/delta_twist_cmds")
		self.declare_parameter("start_service", "/servo_node/start_servo")
		self.declare_parameter("stop_service", "/servo_node/stop_servo")
		self.declare_parameter("frame_id", "tool0")
		self.declare_parameter("enable_smoothing", True)
		self.declare_parameter("smoothing_alpha", 0.25)
		self.declare_parameter("linear_speed", 0.50)
		self.declare_parameter("angular_speed", 0.75)
		self.declare_parameter("publish_rate_hz", 30.0)
		self.declare_parameter("command_timeout", 0.18)

		topic = self.get_parameter("topic").get_parameter_value().string_value
		start_service = self.get_parameter("start_service").get_parameter_value().string_value
		stop_service = self.get_parameter("stop_service").get_parameter_value().string_value
		self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
		self._enable_smoothing = (
			self.get_parameter("enable_smoothing").get_parameter_value().bool_value
		)
		self._smoothing_alpha = (
			self.get_parameter("smoothing_alpha").get_parameter_value().double_value
		)
		self._linear_speed = (
			self.get_parameter("linear_speed").get_parameter_value().double_value
		)
		self._angular_speed = (
			self.get_parameter("angular_speed").get_parameter_value().double_value
		)
		self._command_timeout = (
			self.get_parameter("command_timeout").get_parameter_value().double_value
		)
		publish_rate_hz = (
			self.get_parameter("publish_rate_hz").get_parameter_value().double_value
		)

		self._publisher = self.create_publisher(TwistStamped, topic, 10)
		self._command_lock = threading.Lock()
		self._start_client = self.create_client(Trigger, start_service)
		self._stop_client = self.create_client(Trigger, stop_service)
		self._target_linear = [0.0, 0.0, 0.0]
		self._target_angular = [0.0, 0.0, 0.0]
		self._current_linear = [0.0, 0.0, 0.0]
		self._current_angular = [0.0, 0.0, 0.0]
		self._command_deadline = 0.0
		self._sent_stop = True
		self._quit_requested = False

		period = 1.0 / publish_rate_hz if publish_rate_hz > 0.0 else 1.0 / 30.0
		self.create_timer(period, self._publish_command)
		self.get_logger().info(f"Publishing Servo commands to {topic} in frame {self._frame_id}")
		self.get_logger().info("Hold a key to move. Motion stops shortly after key release.")
		self.get_logger().info(
			f"Command smoothing: {'enabled' if self._enable_smoothing else 'disabled'}"
		)
		self.get_logger().info(f"Servo services: start={start_service}, stop={stop_service}")
		self._call_trigger(self._start_client, "start_servo")

	def request_quit(self) -> None:
		self._quit_requested = True

	@property
	def quit_requested(self) -> bool:
		return self._quit_requested

	def handle_key(self, key: str) -> None:
		linear = [0.0, 0.0, 0.0]
		angular = [0.0, 0.0, 0.0]

		if key == "w":
			linear[0] = self._linear_speed
		elif key == "s":
			linear[0] = -self._linear_speed
		elif key == "a":
			linear[1] = self._linear_speed
		elif key == "d":
			linear[1] = -self._linear_speed
		elif key == "q":
			linear[2] = self._linear_speed
		elif key == "e":
			linear[2] = -self._linear_speed
		elif key == "u":
			angular[0] = self._angular_speed
		elif key == "o":
			angular[0] = -self._angular_speed
		elif key == "i":
			angular[1] = self._angular_speed
		elif key == "k":
			angular[1] = -self._angular_speed
		elif key == "j":
			angular[2] = self._angular_speed
		elif key == "l":
			angular[2] = -self._angular_speed
		elif key == " ":
			self._stop_motion()
			print("\rcommand: stop                ")
			return
		elif key == "v":
			self._call_trigger(self._start_client, "start_servo")
			return
		elif key == "b":
			self._stop_motion()
			self._call_trigger(self._stop_client, "stop_servo")
			return
		elif key == "h":
			print(f"\n{HELP_TEXT}\n")
			return
		elif key == "x":
			self._stop_motion()
			self._call_trigger(self._stop_client, "stop_servo")
			self.request_quit()
			return
		else:
			return

		with self._command_lock:
			self._target_linear = linear
			self._target_angular = angular
			self._command_deadline = time.monotonic() + self._command_timeout
			self._sent_stop = False

		print(f"\rcommand: lin={linear} ang={angular}    ")

	def _stop_motion(self) -> None:
		with self._command_lock:
			self._target_linear = [0.0, 0.0, 0.0]
			self._target_angular = [0.0, 0.0, 0.0]
			self._command_deadline = 0.0
			self._sent_stop = False

	def _publish_command(self) -> None:
		now = time.monotonic()
		with self._command_lock:
			if now <= self._command_deadline:
				target_linear = list(self._target_linear)
				target_angular = list(self._target_angular)
			elif self._sent_stop:
				return
			else:
				target_linear = [0.0, 0.0, 0.0]
				target_angular = [0.0, 0.0, 0.0]
				self._sent_stop = True

			if self._enable_smoothing:
				self._current_linear = self._blend_command(self._current_linear, target_linear)
				self._current_angular = self._blend_command(self._current_angular, target_angular)
			else:
				self._current_linear = target_linear
				self._current_angular = target_angular

			linear = list(self._current_linear)
			angular = list(self._current_angular)

		msg = TwistStamped()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = self._frame_id
		msg.twist.linear.x = linear[0]
		msg.twist.linear.y = linear[1]
		msg.twist.linear.z = linear[2]
		msg.twist.angular.x = angular[0]
		msg.twist.angular.y = angular[1]
		msg.twist.angular.z = angular[2]
		self._publisher.publish(msg)

	def _blend_command(self, current: list[float], target: list[float]) -> list[float]:
		alpha = min(max(self._smoothing_alpha, 0.0), 1.0)
		blended = []
		for current_value, target_value in zip(current, target):
			next_value = current_value + alpha * (target_value - current_value)
			if abs(next_value) < 1e-4 and abs(target_value) < 1e-4:
				next_value = 0.0
			blended.append(next_value)
		return blended

	def _call_trigger(self, client: Trigger, label: str) -> None:
		if not client.wait_for_service(timeout_sec=1.0):
			self.get_logger().warning(f"{label} service not available")
			return

		future = client.call_async(Trigger.Request())
		rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
		if not future.done() or future.result() is None:
			self.get_logger().warning(f"{label} call did not complete")
			return

		response = future.result()
		level = self.get_logger().info if response.success else self.get_logger().warning
		message = response.message if response.message else "ok"
		level(f"{label}: {message}")


def _read_key(timeout: float = 0.1) -> str | None:
	ready, _, _ = select.select([sys.stdin], [], [], timeout)
	if not ready:
		return None
	return sys.stdin.read(1)


def main() -> None:
	rclpy.init()
	node = ServoTeleop()
	old_settings = termios.tcgetattr(sys.stdin)

	print(HELP_TEXT)

	try:
		tty.setraw(sys.stdin.fileno())
		while rclpy.ok() and not node.quit_requested:
			rclpy.spin_once(node, timeout_sec=0.05)
			key = _read_key(timeout=0.05)
			if key is not None:
				node.handle_key(key)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
	main()
