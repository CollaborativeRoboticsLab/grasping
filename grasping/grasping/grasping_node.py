from __future__ import annotations

from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_srvs.srv import Trigger

from geometry_msgs.msg import PoseStamped

from anygrasp_msgs.srv import GetGrasps
from grasping_msgs.action import MoveToPose
from gripper_msgs.action import CloseGripper, OpenGripper


class GraspingNode(Node):
    """Orchestrates the grasping pipeline.

    1) Request a grasp pose from AnyGrasp (`anygrasp_msgs/srv/GetGrasps`)
    2) Send the target pose to the arm-control action server
    3) Close the gripper
    4) Optionally send a post-grasp pose to the arm-control action server

    Trigger the pipeline via the `~run_grasp` Trigger service.
    """

    def __init__(self) -> None:
        super().__init__("ur_grasping_node")

        self.declare_parameter("server_mode", True)
        self.declare_parameter("anygrasp_service", "detection")
        self.declare_parameter("arm_action_name", "move_arm_to_pose")
        self.declare_parameter("do_post_grasp_move", True)
        self.declare_parameter("post_grasp_frame", "base_link")
        self.declare_parameter("post_grasp_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("open_action_name", "/open_gripper")
        self.declare_parameter("close_action_name", "/close_gripper")

        # The grasping node only owns pipeline orchestration. Motion execution is delegated
        # to the arm-control action server so grasp generation and robot control stay decoupled.
        self._anygrasp_client = self.create_client(
            GetGrasps, str(self.get_parameter("anygrasp_service").value)
        )
        self._arm_control_client = ActionClient(
            self, MoveToPose, str(self.get_parameter("arm_action_name").value)
        )
        self._open_client = ActionClient(
            self, OpenGripper, str(self.get_parameter("open_action_name").value)
        )
        self._close_client = ActionClient(
            self, CloseGripper, str(self.get_parameter("close_action_name").value)
        )

        self._srv = None
        if bool(self.get_parameter("server_mode").value):
            self._srv = self.create_service(Trigger, "run_grasp", self._on_run_grasp)

        if bool(self.get_parameter("server_mode").value):
            self.get_logger().info(
                "ur_grasping node ready (server_mode=true). Call ~/run_grasp to execute pipeline."
            )
        else:
            self.get_logger().info(
                "ur_grasping node ready (server_mode=false). Will execute pipeline once and exit."
            )

    def _on_run_grasp(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        try:
            ok, msg = self.run_pipeline()
            res.success = bool(ok)
            res.message = msg
        except Exception as exc:  # noqa: BLE001
            res.success = False
            res.message = f"Pipeline failed: {exc}"
        return res

    def run_pipeline(self) -> Tuple[bool, str]:
        # The pose from AnyGrasp is forwarded directly to the arm-control action server.
        # That server handles frame transforms, MoveIt planning, and planning-scene obstacles.
        grasp_pose = self._request_anygrasp_pose()
        if grasp_pose is None:
            return False, "AnyGrasp returned no pose."

        if not self._move_to_pose(grasp_pose):
            return False, "Arm-control move to grasp pose failed."

        if not self.close_gripper_position():
            return False, "Failed to close gripper."

        if bool(self.get_parameter("do_post_grasp_move").value):
            post_pose = self._get_post_grasp_pose_stamped()
            if not self._move_to_pose(post_pose):
                return False, "Arm-control move to post-grasp pose failed."

        return True, "Grasping pipeline completed."

    def _request_anygrasp_pose(self) -> Optional[PoseStamped]:
        service_name = str(self.get_parameter("anygrasp_service").value)
        if not self._anygrasp_client.service_is_ready():
            self.get_logger().info(f"Waiting for AnyGrasp service '{service_name}'...")
            if not self._anygrasp_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"AnyGrasp service '{service_name}' not available.")
                return None

        req = GetGrasps.Request()
        req.count = 1
        future = self._anygrasp_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done() or future.result() is None:
            self.get_logger().error("AnyGrasp service call did not complete.")
            return None

        resp: GetGrasps.Response = future.result()
        if not resp.success or len(resp.poses) == 0:
            self.get_logger().warn(f"AnyGrasp failed: {resp.message}")
            return None

        return resp.poses[0]

    def _move_to_pose(self, target_pose: PoseStamped) -> bool:
        action_name = str(self.get_parameter("arm_action_name").value)
        if not self._arm_control_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Arm control action server '{action_name}' not available.")
            return False

        # The action goal is intentionally minimal: one PoseStamped target and the server
        # resolves everything else from its own MoveIt and workspace configuration.
        goal = MoveToPose.Goal()
        goal.target_pose = target_pose

        send_future = self._arm_control_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done() or send_future.result() is None:
            self.get_logger().error("Failed to send arm-control goal.")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Arm-control goal was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error("Arm-control result not received.")
            return False

        result = result_future.result().result
        if not result.success:
            self.get_logger().error(result.message)
            return False

        return True

    def open_gripper(self, use_torque_mode: bool = False, torque: float = 0.0) -> bool:
        if not self._open_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("OpenGripper action server not available.")
            return False

        goal = OpenGripper.Goal()
        goal.use_torque_mode = bool(use_torque_mode)
        goal.torque = float(torque)

        send_future = self._open_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.done() or send_future.result() is None:
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)
        if not result_future.done() or result_future.result() is None:
            return False

        return bool(result_future.result().result.success)

    def close_gripper_position(self) -> bool:
        return self._close_gripper(use_torque_mode=False, torque=0.0)

    def close_gripper_torque(self, torque: float) -> bool:
        return self._close_gripper(use_torque_mode=True, torque=float(torque))

    def _close_gripper(self, use_torque_mode: bool, torque: float) -> bool:
        if not self._close_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("CloseGripper action server not available.")
            return False

        goal = CloseGripper.Goal()
        goal.close = True
        goal.use_torque_mode = bool(use_torque_mode)
        goal.torque = float(torque)

        send_future = self._close_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.done() or send_future.result() is None:
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)
        if not result_future.done() or result_future.result() is None:
            return False

        return bool(result_future.result().result.success)

    def _get_post_grasp_pose_stamped(self) -> PoseStamped:
        vals = list(self.get_parameter("post_grasp_pose").value)
        if len(vals) != 7:
            raise RuntimeError("post_grasp_pose must be [x,y,z,qx,qy,qz,qw]")

        # The post-grasp pose stays configurable from launch, but is sent through the same
        # arm-control action path as the main grasp pose.
        frame = str(self.get_parameter("post_grasp_frame").value)
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = frame
        pose_stamped.pose.position.x = float(vals[0])
        pose_stamped.pose.position.y = float(vals[1])
        pose_stamped.pose.position.z = float(vals[2])
        pose_stamped.pose.orientation.x = float(vals[3])
        pose_stamped.pose.orientation.y = float(vals[4])
        pose_stamped.pose.orientation.z = float(vals[5])
        pose_stamped.pose.orientation.w = float(vals[6])
        return pose_stamped


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = GraspingNode()
    try:
        if bool(node.get_parameter("server_mode").value):
            rclpy.spin(node)
        else:
            req = Trigger.Request()
            res = Trigger.Response()
            res = node._on_run_grasp(req, res)
            if res.success:
                node.get_logger().info(res.message)
            else:
                node.get_logger().error(res.message)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
