from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_srvs.srv import Trigger

from geometry_msgs.msg import Pose, PoseStamped

from anygrasp_msgs.srv import GetGrasps

from gripper_msgs.action import CloseGripper, OpenGripper

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
)

from shape_msgs.msg import SolidPrimitive

import tf2_ros

try:
    from tf2_geometry_msgs import do_transform_pose  # type: ignore
except Exception:  # noqa: BLE001
    do_transform_pose = None


@dataclass
class Quaternion:
    x: float
    y: float
    z: float
    w: float


def _normalize_quaternion(q: Quaternion) -> Quaternion:
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if n <= 0.0:
        return Quaternion(0.0, 0.0, 0.0, 1.0)
    return Quaternion(q.x / n, q.y / n, q.z / n, q.w / n)


class GraspingNode(Node):
    """Orchestrates the grasping pipeline:

    1) Request a grasp pose from AnyGrasp (`anygrasp_msgs/srv/GetGrasps`)
    2) Move the robot to the grasp pose using MoveIt (MoveGroup action)
    3) Close the gripper (position-based by default; torque-based helper provided)
    4) Move the robot to a configured post-grasp pose

    Trigger the pipeline via the `~run_grasp` Trigger service.
    """

    def __init__(self) -> None:
        super().__init__("ur_grasping_node")

        self.declare_parameter("server_mode", True)

        # AnyGrasp
        self.declare_parameter("anygrasp_service", "detection")
        self.declare_parameter("anygrasp_frame", "camera_color_optical_frame")

        # MoveIt
        self.declare_parameter("move_group_action_name", "move_action")
        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("planning_frame", "base_link")
        self.declare_parameter("end_effector_link", "tool0")
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 5)
        self.declare_parameter("max_velocity_scaling", 0.2)
        self.declare_parameter("max_acceleration_scaling", 0.2)
        self.declare_parameter("position_tolerance_m", 0.005)
        self.declare_parameter("orientation_tolerance_rad", 0.1)
        self.declare_parameter("planning_pipeline_id", "")
        self.declare_parameter("planner_id", "")

        # Post-grasp move
        self.declare_parameter("do_post_grasp_move", True)
        self.declare_parameter("post_grasp_frame", "base_link")
        self.declare_parameter("post_grasp_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

        # Gripper actions
        self.declare_parameter("open_action_name", "/open_gripper")
        self.declare_parameter("close_action_name", "/close_gripper")

        # TF
        self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Clients
        self._anygrasp_client = self.create_client(
            GetGrasps, str(self.get_parameter("anygrasp_service").value)
        )

        self._movegroup_client = ActionClient(
            self, MoveGroup, str(self.get_parameter("move_group_action_name").value)
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

        if do_transform_pose is None:
            self.get_logger().warn(
                "Python module tf2_geometry_msgs is not available; pose transforms may fail. "
                "Install ros-$ROS_DISTRO-tf2-geometry-msgs."
            )

        if bool(self.get_parameter("server_mode").value):
            self.get_logger().info(
                "ur_grasping node ready (server_mode=true). Call ~/run_grasp to execute pipeline."
            )
        else:
            self.get_logger().info(
                "ur_grasping node ready (server_mode=false). Will execute pipeline once and exit."
            )

    # ------------------------
    # Pipeline trigger
    # ------------------------

    def _on_run_grasp(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        try:
            ok, msg = self.run_pipeline()
            res.success = bool(ok)
            res.message = msg
        except Exception as exc:  # noqa: BLE001
            res.success = False
            res.message = f"Pipeline failed: {exc}"
        return res

    # ------------------------
    # High-level pipeline
    # ------------------------

    def run_pipeline(self) -> Tuple[bool, str]:
        grasp_pose = self._request_anygrasp_pose()
        if grasp_pose is None:
            return False, "AnyGrasp returned no pose."

        if not self._move_to_pose(grasp_pose):
            return False, "MoveIt move to grasp pose failed."

        # Close gripper (position-based for now)
        if not self.close_gripper_position():
            return False, "Failed to close gripper."

        if bool(self.get_parameter("do_post_grasp_move").value):
            post_pose = self._get_post_grasp_pose_stamped()
            if not self._move_to_pose(post_pose):
                return False, "MoveIt move to post-grasp pose failed."

        return True, "Grasping pipeline completed."

    # ------------------------
    # AnyGrasp
    # ------------------------

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

        pose = resp.poses[0]
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = str(self.get_parameter("anygrasp_frame").value)
        pose_stamped.pose = pose

        return self._transform_pose_to_planning_frame(pose_stamped)

    def _transform_pose_to_planning_frame(self, pose: PoseStamped) -> PoseStamped:
        if do_transform_pose is None:
            raise RuntimeError("tf2_geometry_msgs is required to transform PoseStamped")

        target_frame = str(self.get_parameter("planning_frame").value)
        if pose.header.frame_id == target_frame:
            return pose

        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                f"Failed TF lookup {target_frame} <- {pose.header.frame_id}: {exc}"
            )

        out = do_transform_pose(pose, tf)
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = target_frame
        return out

    # ------------------------
    # MoveIt (MoveGroup action)
    # ------------------------

    def _move_to_pose(self, target_pose: PoseStamped) -> bool:
        action_name = str(self.get_parameter("move_group_action_name").value)

        if not self._movegroup_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"MoveGroup action server '{action_name}' not available.")
            return False

        goal = MoveGroup.Goal()
        goal.request = self._build_motion_plan_request(target_pose)
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 0

        send_future = self._movegroup_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done() or send_future.result() is None:
            self.get_logger().error("Failed to send MoveGroup goal.")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error("MoveGroup result not received.")
            return False

        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"MoveGroup failed with error code: {result.error_code.val}")
            return False

        return True

    def _build_motion_plan_request(self, target_pose: PoseStamped) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name = str(self.get_parameter("planning_group").value)
        req.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        req.num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        req.max_velocity_scaling_factor = float(self.get_parameter("max_velocity_scaling").value)
        req.max_acceleration_scaling_factor = float(
            self.get_parameter("max_acceleration_scaling").value
        )

        pipeline_id = str(self.get_parameter("planning_pipeline_id").value)
        planner_id = str(self.get_parameter("planner_id").value)
        if pipeline_id:
            req.pipeline_id = pipeline_id
        if planner_id:
            req.planner_id = planner_id

        req.start_state = RobotState()  # empty = current state
        req.goal_constraints = [self._pose_to_constraints(target_pose)]
        return req

    def _pose_to_constraints(self, target_pose: PoseStamped) -> Constraints:
        planning_frame = str(self.get_parameter("planning_frame").value)
        ee_link = str(self.get_parameter("end_effector_link").value)

        pos_tol = float(self.get_parameter("position_tolerance_m").value)
        ori_tol = float(self.get_parameter("orientation_tolerance_rad").value)

        # Position constraint: sphere around target position
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [max(1e-4, pos_tol)]

        bv = BoundingVolume()
        bv.primitives = [sphere]

        primitive_pose = Pose()
        primitive_pose.position = target_pose.pose.position
        primitive_pose.orientation.w = 1.0
        bv.primitive_poses = [primitive_pose]

        pc = PositionConstraint()
        pc.header.frame_id = planning_frame
        pc.link_name = ee_link
        pc.constraint_region = bv

        q = Quaternion(
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w,
        )
        q = _normalize_quaternion(q)

        oc = OrientationConstraint()
        oc.header.frame_id = planning_frame
        oc.link_name = ee_link
        oc.orientation.x = q.x
        oc.orientation.y = q.y
        oc.orientation.z = q.z
        oc.orientation.w = q.w
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        return c

    # ------------------------
    # Gripper helpers
    # ------------------------

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

        gh = send_future.result()
        if not gh.accepted:
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)
        if not result_future.done() or result_future.result() is None:
            return False

        return bool(result_future.result().result.success)

    def close_gripper_position(self) -> bool:
        """Position-based close.

        Uses the gripper driver's configured `close_position` (implementation-specific).
        """

        return self._close_gripper(use_torque_mode=False, torque=0.0)

    def close_gripper_torque(self, torque: float) -> bool:
        """Torque/current-limited close.

        The numeric scaling/units are driver-specific.
        """

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

        gh = send_future.result()
        if not gh.accepted:
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)
        if not result_future.done() or result_future.result() is None:
            return False

        return bool(result_future.result().result.success)

    # ------------------------
    # Post-grasp pose
    # ------------------------

    def _get_post_grasp_pose_stamped(self) -> PoseStamped:
        vals = list(self.get_parameter("post_grasp_pose").value)
        if len(vals) != 7:
            raise RuntimeError("post_grasp_pose must be [x,y,z,qx,qy,qz,qw]")

        frame = str(self.get_parameter("post_grasp_frame").value)
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = frame
        ps.pose.position.x = float(vals[0])
        ps.pose.position.y = float(vals[1])
        ps.pose.position.z = float(vals[2])
        ps.pose.orientation.x = float(vals[3])
        ps.pose.orientation.y = float(vals[4])
        ps.pose.orientation.z = float(vals[5])
        ps.pose.orientation.w = float(vals[6])

        # If post pose is not in planning frame, transform it.
        if frame != str(self.get_parameter("planning_frame").value):
            ps = self._transform_pose_to_planning_frame(ps)

        return ps


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
