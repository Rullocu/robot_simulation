#!/usr/bin/env python3
"""
motion_planner.py
=================
Motion planning and execution via MoveIt2 ROS2 services/actions.
No moveit_py required — only moveit_msgs + rclpy action client.

Services used
-------------
  /plan_kinematic_path  (moveit_msgs/srv/GetMotionPlan)
  /apply_planning_scene (moveit_msgs/srv/ApplyPlanningScene)

Actions used
------------
  /execute_trajectory   (moveit_msgs/action/ExecuteTrajectory)
"""

from __future__ import annotations

import time
from typing import Optional, Tuple

from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan
from shape_msgs.msg import SolidPrimitive

from .scene_manager import SceneManager


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _wait(future, timeout: float = 30.0):
    """Block (busy-wait) until a future is done. Node must be spinning elsewhere."""
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            return None
        time.sleep(0.02)
    return future.result()


def _pose(x: float, y: float, z: float,
          qx: float, qy: float, qz: float, qw: float) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.x = qx
    p.orientation.y = qy
    p.orientation.z = qz
    p.orientation.w = qw
    return p


def _build_pose_constraints(frame_id: str, link_name: str,
                             goal: Pose,
                             pos_tol: float = 0.002,
                             orient_tol: float = 0.02) -> Constraints:
    """Build a Constraints message for a single Cartesian pose goal."""
    # Position constraint: small sphere around target
    prim = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol])
    bvol = BoundingVolume()
    bvol.primitives.append(prim)
    bvol.primitive_poses.append(goal)

    pos_c = PositionConstraint()
    pos_c.header.frame_id = frame_id
    pos_c.link_name = link_name
    pos_c.constraint_region = bvol
    pos_c.weight = 1.0

    # Orientation constraint
    ori_c = OrientationConstraint()
    ori_c.header.frame_id = frame_id
    ori_c.link_name = link_name
    ori_c.orientation = goal.orientation
    ori_c.absolute_x_axis_tolerance = orient_tol
    ori_c.absolute_y_axis_tolerance = orient_tol
    ori_c.absolute_z_axis_tolerance = orient_tol
    ori_c.weight = 1.0

    c = Constraints()
    c.position_constraints.append(pos_c)
    c.orientation_constraints.append(ori_c)
    return c


def _build_joint_constraints(joint_names: list[str],
                               values: list[float],
                               tol: float = 0.01) -> Constraints:
    c = Constraints()
    for name, val in zip(joint_names, values):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = val
        jc.tolerance_above = tol
        jc.tolerance_below = tol
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    return c


# ---------------------------------------------------------------------------
# MotionPlanner
# ---------------------------------------------------------------------------

# Joint names for the lite6 planning group
LITE6_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
HOME_ANGLES   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class MotionPlanner:
    """
    High-level pick/place motion via MoveIt2 plan+execute services.

    Grasping is simulated: the product CollisionObject is attached to the
    EEF after descending (simulating closing the gripper) and detached
    after arriving at the target (simulating opening the gripper).
    """

    def __init__(
        self,
        node: Node,
        scene: SceneManager,
        arm_group: str,
        eef_link: str,
        base_frame: str,
        planning_time: float,
        vel_scale: float,
        acc_scale: float,
        max_attempts: int,
        orientation: Tuple[float, float, float, float],
        product_dims: Tuple[float, float, float],
    ):
        self._node        = node
        self._scene       = scene
        self._group       = arm_group
        self._eef         = eef_link
        self._frame       = base_frame
        self._plan_time   = planning_time
        self._vel         = vel_scale
        self._acc         = acc_scale
        self._max_att     = max_attempts
        self._orient      = orientation       # (qx, qy, qz, qw)
        self._prod_dims   = product_dims      # (sx, sy, sz) for detach

        # Service clients and action client
        self._plan_cli = node.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._exec_cli = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")

        self._node.get_logger().info("Waiting for /plan_kinematic_path …")
        self._plan_cli.wait_for_service(timeout_sec=30.0)
        self._node.get_logger().info("Waiting for /execute_trajectory …")
        self._exec_cli.wait_for_server(timeout_sec=30.0)
        self._node.get_logger().info("Motion planner ready.")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def move_home(self) -> bool:
        """Move arm to home (all joints = 0)."""
        c = _build_joint_constraints(LITE6_JOINTS, HOME_ANGLES)
        return self._plan_and_execute(c, label="home")

    def move_to_pose(self, x: float, y: float, z: float) -> bool:
        """Move EEF to (x, y, z) with the configured pick orientation."""
        qx, qy, qz, qw = self._orient
        goal = _pose(x, y, z, qx, qy, qz, qw)
        c = _build_pose_constraints(self._frame, self._eef, goal)
        return self._plan_and_execute(c, label=f"({x:.3f},{y:.3f},{z:.3f})")

    def pick_product(self, px: float, py: float, pz: float,
                     approach_h: float, retreat_h: float,
                     product_id: str) -> bool:
        """approach → descend → attach product to EEF → retreat"""
        log = self._node.get_logger()

        if not self.move_to_pose(px, py, pz + approach_h):
            log.warn(f"[pick] approach failed for {product_id}")
            return False

        if not self.move_to_pose(px, py, pz):
            log.warn(f"[pick] descend failed for {product_id}")
            return False

        # Simulate gripper close: attach product so it renders on the EEF
        sx, sy, sz = self._prod_dims
        self._scene.attach_to_eef(product_id, self._eef, sx, sy, sz)
        time.sleep(0.25)

        if not self.move_to_pose(px, py, pz + retreat_h):
            log.warn(f"[pick] retreat failed")

        return True

    def place_product(self, px: float, py: float, pz: float,
                      approach_h: float, retreat_h: float,
                      product_id: str) -> bool:
        """approach → descend → detach (silent) → retreat → add product to world"""
        log = self._node.get_logger()

        if not self.move_to_pose(px, py, pz + approach_h):
            log.warn(f"[place] approach failed for {product_id}")
            return False

        if not self.move_to_pose(px, py, pz):
            log.warn(f"[place] descend failed for {product_id}")
            return False

        # Detach without adding world object first — avoids EEF/product overlap
        # that would block retreat planning
        self._scene.detach_object(product_id, self._eef)
        time.sleep(0.25)

        if not self.move_to_pose(px, py, pz + retreat_h):
            log.warn(f"[place] retreat failed")

        # Now arm is clear — materialise the product at the target position
        sx, sy, sz = self._prod_dims
        self._scene.add_product(product_id, px, py, pz, sx, sy, sz)

        return True

    # ------------------------------------------------------------------
    # Internal: plan + execute
    # ------------------------------------------------------------------

    def _plan_and_execute(self, goal_constraints: Constraints,
                          label: str = "") -> bool:
        for attempt in range(1, self._max_att + 1):
            traj = self._plan(goal_constraints)
            if traj is not None:
                ok = self._execute(traj)
                if ok:
                    return True
                self._node.get_logger().warn(
                    f"Execution failed (attempt {attempt}), retrying…")
            else:
                self._node.get_logger().warn(
                    f"Planning failed for '{label}' (attempt {attempt}/{self._max_att})")
        self._node.get_logger().error(f"All attempts failed for '{label}'")
        return False

    def _plan(self, goal_constraints: Constraints):
        req_msg = MotionPlanRequest()
        req_msg.group_name                   = self._group
        req_msg.num_planning_attempts        = 3
        req_msg.allowed_planning_time        = self._plan_time
        req_msg.max_velocity_scaling_factor  = self._vel
        req_msg.max_acceleration_scaling_factor = self._acc
        req_msg.goal_constraints.append(goal_constraints)
        # Leave start_state empty → move_group uses current robot state

        srv_req = GetMotionPlan.Request()
        srv_req.motion_plan_request = req_msg
        result = _wait(self._plan_cli.call_async(srv_req),
                       timeout=self._plan_time + 10.0)
        if result is None:
            return None

        resp = result.motion_plan_response
        SUCCESS = 1
        if resp.error_code.val != SUCCESS:
            self._node.get_logger().debug(
                f"Plan error code: {resp.error_code.val}")
            return None
        return resp.trajectory

    def _execute(self, trajectory) -> bool:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = self._exec_cli.send_goal_async(goal)
        goal_handle = _wait(send_future, timeout=10.0)
        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().warn("Execution goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        result = _wait(result_future, timeout=60.0)
        if result is None:
            self._node.get_logger().warn("Execution timed out")
            return False

        SUCCESS = 1
        ok = (result.result.error_code.val == SUCCESS)
        if not ok:
            self._node.get_logger().warn(
                f"Execution error code: {result.result.error_code.val}")
        return ok
