#!/usr/bin/env python3
"""
motion_planner.py
=================
Thin wrapper around MoveItPy for the pick-and-place pipeline.

Responsibilities
----------------
- Move the arm to a named state ("home") or to an arbitrary Cartesian pose.
- Execute a pick sequence:
    1. Move to approach pose (above product)
    2. Move straight down to grasp pose
    3. Attach the product collision object (simulates closing the gripper)
    4. Retreat straight up
- Execute a place sequence:
    1. Move to approach pose (above target)
    2. Move straight down to place pose
    3. Detach the product collision object (simulates opening the gripper)
    4. Retreat straight up

Gripper note
------------
The Lite6 gripper is declared as a passive (fixed) joint in xarm_ros2's
SRDF, so it cannot be actuated through MoveIt2 in fake-hardware mode.
Grasping is therefore simulated by attaching/detaching the product's
collision object to the robot's EEF link.  The gripper mesh is still
rendered in RViz so the visual is convincing.
"""

from __future__ import annotations

import copy
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy


# -------------------------------------------------------------------------
# Helpers
# -------------------------------------------------------------------------

def _stamped(x: float, y: float, z: float,
             qx: float, qy: float, qz: float, qw: float,
             frame_id: str) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps


# -------------------------------------------------------------------------
# MotionPlanner
# -------------------------------------------------------------------------

class MotionPlanner:
    """
    Wraps MoveItPy to provide high-level pick/place primitives.

    Parameters
    ----------
    moveit : MoveItPy
        Shared MoveItPy instance (created once by the node).
    node : Node
        The ROS2 node (used for logging).
    arm_group : str
        MoveIt2 planning group name (e.g. "lite6").
    eef_link : str
        End-effector link used for Cartesian pose goals (e.g. "link_tcp").
    base_frame : str
        Reference frame for all poses (e.g. "world").
    planning_time : float
        Maximum time (s) allowed per planning call.
    vel_scale : float
        Velocity scaling factor [0, 1].
    acc_scale : float
        Acceleration scaling factor [0, 1].
    max_attempts : int
        Number of replanning attempts on failure.
    """

    def __init__(
        self,
        moveit: MoveItPy,
        node: Node,
        arm_group: str,
        eef_link: str,
        base_frame: str,
        planning_time: float,
        vel_scale: float,
        acc_scale: float,
        max_attempts: int,
        orientation: tuple,
    ):
        self._moveit = moveit
        self._node = node
        self._arm_group = arm_group
        self._eef_link = eef_link
        self._frame = base_frame
        self._planning_time = planning_time
        self._vel_scale = vel_scale
        self._acc_scale = acc_scale
        self._max_attempts = max_attempts
        # (qx, qy, qz, qw) for top-down approach
        self._orientation = orientation

        self._arm = moveit.get_planning_component(arm_group)
        self._robot_model = moveit.get_robot_model()

    # ------------------------------------------------------------------
    # High-level motions
    # ------------------------------------------------------------------

    def move_home(self) -> bool:
        """Move arm to the 'home' named state (all joints = 0)."""
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(configuration_name="home")
        return self._plan_and_execute()

    def move_to_pose(self, x: float, y: float, z: float) -> bool:
        """Move EEF to (x, y, z) with the default pick orientation."""
        qx, qy, qz, qw = self._orientation
        goal = _stamped(x, y, z, qx, qy, qz, qw, self._frame)
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(pose_stamped_msg=goal, pose_link=self._eef_link)
        return self._plan_and_execute()

    def pick_product(
        self,
        px: float, py: float, pz: float,
        approach_h: float,
        retreat_h: float,
        product_id: str,
    ) -> bool:
        """
        Execute a pick sequence:
          approach → descend → attach object → retreat
        """
        # 1. Approach (above the product)
        if not self.move_to_pose(px, py, pz + approach_h):
            self._node.get_logger().warn(
                f"[pick] Failed to reach approach for {product_id}")
            return False

        # 2. Descend to product centre
        if not self.move_to_pose(px, py, pz):
            self._node.get_logger().warn(
                f"[pick] Failed to descend to {product_id}")
            return False

        # 3. Simulate grasp: attach collision object to EEF
        self._attach_object(product_id)
        time.sleep(0.3)   # brief pause — looks more natural in RViz

        # 4. Retreat
        if not self.move_to_pose(px, py, pz + retreat_h):
            self._node.get_logger().warn(
                f"[pick] Failed to retreat after picking {product_id}")
            # Still return True — we did pick up the object

        return True

    def place_product(
        self,
        px: float, py: float, pz: float,
        approach_h: float,
        retreat_h: float,
        product_id: str,
    ) -> bool:
        """
        Execute a place sequence:
          approach → descend → detach object → retreat
        """
        # 1. Approach
        if not self.move_to_pose(px, py, pz + approach_h):
            self._node.get_logger().warn(
                f"[place] Failed to reach approach for {product_id}")
            return False

        # 2. Descend to place position
        if not self.move_to_pose(px, py, pz):
            self._node.get_logger().warn(
                f"[place] Failed to descend to target for {product_id}")
            return False

        # 3. Simulate release: detach collision object from EEF
        self._detach_object(product_id, px, py, pz)
        time.sleep(0.3)

        # 4. Retreat
        if not self.move_to_pose(px, py, pz + retreat_h):
            self._node.get_logger().warn(
                f"[place] Failed to retreat after placing {product_id}")

        return True

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _plan_and_execute(self) -> bool:
        """Plan and execute with retry logic.  Returns True on success."""
        for attempt in range(1, self._max_attempts + 1):
            plan_result = self._arm.plan()
            if plan_result:
                self._moveit.execute(
                    plan_result.trajectory,
                    blocking=True,
                    controllers=[],
                )
                return True
            self._node.get_logger().warn(
                f"Planning attempt {attempt}/{self._max_attempts} failed, retrying…"
            )
            self._arm.set_start_state_to_current_state()

        self._node.get_logger().error("All planning attempts failed.")
        return False

    def _attach_object(self, object_id: str) -> None:
        """Attach a collision object to the robot EEF (simulated grip)."""
        with self._moveit.get_planning_scene_monitor().read_write() as scene:
            scene.current_state.update()
            # Build attached object: move from world into attached space
            aco = AttachedCollisionObject()
            aco.link_name = self._eef_link
            aco.object.id = object_id
            aco.object.operation = CollisionObject.ADD
            # Allow the object to touch the EEF links
            aco.touch_links = [self._eef_link, "link6", "link5"]
            scene.current_state.update()

        # Use the PSM attach helper which handles the world → attached move
        with self._moveit.get_planning_scene_monitor().read_write() as scene:
            # Remove from world collision objects, add as attached
            scene.apply_attached_collision_object(aco)
            scene.current_state.update()

        self._node.get_logger().debug(f"Attached {object_id} to {self._eef_link}")

    def _detach_object(
        self, object_id: str, px: float, py: float, pz: float
    ) -> None:
        """Detach the collision object from the EEF and place it in the world."""
        aco = AttachedCollisionObject()
        aco.link_name = self._eef_link
        aco.object.id = object_id
        aco.object.operation = CollisionObject.REMOVE

        with self._moveit.get_planning_scene_monitor().read_write() as scene:
            # Detach from robot
            scene.apply_attached_collision_object(aco)
            # Re-add to world at the target position so it blocks future paths
            from shape_msgs.msg import SolidPrimitive
            from geometry_msgs.msg import Pose as GmPose
            co = CollisionObject()
            co.id = object_id
            co.header.frame_id = self._frame
            co.operation = CollisionObject.ADD
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            # Recover original size from the scene if available; use defaults
            prim.dimensions = [0.040, 0.040, 0.030]
            pose = GmPose()
            pose.position.x = px
            pose.position.y = py
            pose.position.z = pz
            pose.orientation.w = 1.0
            co.primitives.append(prim)
            co.primitive_poses.append(pose)
            scene.apply_collision_object(co)
            scene.current_state.update()

        self._node.get_logger().debug(
            f"Detached {object_id} and placed at ({px:.3f},{py:.3f},{pz:.3f})"
        )
