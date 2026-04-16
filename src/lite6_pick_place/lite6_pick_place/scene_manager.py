#!/usr/bin/env python3
"""
scene_manager.py
================
Manages the MoveIt2 planning scene via the /apply_planning_scene service.
No moveit_py required — only moveit_msgs.
"""

from __future__ import annotations

import time
from typing import List

from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    CollisionObject,
    PlanningScene as PlanningSceneMsg,
    AttachedCollisionObject,
)
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _pose(x: float, y: float, z: float) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.w = 1.0
    return p


def _box_co(obj_id: str, frame_id: str,
            cx: float, cy: float, cz: float,
            sx: float, sy: float, sz: float) -> CollisionObject:
    co = CollisionObject()
    co.id = obj_id
    co.header.frame_id = frame_id
    co.operation = CollisionObject.ADD
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [sx, sy, sz]
    co.primitives.append(prim)
    co.primitive_poses.append(_pose(cx, cy, cz))
    return co


def _wait(future, timeout: float = 10.0):
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            return None
        time.sleep(0.02)
    return future.result()


# ---------------------------------------------------------------------------
# SceneManager
# ---------------------------------------------------------------------------

class SceneManager:
    """
    Populates and mutates the MoveIt2 planning scene.

    Open-top box: 5 solid cuboids (floor + 4 walls, no lid) so the robot
    can approach products from above without hitting box geometry.
    """

    def __init__(self, node: Node, frame_id: str = "world"):
        self._node = node
        self._frame = frame_id
        self._client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._node.get_logger().info("Waiting for /apply_planning_scene service …")
        self._client.wait_for_service(timeout_sec=30.0)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def setup_scene(self, grid: dict, prod: dict,
                    src_pos: tuple, tgt_pos: tuple,
                    wall_t: float, box_height: float, box_margin: float) -> None:
        # Inner footprint = product grid + margin on each side for gripper clearance
        inner_x = grid["cols"] * prod["size_x"] + 2 * box_margin
        inner_y = grid["rows"] * prod["size_y"] + 2 * box_margin
        # Wall height is set independently (box_height) so it stays short
        # even when grid_layers is small.

        objects: List[CollisionObject] = []
        objects += self._table_objects(src_pos, tgt_pos, inner_x, inner_y)
        objects += self._open_box_objects("source_box", src_pos,
                                          inner_x, inner_y, box_height, wall_t)
        objects += self._open_box_objects("target_box", tgt_pos,
                                          inner_x, inner_y, box_height, wall_t)
        objects += self._product_objects(grid, prod, src_pos)

        # Apply in batches of 30 so the service call stays responsive
        for i in range(0, len(objects), 30):
            self._apply_objects(objects[i: i + 30])

        self._node.get_logger().info(
            f"Scene ready: table + 2 boxes + {grid['cols']*grid['rows']*grid['layers']} products."
        )

    def remove_collision_object(self, obj_id: str) -> None:
        co = CollisionObject()
        co.id = obj_id
        co.operation = CollisionObject.REMOVE
        self._apply_objects([co])

    def attach_to_eef(self, obj_id: str, eef_link: str,
                      touch_links: List[str] | None = None) -> None:
        """Move a world collision object into the robot's attached-object list."""
        aco = AttachedCollisionObject()
        aco.link_name = eef_link
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = touch_links or [eef_link, "link6", "link5",
                                           "uflite_gripper_link"]

        # Remove from world + attach to robot in one diff
        co_remove = CollisionObject()
        co_remove.id = obj_id
        co_remove.operation = CollisionObject.REMOVE

        diff = PlanningSceneMsg()
        diff.is_diff = True
        diff.robot_state.is_diff = True
        diff.robot_state.attached_collision_objects.append(aco)
        diff.world.collision_objects.append(co_remove)
        self._apply_diff(diff)

    def detach_and_place(self, obj_id: str, eef_link: str,
                         px: float, py: float, pz: float,
                         sx: float, sy: float, sz: float) -> None:
        """Detach a held object and re-add it to the world at (px, py, pz)."""
        aco_remove = AttachedCollisionObject()
        aco_remove.link_name = eef_link
        aco_remove.object.id = obj_id
        aco_remove.object.operation = CollisionObject.REMOVE

        co_add = _box_co(obj_id, self._frame, px, py, pz, sx, sy, sz)

        diff = PlanningSceneMsg()
        diff.is_diff = True
        diff.robot_state.is_diff = True
        diff.robot_state.attached_collision_objects.append(aco_remove)
        diff.world.collision_objects.append(co_add)
        self._apply_diff(diff)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _apply_objects(self, objects: List[CollisionObject]) -> bool:
        diff = PlanningSceneMsg()
        diff.is_diff = True
        diff.world.collision_objects = list(objects)
        return self._apply_diff(diff)

    def _apply_diff(self, diff: PlanningSceneMsg) -> bool:
        req = ApplyPlanningScene.Request()
        req.scene = diff
        result = _wait(self._client.call_async(req), timeout=15.0)
        if result is None:
            self._node.get_logger().warn("apply_planning_scene timed out")
            return False
        return result.success

    # ------------------------------------------------------------------
    # Geometry builders
    # ------------------------------------------------------------------

    def _table_objects(self, src_pos, tgt_pos, inner_x, inner_y):
        t = 0.02  # table thickness (2 cm)
        cx = (src_pos[0] + tgt_pos[0]) / 2.0
        cy = (src_pos[1] + tgt_pos[1]) / 2.0
        z_top = min(src_pos[2], tgt_pos[2])   # bottom of the lower box
        sx = inner_x + 0.15
        sy = abs(src_pos[1] - tgt_pos[1]) + inner_y + 0.15
        return [_box_co("table", self._frame, cx, cy, z_top - t / 2.0, sx, sy, t)]

    def _open_box_objects(self, box_id: str, pos: tuple,
                          inner_x: float, inner_y: float,
                          box_height: float, wall_t: float) -> List[CollisionObject]:
        """5-face open-top box. pos = centre of inner floor."""
        cx, cy, cz_floor = pos[0], pos[1], pos[2]
        outer_x = inner_x + 2 * wall_t
        hx, hy = inner_x / 2.0, inner_y / 2.0
        z_wall_ctr = cz_floor + box_height / 2.0       # wall centre Z
        z_wall_h   = box_height + wall_t               # wall height

        return [
            # Floor (slightly below the product grid)
            _box_co(f"{box_id}_floor", self._frame,
                    cx, cy, cz_floor - wall_t / 2.0,
                    outer_x, inner_y + 2 * wall_t, wall_t),
            # Back wall (−Y)
            _box_co(f"{box_id}_back",  self._frame,
                    cx, cy - hy - wall_t / 2.0, z_wall_ctr,
                    outer_x, wall_t, z_wall_h),
            # Front wall (+Y)
            _box_co(f"{box_id}_front", self._frame,
                    cx, cy + hy + wall_t / 2.0, z_wall_ctr,
                    outer_x, wall_t, z_wall_h),
            # Left wall  (−X)
            _box_co(f"{box_id}_left",  self._frame,
                    cx - hx - wall_t / 2.0, cy, z_wall_ctr,
                    wall_t, inner_y, z_wall_h),
            # Right wall (+X)
            _box_co(f"{box_id}_right", self._frame,
                    cx + hx + wall_t / 2.0, cy, z_wall_ctr,
                    wall_t, inner_y, z_wall_h),
        ]

    def _product_objects(self, grid: dict, prod: dict,
                         src_pos: tuple) -> List[CollisionObject]:
        sx, sy, sz = prod["size_x"], prod["size_y"], prod["size_z"]
        x0 = src_pos[0] - (grid["cols"] * sx) / 2.0 + sx / 2.0
        y0 = src_pos[1] - (grid["rows"] * sy) / 2.0 + sy / 2.0

        objects = []
        for layer in range(grid["layers"]):
            for row in range(grid["rows"]):
                for col in range(grid["cols"]):
                    objects.append(_box_co(
                        f"product_{col}_{row}_{layer}", self._frame,
                        x0 + col * sx,
                        y0 + row * sy,
                        src_pos[2] + layer * sz + sz / 2.0,
                        sx, sy, sz,
                    ))
        return objects
