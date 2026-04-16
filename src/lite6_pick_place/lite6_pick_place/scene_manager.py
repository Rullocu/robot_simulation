#!/usr/bin/env python3
"""
scene_manager.py
================
Manages the MoveIt2 planning scene:
  - Adds source/target boxes as collision objects (open-top, 5 faces)
  - Adds product collision objects per layer
  - Removes product collision objects once picked
  - Adds a ground-plane table for visual reference

All collision objects are applied through the MoveItPy planning scene
monitor so they are immediately visible in RViz and used by the planner.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _make_primitive_pose(x: float, y: float, z: float) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.w = 1.0
    return p


def _solid_box(dims: list[float]) -> SolidPrimitive:
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = list(dims)
    return prim


def _make_box_co(
    obj_id: str,
    frame_id: str,
    cx: float, cy: float, cz: float,
    sx: float, sy: float, sz: float,
) -> CollisionObject:
    """Single solid-box collision object centred at (cx, cy, cz)."""
    co = CollisionObject()
    co.id = obj_id
    co.header.frame_id = frame_id
    co.operation = CollisionObject.ADD
    co.primitives.append(_solid_box([sx, sy, sz]))
    co.primitive_poses.append(_make_primitive_pose(cx, cy, cz))
    return co


# ---------------------------------------------------------------------------
# SceneManager
# ---------------------------------------------------------------------------

class SceneManager:
    """
    Populates the MoveIt2 planning scene.

    Open-top box representation
    ---------------------------
    Each box is made of 5 solid cuboids:
        floor, wall_back, wall_front, wall_left, wall_right
    The open top lets the robot approach products from above without
    triggering a collision with the box geometry.
    """

    def __init__(self, moveit_py_instance, node: Node, frame_id: str = "world"):
        self._moveit = moveit_py_instance
        self._node = node
        self._frame = frame_id

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def setup_scene(
        self,
        grid: dict,
        prod: dict,
        src_pos: tuple,
        tgt_pos: tuple,
        wall_t: float,
    ) -> None:
        """Add table, source box, target box, and all initial products."""
        # Box interior dimensions  (just enough to hold the grid)
        inner_x = grid["cols"] * prod["size_x"]
        inner_y = grid["rows"] * prod["size_y"]
        inner_z = grid["layers"] * prod["size_z"]

        self._add_table(src_pos, tgt_pos, inner_x, inner_y)
        self._add_open_box("source_box", src_pos, inner_x, inner_y, inner_z, wall_t)
        self._add_open_box("target_box", tgt_pos, inner_x, inner_y, inner_z, wall_t)
        self._add_all_products(grid, prod, src_pos)

        self._node.get_logger().info("Planning scene initialised.")

    def remove_product(self, product_id: str) -> None:
        """Remove a product collision object after it has been placed."""
        co = CollisionObject()
        co.id = product_id
        co.operation = CollisionObject.REMOVE

        with self._moveit.get_planning_scene_monitor().read_write() as scene:
            scene.apply_collision_object(co)
            scene.current_state.update()

    def add_product(self, product_id: str, cx: float, cy: float, cz: float,
                    sx: float, sy: float, sz: float) -> None:
        """Add a single product collision object."""
        co = _make_box_co(product_id, self._frame, cx, cy, cz, sx, sy, sz)
        with self._moveit.get_planning_scene_monitor().read_write() as scene:
            scene.apply_collision_object(co)
            scene.current_state.update()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _apply(self, cos: list[CollisionObject]) -> None:
        with self._moveit.get_planning_scene_monitor().read_write() as scene:
            for co in cos:
                scene.apply_collision_object(co)
            scene.current_state.update()

    def _add_table(
        self,
        src_pos: tuple,
        tgt_pos: tuple,
        box_inner_x: float,
        box_inner_y: float,
    ) -> None:
        """Add a wide flat table that both boxes sit on."""
        table_h = src_pos[2]          # floor of boxes = top of table
        table_t = 0.02                # 2 cm thick slab
        # Centre the table between the two boxes
        cx = (src_pos[0] + tgt_pos[0]) / 2.0
        cy = (src_pos[1] + tgt_pos[1]) / 2.0
        # Wide enough to span both boxes plus some margin
        sx = box_inner_x + 0.10
        sy = abs(src_pos[1] - tgt_pos[1]) + box_inner_y + 0.10
        co = _make_box_co(
            "table", self._frame,
            cx, cy, table_h - table_t / 2.0,
            sx, sy, table_t,
        )
        self._apply([co])

    def _add_open_box(
        self,
        box_id: str,
        pos: tuple,           # (cx, cy, cz_floor_inside)
        inner_x: float,
        inner_y: float,
        inner_z: float,
        wall_t: float,
    ) -> None:
        """
        Create an open-top box from 5 solid cuboid collision objects.

            pos = centre of the inner floor face
        """
        cx, cy, cz_floor = pos[0], pos[1], pos[2]

        # Outer total dimensions
        outer_x = inner_x + 2 * wall_t
        outer_y = inner_y + 2 * wall_t
        outer_z = inner_z + wall_t   # wall_t for floor, open on top

        # Half-spans for wall centres
        hx = inner_x / 2.0
        hy = inner_y / 2.0
        hz = inner_z / 2.0

        # Z centre of walls (from floor centre upward)
        z_wall_centre = cz_floor + wall_t / 2.0 + hz

        objects = [
            # Floor
            _make_box_co(f"{box_id}_floor", self._frame,
                         cx, cy, cz_floor + wall_t / 2.0 - wall_t,
                         outer_x, outer_y, wall_t),
            # Back wall  (−Y side)
            _make_box_co(f"{box_id}_wall_back", self._frame,
                         cx, cy - hy - wall_t / 2.0, z_wall_centre,
                         outer_x, wall_t, outer_z),
            # Front wall (+Y side)
            _make_box_co(f"{box_id}_wall_front", self._frame,
                         cx, cy + hy + wall_t / 2.0, z_wall_centre,
                         outer_x, wall_t, outer_z),
            # Left wall  (−X side)
            _make_box_co(f"{box_id}_wall_left", self._frame,
                         cx - hx - wall_t / 2.0, cy, z_wall_centre,
                         wall_t, inner_y, outer_z),
            # Right wall (+X side)
            _make_box_co(f"{box_id}_wall_right", self._frame,
                         cx + hx + wall_t / 2.0, cy, z_wall_centre,
                         wall_t, inner_y, outer_z),
        ]
        self._apply(objects)

    def _add_all_products(
        self,
        grid: dict,
        prod: dict,
        src_pos: tuple,
    ) -> None:
        """Add all product collision objects into the source box."""
        sx, sy, sz = prod["size_x"], prod["size_y"], prod["size_z"]
        cx_floor = src_pos[0]
        cy_floor = src_pos[1]
        cz_floor = src_pos[2]

        # Origin of the first (col=0, row=0) product centre in XY
        x0 = cx_floor - (grid["cols"] * sx) / 2.0 + sx / 2.0
        y0 = cy_floor - (grid["rows"] * sy) / 2.0 + sy / 2.0

        objects = []
        for layer in range(grid["layers"]):
            for row in range(grid["rows"]):
                for col in range(grid["cols"]):
                    pid = f"product_{col}_{row}_{layer}"
                    px = x0 + col * sx
                    py = y0 + row * sy
                    pz = cz_floor + layer * sz + sz / 2.0
                    objects.append(_make_box_co(pid, self._frame, px, py, pz, sx, sy, sz))

        # Apply in batches to avoid holding the lock too long
        batch = 20
        for i in range(0, len(objects), batch):
            self._apply(objects[i: i + batch])

        self._node.get_logger().info(
            f"Added {len(objects)} product collision objects to planning scene."
        )
