#!/usr/bin/env python3
"""
pick_place_node.py
==================
Main entry point for the Lite6 pick-and-place simulation.
Uses only standard rclpy + moveit_msgs — no moveit_py required.

Pipeline
--------
1. Declare and load ROS2 parameters.
2. Wait for MoveIt2 services to become available (handled by SceneManager /
   MotionPlanner constructors).
3. Initialise the planning scene (table, two open-top boxes, product grid).
4. Move arm to home position.
5. For each product (top layer → bottom layer, left-to-right, front-to-back):
     pick (approach → descend → attach → retreat)
     place (approach → descend → detach → retreat)
6. Return home.

Iteration order
---------------
Top layer first → the arm never has to reach past a still-present product
below the one it's picking.  Products land at the same (col, row, layer)
in the target box ("corresponding position").
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .scene_manager import SceneManager
from .motion_planner import MotionPlanner


class PickPlaceNode(Node):

    def __init__(self):
        super().__init__("pick_place_node")

        # Grid
        self.declare_parameter("grid_cols",   3)
        self.declare_parameter("grid_rows",   5)
        self.declare_parameter("grid_layers", 10)

        # Product dimensions
        self.declare_parameter("product_size_x", 0.040)
        self.declare_parameter("product_size_y", 0.040)
        self.declare_parameter("product_size_z", 0.030)

        # Source box (centre of inner floor)
        self.declare_parameter("source_box_x", 0.230)
        self.declare_parameter("source_box_y", 0.180)
        self.declare_parameter("source_box_z", 0.050)

        # Target box
        self.declare_parameter("target_box_x", 0.230)
        self.declare_parameter("target_box_y", -0.180)
        self.declare_parameter("target_box_z", 0.050)

        self.declare_parameter("box_wall_thickness",   0.004)
        self.declare_parameter("box_height",           0.10)
        self.declare_parameter("box_margin",           0.030)

        # Motion
        self.declare_parameter("approach_height",       0.080)
        self.declare_parameter("retreat_height",        0.120)
        self.declare_parameter("arm_group",             "lite6")
        self.declare_parameter("eef_link",              "link_tcp")
        self.declare_parameter("base_frame",            "world")
        self.declare_parameter("planning_time",         5.0)
        self.declare_parameter("velocity_scaling",      0.3)
        self.declare_parameter("acceleration_scaling",  0.3)
        self.declare_parameter("max_planning_attempts", 5)

        # Pick orientation (RPY = π,0,0 → gripper pointing straight down)
        self.declare_parameter("pick_orientation_x", 1.0)
        self.declare_parameter("pick_orientation_y", 0.0)
        self.declare_parameter("pick_orientation_z", 0.0)
        self.declare_parameter("pick_orientation_w", 0.0)

        # Demo limiter (-1 = all)
        self.declare_parameter("max_products", 6)

    # Convenience accessors
    @property
    def grid(self):
        return {
            "cols":   self.get_parameter("grid_cols").value,
            "rows":   self.get_parameter("grid_rows").value,
            "layers": self.get_parameter("grid_layers").value,
        }

    @property
    def prod(self):
        return {
            "size_x": self.get_parameter("product_size_x").value,
            "size_y": self.get_parameter("product_size_y").value,
            "size_z": self.get_parameter("product_size_z").value,
        }

    @property
    def src_pos(self):
        return (self.get_parameter("source_box_x").value,
                self.get_parameter("source_box_y").value,
                self.get_parameter("source_box_z").value)

    @property
    def tgt_pos(self):
        return (self.get_parameter("target_box_x").value,
                self.get_parameter("target_box_y").value,
                self.get_parameter("target_box_z").value)

    @property
    def orientation(self):
        return (self.get_parameter("pick_orientation_x").value,
                self.get_parameter("pick_orientation_y").value,
                self.get_parameter("pick_orientation_z").value,
                self.get_parameter("pick_orientation_w").value)


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()

    # Spin in background so service callbacks are processed
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    log = node.get_logger()

    # ------------------------------------------------------------------
    # Load parameters
    # ------------------------------------------------------------------
    grid         = node.grid
    prod         = node.prod
    src_pos      = node.src_pos
    tgt_pos      = node.tgt_pos
    orientation  = node.orientation
    wall_t       = node.get_parameter("box_wall_thickness").value
    box_height   = node.get_parameter("box_height").value
    box_margin   = node.get_parameter("box_margin").value
    approach_h   = node.get_parameter("approach_height").value
    retreat_h    = node.get_parameter("retreat_height").value
    arm_group    = node.get_parameter("arm_group").value
    eef_link     = node.get_parameter("eef_link").value
    base_frame   = node.get_parameter("base_frame").value
    plan_time    = node.get_parameter("planning_time").value
    vel_scale    = node.get_parameter("velocity_scaling").value
    acc_scale    = node.get_parameter("acceleration_scaling").value
    max_att      = node.get_parameter("max_planning_attempts").value
    max_products = node.get_parameter("max_products").value

    sx = prod["size_x"]
    sy = prod["size_y"]
    sz = prod["size_z"]
    total = grid["cols"] * grid["rows"] * grid["layers"]
    limit = total if max_products < 0 else min(max_products, total)

    log.info(f"Grid {grid['cols']}×{grid['rows']}×{grid['layers']} = {total} products "
             f"(will process {limit})")

    # ------------------------------------------------------------------
    # Initialise subsystems (each waits for its services internally)
    # ------------------------------------------------------------------
    scene   = SceneManager(node, base_frame)
    planner = MotionPlanner(
        node, scene, arm_group, eef_link, base_frame,
        plan_time, vel_scale, acc_scale, max_att, orientation,
        (sx, sy, sz),
    )

    # ------------------------------------------------------------------
    # Build scene
    # ------------------------------------------------------------------
    log.info("Setting up planning scene …")
    scene.setup_scene(grid, prod, src_pos, tgt_pos, wall_t, box_height, box_margin)

    # ------------------------------------------------------------------
    # Home
    # ------------------------------------------------------------------
    log.info("Moving to home …")
    planner.move_home()

    # ------------------------------------------------------------------
    # Grid origins  (centre of the [col=0, row=0] product)
    # ------------------------------------------------------------------
    src_x0 = src_pos[0] - (grid["cols"] * sx) / 2.0 + sx / 2.0
    src_y0 = src_pos[1] - (grid["rows"] * sy) / 2.0 + sy / 2.0
    tgt_x0 = tgt_pos[0] - (grid["cols"] * sx) / 2.0 + sx / 2.0
    tgt_y0 = tgt_pos[1] - (grid["rows"] * sy) / 2.0 + sy / 2.0

    # ------------------------------------------------------------------
    # Pick-and-place loop  (top layer first)
    # ------------------------------------------------------------------
    log.info("━" * 52)
    log.info("Starting pick-and-place loop …")
    log.info("━" * 52)

    count, success = 0, 0
    failed = []
    done = False

    for layer in reversed(range(grid["layers"])):
        if done:
            break
        for row in range(grid["rows"]):
            if done:
                break
            for col in range(grid["cols"]):
                if count >= limit:
                    done = True
                    break

                pid = f"product_{col}_{row}_{layer}"

                pick_x  = src_x0 + col * sx
                pick_y  = src_y0 + row * sy
                pick_z  = src_pos[2] + layer * sz + sz / 2.0

                place_x = tgt_x0 + col * sx
                place_y = tgt_y0 + row * sy
                place_z = tgt_pos[2] + layer * sz + sz / 2.0

                log.info(
                    f"[{count+1}/{limit}] {pid}  "
                    f"src=({pick_x:.3f},{pick_y:.3f},{pick_z:.3f})  "
                    f"dst=({place_x:.3f},{place_y:.3f},{place_z:.3f})"
                )

                ok = planner.pick_product(
                    pick_x, pick_y, pick_z, approach_h, retreat_h, pid)

                if ok:
                    planner.place_product(
                        place_x, place_y, place_z, approach_h, retreat_h, pid)
                    scene.remove_collision_object(pid)
                    success += 1
                    log.info(f"  ✓ {pid} placed")
                else:
                    failed.append(pid)
                    log.warn(f"  ✗ {pid} skipped")

                count += 1

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    log.info("━" * 52)
    log.info(f"Done — {success}/{count} products transferred successfully.")
    if failed:
        log.warn(f"Failed: {failed}")

    log.info("Returning to home …")
    planner.move_home()
    log.info("Pick-and-place node finished.")

    rclpy.shutdown()
    spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
