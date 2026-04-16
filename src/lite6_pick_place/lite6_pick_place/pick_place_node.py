#!/usr/bin/env python3
"""
pick_place_node.py
==================
Main entry point for the Lite6 pick-and-place simulation.

Pipeline
--------
1. Declare and load all ROS2 parameters.
2. Wait for MoveIt2 (move_group) to be fully ready.
3. Initialise the planning scene (table, two boxes, products).
4. Move to home position.
5. For each product (top layer → bottom layer):
     a. pick_product  – approach, descend, attach, retreat
     b. place_product – approach, descend, detach, retreat
6. Return to home when done (or max_products reached).

Iteration order
---------------
Products are picked from the **top layer downwards** in the source box so
that the robot never has to reach over a product it already picked.
In the target box products land in the **same layer / position** they came
from ("corresponding position" per the task spec).  Because the target box
starts empty, no collision avoidance between placed products is required
within the target.
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from moveit.planning import MoveItPy

from .scene_manager import SceneManager
from .motion_planner import MotionPlanner


# ---------------------------------------------------------------------------
# Parameter node
# ---------------------------------------------------------------------------

class PickPlaceNode(Node):
    """ROS2 node that holds parameters and drives the pick-and-place loop."""

    def __init__(self):
        super().__init__("pick_place_node")

        # --- grid ---
        self.declare_parameter("grid_cols",   3)
        self.declare_parameter("grid_rows",   5)
        self.declare_parameter("grid_layers", 10)

        # --- product dimensions ---
        self.declare_parameter("product_size_x", 0.040)
        self.declare_parameter("product_size_y", 0.040)
        self.declare_parameter("product_size_z", 0.030)

        # --- source box ---
        self.declare_parameter("source_box_x", 0.230)
        self.declare_parameter("source_box_y", 0.180)
        self.declare_parameter("source_box_z", 0.050)

        # --- target box ---
        self.declare_parameter("target_box_x", 0.230)
        self.declare_parameter("target_box_y", -0.180)
        self.declare_parameter("target_box_z", 0.050)

        self.declare_parameter("box_wall_thickness", 0.004)

        # --- motion ---
        self.declare_parameter("approach_height",     0.080)
        self.declare_parameter("retreat_height",      0.120)
        self.declare_parameter("arm_group",           "lite6")
        self.declare_parameter("eef_link",            "link_tcp")
        self.declare_parameter("base_frame",          "world")
        self.declare_parameter("planning_time",       5.0)
        self.declare_parameter("velocity_scaling",    0.3)
        self.declare_parameter("acceleration_scaling", 0.3)
        self.declare_parameter("max_planning_attempts", 5)

        # --- pick orientation (quaternion, top-down: RPY = π, 0, 0) ---
        self.declare_parameter("pick_orientation_x", 1.0)
        self.declare_parameter("pick_orientation_y", 0.0)
        self.declare_parameter("pick_orientation_z", 0.0)
        self.declare_parameter("pick_orientation_w", 0.0)

        # --- demo limiter ---
        self.declare_parameter("max_products", 6)

    # ----- convenience getters -----------------------------------------

    @property
    def grid(self) -> dict:
        return {
            "cols":   self.get_parameter("grid_cols").value,
            "rows":   self.get_parameter("grid_rows").value,
            "layers": self.get_parameter("grid_layers").value,
        }

    @property
    def prod(self) -> dict:
        return {
            "size_x": self.get_parameter("product_size_x").value,
            "size_y": self.get_parameter("product_size_y").value,
            "size_z": self.get_parameter("product_size_z").value,
        }

    @property
    def src_pos(self) -> tuple:
        return (
            self.get_parameter("source_box_x").value,
            self.get_parameter("source_box_y").value,
            self.get_parameter("source_box_z").value,
        )

    @property
    def tgt_pos(self) -> tuple:
        return (
            self.get_parameter("target_box_x").value,
            self.get_parameter("target_box_y").value,
            self.get_parameter("target_box_z").value,
        )

    @property
    def orientation(self) -> tuple:
        return (
            self.get_parameter("pick_orientation_x").value,
            self.get_parameter("pick_orientation_y").value,
            self.get_parameter("pick_orientation_z").value,
            self.get_parameter("pick_orientation_w").value,
        )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()

    # Spin the parameter node in a background thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    logger = node.get_logger()

    # ------------------------------------------------------------------
    # Initialise MoveItPy — this blocks until move_group is ready
    # ------------------------------------------------------------------
    logger.info("Waiting for MoveIt2 (move_group) to start …")
    moveit = MoveItPy(node_name="moveit_py")
    logger.info("MoveItPy ready.")

    # ------------------------------------------------------------------
    # Load parameters
    # ------------------------------------------------------------------
    grid          = node.grid
    prod          = node.prod
    src_pos       = node.src_pos
    tgt_pos       = node.tgt_pos
    orientation   = node.orientation
    wall_t        = node.get_parameter("box_wall_thickness").value
    approach_h    = node.get_parameter("approach_height").value
    retreat_h     = node.get_parameter("retreat_height").value
    arm_group     = node.get_parameter("arm_group").value
    eef_link      = node.get_parameter("eef_link").value
    base_frame    = node.get_parameter("base_frame").value
    plan_time     = node.get_parameter("planning_time").value
    vel_scale     = node.get_parameter("velocity_scaling").value
    acc_scale     = node.get_parameter("acceleration_scaling").value
    max_att       = node.get_parameter("max_planning_attempts").value
    max_products  = node.get_parameter("max_products").value

    total_products = grid["cols"] * grid["rows"] * grid["layers"]
    limit = total_products if max_products < 0 else min(max_products, total_products)
    logger.info(
        f"Grid: {grid['cols']}×{grid['rows']}×{grid['layers']} = "
        f"{total_products} products. Will process {limit}."
    )

    # ------------------------------------------------------------------
    # Build scene
    # ------------------------------------------------------------------
    scene = SceneManager(moveit, node, base_frame)
    planner = MotionPlanner(
        moveit, node, arm_group, eef_link, base_frame,
        plan_time, vel_scale, acc_scale, max_att, orientation,
    )

    logger.info("Setting up planning scene …")
    scene.setup_scene(grid, prod, src_pos, tgt_pos, wall_t)
    # Brief pause so RViz can render the scene before planning starts
    time.sleep(1.0)

    # ------------------------------------------------------------------
    # Home position
    # ------------------------------------------------------------------
    logger.info("Moving to home position …")
    if not planner.move_home():
        logger.warn("Could not reach home — continuing anyway.")

    # ------------------------------------------------------------------
    # Pre-compute grid origins
    #   x0 / y0 = centre of the (col=0, row=0) product in XY
    # ------------------------------------------------------------------
    sx, sy, sz = prod["size_x"], prod["size_y"], prod["size_z"]

    src_x0 = src_pos[0] - (grid["cols"] * sx) / 2.0 + sx / 2.0
    src_y0 = src_pos[1] - (grid["rows"] * sy) / 2.0 + sy / 2.0

    tgt_x0 = tgt_pos[0] - (grid["cols"] * sx) / 2.0 + sx / 2.0
    tgt_y0 = tgt_pos[1] - (grid["rows"] * sy) / 2.0 + sy / 2.0

    # ------------------------------------------------------------------
    # Pick-and-place loop
    # Iterate top-layer first so lower products are never occluded.
    # ------------------------------------------------------------------
    count = 0
    success = 0
    failed = []

    logger.info("─" * 50)
    logger.info("Starting pick-and-place loop …")
    logger.info("─" * 50)

    outer_break = False
    for layer in reversed(range(grid["layers"])):
        if outer_break:
            break
        for row in range(grid["rows"]):
            if outer_break:
                break
            for col in range(grid["cols"]):
                if count >= limit:
                    outer_break = True
                    break

                product_id = f"product_{col}_{row}_{layer}"

                pick_x = src_x0 + col * sx
                pick_y = src_y0 + row * sy
                pick_z = src_pos[2] + layer * sz + sz / 2.0

                place_x = tgt_x0 + col * sx
                place_y = tgt_y0 + row * sy
                place_z = tgt_pos[2] + layer * sz + sz / 2.0

                logger.info(
                    f"[{count + 1}/{limit}] {product_id}  "
                    f"src=({pick_x:.3f},{pick_y:.3f},{pick_z:.3f})  "
                    f"dst=({place_x:.3f},{place_y:.3f},{place_z:.3f})"
                )

                ok = planner.pick_product(
                    pick_x, pick_y, pick_z,
                    approach_h, retreat_h,
                    product_id,
                )

                if ok:
                    planner.place_product(
                        place_x, place_y, place_z,
                        approach_h, retreat_h,
                        product_id,
                    )
                    # Remove product from source scene (already placed)
                    scene.remove_product(product_id)
                    success += 1
                    logger.info(f"  ✓ placed {product_id}")
                else:
                    failed.append(product_id)
                    logger.warn(f"  ✗ skipped {product_id}")

                count += 1

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    logger.info("─" * 50)
    logger.info(
        f"Done. Processed {count} products: {success} succeeded, "
        f"{len(failed)} failed."
    )
    if failed:
        logger.warn(f"Failed products: {failed}")

    # Return home
    logger.info("Returning to home position …")
    planner.move_home()
    logger.info("Pick-and-place node finished.")

    rclpy.shutdown()
    spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
