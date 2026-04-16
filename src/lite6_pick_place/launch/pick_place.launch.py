#!/usr/bin/env python3
"""
pick_place.launch.py
====================
Launches the full Lite6 pick-and-place simulation:

  1. lite6_moveit_fake  – Lite6 robot + MoveIt2 + RViz (fake hardware)
  2. pick_place_node    – our pick-and-place orchestrator

Usage
-----
  ros2 launch lite6_pick_place pick_place.launch.py

Optional overrides:
  ros2 launch lite6_pick_place pick_place.launch.py \
      params_file:=/ws/config/params.yaml \
      max_products:=10
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ------------------------------------------------------------------
    # Declare overridable arguments
    # ------------------------------------------------------------------
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("lite6_pick_place"), "config", "params.yaml"
        ]),
        description="Path to the pick-and-place parameter file",
    )

    add_gripper_arg = DeclareLaunchArgument(
        "add_gripper",
        default_value="true",
        description="Attach gripper mesh to Lite6 model",
    )

    # ------------------------------------------------------------------
    # Lite6 MoveIt2 fake-hardware simulation
    # Starts: move_group, joint_state_broadcaster, RViz
    # ------------------------------------------------------------------
    lite6_moveit_fake = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("xarm_moveit_config"),
                "launch",
                "lite6_moveit_fake.launch.py",
            ])
        ),
        launch_arguments={
            "add_gripper": LaunchConfiguration("add_gripper"),
            # Keep RViz open so the user can watch the motion
            "no_gui_ctrl": "false",
        }.items(),
    )

    # ------------------------------------------------------------------
    # Pick-and-place node
    # Delayed by 8 s to let move_group finish initialising.
    # MoveItPy also waits internally, so this is just a safety margin.
    # ------------------------------------------------------------------
    pick_place_node = Node(
        package="lite6_pick_place",
        executable="pick_place_node",
        name="pick_place_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    delayed_pick_place = TimerAction(
        period=8.0,
        actions=[pick_place_node],
    )

    return LaunchDescription([
        params_file_arg,
        add_gripper_arg,
        lite6_moveit_fake,
        delayed_pick_place,
    ])
