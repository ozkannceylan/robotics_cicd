# Copyright 2026 Ozkan Ceylan
# SPDX-License-Identifier: Apache-2.0
"""Goal-reach SIL integration scenario (Phase 3).

Headless sim (robot + offset obstacle) + go-to-goal nav. Asserts, on sim time:
the robot reaches the goal within the budget AND never comes within the clearance
bound of the obstacle. No wall-clock sleeps — we spin and check (Hard Rule #3);
the wall deadline is only a safety net. A rosbag is recorded for debugging and
uploaded by CI on failure.
"""

import math
import os
import time
import unittest

import launch_testing
import launch_testing.actions
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

GOAL_X, GOAL_Y = 3.0, 0.0
OBSTACLE_X, OBSTACLE_Y = 1.5, 0.5
CLEARANCE_MIN = 0.3   # robot centre to obstacle centre [m]
GOAL_TOLERANCE = 0.15  # [m]
SIM_BUDGET_S = 20.0
BAG_DIR = os.environ.get("OTONAV_BAG_DIR", "/tmp/otonav_goal_reach_bag")


@pytest.mark.launch_test
def generate_test_description():
    model = PathJoinSubstitution(
        [FindPackageShare("otonav_description"), "model", "scene_goal.xml"]
    )
    bridge = Node(
        package="otonav_mujoco_bridge",
        executable="otonav_bridge_node",
        name="otonav_bridge",
        parameters=[
            {"use_sim_time": False},
            {"model_path": model},
            {"backend": "mujoco"},
            {"wheel_radius": 0.05},
            {"track_width": 0.30},
            {"publish_rate": 100.0},
            {"lidar_beams": 13},
        ],
    )
    nav = Node(
        package="otonav_nav",
        executable="otonav_nav_node",
        name="otonav_nav",
        parameters=[
            {"use_sim_time": True},
            {"use_initial_goal": True},
            {"goal_x": GOAL_X},
            {"goal_y": GOAL_Y},
            {"obstacle_stop_distance": 0.4},
            {"front_half_angle": 0.35},
        ],
    )
    bag = ExecuteProcess(
        cmd=["bash", "-c", f"rm -rf {BAG_DIR}; exec ros2 bag record -o {BAG_DIR} "
             "/clock /odom /scan /cmd_vel"],
        output="log",
    )
    return LaunchDescription([bridge, nav, bag, launch_testing.actions.ReadyToTest()])


class TestGoalReach(unittest.TestCase):
    def test_reaches_goal_with_clearance(self):
        rclpy.init()
        node = rclpy.create_node("goal_reach_probe")
        try:
            st = {"x": 0.0, "y": 0.0, "have": False, "sim": 0.0,
                  "min_clear": 9e9, "reached_sim": None}

            def on_odom(m):
                st["x"] = m.pose.pose.position.x
                st["y"] = m.pose.pose.position.y
                st["have"] = True
                st["min_clear"] = min(
                    st["min_clear"], math.hypot(st["x"] - OBSTACLE_X, st["y"] - OBSTACLE_Y)
                )
                if (
                    st["reached_sim"] is None
                    and math.hypot(GOAL_X - st["x"], GOAL_Y - st["y"]) < GOAL_TOLERANCE
                ):
                    st["reached_sim"] = st["sim"]

            def on_clock(m):
                st["sim"] = m.clock.sec + m.clock.nanosec * 1e-9

            node.create_subscription(Odometry, "odom", on_odom, 10)
            node.create_subscription(Clock, "/clock", on_clock, 10)

            wall_deadline = time.time() + 120.0
            while time.time() < wall_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if st["reached_sim"] is not None:
                    break
                if st["sim"] > SIM_BUDGET_S + 10.0:  # sim clearly overran the budget
                    break

            self.assertTrue(st["have"], "no /odom received — sim did not start")
            self.assertIsNotNone(
                st["reached_sim"],
                f"goal not reached: pos=({st['x']:.2f},{st['y']:.2f}) sim={st['sim']:.1f}s",
            )
            self.assertLessEqual(
                st["reached_sim"], SIM_BUDGET_S,
                f"reached too late: {st['reached_sim']:.1f}s sim > {SIM_BUDGET_S}s",
            )
            self.assertGreaterEqual(
                st["min_clear"], CLEARANCE_MIN,
                f"obstacle clearance {st['min_clear']:.2f}m < {CLEARANCE_MIN}m",
            )
        finally:
            node.destroy_node()
            rclpy.shutdown()
