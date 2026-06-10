# Copyright 2026 Ozkan Ceylan
# SPDX-License-Identifier: Apache-2.0
"""Headless SIL smoke test (Phase 2).

Launches the sim headless and asserts the bridge publishes its core topics
(/clock, /odom, /imu, /scan) within a budget. No wall-clock sleeps: we spin and
check, exiting as soon as all topics are seen (Hard Rule #3). The wall deadline
is only a safety timeout. The full goal-reach scenario is Phase 3.
"""

import time
import unittest

import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("otonav_bringup"), "/launch/sim.launch.py"]
        ),
        launch_arguments={"gui": "false"}.items(),
    )
    return LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class TestSilSmoke(unittest.TestCase):
    def test_core_topics_publish(self):
        rclpy.init()
        node = rclpy.create_node("sil_smoke")
        try:
            counts = {"clock": 0, "odom": 0, "imu": 0, "scan": 0}
            node.create_subscription(
                Clock, "/clock", lambda _m: counts.__setitem__("clock", counts["clock"] + 1), 10
            )
            node.create_subscription(
                Odometry, "odom", lambda _m: counts.__setitem__("odom", counts["odom"] + 1), 10
            )
            # /imu and /scan publish with SensorDataQoS (best-effort); match it
            # or a reliable reader silently receives nothing.
            node.create_subscription(
                Imu,
                "imu",
                lambda _m: counts.__setitem__("imu", counts["imu"] + 1),
                qos_profile_sensor_data,
            )
            node.create_subscription(
                LaserScan,
                "scan",
                lambda _m: counts.__setitem__("scan", counts["scan"] + 1),
                qos_profile_sensor_data,
            )
            # Nudge the robot so /odom is exercised, not just idle.
            cmd_pub = node.create_publisher(Twist, "cmd_vel", 10)
            fwd = Twist()
            fwd.linear.x = 0.3

            deadline = time.time() + 45.0
            while time.time() < deadline and not all(c > 0 for c in counts.values()):
                cmd_pub.publish(fwd)
                rclpy.spin_once(node, timeout_sec=0.2)

            missing = [name for name, c in counts.items() if c == 0]
            self.assertEqual(missing, [], f"topics not publishing: {missing} (counts={counts})")
        finally:
            node.destroy_node()
            rclpy.shutdown()
