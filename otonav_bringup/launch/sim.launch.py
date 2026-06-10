"""Bring up the OtoNav MuJoCo SIL bridge.

Headless by default (gui:=false); the viewer is opt-in and never a precondition
for physics or tests (Hard Rule #7). All non-bridge nodes consume sim time
(use_sim_time:=true, ADR-2); the bridge itself is the /clock source.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    gui = LaunchConfiguration("gui")
    backend = LaunchConfiguration("backend")

    params_file = PathJoinSubstitution(
        [FindPackageShare("otonav_bringup"), "config", "bridge_params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="Open the MuJoCo viewer (local dev only; headless build is a no-op).",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Consumer nodes follow /clock from the bridge (ADR-2).",
            ),
            DeclareLaunchArgument(
                "backend",
                default_value="mujoco",
                description="Hardware backend: 'mujoco' or 'canbus'.",
            ),
            Node(
                package="otonav_mujoco_bridge",
                executable="otonav_bridge_node",
                name="otonav_bridge",
                output="screen",
                parameters=[
                    params_file,
                    # The bridge produces /clock, so it must not follow it.
                    {"use_sim_time": False},
                    {"backend": backend},
                    {"enable_viewer": gui},
                ],
            ),
            # Consumer nodes (nav, teleop) arrive in later phases and will be
            # launched with use_sim_time:=<the declared arg> to follow /clock.
        ]
    )
