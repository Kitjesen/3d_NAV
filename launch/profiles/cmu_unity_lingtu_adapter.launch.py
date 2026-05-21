"""Launch the CMU Unity <-> LingTu simulation topic adapter."""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _adapter_env(repo_root: str) -> dict[str, str]:
    src_root = os.path.join(repo_root, "src")
    existing = os.environ.get("PYTHONPATH", "")
    paths = [repo_root, src_root]
    if existing:
        paths.append(existing)
    return {"PYTHONPATH": os.pathsep.join(paths)}


def _launch_adapter(context, *_args, **_kwargs):
    relay_cmd_vel = (
        LaunchConfiguration("relay_cmd_vel_to_sim").perform(context).lower() == "true"
    )
    allow_default_domain = (
        LaunchConfiguration("allow_default_ros_domain").perform(context).lower() == "true"
    )
    required_only = LaunchConfiguration("required_only").perform(context).lower() == "true"
    cmd_vel_frame_id = LaunchConfiguration("cmd_vel_frame_id").perform(context)

    cmd = [
        "python3",
        "-m",
        "sim.engine.bridge.cmu_unity_lingtu_adapter",
        "--cmd-vel-frame-id",
        cmd_vel_frame_id,
    ]
    if relay_cmd_vel:
        cmd.append("--relay-cmd-vel-to-sim")
    if allow_default_domain:
        cmd.append("--allow-default-ros-domain")
    if required_only:
        cmd.append("--required-only")

    return [
        ExecuteProcess(
            cmd=cmd,
            cwd=_REPO_ROOT,
            additional_env=_adapter_env(_REPO_ROOT),
            name="lingtu_cmu_unity_adapter",
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "relay_cmd_vel_to_sim",
                default_value="false",
                description=(
                    "Relay LingTu /nav/cmd_vel to CMU /cmd_vel. Use only in "
                    "an isolated simulation ROS_DOMAIN_ID."
                ),
            ),
            DeclareLaunchArgument(
                "allow_default_ros_domain",
                default_value="false",
                description="Allow command relay on unset/0 ROS_DOMAIN_ID.",
            ),
            DeclareLaunchArgument(
                "required_only",
                default_value="false",
                description="Relay only required integration topics.",
            ),
            DeclareLaunchArgument("cmd_vel_frame_id", default_value="vehicle"),
            OpaqueFunction(function=_launch_adapter),
        ]
    )
