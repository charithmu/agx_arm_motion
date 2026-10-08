"""Shared MoveIt config builder for agx_arm_motion launch files.

Centralises ``MoveItConfigsBuilder`` setup so all launch files stay in sync
when the backend configuration changes.

Supported profiles
------------------
``sim``
    Gazebo Harmonic simulation via ``agx_arm_gzsim``.
    Uses ``agx_arm_moveit``'s standard URDF/SRDF xacro (robot name
    ``"agx_arm"``) with piper + agx_gripper mappings, matching exactly what
    the interactive ``piper_with_gripper_moveit_gzsim.launch.py`` does.
    Trajectory execution is overridden to the sim controller config.
    ``use_sim_time`` should be ``True``.

``real``
    Real hardware via the standard ``agx_arm_moveit`` config.
    Same URDF/SRDF, hardware controller config.
    ``use_sim_time`` should be ``False``.

Usage in a launch file::

    from launch.actions import OpaqueFunction
    from launch.substitutions import LaunchConfiguration
    from agx_arm_motion.launch_utils import build_moveit_config, use_sim_time

    def launch_setup(context, *args, **kwargs):
        profile = LaunchConfiguration("profile").perform(context)
        moveit_config = build_moveit_config(profile)
        ...
"""

import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

_DEFAULT_TCP_OFFSET_XYZ = "0 0 0.1358"
_DEFAULT_TCP_OFFSET_RPY = "0 0 0"

# Fixed robot configuration — piper arm with AGX gripper.
# These mappings mirror what piper_with_gripper_moveit_gzsim.launch.py uses
# so MoveItPy loads the identical URDF/SRDF that move_group does.
_URDF_MAPPINGS = {
    "arm_type": "piper",
    "effector_type": "agx_gripper",
    "revo2_type": "left",
    "tcp_offset_xyz": _DEFAULT_TCP_OFFSET_XYZ,
    "tcp_offset_rpy": _DEFAULT_TCP_OFFSET_RPY,
}
_SRDF_MAPPINGS = {
    "arm_type": "piper",
    "effector_type": "agx_gripper",
    "revo2_type": "left",
}


def build_moveit_config(profile: str = "sim"):
    """Return a ``MoveItConfigs`` object for the given backend profile.

    Raises ``ValueError`` for unknown profile names.
    """
    pkg_motion = get_package_share_directory("agx_arm_motion")
    pkg_moveit = get_package_share_directory("agx_arm_moveit")
    moveit_cpp_yaml = os.path.join(pkg_motion, "config", "moveit_cpp.yaml")

    if profile == "sim":
        pkg_gzsim = get_package_share_directory("agx_arm_gzsim")
        return (
            MoveItConfigsBuilder("agx_arm", package_name="agx_arm_moveit")
            # agx_arm.urdf.xacro has robot name="agx_arm", matching the SRDF.
            # Do NOT use piper_with_gripper_gzsim.urdf.xacro here — that URDF
            # has robot name="piper" and is only for Gazebo/RSP, not for MoveIt.
            .robot_description(
                file_path="config/agx_arm.urdf.xacro",
                mappings=_URDF_MAPPINGS,
            )
            # agx_arm.srdf.xacro contains all disable_collisions entries for
            # gripper/arm links. The plain agx_arm.srdf has none of them.
            .robot_description_semantic(
                file_path="config/agx_arm.srdf.xacro",
                mappings=_SRDF_MAPPINGS,
            )
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .trajectory_execution(
                file_path=os.path.join(pkg_gzsim, "config", "moveit_controllers.yaml")
            )
            .moveit_cpp(file_path=moveit_cpp_yaml)
            .to_moveit_configs()
        )

    if profile == "real":
        return (
            MoveItConfigsBuilder("agx_arm", package_name="agx_arm_moveit")
            .robot_description(
                file_path="config/agx_arm.urdf.xacro",
                mappings=_URDF_MAPPINGS,
            )
            .robot_description_semantic(
                file_path="config/agx_arm.srdf.xacro",
                mappings=_SRDF_MAPPINGS,
            )
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .trajectory_execution(
                file_path=os.path.join(
                    pkg_moveit, "config", "moveit_controllers_gripper.yaml"
                )
            )
            .moveit_cpp(file_path=moveit_cpp_yaml)
            .to_moveit_configs()
        )

    raise ValueError(f"Unknown profile '{profile}'. Valid choices: sim, real.")


def use_sim_time(profile: str) -> bool:
    """Return ``True`` when the profile requires sim time."""
    return profile == "sim"
