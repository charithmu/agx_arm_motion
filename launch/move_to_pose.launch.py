"""
Move the Piper TCP to a target pose (one-shot).

Arguments
---------
  x        Target X position in metres         (default: 0.25)
  y        Target Y position in metres         (default: 0.0)
  z        Target Z position in metres         (default: 0.30)
  roll     Roll  in radians (rotation about X) (default: 0.0)
  pitch    Pitch in radians (rotation about Y) (default: 1.5708  ≈ π/2)
  yaw      Yaw   in radians (rotation about Z) (default: 0.0)
  frame_id Reference frame for the pose        (default: base_link)

Examples
--------
  # Top-down approach directly in front of the arm:
  ros2 launch agx_arm_motion move_to_pose.launch.py x:=0.3 z:=0.25

  # Horizontal reach to the left:
  ros2 launch agx_arm_motion move_to_pose.launch.py x:=0.25 y:=0.15 z:=0.20 pitch:=0.0

  # Goal expressed in a custom bench frame:
  ros2 launch agx_arm_motion move_to_pose.launch.py x:=0.1 z:=0.05 frame_id:=bench
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    pkg_gzsim = get_package_share_directory("agx_arm_gzsim")

    # Load MoveIt config (same override as in the simulation launch).
    # MoveItPy needs robot_description_semantic, kinematics, planning pipelines, etc.
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit")
        .robot_description(
            file_path=os.path.join(pkg_gzsim, "urdf", "piper_with_gripper_gzsim.urdf.xacro"),
            mappings={
                "initial_positions_file": os.path.join(
                    pkg_gzsim, "config", "initial_positions.yaml"
                )
            },
        )
        .trajectory_execution(
            file_path=os.path.join(pkg_gzsim, "config", "moveit_controllers.yaml")
        )
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("agx_arm_motion"), "config", "moveit_cpp.yaml"
            )
        )
        .to_moveit_configs()
    )

    # (name, default_as_string, description)
    float_args = [
        ("x",     "0.25",   "TCP target X position in metres"),
        ("y",     "0.0",    "TCP target Y position in metres"),
        ("z",     "0.30",   "TCP target Z position in metres"),
        ("roll",  "0.0",    "Roll  in radians (rotation about X-axis)"),
        ("pitch", "1.5708", "Pitch in radians (rotation about Y-axis); π/2 = top-down approach"),
        ("yaw",   "0.0",    "Yaw   in radians (rotation about Z-axis)"),
    ]
    str_args = [
        ("frame_id", "base_link", "TF frame in which the target pose is expressed"),
    ]

    declared = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in float_args + str_args
    ]

    # Pose parameters
    pose_params: dict = {
        name: ParameterValue(LaunchConfiguration(name), value_type=float)
        for name, _, _ in float_args
    }
    pose_params.update({name: LaunchConfiguration(name) for name, _, _ in str_args})

    node = Node(
        package="agx_arm_motion",
        executable="move_to_pose",
        name="moveit_py",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            pose_params,
        ],
        output="screen",
    )

    return LaunchDescription(declared + [node])
