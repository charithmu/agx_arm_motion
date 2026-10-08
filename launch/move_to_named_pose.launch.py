"""
Move the Piper TCP to a named pose from config/example_poses.yaml (one-shot).

Arguments
---------
  profile    Backend profile: sim (default) or real
  pose_name  Key in example_poses.yaml
             (home | ready | bench_center_above | bench_center_pick |
              front_horizontal | left_side)

  ``home`` uses joint-space planning via the SRDF named configuration
  (all joints at zero).  All other poses use Cartesian IK planning.

Examples
--------
  ros2 launch agx_arm_motion move_to_named_pose.launch.py pose_name:=home

  ros2 launch agx_arm_motion move_to_named_pose.launch.py pose_name:=ready

  ros2 launch agx_arm_motion move_to_named_pose.launch.py \\
      profile:=real pose_name:=bench_center_above
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from agx_arm_motion.launch_utils import build_moveit_config, use_sim_time


def launch_setup(context, *args, **kwargs):
    profile = LaunchConfiguration("profile").perform(context)
    pose_name = LaunchConfiguration("pose_name").perform(context)
    moveit_config = build_moveit_config(profile)

    node = Node(
        package="agx_arm_motion",
        executable="move_to_named_pose",
        name="moveit_py",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time(profile)},
            {"pose_name": pose_name},
        ],
        output="screen",
    )
    return [node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "profile",
            default_value="sim",
            choices=["sim", "real"],
            description="Backend profile: sim (Gazebo) or real (hardware)",
        ),
        DeclareLaunchArgument(
            "pose_name",
            default_value="ready",
            description=(
                "Named pose from example_poses.yaml: "
                "home | ready | bench_center_above | bench_center_pick | "
                "front_horizontal | left_side"
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
