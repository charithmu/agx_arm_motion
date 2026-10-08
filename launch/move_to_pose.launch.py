"""
Move the Piper TCP to a target pose (one-shot).

Arguments
---------
  profile  Backend profile: sim (default) or real
  x        Target X position in metres         (default: 0.25)
  y        Target Y position in metres         (default: 0.0)
  z        Target Z position in metres         (default: 0.30)
  roll     Roll  in radians (rotation about X) (default: 0.0)
  pitch    Pitch in radians (rotation about Y) (default: 1.5708 ≈ π/2)
  yaw      Yaw   in radians (rotation about Z) (default: 0.0)
  frame_id Reference TF frame for the pose     (default: base_link)

Examples
--------
  # Top-down approach directly in front of the arm (sim):
  ros2 launch agx_arm_motion move_to_pose.launch.py x:=0.3 z:=0.25

  # Horizontal reach to the left:
  ros2 launch agx_arm_motion move_to_pose.launch.py x:=0.25 y:=0.15 z:=0.20 pitch:=0.0

  # Real hardware:
  ros2 launch agx_arm_motion move_to_pose.launch.py profile:=real x:=0.3 z:=0.25

  # Goal in a custom TF frame:
  ros2 launch agx_arm_motion move_to_pose.launch.py x:=0.1 z:=0.05 frame_id:=bench
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

from agx_arm_motion.launch_utils import build_moveit_config, use_sim_time


def launch_setup(context, *args, **kwargs):
    profile = LaunchConfiguration("profile").perform(context)
    moveit_config = build_moveit_config(profile)

    float_params = {
        name: ParameterValue(LaunchConfiguration(name), value_type=float)
        for name in ("x", "y", "z", "roll", "pitch", "yaw")
    }
    float_params["frame_id"] = LaunchConfiguration("frame_id")

    node = Node(
        package="agx_arm_motion",
        executable="move_to_pose",
        name="moveit_py",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time(profile)},
            float_params,
        ],
        output="screen",
    )
    return [node]


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            "profile",
            default_value="sim",
            choices=["sim", "real"],
            description="Backend profile: sim (Gazebo) or real (hardware)",
        ),
        DeclareLaunchArgument("x", default_value="0.25",
                              description="TCP target X in metres"),
        DeclareLaunchArgument("y", default_value="0.0",
                              description="TCP target Y in metres"),
        DeclareLaunchArgument("z", default_value="0.30",
                              description="TCP target Z in metres"),
        DeclareLaunchArgument("roll", default_value="0.0",
                              description="Roll in radians (about X)"),
        DeclareLaunchArgument("pitch", default_value="1.5708",
                              description="Pitch in radians (about Y); π/2 = top-down"),
        DeclareLaunchArgument("yaw", default_value="0.0",
                              description="Yaw in radians (about Z)"),
        DeclareLaunchArgument("frame_id", default_value="base_link",
                              description="TF frame of the target pose"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
