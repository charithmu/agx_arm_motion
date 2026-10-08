"""
Start the persistent Piper pose goal server.

Arguments
---------
  profile  Backend profile: sim (default) or real

The server listens on /piper/target_pose (geometry_msgs/PoseStamped) and
executes a MoveIt trajectory for each received goal.  Poses can be in any
TF-known frame; the server transforms to base_link automatically.

Usage
-----
  ros2 launch agx_arm_motion pose_goal_server.launch.py

  # Real hardware:
  ros2 launch agx_arm_motion pose_goal_server.launch.py profile:=real

  # Send a goal:
  ros2 topic pub --once /piper/target_pose geometry_msgs/msg/PoseStamped \\
    "{header: {frame_id: base_link},
      pose: {position: {x: 0.3, y: 0.0, z: 0.25},
             orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"

  # Monitor status:
  ros2 topic echo /piper/motion_status
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from agx_arm_motion.launch_utils import build_moveit_config, use_sim_time


def launch_setup(context, *args, **kwargs):
    profile = LaunchConfiguration("profile").perform(context)
    moveit_config = build_moveit_config(profile)

    node = Node(
        package="agx_arm_motion",
        executable="pose_goal_server",
        name="moveit_py",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time(profile)},
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
        OpaqueFunction(function=launch_setup),
    ])
