"""
Start the persistent Piper pose goal server.

The server listens on /piper/target_pose (geometry_msgs/PoseStamped) and
executes a MoveIt trajectory for each received goal.  Poses can be in any
TF-known frame; the server transforms them to base_link automatically.

Usage
-----
  ros2 launch agx_arm_motion pose_goal_server.launch.py

  # Then send goals from any node or the CLI:
  ros2 topic pub --once /piper/target_pose geometry_msgs/msg/PoseStamped \\
    "{header: {frame_id: base_link},
      pose: {position: {x: 0.3, y: 0.0, z: 0.25},
             orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}}"

  # Monitor status:
  ros2 topic echo /piper/motion_status
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    pkg_gzsim = get_package_share_directory("agx_arm_gzsim")

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

    return LaunchDescription([
        Node(
            package="agx_arm_motion",
            executable="pose_goal_server",
            name="moveit_py",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": True},
            ],
            output="screen",
        ),
    ])
