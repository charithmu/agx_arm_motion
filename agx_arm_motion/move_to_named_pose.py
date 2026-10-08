"""One-shot node: look up a named pose from example_poses.yaml and execute.

Parameter:

  pose_name – Key in ``config/example_poses.yaml``
              (e.g. ``home``, ``ready``, ``bench_center_above``,
              ``bench_center_pick``, ``front_horizontal``, ``left_side``)

YAML entries may specify either:

* ``srdf_state`` — delegates to joint-space planning via the SRDF named
  configuration (preferred for robot-defined poses such as ``home``).
* ``position`` + ``orientation_rpy`` + ``frame_id`` — Cartesian IK planning.

Exits with code 0 on success, 1 on failure.
"""

import os
import sys

import numpy as np
import rclpy
import rclpy.node
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped

from agx_arm_motion.motion_facade import MotionFacade


def _rpy_to_quat(roll: float, pitch: float, yaw: float):
    """Convert roll/pitch/yaw (intrinsic XYZ) to quaternion (x, y, z, w)."""
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def main():
    rclpy.init()

    node = rclpy.node.Node("moveit_py")
    node.declare_parameter("pose_name", "ready")
    pose_name = node.get_parameter("pose_name").get_parameter_value().string_value

    # Load named poses.
    poses_file = (
        get_package_share_directory("agx_arm_motion") + "/config/example_poses.yaml"
    )
    with open(poses_file) as fh:
        data = yaml.safe_load(fh)

    poses = data.get("poses", {})
    if pose_name not in poses:
        node.get_logger().error(
            f"Unknown pose_name '{pose_name}'. Available: {list(poses.keys())}"
        )
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    entry = poses[pose_name]
    facade = MotionFacade(node_name="moveit_py")

    # --- Joint-space path (SRDF named state) ---
    if "srdf_state" in entry:
        srdf_state = entry["srdf_state"]
        node.get_logger().info(
            f"Named pose '{pose_name}' → SRDF state '{srdf_state}' (joint-space)"
        )
        success = facade.move_to_named_state(srdf_state)

    # --- Cartesian path ---
    else:
        pos = entry["position"]
        rpy = entry["orientation_rpy"]
        frame_id = entry.get("frame_id", "base_link")

        node.get_logger().info(
            f"Named pose '{pose_name}': pos={pos}  rpy={rpy}  frame={frame_id}"
        )

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(pos[0])
        pose.pose.position.y = float(pos[1])
        pose.pose.position.z = float(pos[2])
        qx, qy, qz, qw = _rpy_to_quat(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        success = facade.move_to_pose(pose)

    rc = 0 if success else 1
    # Use os._exit to skip MoveItPy C++ destructor which races with Python
    # cleanup and causes SIGSEGV on exit.
    os._exit(rc)
