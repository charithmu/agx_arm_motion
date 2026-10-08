"""One-shot node: plan and execute a single Cartesian goal then exit.

Parameters (declared in the launch file and passed to the ``moveit_py`` node
namespace):

  x, y, z       – TCP target position in base_link (metres)
  roll, pitch, yaw  – TCP orientation (radians, intrinsic XYZ)
  frame_id      – TF frame of the target pose (default: base_link)

If ``frame_id`` differs from ``base_link`` the node performs a TF2 lookup
before planning.  Exits with code 0 on success, 1 on failure.
"""

import os
import sys
import threading

import numpy as np
import rclpy
import rclpy.duration
import rclpy.executors
import rclpy.node
import rclpy.time
import tf2_ros
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose_stamped

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

    # Read pose parameters from the "moveit_py" namespace (matches the launch
    # file's Node name so params are forwarded correctly).
    node = rclpy.node.Node("moveit_py")
    node.declare_parameter("x", 0.25)
    node.declare_parameter("y", 0.0)
    node.declare_parameter("z", 0.30)
    node.declare_parameter("roll", 0.0)
    node.declare_parameter("pitch", 1.5708)
    node.declare_parameter("yaw", 0.0)
    node.declare_parameter("frame_id", "base_link")

    x = node.get_parameter("x").get_parameter_value().double_value
    y = node.get_parameter("y").get_parameter_value().double_value
    z = node.get_parameter("z").get_parameter_value().double_value
    roll = node.get_parameter("roll").get_parameter_value().double_value
    pitch = node.get_parameter("pitch").get_parameter_value().double_value
    yaw = node.get_parameter("yaw").get_parameter_value().double_value
    frame_id = node.get_parameter("frame_id").get_parameter_value().string_value

    node.get_logger().info(
        f"Target TCP: pos=({x:.3f}, {y:.3f}, {z:.3f}) m  "
        f"rpy=({roll:.3f}, {pitch:.3f}, {yaw:.3f}) rad  frame={frame_id}"
    )

    # Build the target pose.
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    qx, qy, qz, qw = _rpy_to_quat(roll, pitch, yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    # Transform to base_link when the goal is expressed in a different frame.
    if frame_id != "base_link":
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, node)  # noqa: F841

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        try:
            transform = tf_buffer.lookup_transform(
                "base_link",
                frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=5.0),
            )
            pose = do_transform_pose_stamped(pose, transform)
        except Exception as exc:
            node.get_logger().error(f"TF2 lookup failed: {exc}")
            executor.shutdown(timeout_sec=0)
            os._exit(1)
        finally:
            executor.shutdown(timeout_sec=0)

    # Plan and execute.
    facade = MotionFacade(node_name="moveit_py")
    success = facade.move_to_pose(pose)

    # Use os._exit to skip MoveItPy C++ destructor which races with Python
    # cleanup and causes SIGSEGV on exit.
    os._exit(0 if success else 1)
