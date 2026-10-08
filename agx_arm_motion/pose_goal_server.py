"""Persistent pose goal server.

Subscribes to ``/piper/target_pose`` (geometry_msgs/PoseStamped) and executes
a MoveIt trajectory for each received goal.  Goals received while a motion is
in progress are dropped with a warning.

Status is published on ``/piper/motion_status`` (std_msgs/String):

  ``IDLE`` → ``PLANNING`` → ``EXECUTING`` → ``SUCCESS`` (or ``FAILED``)

Poses may be expressed in any TF-known frame; the server transforms to
``base_link`` automatically.
"""

import threading

import rclpy
import rclpy.duration
import rclpy.node
import rclpy.time
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_pose_stamped

from agx_arm_motion.motion_facade import MotionFacade

_TARGET_TOPIC = "/piper/target_pose"
_STATUS_TOPIC = "/piper/motion_status"
_BASE_FRAME = "base_link"
_TF_TIMEOUT_SEC = 5.0


class PoseGoalServer(rclpy.node.Node):
    """ROS 2 node that executes incoming pose goals one at a time."""

    def __init__(self) -> None:
        # Node name differs from "moveit_py" so the two nodes are distinct.
        # Absolute topic names (/piper/...) are unaffected by the node name.
        super().__init__("pose_goal_server")

        self._facade = MotionFacade(node_name="moveit_py")
        self._status_pub = self.create_publisher(String, _STATUS_TOPIC, 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._busy = threading.Event()

        self._sub = self.create_subscription(
            PoseStamped, _TARGET_TOPIC, self._goal_cb, 10
        )
        self._publish_status("IDLE")
        self.get_logger().info(
            f"Ready. Publish geometry_msgs/PoseStamped to {_TARGET_TOPIC}."
        )

    # ------------------------------------------------------------------

    def _goal_cb(self, msg: PoseStamped) -> None:
        if self._busy.is_set():
            self.get_logger().warn("Goal received while busy — dropping.")
            return
        self._busy.set()
        threading.Thread(target=self._execute, args=(msg,), daemon=True).start()

    def _execute(self, pose_stamped: PoseStamped) -> None:
        try:
            self._publish_status("PLANNING")

            # Transform to base_link if the goal is in a different frame.
            if pose_stamped.header.frame_id != _BASE_FRAME:
                try:
                    transform = self._tf_buffer.lookup_transform(
                        _BASE_FRAME,
                        pose_stamped.header.frame_id,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=_TF_TIMEOUT_SEC),
                    )
                    pose_stamped = do_transform_pose_stamped(pose_stamped, transform)
                except Exception as exc:
                    self.get_logger().error(f"TF2 lookup failed: {exc}")
                    self._publish_status("FAILED")
                    return

            success = self._facade.move_to_pose(
                pose_stamped,
                on_executing=lambda: self._publish_status("EXECUTING"),
            )
            self._publish_status("SUCCESS" if success else "FAILED")

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Execution error: {exc}")
            self._publish_status("FAILED")
        finally:
            self._busy.clear()
            self._publish_status("IDLE")

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    node = PoseGoalServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._facade.shutdown()
        node.destroy_node()
        rclpy.shutdown()
