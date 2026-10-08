"""MoveItPy facade for agx_arm_motion.

Wraps MoveIt2 arm planning and execution behind a minimal interface so
higher-level packages never touch MoveItPy directly.

Usage::

    import rclpy
    from agx_arm_motion import MotionFacade

    rclpy.init()
    facade = MotionFacade()
    success = facade.move_to_pose(pose_stamped)   # pose must be in base_link
    facade.shutdown()
    rclpy.shutdown()

The caller is responsible for calling ``rclpy.init()`` before constructing
this object.  MoveIt configuration (URDF, SRDF, kinematics, pipelines) is
read from ROS parameters already loaded by the launch system under the
``moveit_py`` node name.
"""

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from rclpy.logging import get_logger


class MotionFacade:
    """Thin wrapper around MoveItPy for Piper arm planning and execution.

    Only the arm planning group is exposed here.  Gripper actuation is
    delegated to the hardware controller layer (``agx_arm_ctrl``).
    """

    ARM_GROUP = "arm"
    TCP_LINK = "tcp_link"

    def __init__(self, node_name: str = "moveit_py") -> None:
        self._logger = get_logger(node_name)
        self._moveit = MoveItPy(node_name=node_name)
        self._arm = self._moveit.get_planning_component(self.ARM_GROUP)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def move_to_pose(self, pose_stamped: PoseStamped, on_executing=None) -> bool:
        """Plan and execute a Cartesian end-effector goal.

        The pose must be expressed in ``base_link``.  Use TF2 to transform
        to ``base_link`` before calling this method when needed.

        ``on_executing`` is an optional zero-argument callable invoked between
        planning and execution (e.g. to emit a status update).

        Returns ``True`` on success, ``False`` if planning or execution fails.
        """
        pos = pose_stamped.pose.position
        ori = pose_stamped.pose.orientation
        self._logger.info(
            f"Target TCP: pos=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) m  "
            f"ori=({ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f})  "
            f"frame={pose_stamped.header.frame_id}"
        )
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(
            pose_stamped_msg=pose_stamped, pose_link=self.TCP_LINK
        )
        self._logger.info("Planning...")
        plan_result = self._arm.plan()
        if not plan_result:
            self._logger.error("Planning failed.")
            return False

        if on_executing is not None:
            on_executing()
        self._logger.info("Executing...")
        self._moveit.execute(plan_result.trajectory, controllers=[])
        self._logger.info("Done.")
        return True

    def move_to_named_state(self, state_name: str) -> bool:
        """Move to a named joint configuration defined in the SRDF.

        The base SRDF defines ``home`` (all joints at zero).

        Returns ``True`` on success.
        """
        self._logger.info(f"Moving to named state: '{state_name}'")
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(configuration_name=state_name)
        self._logger.info("Planning...")
        plan_result = self._arm.plan()
        if not plan_result:
            self._logger.error(f"Planning to '{state_name}' failed.")
            return False

        self._logger.info("Executing...")
        self._moveit.execute(plan_result.trajectory, controllers=[])
        self._logger.info("Done.")
        return True

    def shutdown(self) -> None:
        """Shut down the MoveItPy instance."""
        self._moveit.shutdown()
