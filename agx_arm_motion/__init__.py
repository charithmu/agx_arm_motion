"""agx_arm_motion — MoveIt2 convenience layer for Piper Studio.

Higher-level packages (agx_arm_manipulation, etc.) can import the facade
directly::

    from agx_arm_motion import MotionFacade
"""

from agx_arm_motion.motion_facade import MotionFacade

__all__ = ["MotionFacade"]
