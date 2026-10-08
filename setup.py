from setuptools import setup, find_packages
import os
from glob import glob

package_name = "agx_arm_motion"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Charith Munasinghe",
    maintainer_email="mung@zhaw.ch",
    description="MoveIt2 convenience layer for Piper Studio — pose executors and goal server.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "move_to_pose = agx_arm_motion.move_to_pose:main",
            "pose_goal_server = agx_arm_motion.pose_goal_server:main",
            "move_to_named_pose = agx_arm_motion.move_to_named_pose:main",
        ],
    },
)
