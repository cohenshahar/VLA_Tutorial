from setuptools import setup
import os
from glob import glob

package_name = "arm_teleop"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("../models/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Shahar Cohen",
    maintainer_email="cohenshahar17@gmail.com",
    description="Mini teleop project for KUKA KR6 in MuJoCo (learning).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mujoco_sim_node = arm_teleop.mujoco_sim_node:main",
            "keyboard_teleop_node = arm_teleop.keyboard_teleop_node:main",
            "cartesian_controller_node = arm_teleop.cartesian_controller_node:main",
            "gripper_node = arm_teleop.gripper_node:main",
        ],
    },
)
