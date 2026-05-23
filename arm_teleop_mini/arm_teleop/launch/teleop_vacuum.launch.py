"""Launch the arm + vacuum end-effector + keyboard teleop + Cartesian controller."""
# TODO Phase 11: full implementation. Skeleton below.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Node(package="arm_teleop", executable="mujoco_sim_node",
        #      parameters=[{"model_path": ".../models/kr6_with_vacuum.xml"}]),
        # Node(package="arm_teleop", executable="cartesian_controller_node"),
        # Node(package="arm_teleop", executable="gripper_node",
        #      parameters=[{"ee_type": "vacuum"}]),
        # Node(package="arm_teleop", executable="keyboard_teleop_node",
        #      output="screen"),
    ])
