"""Launch the arm + vacuum end-effector + Cartesian controller.

NOTE: keyboard_teleop_node is NOT launched here — curses requires a real TTY.
Run it separately in its own terminal:
    ros2 run arm_teleop keyboard_teleop_node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("arm_teleop")
    vacuum_model = os.path.join(pkg_share, "models", "kr6_with_vacuum.xml")
    params_yaml  = os.path.join(pkg_share, "config", "kr6_params.yaml")

    return LaunchDescription([
        Node(
            package="arm_teleop",
            executable="mujoco_sim_node",
            parameters=[{
                "model_path":        vacuum_model,
                "joint_limits_yaml": params_yaml,
            }],
        ),
        Node(
            package="arm_teleop",
            executable="cartesian_controller_node",
            parameters=[{"model_path": vacuum_model}],
        ),
        Node(
            package="arm_teleop",
            executable="gripper_node",
            parameters=[{"ee_type": "vacuum"}],
        ),
        # keyboard_teleop_node intentionally omitted — run it in a dedicated terminal.
    ])
