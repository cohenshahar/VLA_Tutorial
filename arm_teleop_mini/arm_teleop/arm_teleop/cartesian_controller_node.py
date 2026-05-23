"""
cartesian_controller_node — Jacobian IK controller.

Introduced in Phase 9. Extended in Phase 10 (frame toggle).

Subscribes to:
  - /joint_states  (sensor_msgs/JointState)         — current q
  - /ee_pose       (geometry_msgs/PoseStamped)      — current EE pose (for tool-frame rotation)
  - /cmd_ee_vel    (geometry_msgs/Twist)            — desired EE velocity (linear+angular)

Publishes:
  - /cmd_joint_vel (std_msgs/Float64MultiArray)     — computed q_dot

Algorithm (Phase 9):
  q_dot = J^T @ inv(J @ J^T + lambda^2 * I) @ x_dot      (damped least squares)

  where J is the 6x6 geometric Jacobian at current q.

Frame handling (Phase 10):
  - If cmd is in base frame: use as-is.
  - If cmd is in tool frame: rotate the linear part by R_base_tool from /ee_pose.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64MultiArray

# import numpy as np                       # TODO Phase 9
# from .kinematics import damped_least_squares_ik, compute_jacobian  # TODO Phase 9

IK_LAMBDA_SQ = 0.01


class CartesianControllerNode(Node):
    def __init__(self):
        super().__init__("cartesian_controller_node")
        # TODO Phase 9:
        #   - load MuJoCo model (read-only, for Jacobian computation) OR
        #     receive J from mujoco_sim_node via a topic. Simpler: load model here too.
        #   - subscribe /joint_states  -> self._on_joint_states
        #   - subscribe /cmd_ee_vel    -> self._on_cmd_ee_vel
        #   - publisher  /cmd_joint_vel
        #   - declare param `frame` (default "base"); subscribe a small topic if we
        #     want runtime toggling, OR re-read this from /cmd_ee_vel header.frame_id
        # TODO Phase 10:
        #   - subscribe /ee_pose to know current EE orientation (for tool->base rotation)
        pass

    def _on_joint_states(self, msg: JointState):
        # TODO Phase 9: store self.q = np.array(msg.position[:6])
        pass

    def _on_ee_pose(self, msg: PoseStamped):
        # TODO Phase 10: store self.R_base_tool from msg.pose.orientation (quat -> R)
        pass

    def _on_cmd_ee_vel(self, msg: Twist):
        """Twist in (linear xyz, angular xyz). 6-vector x_dot."""
        # TODO Phase 9:
        #   - x_dot = np.array([msg.linear.x, msg.linear.y, msg.linear.z,
        #                       msg.angular.x, msg.angular.y, msg.angular.z])
        # TODO Phase 10:
        #   - if msg.header.frame_id == "tool": rotate linear part by self.R_base_tool
        # TODO Phase 9:
        #   - J = compute_jacobian(self.model, self.data, body_name="tool0")
        #   - q_dot = damped_least_squares_ik(J, x_dot, IK_LAMBDA_SQ)
        #   - publish q_dot as Float64MultiArray on /cmd_joint_vel
        #   - if condition number too high, log "IK_FAIL: near singularity" and publish zeros
        pass


def main(args=None):
    rclpy.init(args=args)
    node = CartesianControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
