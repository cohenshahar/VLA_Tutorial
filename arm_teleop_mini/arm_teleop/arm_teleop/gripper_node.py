"""
gripper_node — black-box end-effector controller.

Introduced in Phase 11 (vacuum) and Phase 12 (claw).

One node, two modes selected by ROS param `ee_type` ("vacuum" or "claw").

Vacuum mode:
  - Service /vacuum/set (std_srvs/SetBool) -> turns vacuum on/off
  - Publishes /vacuum/state (std_msgs/Bool) at 10 Hz
  - Visual effect: publishes state; mujoco_sim_node swaps cup color

Claw mode:
  - Service /claw/set (std_srvs/SetBool) -> data=True means CLOSE, False means OPEN
  - Publishes /claw/state (std_msgs/Bool) at 10 Hz
  - Visual effect only: drive finger joints to closed/open positions in MuJoCo
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class GripperNode(Node):
    def __init__(self):
        super().__init__("gripper_node")

        self.declare_parameter("ee_type", "vacuum")
        ee_type = self.get_parameter("ee_type").get_parameter_value().string_value
        self.state = False

        if ee_type == "vacuum":
            self._srv = self.create_service(SetBool, "/vacuum/set", self._on_set)
            self._pub = self.create_publisher(Bool, "/vacuum/state", 10)
            self.create_timer(0.1, self._publish_state)
            self.get_logger().info("Gripper node: vacuum mode. Press 'v' to toggle.")
        elif ee_type == "claw":
            self._srv = self.create_service(SetBool, "/claw/set", self._on_set)
            self._pub = self.create_publisher(Bool, "/claw/state", 10)
            self.create_timer(0.1, self._publish_state)
            self.get_logger().info("Gripper node: claw mode. Press '['/']' to open/close.")
        else:
            self.get_logger().error(f"Unknown ee_type: '{ee_type}'. Use 'vacuum' or 'claw'.")

    def _on_set(self, request, response):
        self.state = request.data
        response.success = True
        response.message = f"vacuum {'ON' if self.state else 'OFF'}"
        self.get_logger().info(response.message)
        return response

    def _publish_state(self):
        msg = Bool()
        msg.data = self.state
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
