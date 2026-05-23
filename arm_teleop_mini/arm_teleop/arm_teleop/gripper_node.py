"""
gripper_node — black-box end-effector controller.

Introduced in Phase 11 (vacuum) and Phase 12 (claw).

One node, two modes selected by ROS param `ee_type` ("vacuum" or "claw").

Vacuum mode:
  - Service /vacuum/set (std_srvs/SetBool) -> turns vacuum on/off
  - Publishes /vacuum/state at 10 Hz
  - Visual effect only (black box): change tube material color in MuJoCo

Claw mode:
  - Service /claw/set (std_srvs/SetBool) -> data=True means CLOSE, False means OPEN
  - Publishes /claw/state at 10 Hz
  - Visual effect only: drive finger joints to closed/open positions in MuJoCo
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Bool


class GripperNode(Node):
    def __init__(self):
        super().__init__("gripper_node")
        # TODO Phase 11:
        #   - declare param ee_type ("vacuum" by default)
        #   - if ee_type == "vacuum": create service /vacuum/set, publisher /vacuum/state
        #   - self.state = False
        # TODO Phase 12:
        #   - if ee_type == "claw": create service /claw/set, publisher /claw/state
        pass

    def _on_set(self, request, response):
        # TODO Phase 11/12:
        #   - self.state = request.data
        #   - apply to MuJoCo (color swap or finger qpos target) via a side channel
        #     (e.g., a topic the mujoco_sim_node listens to, OR shared model handle)
        #   - response.success = True
        #   - response.message = f"set to {self.state}"
        #   - return response
        pass


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
