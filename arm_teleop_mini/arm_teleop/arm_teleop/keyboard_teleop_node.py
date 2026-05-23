"""
keyboard_teleop_node — keyboard → ROS2 commands.

Introduced in Phase 5. Extended in Phases 10, 11, 12.

Joint mode (always active, top of keyboard):
  q/a w/s e/d r/f t/g y/h -> joints 1..6 (+/-)

Cartesian mode (always active, numpad):
  8/2 = +X/-X    4/6 = +Y/-Y    +/- = +Z/-Z
  m              = toggle base <-> tool frame

Gripper (depends on which gripper_node is launched):
  v       = vacuum toggle
  [ / ]   = claw open / close

Misc:
  0       = go to home pose (TODO: publish a special command)
  Esc     = quit

Deadman: if no key event in DEADMAN_TIMEOUT_MS, publish zero command.

Owns topics:
  - PUB  /cmd_joint_vel    std_msgs/Float64MultiArray
  - PUB  /cmd_ee_vel       geometry_msgs/Twist
  - CALL /vacuum/set       std_srvs/SetBool  (Phase 11)
  - CALL /claw/set         std_srvs/SetBool  (Phase 12)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
# from std_srvs.srv import SetBool   # TODO Phase 11

# import curses  # TODO Phase 5

DEADMAN_TIMEOUT_MS = 200
JOINT_VEL_STEP = 0.3       # rad/s per held key
CART_LIN_STEP = 0.05       # m/s
CART_ANG_STEP = 0.3        # rad/s

# Keymap (default — override via config/keymap.yaml later)
JOINT_KEYS = {
    "q": ( 0, +1), "a": ( 0, -1),
    "w": ( 1, +1), "s": ( 1, -1),
    "e": ( 2, +1), "d": ( 2, -1),
    "r": ( 3, +1), "f": ( 3, -1),
    "t": ( 4, +1), "g": ( 4, -1),
    "y": ( 5, +1), "h": ( 5, -1),
}
CART_KEYS = {
    # numpad codes -- fill exact curses key constants in Phase 10
    "KP_8": ("x", +1), "KP_2": ("x", -1),
    "KP_4": ("y", +1), "KP_6": ("y", -1),
    "+":    ("z", +1), "-":    ("z", -1),
}


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__("keyboard_teleop_node")
        # TODO Phase 5:
        #   - create publisher /cmd_joint_vel
        #   - init self.key_state (dict of currently-held keys)
        #   - init self.last_key_time = now
        #   - start curses screen on a background thread OR via timer
        #   - timer @ 50 Hz: self._publish_cmd
        # TODO Phase 10:
        #   - create publisher /cmd_ee_vel
        #   - self.frame = "base"   # toggled by 'm'
        # TODO Phase 11/12:
        #   - create service clients for /vacuum/set and /claw/set
        pass

    def _read_keys(self):
        """Poll curses for keypresses; update self.key_state with timestamps."""
        # TODO Phase 5:
        #   - non-blocking getch
        #   - for each pressed key, set key_state[key] = now
        #   - if Esc, request shutdown
        pass

    def _publish_cmd(self):
        """Timer callback: read held keys, build cmd messages, publish."""
        # TODO Phase 5:
        #   - prune keys older than DEADMAN_TIMEOUT_MS
        #   - build qdot vector (6,) from JOINT_KEYS
        #   - publish Float64MultiArray on /cmd_joint_vel
        # TODO Phase 10:
        #   - build Twist from CART_KEYS
        #   - if self.frame == "tool": leave Twist as-is (cartesian_controller_node rotates)
        #     else: also leave as-is (base) -- the *flag* travels via TF or a param
        #   - publish /cmd_ee_vel
        pass

    def _on_frame_toggle(self):
        """Handle 'm' keypress."""
        # TODO Phase 10:
        #   self.frame = "tool" if self.frame == "base" else "base"
        #   self.get_logger().info(f"[FRAME] {self.frame}")
        pass

    def _on_vacuum_toggle(self):
        """Handle 'v' keypress — call /vacuum/set with the opposite state."""
        # TODO Phase 11
        pass

    def _on_claw(self, close: bool):
        """Handle '[' or ']' — call /claw/set with close=True/False."""
        # TODO Phase 12
        pass


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
