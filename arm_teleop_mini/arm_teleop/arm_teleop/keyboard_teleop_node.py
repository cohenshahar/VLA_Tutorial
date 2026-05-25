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

import curses
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
# from std_srvs.srv import SetBool   # TODO Phase 11

DEADMAN_TIMEOUT_MS = 200
JOINT_VEL_STEP = 0.3       # rad/s per held key
CART_LIN_STEP  = 0.05      # m/s  per held key

# Keymap — joint mode
JOINT_KEYS = {
    "q": (0, +1), "a": (0, -1),
    "w": (1, +1), "s": (1, -1),
    "e": (2, +1), "d": (2, -1),
    "r": (3, +1), "f": (3, -1),
    "t": (4, +1), "g": (4, -1),
    "y": (5, +1), "h": (5, -1),
}

# Keymap — Cartesian mode (numpad with numlock ON)
# key → unit direction vector [x, y, z]
CART_KEYS = {
    "8": np.array([ 1.,  0.,  0.]),   # numpad 8  → +X
    "2": np.array([-1.,  0.,  0.]),   # numpad 2  → -X
    "4": np.array([ 0.,  1.,  0.]),   # numpad 4  → +Y
    "6": np.array([ 0., -1.,  0.]),   # numpad 6  → -Y
    "+": np.array([ 0.,  0.,  1.]),   # numpad +  → +Z
    "-": np.array([ 0.,  0., -1.]),   # numpad -  → -Z
}

HELP = (
    "Joint: q/a:J1 w/s:J2 e/d:J3 r/f:J4 t/g:J5 y/h:J6  "
    "| Cart(numpad): 8/2=X 4/6=Y +/-=Z  m=frame  Esc=quit"
)


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__("keyboard_teleop_node")

        # ── Publishers ────────────────────────────────────────────────────
        self.joint_vel_pub = self.create_publisher(Float64MultiArray, "/cmd_joint_vel", 10)
        self.ee_vel_pub    = self.create_publisher(TwistStamped, "/cmd_ee_vel", 10)
        self.frame = "base"   # Phase 10: toggles between "base" and "tool"

        # ── Key state: key -> timestamp of last press (seconds) ───────────
        self._key_state: dict[str, float] = {}
        self._lock = threading.Lock()
        self._running = True

        # ── curses keyboard thread ────────────────────────────────────────
        self._kb_thread = threading.Thread(target=self._run_curses, daemon=True)
        self._kb_thread.start()

        # ── 50 Hz publish timer ───────────────────────────────────────────
        self.create_timer(0.02, self._publish_cmd)
        self.get_logger().info("keyboard_teleop_node ready — " + HELP)

    # ── Curses thread ─────────────────────────────────────────────────────
    def _run_curses(self):
        curses.wrapper(self._curses_loop)

    def _curses_loop(self, stdscr):
        curses.cbreak()
        curses.noecho()
        stdscr.nodelay(True)          # non-blocking getch
        stdscr.addstr(0, 0, HELP[:curses.COLS - 1])
        stdscr.refresh()
        while self._running:
            ch = stdscr.getch()
            if ch == curses.ERR:      # no key pressed
                time.sleep(0.01)
                continue
            if ch == 27:              # Esc
                self._running = False
                break
            key = chr(ch) if 0 < ch < 128 else None
            if key == 'm':
                self._on_frame_toggle()   # one-shot toggle, not a held key
                continue
            if key == 'v':
                self._on_vacuum_toggle()
                continue
            if key:
                with self._lock:
                    self._key_state[key] = time.monotonic()

    # ── 50 Hz timer ───────────────────────────────────────────────────────
    def _publish_cmd(self):
        if not self._running:
            return
        now = time.monotonic()
        deadline = DEADMAN_TIMEOUT_MS / 1000.0

        with self._lock:
            # Prune stale keys
            active = {k: t for k, t in self._key_state.items() if now - t < deadline}
            self._key_state = active

        # Build joint velocity vector
        qdot = [0.0] * 6
        for key, (joint_idx, sign) in JOINT_KEYS.items():
            if key in active:
                qdot[joint_idx] += sign * JOINT_VEL_STEP

        # Only publish joint velocity when at least one joint key is active
        # (avoids fighting with cartesian_controller_node on /cmd_joint_vel)
        if any(k in active for k in JOINT_KEYS):
            msg = Float64MultiArray()
            msg.data = qdot
            self.joint_vel_pub.publish(msg)

        # Build and publish Cartesian velocity (TwistStamped)
        cart_vel = np.zeros(3)
        for key, direction in CART_KEYS.items():
            if key in active:
                cart_vel += direction * CART_LIN_STEP

        twist_msg = TwistStamped()
        twist_msg.header.stamp    = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.frame
        twist_msg.twist.linear.x  = float(cart_vel[0])
        twist_msg.twist.linear.y  = float(cart_vel[1])
        twist_msg.twist.linear.z  = float(cart_vel[2])
        self.ee_vel_pub.publish(twist_msg)

    def _on_frame_toggle(self):
        """Handle 'm' keypress — toggle base ↔ tool frame."""
        self.frame = "tool" if self.frame == "base" else "base"
        self.get_logger().info(f"[FRAME] {self.frame}")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

# *VLA Research | Shahar Cohen | BGU Mechatronics | 2026-05-24*
        #   self.get_logger().info(f"[FRAME] {self.frame}")
        pass

    def _on_vacuum_toggle(self):
        """Handle 'v' keypress — call /vacuum/set with the opposite state."""
        if not hasattr(self, '_vacuum_client'):
            from std_srvs.srv import SetBool
            self._vacuum_client = self.create_client(SetBool, '/vacuum/set')
            self._vacuum_state  = False
        if not self._vacuum_client.service_is_ready():
            self.get_logger().warn('Vacuum service not available')
            return
        from std_srvs.srv import SetBool
        self._vacuum_state = not self._vacuum_state
        req = SetBool.Request()
        req.data = self._vacuum_state
        self._vacuum_client.call_async(req)
        self.get_logger().info(f"[VACUUM] {'ON' if self._vacuum_state else 'OFF'}")

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
