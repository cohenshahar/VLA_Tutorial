"""
MuJoCo Bridge Node — Phase 9

Runs a MuJoCo simulation in a background thread and bridges its state to ROS2:

Topics published:
  /bridge/status              std_msgs/String              1 Hz    heartbeat
  /joint_states               sensor_msgs/JointState      100 Hz  pos + vel + torque
  /joint_torques              sensor_msgs/JointState      100 Hz  actuator forces
  /ft_sensor                  geometry_msgs/WrenchStamped 100 Hz  EE force/torque
  /proximity                  sensor_msgs/Range            50 Hz  rangefinder
  /em_state                   std_msgs/Bool               on-change  EM active?
  /target_contact             std_msgs/Bool                50 Hz  box on target zone?
  /camera/overhead/image_raw  sensor_msgs/Image              6 Hz  overhead RGB
  /camera/side/image_raw      sensor_msgs/Image              6 Hz  side RGB
  /camera/wrist/image_raw     sensor_msgs/Image              6 Hz  wrist RGB

Topics subscribed:
  /joint_commands             trajectory_msgs/JointTrajectory  → writes data.ctrl

Tasks 9.2 – 9.12
"""

import os
import sys
import threading
from pathlib import Path

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState, Range, Image
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory
from cv_bridge import CvBridge

# ── paths ─────────────────────────────────────────────────────────────────────────
def _resolve_sim_root() -> str:
    """Resolve VLATraining/sim. Priority: VLA_SIM_ROOT env var, then dev-tree fallback."""
    env = os.environ.get('VLA_SIM_ROOT')
    if env:
        return os.path.abspath(env)
    # Dev-tree fallback: walk up from __file__ looking for VLATraining/sim.
    # Works from both the source tree and the install tree.
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / 'VLATraining' / 'sim'
        if candidate.is_dir():
            return str(candidate)
    raise RuntimeError(
        f"Cannot locate VLATraining/sim. "
        f"Set VLA_SIM_ROOT env var, or run from the VLA_Tutorial dev tree. "
        f"Searched ancestors of: {here}"
    )

_SIM_ROOT  = _resolve_sim_root()
_WORLD_XML = os.path.join(_SIM_ROOT, 'scene', 'world.xml')

if _SIM_ROOT not in sys.path:
    sys.path.insert(0, _SIM_ROOT)

# Camera names (must match XML)
_CAMERA_NAMES = ['cam_overhead', 'cam_side', 'cam_wrist']
_CAMERA_TOPICS = {
    'cam_overhead': '/camera/overhead/image_raw',
    'cam_side':     '/camera/side/image_raw',
    'cam_wrist':    '/camera/wrist/image_raw',
}
_CAMERA_WIDTH  = 640
_CAMERA_HEIGHT = 480

# Joint / actuator ordering expected by the model
_JOINT_NAMES = ['joint_a1', 'joint_a2', 'joint_a3',
                'joint_a4', 'joint_a5', 'joint_a6']

# Human-readable names for ROS2 messages
_JOINT_LABELS = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']

# Sensor name → sensordata address (resolved at init)
_POS_SENSORS    = [f'sensor_pos_A{i}'    for i in range(1, 7)]
_VEL_SENSORS    = [f'sensor_vel_A{i}'    for i in range(1, 7)]
_TORQUE_SENSORS = [f'sensor_torque_A{i}' for i in range(1, 7)]

_EE_FORCE_SENSOR  = 'sensor_ee_force'
_EE_TORQUE_SENSOR = 'sensor_ee_torque'
_PROXIMITY_SENSOR = 'sensor_proximity'
_TOUCH_SENSOR     = 'target_touch_sensor'
_EM_WELD_NAME     = 'em_weld'

_PROXIMITY_MIN_RANGE = 0.001   # m
_PROXIMITY_MAX_RANGE = 1.0     # m


def _ros_stamp(node: Node) -> Time:
    t = node.get_clock().now().to_msg()
    return t


class BridgeNode(Node):
    """
    MuJoCo ↔ ROS2 bridge node — Phase 9.

    Lock contract
    -------------
    Any read or write of self._model or self._data fields outside _sim_loop
    must be done while holding self._lock.

      * _sim_loop holds the lock for the duration of mj_step.
      * Every ROS2 timer callback that reads sensordata, qpos, qvel, or
        eq_active holds the lock only for the duration of the array copy.
      * _publish_cameras copies qpos/qvel/ctrl/act under lock (~0.1 ms),
        then releases the lock before calling mj_forward and rendering.
        Rendering (~45 ms) runs lock-free so _sim_loop is not starved.
      * Callbacks copy out (.copy()) any numpy view they need, then
        release the lock before constructing/publishing the ROS message.

    Violating this contract = silent data races during mj_step, and
    intermittent garbage values in the published topics.
    """

    def __init__(self):
        super().__init__('mujoco_bridge')

        # ── load model ────────────────────────────────────────────────
        self.get_logger().info(f'Loading MuJoCo model: {_WORLD_XML}')
        self._model = mujoco.MjModel.from_xml_path(_WORLD_XML)
        self._data  = mujoco.MjData(self._model)

        # Resolve joint indices (for qpos / qvel access)
        self._jnt_ids = [self._model.joint(n).id for n in _JOINT_NAMES]
        self._act_ids = list(range(self._model.nu))   # actuators 0–5

        # Resolve sensor addresses
        def _adr(name):
            sid = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid < 0:
                raise RuntimeError(f"Sensor '{name}' not found in model")
            return self._model.sensor_adr[sid]

        self._pos_adrs    = [_adr(n) for n in _POS_SENSORS]
        self._vel_adrs    = [_adr(n) for n in _VEL_SENSORS]
        self._torque_adrs = [_adr(n) for n in _TORQUE_SENSORS]
        self._ee_force_adr  = _adr(_EE_FORCE_SENSOR)
        self._ee_torque_adr = _adr(_EE_TORQUE_SENSOR)
        self._proximity_adr = _adr(_PROXIMITY_SENSOR)
        self._touch_adr     = _adr(_TOUCH_SENSOR)

        # EM weld constraint index
        self._em_weld_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_EQUALITY, _EM_WELD_NAME)
        if self._em_weld_id < 0:
            raise RuntimeError(f"Equality constraint '{_EM_WELD_NAME}' not found")
        self._em_state_last: bool | None = None   # for on-change publishing

        # cv_bridge + CameraPublisher
        self._cv_bridge = CvBridge()
        from scene.camera_utils import CameraPublisher
        self._cam_publisher = CameraPublisher(
            camera_names=_CAMERA_NAMES,
            width=_CAMERA_WIDTH,
            height=_CAMERA_HEIGHT,
        )
        self._cam_pubs = {
            name: self.create_publisher(Image, topic, 10)
            for name, topic in _CAMERA_TOPICS.items()
        }

        # Lock guards shared MjData between sim thread and ROS timers
        self._lock = threading.Lock()

        # Dedicated MjData for camera rendering — never touched by _sim_loop.
        # _publish_cameras copies state under lock (~0.1 ms), then renders
        # outside the lock so physics advances freely during the ~45 ms render.
        self._data_render = mujoco.MjData(self._model)

        # Pending joint command (set by subscriber, consumed by sim thread)
        self._cmd_ctrl: np.ndarray | None = None

        # ── publishers ────────────────────────────────────────────────
        self._status_pub    = self.create_publisher(String,         '/bridge/status',   10)
        self._js_pub        = self.create_publisher(JointState,     '/joint_states',    10)
        self._torque_pub    = self.create_publisher(JointState,     '/joint_torques',   10)
        self._ft_pub        = self.create_publisher(WrenchStamped,  '/ft_sensor',       10)
        self._proximity_pub = self.create_publisher(Range,          '/proximity',       10)
        self._em_pub        = self.create_publisher(Bool,           '/em_state',        10)
        self._touch_pub     = self.create_publisher(Bool,           '/target_contact',  10)

        # ── subscribers ───────────────────────────────────────────────
        self.create_subscription(
            JointTrajectory, '/joint_commands',
            self._on_joint_command, 10)

        # ── timers ────────────────────────────────────────────────────
        self.create_timer(1.0,   self._publish_status)          # 1 Hz
        self.create_timer(0.01,  self._publish_joint_states)    # 100 Hz
        self.create_timer(0.01,  self._publish_ft)              # 100 Hz
        self.create_timer(0.02,  self._publish_proximity)       # 50 Hz
        self.create_timer(0.02,  self._publish_target_contact)  # 50 Hz
        self.create_timer(0.05,  self._publish_em_state)        # poll 20 Hz, emit on change
        self.create_timer(1.0/6, self._publish_cameras)         # 6 Hz

        # ── simulation thread ─────────────────────────────────────────
        self._running = True
        self._sim_thread = threading.Thread(
            target=self._sim_loop, daemon=True)
        self._sim_thread.start()

        self.get_logger().info('MuJoCo bridge node started')

    # ── simulation loop (background thread) ───────────────────────────────
    def _sim_loop(self):
        """Physics thread. Holds self._lock during mj_step. Logs RTF every 5 s.

        Rate-limited to 1× real time so the ROS2 executor is not GIL-starved.
        After each step we yield the GIL (time.sleep(0)) then sleep the
        remainder of the timestep budget, keeping RTF ≈ 1.0.
        """
        import time
        target_hz = 1.0 / self._model.opt.timestep   # e.g. 1000.0 for 1 ms timestep
        target_dt = self._model.opt.timestep          # wall-clock budget per step
        step_count = 0
        last_log = time.monotonic()
        while self._running:
            t0 = time.monotonic()
            with self._lock:
                # Apply pending command if any
                if self._cmd_ctrl is not None:
                    np.copyto(self._data.ctrl, self._cmd_ctrl)
                    self._cmd_ctrl = None
                mujoco.mj_step(self._model, self._data)
            # Yield GIL — lets the ROS2 executor fire pending timer callbacks
            time.sleep(0)
            # Sleep remainder of timestep budget to target RTF ≈ 1.0
            elapsed = time.monotonic() - t0
            remaining = target_dt - elapsed
            if remaining > 1e-4:
                time.sleep(remaining)
            step_count += 1
            now = time.monotonic()
            if now - last_log >= 5.0:
                real_hz = step_count / (now - last_log)
                rtf = real_hz / target_hz
                self.get_logger().info(
                    f'sim_loop: real {real_hz:.0f} Hz / target {target_hz:.0f} Hz '
                    f'= RTF {rtf:.3f}'
                )
                step_count = 0
                last_log = now

    # ── ROS callbacks ─────────────────────────────────────────────────────
    def _publish_status(self):
        msg = String()
        msg.data = 'MuJoCo bridge alive'
        self._status_pub.publish(msg)

    def _publish_joint_states(self):
        stamp = _ros_stamp(self)
        with self._lock:
            sd = self._data.sensordata.copy()

        pos    = [float(sd[a]) for a in self._pos_adrs]
        vel    = [float(sd[a]) for a in self._vel_adrs]
        effort = [float(sd[a]) for a in self._torque_adrs]

        # /joint_states — position + velocity + effort
        js = JointState()
        js.header.stamp = stamp
        js.name     = _JOINT_LABELS
        js.position = pos
        js.velocity = vel
        js.effort   = effort
        self._js_pub.publish(js)

        # /joint_torques — effort only (same data, dedicated topic)
        jt = JointState()
        jt.header.stamp = stamp
        jt.name   = _JOINT_LABELS
        jt.effort = effort
        self._torque_pub.publish(jt)

    def _publish_ft(self):
        """Task 9.6 — EE force/torque on /ft_sensor at 100 Hz."""
        stamp = _ros_stamp(self)
        with self._lock:
            sd = self._data.sensordata.copy()

        msg = WrenchStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'em_contact_site'
        msg.wrench.force.x  = float(sd[self._ee_force_adr])
        msg.wrench.force.y  = float(sd[self._ee_force_adr + 1])
        msg.wrench.force.z  = float(sd[self._ee_force_adr + 2])
        msg.wrench.torque.x = float(sd[self._ee_torque_adr])
        msg.wrench.torque.y = float(sd[self._ee_torque_adr + 1])
        msg.wrench.torque.z = float(sd[self._ee_torque_adr + 2])
        self._ft_pub.publish(msg)

    def _publish_proximity(self):
        """Task 9.7 — rangefinder on /proximity at 50 Hz."""
        stamp = _ros_stamp(self)
        with self._lock:
            raw = float(self._data.sensordata[self._proximity_adr])

        msg = Range()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'em_contact_site'
        msg.radiation_type  = Range.INFRARED
        msg.field_of_view   = 0.0
        msg.min_range       = _PROXIMITY_MIN_RANGE
        msg.max_range       = _PROXIMITY_MAX_RANGE
        msg.range           = float(np.clip(raw, _PROXIMITY_MIN_RANGE,
                                            _PROXIMITY_MAX_RANGE))
        self._proximity_pub.publish(msg)

    def _publish_em_state(self):
        """Task 9.8 — EM weld active state on /em_state, emitted on change."""
        with self._lock:
            active = bool(self._data.eq_active[self._em_weld_id])

        if active != self._em_state_last:
            msg = Bool()
            msg.data = active
            self._em_pub.publish(msg)
            self._em_state_last = active

    def _publish_target_contact(self):
        """Task 9.9 — target touch sensor on /target_contact at 50 Hz."""
        with self._lock:
            raw = float(self._data.sensordata[self._touch_adr])

        msg = Bool()
        msg.data = raw > 0.0
        self._touch_pub.publish(msg)

    def _snapshot_for_render(self) -> None:
        """Copy live physics state into _data_render. Must be called under self._lock."""
        d, dr = self._data, self._data_render
        dr.qpos[:] = d.qpos
        dr.qvel[:] = d.qvel
        dr.act[:]  = d.act
        dr.ctrl[:] = d.ctrl
        if d.mocap_pos.size:
            dr.mocap_pos[:]  = d.mocap_pos
            dr.mocap_quat[:] = d.mocap_quat
        dr.time = d.time

    def _publish_cameras(self):
        """Tasks 9.10–9.12 — render all 3 cameras and publish at 6 Hz.

        Lock is held only for the state copy (~0.1 ms).  The expensive
        mj_forward + render (~45 ms) runs outside the lock so _sim_loop
        can advance physics freely during the render.
        """
        stamp = _ros_stamp(self)
        # 1. Snapshot under lock — very fast
        with self._lock:
            self._snapshot_for_render()
        # 2. Recompute derived quantities on the snapshot (no lock needed)
        mujoco.mj_forward(self._model, self._data_render)
        # 3. Render from snapshot (no lock, ~45 ms)
        frames = self._cam_publisher.get_frames(self._model, self._data_render)
        for cam_name, rgb in frames.items():
            img_msg = self._cv_bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = cam_name
            self._cam_pubs[cam_name].publish(img_msg)

    def _on_joint_command(self, msg: JointTrajectory):
        """Task 9.5 — apply the first trajectory point to data.ctrl."""
        if not msg.points:
            return
        pt = msg.points[0]
        if len(pt.positions) != len(_JOINT_LABELS):
            self.get_logger().warn(
                f'Expected {len(_JOINT_LABELS)} positions, '
                f'got {len(pt.positions)} — ignoring')
            return
        with self._lock:
            self._cmd_ctrl = np.array(pt.positions, dtype=np.float64)
        self.get_logger().info(
            f'Joint command received: '
            + ', '.join(f'{l}={v:.3f}' for l, v
                        in zip(_JOINT_LABELS, pt.positions)))

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
