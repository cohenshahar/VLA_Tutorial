---
applyTo: "**/VLATraining/sim/bridge/**/*.py"
description: "SimBus and ROS2 bridge conventions for the VLA simulation project."
---

## Message bus conventions

The bridge layer has two interchangeable backends. **Default: SimBus** (pure Python, zero extra deps).
Switch to ROS2 only after confirming `rclpy` is importable in the execution environment.

---

### SimBus — canonical implementation pattern

```python
import queue, threading

class SimBus:
    def __init__(self):
        self._queues: dict[str, queue.Queue] = {}
        self._callbacks: dict[str, list] = {}
        self._lock = threading.Lock()

    def put(self, topic: str, msg):
        with self._lock:
            if topic not in self._queues:
                self._queues[topic] = queue.Queue(maxsize=10)
        try:
            self._queues[topic].put_nowait(msg)
        except queue.Full:
            self._queues[topic].get_nowait()   # drop oldest
            self._queues[topic].put_nowait(msg)
        for cb in self._callbacks.get(topic, []):
            cb(msg)

    def get(self, topic: str, block=False, timeout=None):
        return self._queues[topic].get(block=block, timeout=timeout)

    def subscribe(self, topic: str, callback):
        self._callbacks.setdefault(topic, []).append(callback)
```

---

### Canonical topic names and message shapes

| Topic | Shape / type | Rate | Notes |
|---|---|---|---|
| `/joint_states` | `np.ndarray (12,)` — `qpos[6] \|\| qvel[6]` | 100 Hz | |
| `/joint_torques` | `np.ndarray (6,)` | 100 Hz | `actuator_force` |
| `/joint_commands` | `np.ndarray (6,)` — target `qpos` in radians | on-demand | |
| `/ft_sensor` | `np.ndarray (6,)` — `force[3] \|\| torque[3]` | 100 Hz | Fz index 2 |
| `/proximity` | `float` — metres | 50 Hz | < 0.02 triggers EM pre-check |
| `/em_state` | `bool` | on-change | True = magnet active |
| `/target_contact` | `bool` | 50 Hz | True = box on target zone |
| `/camera/wrist/image_raw` | `np.ndarray (480, 640, 3)` uint8 | 6 Hz | primary VLA input |
| `/camera/overhead/image_raw` | `np.ndarray (480, 640, 3)` uint8 | 6 Hz | |
| `/camera/side/image_raw` | `np.ndarray (480, 640, 3)` uint8 | 6 Hz | |
| `/task_tree/status` | `str` (JSON) | on-change | `{"subtask_id", "subtask_name", "status", "postcondition_result"}` |

---

### Publisher loop pattern (sensor publisher)

```python
import time, threading

class SensorPublisher:
    def __init__(self, model, data, bus: SimBus):
        self.model = model
        self.data = data
        self.bus = bus

    def publish_once(self):
        # joint states
        import numpy as np
        qpos = self.data.qpos[:6].copy()
        qvel = self.data.qvel[:6].copy()
        self.bus.put("/joint_states", np.concatenate([qpos, qvel]))

        # proximity
        prox_adr = self.model.sensor_adr[self.model.sensor("proximity").id]
        self.bus.put("/proximity", float(self.data.sensordata[prox_adr]))

        # add other topics following the same pattern
```

---

### ROS2 adapter pattern (Option A — use only if SimBus is insufficient)

When ROS2 is available, wrap SimBus topics with a thin rclpy adapter.
**Never import `rclpy` at module level** — always guard with `try/except ImportError` so the code
remains importable in environments without ROS2.

```python
try:
    import rclpy
    from sensor_msgs.msg import JointState
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

class Ros2Bridge:
    """Publishes SimBus messages to ROS2 topics. No-op if ROS2 not available."""

    def __init__(self, bus: SimBus):
        self.bus = bus
        if not ROS2_AVAILABLE:
            return
        rclpy.init()
        self.node = rclpy.create_node("mujoco_bridge")
        self._js_pub = self.node.create_publisher(JointState, "/joint_states", 10)
        # subscribe SimBus and forward
        bus.subscribe("/joint_states", self._forward_joint_states)

    def _forward_joint_states(self, msg):
        if not ROS2_AVAILABLE:
            return
        # convert numpy array → JointState message and publish
        ...
```

---

### Anti-patterns to avoid
- Do NOT use blocking `queue.get()` on the physics thread — it will stall the sim.
- Do NOT use topic names other than the canonical list above — the TaskTreeManager subscribes by exact string.
- Do NOT import `rclpy` unconditionally — always guard with `try/except ImportError`.
- Do NOT create new topics outside the canonical list without updating AGENTS.md first.
