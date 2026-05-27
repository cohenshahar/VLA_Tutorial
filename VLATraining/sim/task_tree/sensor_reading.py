"""
sim/task_tree/sensor_reading.py  —  Phase 10, Task 10.10

Flat dataclass representing all sensor readings from the MuJoCo simulation.
Includes a classmethod to load directly from MuJoCo model/data.
"""

import sys
import pathlib
import numpy as np

# Path bootstrap to allow relative/absolute imports
_SIM_DIR = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_SIM_DIR))

try:
    import mujoco
except ImportError:
    pass  # Allow import of the dataclass without mujoco installed

from dataclasses import dataclass
from typing import Optional


@dataclass
class SensorReading:
    t:              float           # simulation time (s)
    qpos:           np.ndarray      # shape (6,) — joint positions (rad)
    qvel:           np.ndarray      # shape (6,) — joint velocities (rad/s)
    torque:         np.ndarray      # shape (6,) — actuator forces (N·m)
    ee_force:       np.ndarray      # shape (3,) — [fx, fy, fz] at em_contact_site (N)
    ee_torque:      np.ndarray      # shape (3,) — [tx, ty, tz] at em_contact_site (N·m)
    proximity:      float           # rangefinder (m); −1.0 = no hit
    em_active:      bool            # weld constraint active?
    target_contact: float           # target_touch_sensor value (N)
    overhead_bgr:   Optional[np.ndarray]      # shape (H, W, 3) uint8 — may be None
    wrist_bgr:      Optional[np.ndarray]      # shape (H, W, 3) uint8 — may be None
    box_pos:        np.ndarray      # shape (3,) world position of metal_box body
    ee_pos:         np.ndarray      # shape (3,) world position of em_contact_site

    @classmethod
    def from_mujoco(cls, model: "mujoco.MjModel", data: "mujoco.MjData", cam_publisher=None) -> "SensorReading":
        """
        Create a SensorReading directly from MuJoCo model and data.
        """
        import cv2

        def get_sensor_val(name: str, size: int = 1) -> np.ndarray:
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid < 0:
                raise RuntimeError(f"Sensor '{name}' not found in model.")
            adr = model.sensor_adr[sid]
            return data.sensordata[adr:adr + size]

        # 1. Joint state arrays (1-6)
        qpos_arr = np.array([get_sensor_val(f"sensor_pos_A{i}")[0] for i in range(1, 7)], dtype=np.float64)
        qvel_arr = np.array([get_sensor_val(f"sensor_vel_A{i}")[0] for i in range(1, 7)], dtype=np.float64)
        torque_arr = np.array([get_sensor_val(f"sensor_torque_A{i}")[0] for i in range(1, 7)], dtype=np.float64)

        # 2. End-effector force + torque
        ee_force = get_sensor_val("sensor_ee_force", 3).copy()
        ee_torque = get_sensor_val("sensor_ee_torque", 3).copy()

        # 3. Proximity rangefinder & Target touch
        proximity = float(get_sensor_val("sensor_proximity")[0])
        target_contact = float(get_sensor_val("target_touch_sensor")[0])

        # 4. EM weld constraint state
        weld_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "em_weld")
        if weld_id < 0:
            raise RuntimeError("Equality constraint 'em_weld' not found.")
        # Support both MuJoCo 2 data.eq_active and MuJoCo 3 eq_active
        em_active = bool(data.eq_active[weld_id])

        # 5. Object & EE world positions
        box_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "metal_box")
        if box_bid < 0:
            raise RuntimeError("Body 'metal_box' not found.")
        box_pos = data.xpos[box_bid].copy()

        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "em_contact_site")
        if site_id < 0:
            raise RuntimeError("Site 'em_contact_site' not found.")
        ee_pos = data.site_xpos[site_id].copy()

        # 6. Cameras (optional BGR outputs)
        overhead_bgr = None
        wrist_bgr = None
        if cam_publisher is not None:
            frames = cam_publisher.get_frames(model, data)
            if "cam_overhead" in frames:
                overhead_bgr = cv2.cvtColor(frames["cam_overhead"], cv2.COLOR_RGB2BGR)
            if "cam_wrist" in frames:
                wrist_bgr = cv2.cvtColor(frames["cam_wrist"], cv2.COLOR_RGB2BGR)

        return cls(
            t=float(data.time),
            qpos=qpos_arr,
            qvel=qvel_arr,
            torque=torque_arr,
            ee_force=ee_force,
            ee_torque=ee_torque,
            proximity=proximity,
            em_active=em_active,
            target_contact=target_contact,
            overhead_bgr=overhead_bgr,
            wrist_bgr=wrist_bgr,
            box_pos=box_pos,
            ee_pos=ee_pos
        )


if __name__ == "__main__":
    # Standalone test
    print("Running SensorReading standalone test...")
    WORLD_XML = str(_SIM_DIR / "scene" / "world.xml")
    
    model = mujoco.MjModel.from_xml_path(WORLD_XML)
    data  = mujoco.MjData(model)
    mujoco.mj_step(model, data)
    
    reading = SensorReading.from_mujoco(model, data)
    
    print("\nParsed Sensor Reading successfully!")
    print(f"Time (t)         : {reading.t:.4f} s")
    print(f"qpos (joints 1-6): {reading.qpos}")
    print(f"qvel (joints 1-6): {reading.qvel}")
    print(f"torque (act 1-6) : {reading.torque}")
    print(f"ee_force [X,Y,Z] : {reading.ee_force}")
    print(f"ee_torque [X,Y,Z]: {reading.ee_torque}")
    print(f"proximity sensor : {reading.proximity:.4f} m")
    print(f"EM active state  : {reading.em_active}")
    print(f"target_contact   : {reading.target_contact:.4f} N")
    print(f"box_pos          : {reading.box_pos}")
    print(f"ee_pos           : {reading.ee_pos}")
    
    # Assertions
    assert isinstance(reading.t, float)
    assert reading.qpos.shape == (6,)
    assert reading.qvel.shape == (6,)
    assert reading.torque.shape == (6,)
    assert reading.ee_force.shape == (3,)
    assert reading.ee_torque.shape == (3,)
    assert isinstance(reading.proximity, float)
    assert isinstance(reading.em_active, bool)
    assert isinstance(reading.target_contact, float)
    assert reading.box_pos.shape == (3,)
    assert reading.ee_pos.shape == (3,)
    print("\n✅ Standalone assertions PASSED!")
