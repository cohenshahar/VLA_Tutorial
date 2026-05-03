"""
scene/sensor_logger.py
──────────────────────────────────────────────────────────────────────────
Phase 7, Task 7.6 — SensorLogger: writes one CSV row per simulation step.

CSV layout (27 columns):
  Col  0     : t            — simulation time (seconds)
  Col  1     : touch        — target_touch_sensor
  Cols  2– 7 : pos_A1…A6   — joint positions (rad)
  Cols  8–13 : vel_A1…A6   — joint velocities (rad/s)
  Cols 14–19 : torque_A1…A6— actuator forces (N·m)
  Cols 20–22 : ee_fx/fy/fz — EE force in site frame (N)
  Cols 23–25 : ee_tx/ty/tz — EE torque in site frame (N·m)
  Col 26     : proximity    — rangefinder (m; -1.0 = no hit)

Usage:
    logger = SensorLogger(model, output_path)
    for each step:
        mujoco.mj_step(model, data)
        logger.step(data)
    logger.finish()   # flushes and closes the file
"""
import csv
import pathlib
import mujoco
import numpy as np


# Ordered sensor names → defines column order (matches sensordata layout)
_SENSOR_NAMES = [
    "target_touch_sensor",
    "sensor_pos_A1", "sensor_pos_A2", "sensor_pos_A3",
    "sensor_pos_A4", "sensor_pos_A5", "sensor_pos_A6",
    "sensor_vel_A1", "sensor_vel_A2", "sensor_vel_A3",
    "sensor_vel_A4", "sensor_vel_A5", "sensor_vel_A6",
    "sensor_torque_A1", "sensor_torque_A2", "sensor_torque_A3",
    "sensor_torque_A4", "sensor_torque_A5", "sensor_torque_A6",
    "sensor_ee_force",    # 3 floats: fx, fy, fz
    "sensor_ee_torque",   # 3 floats: tx, ty, tz
    "sensor_proximity",   # 1 float
]

# Human-readable column headers (1 per scalar output)
_HEADERS = [
    "t",
    "touch",
    "pos_A1", "pos_A2", "pos_A3", "pos_A4", "pos_A5", "pos_A6",
    "vel_A1", "vel_A2", "vel_A3", "vel_A4", "vel_A5", "vel_A6",
    "torque_A1", "torque_A2", "torque_A3", "torque_A4", "torque_A5", "torque_A6",
    "ee_fx", "ee_fy", "ee_fz",
    "ee_tx", "ee_ty", "ee_tz",
    "proximity",
]
assert len(_HEADERS) == 27, f"Expected 27 columns, got {len(_HEADERS)}"


class SensorLogger:
    """
    Writes one CSV row per call to step().

    Parameters
    ----------
    model : mujoco.MjModel
    output_path : str | pathlib.Path
        Destination CSV file.  Parent directory is created if needed.
    """

    def __init__(self, model: mujoco.MjModel, output_path):
        output_path = pathlib.Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # Build (adr, dim) index for each sensor, in column order
        self._index = []
        for name in _SENSOR_NAMES:
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid < 0:
                raise RuntimeError(f"Sensor '{name}' not found in model.")
            self._index.append((model.sensor_adr[sid], model.sensor_dim[sid]))

        # Verify total scalar count matches expected 26 sensordata values
        total = sum(dim for _, dim in self._index)
        assert total == 26, f"Expected 26 sensordata scalars, got {total}"

        self._fh     = open(output_path, "w", newline="")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(_HEADERS)
        self._rows   = 0
        self._path   = output_path

    def step(self, data: mujoco.MjData) -> None:
        """Write one row: simulation time + all 26 sensordata values."""
        row = [f"{data.time:.6f}"]
        sd  = data.sensordata
        for adr, dim in self._index:
            for v in sd[adr:adr+dim]:
                row.append(f"{v:.6f}")
        self._writer.writerow(row)
        self._rows += 1

    def finish(self) -> pathlib.Path:
        """Flush, close, and return the output path."""
        self._fh.flush()
        self._fh.close()
        return self._path

    @property
    def rows_written(self) -> int:
        return self._rows

    @staticmethod
    def headers() -> list:
        return list(_HEADERS)
