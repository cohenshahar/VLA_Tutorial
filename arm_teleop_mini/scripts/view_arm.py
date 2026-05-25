"""Standalone MuJoCo viewer for the KR6 — Phase 2 smoke test."""

from pathlib import Path

import mujoco
import mujoco.viewer

_MODEL_PATH = Path(__file__).parent.parent / "models" / "kr6.xml"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(_MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()

# *VLA Research | Shahar Cohen | BGU Mechatronics | 2026-05-23*
