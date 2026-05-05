"""
scene/camera_utils.py
─────────────────────────────────────────────────────────────────────────────
Phase 8, Tasks 8.8 — Camera rendering utility + CameraPublisher.

Public API
----------
render_camera(model, data, cam_name, width=640, height=480) -> np.ndarray
    Render one frame from the named camera. Returns uint8 RGB (H, W, 3).

CameraPublisher(camera_names, width=640, height=480)
    .get_frames(model, data) -> dict[str, np.ndarray]
        Returns {cam_name: uint8 RGB array} for every registered camera.
    .close()
        Release the offscreen renderer.

Usage (standalone test):
    python -m scene.camera_utils
"""

import os
import numpy as np
import mujoco


# ── module-level renderer cache ───────────────────────────────────────────
# One renderer is shared for all render_camera() calls with the same
# (model, width, height) signature to avoid re-creating the GL context.

_renderer_cache: dict = {}   # key: (id(model), width, height) → Renderer


def _get_renderer(model: mujoco.MjModel, width: int, height: int) -> mujoco.Renderer:
    key = (id(model), width, height)
    if key not in _renderer_cache:
        _renderer_cache[key] = mujoco.Renderer(model, height=height, width=width)
    return _renderer_cache[key]


def render_camera(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    cam_name: str,
    width: int = 640,
    height: int = 480,
) -> np.ndarray:
    """Render one offscreen frame from *cam_name*.

    Parameters
    ----------
    model, data : MjModel / MjData
    cam_name : str
        Must match a camera name defined in the XML (e.g. 'cam_overhead').
    width, height : int
        Output resolution in pixels.

    Returns
    -------
    np.ndarray
        uint8 RGB array of shape (height, width, 3).
    """
    renderer = _get_renderer(model, width, height)
    renderer.update_scene(data, camera=cam_name)
    return renderer.render().copy()


# ── CameraPublisher ───────────────────────────────────────────────────────

class CameraPublisher:
    """
    Holds a list of camera names and renders all of them on demand.

    Designed to be called once per simulation step from the ROS2 bridge
    (Phase 9), but works equally well in headless test scripts.

    Parameters
    ----------
    camera_names : list[str]
        Camera names as defined in the XML.
        Defaults to the three Phase 8 cameras:
        ['cam_overhead', 'cam_side', 'cam_wrist'].
    width, height : int
        Rendering resolution for every camera (pixels).
    """

    DEFAULT_CAMERAS = ["cam_overhead", "cam_side", "cam_wrist"]

    def __init__(
        self,
        camera_names: list | None = None,
        width: int = 640,
        height: int = 480,
    ):
        self._cameras = list(camera_names or self.DEFAULT_CAMERAS)
        self._width   = width
        self._height  = height
        self._renderer: mujoco.Renderer | None = None

    # lazy-init renderer on first call (model is not known at construction time)
    def _ensure_renderer(self, model: mujoco.MjModel) -> mujoco.Renderer:
        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                model, height=self._height, width=self._width
            )
        return self._renderer

    def get_frames(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
    ) -> dict[str, np.ndarray]:
        """Render all registered cameras and return a frame dict.

        Returns
        -------
        dict[str, np.ndarray]
            Maps each camera name to its uint8 RGB frame of shape
            (height, width, 3).
        """
        renderer = self._ensure_renderer(model)
        frames = {}
        for cam in self._cameras:
            renderer.update_scene(data, camera=cam)
            frames[cam] = renderer.render().copy()
        return frames

    def close(self) -> None:
        """Release the offscreen renderer context."""
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    @property
    def camera_names(self) -> list[str]:
        return list(self._cameras)

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height


# ── standalone test ───────────────────────────────────────────────────────

if __name__ == "__main__":
    import math
    import pathlib
    import sys

    SIM_DIR = pathlib.Path(__file__).parent.parent
    sys.path.insert(0, str(SIM_DIR))
    from arm.load_arm import apply_gains

    WORLD = str(SIM_DIR / "scene" / "world.xml")
    OUT   = SIM_DIR / "outputs"
    OUT.mkdir(parents=True, exist_ok=True)

    print("Loading model …")
    model = mujoco.MjModel.from_xml_path(WORLD)
    data  = mujoco.MjData(model)

    apply_gains(model, {
        "act_a1": (800., 80.), "act_a2": (800., 80.), "act_a3": (500., 50.),
        "act_a4": (300., 30.), "act_a5": (500., 50.), "act_a6": (300., 30.),
    })

    # Snap pose so wrist camera has something interesting to look at
    SNAP = {"a1": 0, "a2": -55, "a3": 90, "a4": 0, "a5": 55, "a6": 0}
    for name, deg in SNAP.items():
        data.ctrl[model.actuator(f"act_{name}").id] = math.radians(deg)
    for _ in range(3000):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    # ── Task 8.8a: render_camera() ────────────────────────────────────────
    print("\nTask 8.8a — render_camera()")
    for cam_name in ["cam_overhead", "cam_side", "cam_wrist"]:
        frame = render_camera(model, data, cam_name, width=640, height=480)
        assert frame.shape == (480, 640, 3), f"Unexpected shape: {frame.shape}"
        assert frame.dtype == np.uint8
        out_path = OUT / f"cam_{cam_name.replace('cam_', '')}_test.png"
        # save with PIL (avoids opencv dependency)
        try:
            from PIL import Image
            Image.fromarray(frame).save(out_path)
            print(f"  {cam_name}: shape={frame.shape}  saved → {out_path.name}")
        except ImportError:
            import struct, zlib
            # minimal PNG writer — no external dep
            def _write_png(path, arr):
                h, w = arr.shape[:2]
                raw = b"".join(b"\x00" + arr[y].tobytes() for y in range(h))
                def chunk(tag, data):
                    crc = zlib.crc32(tag + data) & 0xFFFFFFFF
                    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)
                png = (b"\x89PNG\r\n\x1a\n"
                       + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                       + chunk(b"IDAT", zlib.compress(raw, 9))
                       + chunk(b"IEND", b""))
                with open(path, "wb") as f:
                    f.write(png)
            _write_png(out_path, frame)
            print(f"  {cam_name}: shape={frame.shape}  saved → {out_path.name}")

    # ── Task 8.8b: CameraPublisher.get_frames() ───────────────────────────
    print("\nTask 8.8b — CameraPublisher.get_frames()")
    pub = CameraPublisher()
    frames = pub.get_frames(model, data)

    assert set(frames.keys()) == {"cam_overhead", "cam_side", "cam_wrist"}, \
        f"Unexpected keys: {set(frames.keys())}"
    for name, arr in frames.items():
        assert arr.shape == (480, 640, 3), f"{name}: bad shape {arr.shape}"
        assert arr.dtype == np.uint8
        print(f"  {name}: shape={arr.shape}  dtype={arr.dtype}  ✓")

    pub.close()

    print("\n══════════════════════════════════════════════")
    print("  Phase 8 (Task 8.8) CameraPublisher: PASS ✓")
    print("══════════════════════════════════════════════")
