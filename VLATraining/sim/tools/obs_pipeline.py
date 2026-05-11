"""
tools/obs_pipeline.py  —  Phase 10, Task 10.7
═══════════════════════════════════════════════════════════════════════════════
ObsPipeline: convert a raw camera frame into the observation dict expected by
the Octo VLA model.

Octo observation format
  "image"       : np.ndarray shape (256, 256, 3), dtype float32, values [0, 1]
  "instruction" : str

No ROS2 dependency — runs standalone (offline preprocessing).

Standalone test:
  python3 VLATraining/sim/tools/obs_pipeline.py
"""

import pathlib
import sys

import numpy as np

# ── Constants ─────────────────────────────────────────────────────────────────
TARGET_H = 256
TARGET_W = 256


class ObsPipeline:
    """
    Preprocess raw camera frames into Octo-compatible observation dicts.

    Usage
    -----
    pipeline = ObsPipeline()
    obs = pipeline.get_observation(image_bgr, instruction="pick up the box")
    # obs["image"]  → np.ndarray (256, 256, 3) float32 [0, 1]
    # obs["instruction"]  → str

    Notes
    -----
    • Input must be a BGR numpy array of any spatial size (H×W×3, uint8 or
      float32).  Any other dtype is accepted but treated as [0, 255] scale if
      uint8, or [0, 1] scale if float32 / float64.
    • Resize uses nearest-neighbour to avoid a heavy dependency on Pillow or
      OpenCV.  If cv2 is installed, bilinear resize is used automatically.
    """

    def __init__(self):
        self._use_cv2 = self._check_cv2()

    # ── Public API ────────────────────────────────────────────────────────────

    def get_observation(self, image_bgr: np.ndarray,
                        instruction: str) -> dict:
        """
        Convert a BGR image and instruction string into an Octo observation.

        Parameters
        ----------
        image_bgr   : (H, W, 3) numpy array, BGR channel order.
        instruction : str  Language command for the VLA model.

        Returns
        -------
        dict with keys:
            "image"       : np.ndarray (256, 256, 3) float32 RGB, values [0, 1]
            "instruction" : str
        """
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise ValueError(
                f"image_bgr must be (H, W, 3); got shape {image_bgr.shape}")

        # BGR → RGB
        rgb = image_bgr[:, :, ::-1].copy()

        # Ensure uint8 for consistent normalisation
        if rgb.dtype != np.uint8:
            rgb = np.clip(rgb, 0, 255).astype(np.uint8)

        # Resize to 256×256
        rgb = self._resize(rgb, TARGET_H, TARGET_W)

        # Normalise to [0, 1] float32
        rgb_f32 = rgb.astype(np.float32) / 255.0

        return {
            "image":       rgb_f32,
            "instruction": instruction,
        }

    # ── Private helpers ───────────────────────────────────────────────────────

    @staticmethod
    def _check_cv2() -> bool:
        try:
            import cv2   # noqa: F401
            return True
        except ImportError:
            return False

    def _resize(self, img: np.ndarray, h: int, w: int) -> np.ndarray:
        """Resize img to (h, w, 3) using cv2 bilinear if available."""
        if img.shape[0] == h and img.shape[1] == w:
            return img
        if self._use_cv2:
            import cv2
            return cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)
        # Pure-numpy nearest-neighbour fallback
        row_idx = (np.arange(h) * img.shape[0] / h).astype(int)
        col_idx = (np.arange(w) * img.shape[1] / w).astype(int)
        return img[np.ix_(row_idx, col_idx)]


# ══════════════════════════════════════════════════════════════════════════════
#  Standalone test
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    """Smoke-test ObsPipeline without ROS2."""
    pipeline = ObsPipeline()

    # Test 1 — random uint8 image any size
    rng  = np.random.default_rng(42)
    fake = rng.integers(0, 256, (480, 640, 3), dtype=np.uint8)   # 640×480 BGR
    obs  = pipeline.get_observation(fake, instruction="pick up the metal box")

    print("── ObsPipeline smoke test ──────────────────────────────────")
    print(f"  Input  : shape={fake.shape}, dtype={fake.dtype}")
    print(f"  Output image : shape={obs['image'].shape}, "
          f"dtype={obs['image'].dtype}, "
          f"min={obs['image'].min():.3f}, max={obs['image'].max():.3f}")
    print(f"  Instruction  : '{obs['instruction']}'")

    # Assertions
    assert obs["image"].shape   == (TARGET_H, TARGET_W, 3), "Shape mismatch"
    assert obs["image"].dtype   == np.float32,              "dtype must be float32"
    assert obs["image"].min()   >= 0.0,                     "values must be >= 0"
    assert obs["image"].max()   <= 1.0,                     "values must be <= 1"
    assert isinstance(obs["instruction"], str),             "instruction must be str"

    # Test 2 — already 256×256 (should pass through without resize)
    small = rng.integers(0, 256, (256, 256, 3), dtype=np.uint8)
    obs2  = pipeline.get_observation(small, "no resize needed")
    assert obs2["image"].shape == (256, 256, 3)

    print("\n  ✅ All assertions passed.")
    print(f"  cv2 available : {pipeline._use_cv2}")


if __name__ == "__main__":
    main()
