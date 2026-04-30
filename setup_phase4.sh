#!/bin/bash
# =============================================================
#  Phase 4 — Robot Simulation Stack Setup
#  Ubuntu 22 עם display
#  Usage: bash setup_phase4.sh
# =============================================================

set -e

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║   Phase 4 — Robot Simulation Stack Setup         ║"
echo "║   MuJoCo · dm_control · Keyboard Control         ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""

# ── System dependencies ────────────────────────────────────
echo "[0/6] Installing system dependencies..."
sudo apt-get update -qq
sudo apt-get install -y \
    libglfw3 libglfw3-dev \
    libgl1-mesa-dev \
    libxrandr-dev libxinerama-dev \
    libxcursor-dev libxi-dev \
    ffmpeg \
    2>/dev/null || echo "  (some packages may already be installed)"

# ── Virtual environment ────────────────────────────────────
echo "[1/6] Creating virtual environment: phase4_env"
python3 -m venv phase4_env
source phase4_env/bin/activate
pip install --upgrade pip --quiet

# ── Core stack ─────────────────────────────────────────────
echo "[2/6] Installing PyTorch (CPU)..."
pip install torch --index-url https://download.pytorch.org/whl/cpu --quiet

echo "[3/6] Installing MuJoCo..."
pip install mujoco --quiet

echo "[4/6] Installing dm_control..."
pip install dm_control --quiet

echo "[5/6] Installing Gymnasium Robotics..."
pip install "gymnasium[robotics]" --quiet

echo "[6/6] Installing keyboard + visualization tools..."
pip install pynput pygame numpy matplotlib scipy --quiet

# ── Verification ───────────────────────────────────────────
echo ""
echo "─── Verifying installations ─────────────────────────"
python3 << 'EOF'
try:
    import mujoco
    print(f"  ✅ MuJoCo      {mujoco.__version__}")
except Exception as e:
    print(f"  ❌ MuJoCo:     {e}")

try:
    import dm_control
    print(f"  ✅ dm_control  {dm_control.__version__}")
except Exception as e:
    print(f"  ❌ dm_control: {e}")

try:
    import gymnasium
    print(f"  ✅ Gymnasium   {gymnasium.__version__}")
except Exception as e:
    print(f"  ❌ Gymnasium:  {e}")

try:
    import pynput
    print(f"  ✅ pynput      (keyboard control)")
except Exception as e:
    print(f"  ❌ pynput:     {e}")

try:
    import pygame
    print(f"  ✅ pygame      {pygame.__version__}")
except Exception as e:
    print(f"  ❌ pygame:     {e}")

print()
print("  Ready for interactive robot simulation!")
EOF

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║  Setup complete! בכל terminal חדש:               ║"
echo "║    source phase4_env/bin/activate                ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""
