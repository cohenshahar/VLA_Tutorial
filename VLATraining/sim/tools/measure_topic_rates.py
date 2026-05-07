#!/usr/bin/env python3
"""
measure_topic_rates.py — Phase 9 verification tool

מודד את קצב כל 10 ה-topics בו-זמנית (לא באופן סידורי) על פני חלון של 30 שניות.
כותב קובץ outputs/topic_rates_<YYYYMMDD>.txt עם mean, min, max, stddev, jitter%,
ו-pass/fail לכל topic.

Usage (bridge must be running first):
    source /opt/ros/humble/setup.bash
    source ~/Desktop/VLA_Tutorial/VLATraining/vla_ws/install/setup.bash
    python3 measure_topic_rates.py [--duration 30] [--output-dir ~/Desktop/VLA_Tutorial/VLATraining/sim/outputs]

Why not `ros2 topic hz` in a loop?
    ros2 topic hz measures one topic at a time — 10 topics × 30 s = 300 s.
    By topic 5, the system state is different from topic 1.
    This script subscribes to all 10 topics simultaneously and measures
    for exactly 30 s, then computes statistics from the recorded timestamps.

Requirements: rclpy, standard ROS2 message types (all installed with ROS2 Humble)
"""

import argparse
import math
import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState, Range, Image
from geometry_msgs.msg import WrenchStamped


# ── topic spec ────────────────────────────────────────────────────────────────
# (topic_name, msg_type, target_hz_or_None, note)
TOPICS = [
    ('/bridge/status',             String,         1.0,    ''),
    ('/joint_states',              JointState,     100.0,  ''),
    ('/joint_torques',             JointState,     100.0,  ''),
    ('/ft_sensor',                 WrenchStamped,  100.0,  ''),
    ('/proximity',                 Range,          50.0,   ''),
    ('/em_state',                  Bool,           None,   'on-change — may be 0 Hz if EM never toggled'),
    ('/target_contact',            Bool,           50.0,   ''),
    ('/camera/overhead/image_raw', Image,          6.0,    ''),
    ('/camera/side/image_raw',     Image,          6.0,    ''),
    ('/camera/wrist/image_raw',    Image,          6.0,    ''),
]

TOLERANCE_PCT = 5.0   # ±5% rate accuracy target
JITTER_PCT    = 5.0   # σ/mean < 5% jitter target


# ── measurement node ──────────────────────────────────────────────────────────
class RateMeasurer(Node):
    def __init__(self, duration: float):
        super().__init__('rate_measurer')
        self._duration = duration
        self._lock = threading.Lock()
        self._arrivals: dict[str, list[float]] = {t[0]: [] for t in TOPICS}
        self._done = threading.Event()

        # Subscribe to every topic with a timestamp-recording callback
        for topic_name, msg_type, _, _ in TOPICS:
            self.create_subscription(
                msg_type,
                topic_name,
                lambda _msg, tn=topic_name: self._record(tn),
                10,
            )

        # Single-shot timer — fires once after `duration` seconds
        self.create_timer(duration, self._on_finish)
        self.get_logger().info(
            f'[rate_measurer] Measuring {len(TOPICS)} topics for {duration:.0f} s …'
        )

    def _record(self, topic_name: str) -> None:
        now = time.monotonic()
        with self._lock:
            self._arrivals[topic_name].append(now)

    def _on_finish(self) -> None:
        self._done.set()

    def wait_for_finish(self) -> None:
        self._done.wait()

    def compute_stats(self) -> dict[str, dict]:
        """Compute per-topic rate statistics from recorded timestamps."""
        with self._lock:
            arrivals_copy = {k: list(v) for k, v in self._arrivals.items()}

        results = {}
        for topic_name, _, target_hz, note in TOPICS:
            ts = arrivals_copy[topic_name]
            n = len(ts)

            entry: dict = {
                'count': n,
                'target_hz': target_hz,
                'note': note,
            }

            if n < 2:
                entry.update({
                    'mean_hz': 0.0, 'min_hz': 0.0, 'max_hz': 0.0,
                    'stddev_hz': 0.0, 'jitter_pct': None,
                    'status': '⚠️  no data (0 msgs)' if n == 0 else '⚠️  only 1 msg',
                })
                results[topic_name] = entry
                continue

            # Instantaneous rates from consecutive inter-message intervals
            intervals = [ts[i + 1] - ts[i] for i in range(n - 1)]
            inst_rates = [1.0 / iv for iv in intervals if iv > 1e-9]

            if not inst_rates:
                entry.update({'mean_hz': 0.0, 'status': '⚠️  zero-length intervals'})
                results[topic_name] = entry
                continue

            mean_hz   = sum(inst_rates) / len(inst_rates)
            min_hz    = min(inst_rates)
            max_hz    = max(inst_rates)
            variance  = sum((r - mean_hz) ** 2 for r in inst_rates) / len(inst_rates)
            stddev_hz = math.sqrt(variance)
            jitter_pct = (stddev_hz / mean_hz * 100.0) if mean_hz > 1e-9 else None

            # Pass / fail logic
            if target_hz is None:
                status = f'ℹ️  on-change  ({n} transitions in {self._duration:.0f} s)'
            else:
                rate_err_pct = abs(mean_hz - target_hz) / target_hz * 100.0
                rate_ok  = rate_err_pct <= TOLERANCE_PCT
                jitter_ok = (jitter_pct is not None and jitter_pct < JITTER_PCT)
                if rate_ok and jitter_ok:
                    status = '✅ PASS'
                elif rate_ok and not jitter_ok:
                    status = f'⚠️  rate OK  jitter {jitter_pct:.1f}% > {JITTER_PCT}%'
                else:
                    status = f'❌ FAIL  rate {mean_hz:.1f} Hz ({rate_err_pct:.1f}% off {target_hz} Hz spec)'

            entry.update({
                'mean_hz': mean_hz,
                'min_hz': min_hz,
                'max_hz': max_hz,
                'stddev_hz': stddev_hz,
                'jitter_pct': jitter_pct,
                'status': status,
            })
            results[topic_name] = entry

        return results


# ── report writer ─────────────────────────────────────────────────────────────
def write_report(stats: dict, duration: float, output_path: Path, commit: str) -> None:
    lines = [
        '# Topic rates — Phase 9 verification',
        f'# Captured:    {datetime.now().isoformat(timespec="seconds")}',
        f'# Bridge commit: {commit}',
        f'# Window:      {duration:.0f} s (all topics measured simultaneously)',
        f'# Targets:     rate ±{TOLERANCE_PCT}%  jitter σ/mean < {JITTER_PCT}%',
        '',
        f'{"Topic":<40} {"Target":>8} {"Mean":>8} {"Min":>8} {"Max":>8} '
        f'{"Stddev":>8} {"Jitter%":>8} {"N":>6}  Status',
        '-' * 120,
    ]

    all_pass = True
    for topic_name, _, target_hz, _ in TOPICS:
        s = stats[topic_name]
        target_str = f'{target_hz:.0f} Hz' if target_hz else 'on-chg'
        mean_str   = f'{s["mean_hz"]:.1f}'  if s.get("mean_hz") else '—'
        min_str    = f'{s.get("min_hz", 0):.1f}'
        max_str    = f'{s.get("max_hz", 0):.1f}'
        std_str    = f'{s.get("stddev_hz", 0):.2f}'
        jit_str    = f'{s["jitter_pct"]:.1f}' if s.get("jitter_pct") is not None else '—'
        status     = s.get('status', '?')
        n          = s.get('count', 0)
        if '❌' in status or '⚠️' in status:
            all_pass = False
        lines.append(
            f'{topic_name:<40} {target_str:>8} {mean_str:>8} {min_str:>8} {max_str:>8} '
            f'{std_str:>8} {jit_str:>8} {n:>6}  {status}'
        )

    lines += [
        '',
        f'Overall: {"✅ ALL PASS" if all_pass else "❌ SEE ABOVE"}',
        '',
        '# Note: /em_state is on-change and may show 0 Hz if EM was never toggled.',
        '# Note: camera topics may show lower mean if camera timer fired late on startup.',
    ]

    report = '\n'.join(lines) + '\n'
    output_path.write_text(report, encoding='utf-8')
    print(report)
    print(f'\nSaved → {output_path}')


# ── main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(description='Measure ROS2 topic rates simultaneously.')
    parser.add_argument('--duration',   type=float, default=30.0,
                        help='Measurement window in seconds (default: 30)')
    parser.add_argument('--output-dir', type=str,
                        default=str(Path(__file__).resolve().parent),
                        help='Directory for output file (default: same dir as this script)')
    args = parser.parse_args()

    # Resolve output path
    out_dir = Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    date_str = datetime.now().strftime('%Y%m%d')
    out_path = out_dir / f'topic_rates_{date_str}.txt'

    # Get current git commit for provenance
    import subprocess
    try:
        commit = subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=Path(__file__).parent, stderr=subprocess.DEVNULL
        ).decode().strip()
    except Exception:
        commit = 'unknown'

    rclpy.init()
    node = RateMeasurer(duration=args.duration)

    # Spin in a background thread so wait_for_finish() blocks the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.wait_for_finish()

    stats = node.compute_stats()
    write_report(stats, args.duration, out_path, commit)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
