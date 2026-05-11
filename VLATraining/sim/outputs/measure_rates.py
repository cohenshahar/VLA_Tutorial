#!/usr/bin/env python3
"""
Topic rate measurement script — Phase 9 Task 5
Measures each topic one at a time: 4s warm-up + 10s count window.
Run with the bridge node already running in another terminal.
"""
import sys, time, threading, collections, os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState, Range, Image
from geometry_msgs.msg import WrenchStamped

TOPICS = [
    ('/bridge/status',             String,        1),
    ('/joint_states',              JointState,    100),
    ('/joint_torques',             JointState,    100),
    ('/ft_sensor',                 WrenchStamped, 100),
    ('/proximity',                 Range,         50),
    ('/em_state',                  Bool,          None),
    ('/target_contact',            Bool,          50),
    ('/camera/overhead/image_raw', Image,         6),
    ('/camera/side/image_raw',     Image,         6),
    ('/camera/wrist/image_raw',    Image,         6),
]
WARMUP = 4   # seconds for DDS discovery
WINDOW = 10  # seconds to count messages

results = []

for topic, msg_type, spec in TOPICS:
    count = [0]
    measuring = threading.Event()

    rclpy.init()
    node = rclpy.create_node('rate_meas_' + topic.replace('/', '_').strip('_'))

    sub = node.create_subscription(
        msg_type, topic,
        lambda msg: count.__setitem__(0, count[0] + 1) if measuring.is_set() else None,
        100)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    sys.stdout.write(f'  {topic:<42}  warming up {WARMUP}s... ')
    sys.stdout.flush()
    time.sleep(WARMUP)

    measuring.set()
    t0 = time.monotonic()
    time.sleep(WINDOW)
    elapsed = time.monotonic() - t0

    hz = count[0] / elapsed

    rclpy.shutdown()
    spin_thread.join(timeout=2)

    if spec:
        diff_pct = (hz - spec) / spec * 100
        flag = '✅' if abs(diff_pct) <= 5 else ('⚠️ ' if abs(diff_pct) <= 10 else '❌ ')
        line = f'{hz:6.1f} Hz  spec={spec:3}  {diff_pct:+6.1f}%  {flag}'
    else:
        flag = '—'
        line = f'{hz:6.1f} Hz  spec=on-change  {flag}'

    print(line)
    results.append((topic, hz, spec))

# ── Write output file ──────────────────────────────────────────────────────
import subprocess, datetime
out_dir = os.path.dirname(os.path.abspath(__file__))
date_str = datetime.date.today().strftime('%Y%m%d')
out_file = os.path.join(out_dir, f'topic_rates_{date_str}.txt')

try:
    commit = subprocess.check_output(
        ['git', '-C', out_dir, 'rev-parse', '--short', 'HEAD'],
        stderr=subprocess.DEVNULL).decode().strip()
except Exception:
    commit = 'unknown'

with open(out_file, 'w') as f:
    f.write('# Topic rates — Phase 9 verification\n')
    f.write(f'# Captured: {datetime.datetime.now().isoformat()}\n')
    f.write(f'# Bridge commit: {commit}\n')
    f.write(f'# Workload: bridge only\n')
    f.write(f'# Measurement: {WARMUP}s warm-up + {WINDOW}s count window per topic\n\n')
    f.write(f'{"Topic":<42}  {"Hz":>7}  {"Spec":>8}  {"Diff%":>7}  Flag\n')
    f.write('-' * 75 + '\n')
    for topic, hz, spec in results:
        if spec:
            diff_pct = (hz - spec) / spec * 100
            flag = '✅' if abs(diff_pct) <= 5 else ('⚠️ ' if abs(diff_pct) <= 10 else '❌')
        else:
            diff_pct = None
            flag = '—'
        diff_str = f'{diff_pct:+6.1f}%' if diff_pct is not None else '      —'
        spec_str = str(spec) if spec else 'on-chg'
        f.write(f'{topic:<42}  {hz:>7.1f}  {spec_str:>8}  {diff_str}  {flag}\n')

print(f'\nResults saved to: {out_file}')
