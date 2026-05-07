# Phase 9 Execution Guide — כשחוזרים למכונה

**Date:** 2026-05-07
**Pre-condition:** `git pull` — HEAD = `6d3abf1`
**Goal:** Phase 9 characterized → commit pushed → Phase 10 unlocked

---

## מה מוכן פה ומה צריך לעשות שם

| # | משימה | קובץ מוכן? | עבודה שנשארת |
|---|-------|-----------|--------------|
| Task 1 | path fix | ✅ `phase9_patch/bridge_node_patched.py` | `cp` בלבד |
| Task 2 | lock docstring | ✅ נוסף לפאץ' | כלול ב-Task 1 |
| Task 3 | RTF logger | ✅ נוסף לפאץ' | כלול ב-Task 1 |
| Task 4 | colcon build + smoke | — | להריץ |
| Task 5 | topic rate capture | ✅ `phase9_patch/measure_topic_rates.py` | להריץ + להעתיק |
| Task 6 | RTF cameras on | — | לקרוא מ-log |
| Task 7 | RTF cameras off | — | לשנות 1 שורה + לקרוא |
| Task 8 | SESSION_LOG + PHASES.md | — | למלא ערכים |
| Task 9 | commit + push | — | 1 פקודה |

---

## סדר ביצוע — פקודות מדויקות

### שלב 1 — sync + העתקת קובץ מוכן

```bash
cd ~/Desktop/VLA_Tutorial
[ -f .git/index.lock ] && rm .git/index.lock
git reset --hard origin/main
git rev-parse --short HEAD   # expect: 6d3abf1

# העתק את הפאץ' המוכן (Tasks 1+2+3 בבת אחת)
cp "<cowork>/Notes/phase9_patch/bridge_node_patched.py" \
   VLATraining/vla_ws/src/mujoco_bridge/mujoco_bridge/bridge_node.py

# ודא:
grep -c '~/Desktop/VLA_Tutorial' VLATraining/vla_ws/src/mujoco_bridge/mujoco_bridge/bridge_node.py
# expected: 0
grep -c 'Lock contract' VLATraining/vla_ws/src/mujoco_bridge/mujoco_bridge/bridge_node.py
# expected: 1
grep -c 'step_count' VLATraining/vla_ws/src/mujoco_bridge/mujoco_bridge/bridge_node.py
# expected: 1
```

`<cowork>` = הנתיב לתיקייה הנבחרת שלך (ב-Linux: ראה `echo $VLA_COWORK` אם הגדרת, אחרת נתיב מלא).

---

### שלב 2 — build + smoke test (Terminal 1)

```bash
cd ~/Desktop/VLA_Tutorial/VLATraining/vla_ws
colcon build --packages-select mujoco_bridge
source install/setup.bash
ros2 run mujoco_bridge bridge_node
```

**לצפות ב-log כל 5 שניות:**
```
[mujoco_bridge]: sim_loop: real 943 Hz / target 1000 Hz = RTF 0.943
```

**אם השורה לא מופיעה** — הפאץ' לא הוחל. עצור ובדוק שוב.

---

### שלב 3 — מדידת topic rates (Terminal 2, bridge פועל)

```bash
# העתק את כלי המדידה לתוך הריפו
cp "<cowork>/Notes/phase9_patch/measure_topic_rates.py" \
   ~/Desktop/VLA_Tutorial/VLATraining/sim/tools/measure_topic_rates.py

# הרץ (תוך שה-bridge פועל ב-Terminal 1)
source /opt/ros/humble/setup.bash
source ~/Desktop/VLA_Tutorial/VLATraining/vla_ws/install/setup.bash

python3 ~/Desktop/VLA_Tutorial/VLATraining/sim/tools/measure_topic_rates.py \
    --duration 30 \
    --output-dir ~/Desktop/VLA_Tutorial/VLATraining/sim/outputs
```

**מה זה עושה:** מנוי לכל 10 ה-topics בו-זמנית למשך 30 שניות,
מחשב mean/min/max/stddev/jitter לכל topic, ומציג pass/fail.
**זמן ריצה: 30 שניות בדיוק**, לא 300.

**קובץ פלט:** `outputs/topic_rates_<YYYYMMDD>.txt` — נוצר אוטומטית.

> **הערה על `/em_state`:** אם ה-EM לא הופעל במהלך ה-30 שניות — יופיע `ℹ️ on-change (0 transitions)`. זה תקין. אם רוצים לבדוק, להריץ `demo_pickup_throw.py` בטרמינל נפרד כדי שה-EM יתפעל.

---

### שלב 4 — מדידת RTF cameras on (Terminal 1 עדיין פועל)

צפה ב-Terminal 1 למשך **60 שניות** (≥12 שורות של `sim_loop: real ...`).

רשום לאנשהו: mean, min, max של ה-RTF values שאתה רואה.

```bash
# שמור לקובץ:
DATE=$(date +%Y%m%d)
cat > ~/Desktop/VLA_Tutorial/VLATraining/sim/outputs/sim_rate_cameras_on_${DATE}.txt << 'EOF'
Scenario: cameras_on
Bridge commit: <החלף ב-git rev-parse --short HEAD>
Workload: bridge idle (no joint_commands)
Capture window: 60 s
RTF samples: <הדבק מה-log>
RTF mean: <חשב>
RTF min: <min מהדוגמאות>
RTF max: <max מהדוגמאות>
EOF
```

---

### שלב 5 — מדידת RTF cameras off

עצור את ה-bridge (Ctrl+C ב-Terminal 1).

```bash
# מצא את שורת יצירת ה-camera timer ב-bridge_node.py:
grep -n 'publish_cameras' VLATraining/vla_ws/src/mujoco_bridge/mujoco_bridge/bridge_node.py
```

בשורה שמוצאת — הוסף `#` בתחילתה (comment out הטיימר בלבד).

```bash
colcon build --packages-select mujoco_bridge
source install/setup.bash
ros2 run mujoco_bridge bridge_node
```

צפה 60 שניות. שמור ל-`outputs/sim_rate_cameras_off_${DATE}.txt`.

אחרי המדידה — **בטל את ה-comment** על שורת הטיימר, build שוב, ולוודא:
```bash
ros2 topic hz /camera/overhead/image_raw   # expect ~6 Hz
```

---

### שלב 6 — עדכון SESSION_LOG.md

פתח `VLATraining/sim/SESSION_LOG.md`.
מלא את הטבלה הזו (כבר קיים שם מקום):

```markdown
## Session 2026-05-06 (continued) — Phase 9 polish

### Code changes
- bridge_node.py: env-var-based sim root resolution (no more hardcoded ~/Desktop/...)
- bridge_node.py: Lock contract docstring
- bridge_node.py: _sim_loop logs real Hz and RTF every 5 s

### Bounds measured

| Scenario | RTF mean | RTF min | Pass? |
|---|---|---|---|
| Cameras on (3 × 6 Hz) | <NUM> | <NUM> | <YES/NO> |
| Cameras off           | <NUM> | <NUM> | <YES/NO> |

### Topic rates
(הדבק את תוכן topic_rates_*.txt כאן)

### Phase 9 status: ✅ characterized
```

---

### שלב 7 — commit ו-push

```bash
cd ~/Desktop/VLA_Tutorial
git add VLATraining/vla_ws/src/mujoco_bridge/mujoco_bridge/bridge_node.py
git add VLATraining/sim/SESSION_LOG.md
git add PHASES.md
git add VLATraining/sim/tools/measure_topic_rates.py
git add VLATraining/sim/outputs/topic_rates_*.txt
git add VLATraining/sim/outputs/sim_rate_cameras_on_*.txt
git add VLATraining/sim/outputs/sim_rate_cameras_off_*.txt
git status   # בדוק שרק הקבצים האלה מסומנים

git commit -m "Phase 9 polish: env-var paths, lock contract, RTF + topic rate measurements

- bridge_node.py: replace hardcoded ~/Desktop path with VLA_SIM_ROOT + dev-tree fallback
- bridge_node.py: BridgeNode Lock contract docstring
- bridge_node.py: _sim_loop logs real Hz and RTF every 5 s
- tools/measure_topic_rates.py: simultaneous rate measurement for all 10 topics
- outputs/topic_rates_*.txt: 30 s capture (all topics simultaneously)
- outputs/sim_rate_cameras_{on,off}_*.txt: 60 s RTF measurement
- SESSION_LOG.md, PHASES.md: documented bounds

Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>"

git push origin main
```

---

## קריטריוני עבור/נכשל

| יעד | target | floor |
|-----|--------|-------|
| RTF cameras on | ≥ 0.95 | < 0.70 → פאץ' אדריכלי נדרש |
| RTF cameras off | ≥ 1.5 | < 1.0 → חקור |
| Topic rate accuracy | ±5% | ±10% |
| Topic rate jitter σ/mean | < 5% | < 10% |

**RTF cameras on בין 0.70–0.95:** תעד כגבול ושלח as-is. Phase 10 יכול להתחיל.

**RTF cameras on < 0.70:** הפאץ' האדריכלי (`_data_render` snapshot) נדרש לפני Phase 10. ראה CLAUDE_CODE_TASKS.md קטע "Branch: if RTF cameras-on < 0.70".

---

## הערות

**למה `measure_topic_rates.py` ולא `ros2 topic hz` בלולאה?**

`ros2 topic hz` מודד topic אחד × 30 שניות = 5 דקות לכל 10 ה-topics.
במהלך הזמן הזה המצב של המערכת משתנה, העומס משתנה, ו-`demo_pickup_throw.py`
(אם הורץ) כבר הסתיים זמן רב לפני ה-topic ה-8.

`measure_topic_rates.py` פותח 10 subscriptions בו-זמנית ורושם timestamps
לכל הודעה שמגיעה. אחרי 30 שניות מחשב סטטיסטיקה מ-timestamps בדיוק כמו
שעושה `ros2 topic hz` — רק לכולם בו-זמנית.

---

*VLA Research | Shahar Cohen | BGU Mechatronics | 2026-05-07*
