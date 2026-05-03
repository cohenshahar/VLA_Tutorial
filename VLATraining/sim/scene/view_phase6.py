"""
scene/view_phase6.py — Phase 6 live viewer (loops forever).
Run: PYTHONPATH=. python scene/view_phase6.py
"""
import math, pathlib, time, sys
import mujoco, mujoco.viewer
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from arm.load_arm import apply_gains
from scene.em_controller import em_activate, em_deactivate

_GAINS = {"act_a1":(800.,80.),"act_a2":(800.,80.),"act_a3":(500.,50.),
          "act_a4":(300.,30.),"act_a5":(500.,50.),"act_a6":(300.,30.)}
# EM face points straight down (-Z) in snap pose
HOME  = {"a1":0,"a2":0,  "a3":0, "a4":0,"a5":0, "a6":0}
SNAP  = {"a1":0,"a2":-55,"a3":90,"a4":0,"a5":55,"a6":0}
LIFT  = {"a1":0,"a2":-65,"a3":90,"a4":0,"a5":55,"a6":0}
CARRY = {"a1":45,"a2":-65,"a3":90,"a4":0,"a5":55,"a6":0}
WORLD = str(pathlib.Path(__file__).parent/"world.xml")
RENDER_EVERY = 16

def ctrl(data, model, pose):
    for k,v in pose.items():
        data.ctrl[model.actuator("act_"+k).id] = math.radians(v)

def main():
    model = mujoco.MjModel.from_xml_path(WORLD)
    data  = mujoco.MjData(model)
    apply_gains(model, _GAINS)
    DT = model.opt.timestep
    em_id = model.site("em_contact_site").id
    jnt   = model.joint("box_freejoint").id
    qpa   = model.jnt_qposadr[jnt]
    doa   = model.jnt_dofadr[jnt]

    ctrl(data, model, SNAP)
    for _ in range(3000): mujoco.mj_step(model, data)
    snap = data.site_xpos[em_id].copy()

    def reset():
        mujoco.mj_resetData(model, data)
        apply_gains(model, _GAINS)
        # Place box on table directly below EM contact site
        bz = snap[2] - 0.05 - 0.015
        data.qpos[qpa:qpa+3]   = [snap[0], snap[1], bz]
        data.qpos[qpa+3:qpa+7] = [1, 0, 0, 0]
        mujoco.mj_forward(model, data)
        return data.qpos[qpa:qpa+7].copy()

    plen  = [int(2/DT), int(3/DT), int(2/DT), int(2/DT), int(2/DT), int(1/DT)]
    pname = ["HOME", "DESCEND -> EM activates", "LIFT", "CARRY A1+45", "RETURN+RELEASE", "PAUSE"]
    bi    = reset()
    phase = 0; pstep = 0; em_on = False
    ctrl(data, model, HOME)
    print(f"\n  Phase 0: {pname[0]}")

    with mujoco.viewer.launch_passive(model, data) as v:
        v.cam.lookat[:] = [snap[0]*0.7, snap[1]-0.05, 1.1]
        v.cam.distance  = 1.8
        v.cam.azimuth   = 150
        v.cam.elevation = -25
        tr = time.time()
        while v.is_running():
            if phase == 0:
                data.qpos[qpa:qpa+7] = bi
                data.qvel[doa:doa+6] = 0.
                mujoco.mj_forward(model, data)
            elif phase == 1:
                if not em_on:
                    data.qpos[qpa:qpa+7] = bi
                    data.qvel[doa:doa+6] = 0.
                    mujoco.mj_forward(model, data)
                    if em_activate(model, data, threshold_m=0.05):
                        em_on = True
                        print("  *** EM ACTIVATED ***")
            elif phase == 4 and pstep == 0:
                em_deactivate(model, data)
                em_on = False
                data.qvel[:] = 0.
                print("  *** EM DEACTIVATED — box drops ***")
            mujoco.mj_step(model, data)
            pstep += 1
            if pstep >= plen[phase]:
                phase = (phase+1) % len(plen); pstep = 0
                print(f"\n  Phase {phase}: {pname[phase]}")
                if   phase == 0: bi = reset(); em_on = False; ctrl(data, model, HOME)
                elif phase == 1: ctrl(data, model, SNAP)
                elif phase == 2: ctrl(data, model, LIFT)
                elif phase == 3: ctrl(data, model, CARRY)
                elif phase == 4: ctrl(data, model, SNAP)
            if pstep % RENDER_EVERY == 0:
                v.sync()
                tgt = tr + RENDER_EVERY * DT
                now = time.time()
                if tgt > now: time.sleep(tgt - now)
                tr = time.time()

if __name__ == "__main__":
    main()

