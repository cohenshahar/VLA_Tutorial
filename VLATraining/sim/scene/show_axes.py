"""
scene/show_axes.py  — visualise axis misalignment between EM pad and box top.
RED   arrow = link_6 X axis (EM tool direction / face normal)
GREEN arrow = box top-face normal (world +Z)
Run:  python -m scene.show_axes
"""
import math, pathlib
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import mujoco

_GAINS = {"act_a1":(800.,80.),"act_a2":(800.,80.),"act_a3":(500.,50.),
          "act_a4":(300.,30.),"act_a5":(500.,50.),"act_a6":(300.,30.)}

def _apply_gains(model):
    for n,(kp,kv) in _GAINS.items():
        ai=model.actuator(n).id
        model.actuator_gainprm[ai,0]=kp; model.actuator_biasprm[ai,1]=-kp; model.actuator_biasprm[ai,2]=-kv
    for i in range(model.nv): model.dof_armature[i]=0.5

def _project(pt_world, cam_pos, cam_fwd, cam_up, near, frustum_bottom, frustum_top, frustum_center, frustum_width, W, H):
    right = np.cross(cam_fwd, cam_up)
    V = np.array([[right[0],right[1],right[2],-np.dot(right,cam_pos)],
                  [cam_up[0],cam_up[1],cam_up[2],-np.dot(cam_up,cam_pos)],
                  [-cam_fwd[0],-cam_fwd[1],-cam_fwd[2],np.dot(cam_fwd,cam_pos)],
                  [0,0,0,1]])
    l = frustum_center - frustum_width/2
    r = frustum_center + frustum_width/2
    b = frustum_bottom; t = frustum_top
    n = near; f = near * 1000
    P = np.array([[2*n/(r-l),0,(r+l)/(r-l),0],
                  [0,2*n/(t-b),(t+b)/(t-b),0],
                  [0,0,-(f+n)/(f-n),-2*f*n/(f-n)],
                  [0,0,-1,0]])
    pw4 = np.array([pt_world[0],pt_world[1],pt_world[2],1.0])
    clip = P @ V @ pw4
    if clip[3] <= 0: return None
    ndc = clip[:3]/clip[3]
    return (int((ndc[0]*0.5+0.5)*W), int((-ndc[1]*0.5+0.5)*H))

def _arrow(draw, p0, p1, color, w=5, hs=22):
    if not p0 or not p1: return
    draw.line([p0, p1], fill=color, width=w)
    dx,dy = p1[0]-p0[0], p1[1]-p0[1]
    L = math.hypot(dx,dy)
    if L<1: return
    ux,uy = dx/L, dy/L
    px,py = -uy, ux
    h=hs
    b1=(int(p1[0]-h*ux+h*.4*px), int(p1[1]-h*uy+h*.4*py))
    b2=(int(p1[0]-h*ux-h*.4*px), int(p1[1]-h*uy-h*.4*py))
    draw.polygon([p1,b1,b2], fill=color)

def main():
    sim_dir = pathlib.Path(__file__).parent
    out_dir  = sim_dir.parent/"outputs"; out_dir.mkdir(exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(sim_dir/"world.xml"))
    data  = mujoco.MjData(model)
    _apply_gains(model)

    SNAP = {"a1":0,"a2":-60,"a3":50,"a4":0,"a5":30,"a6":0}
    for jn,deg in SNAP.items():
        data.ctrl[model.actuator("act_"+jn).id]=math.radians(deg)
    for _ in range(3000): mujoco.mj_step(model,data)

    em_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "em_contact_site")
    box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "metal_box")
    l6_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_6")

    em_pos   = data.site_xpos[em_id].copy()
    box_pos  = data.xpos[box_id].copy()
    box_top  = box_pos + np.array([0,0,0.05])
    link6_x  = data.xmat[l6_id].reshape(3,3)[:,0]
    world_z  = np.array([0.,0.,1.])

    # place box under EM
    adr = model.jnt_qposadr[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,"box_freejoint")]
    data.qpos[adr:adr+3]=[em_pos[0],em_pos[1],em_pos[2]-0.05-0.03]
    data.qpos[adr+3:adr+7]=[1,0,0,0]
    mujoco.mj_forward(model,data)
    em_pos  = data.site_xpos[em_id].copy()
    box_top = data.xpos[box_id]+np.array([0,0,0.05])

    W,H = 1280,720
    cam = mujoco.MjvCamera()
    cam.type=mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:]=(em_pos+box_top)*0.5
    cam.distance=1.3; cam.azimuth=200; cam.elevation=-18

    renderer = mujoco.Renderer(model,height=H,width=W)
    renderer.update_scene(data, camera=cam)
    img = Image.fromarray(renderer.render())

    # Get camera params from populated scene
    scn=renderer._scene
    c=scn.camera[0]
    cam_pos=np.array(c.pos); cam_fwd=np.array(c.forward); cam_up=np.array(c.up)
    near=c.frustum_near; fb=c.frustum_bottom; ft=c.frustum_top
    fc=c.frustum_center; fw=c.frustum_width
    # When frustum_width==0 the horizontal FOV is derived from aspect ratio
    if fw == 0.0:
        fw = (ft - fb) * (W / H)
        fc = 0.0
    print(f"frustum: near={near:.4f} bottom={fb:.4f} top={ft:.4f} center={fc:.4f} width={fw:.4f}")

    def w2p(pt): return _project(pt,cam_pos,cam_fwd,cam_up,near,fb,ft,fc,fw,W,H)

    AL = 0.18
    p_em_base = w2p(em_pos)
    p_em_tip  = w2p(em_pos + link6_x*AL)
    p_bx_base = w2p(box_top)
    p_bx_tip  = w2p(box_top + world_z*AL)

    draw = ImageDraw.Draw(img)
    _arrow(draw, p_em_base, p_em_tip,  (255,50,50),  w=6, hs=24)  # RED
    _arrow(draw, p_bx_base, p_bx_tip,  (50,230,50),  w=6, hs=24)  # GREEN

    try:
        fb_font=ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",24)
        fn_font=ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",19)
    except: fb_font=fn_font=ImageFont.load_default()

    angle=math.degrees(math.acos(float(np.clip(np.dot(link6_x,world_z),-1,1))))
    draw.rectangle([(0,0),(W,110)], fill=(15,15,15))
    draw.text((14, 5), "RED   = link_6 X axis  (EM face-normal / tool direction)",        fill=(255,70,70),  font=fn_font)
    draw.text((14,30), "GREEN = box top-face normal  (world +Z — what must face the EM)", fill=(50,220,50),  font=fn_font)
    draw.text((14,55), f"Angle between them: {angle:.1f}°  →  need 180° so they face each other for flush contact", fill=(255,215,0), font=fb_font)
    draw.text((14,88), f"link_6 X = {np.round(link6_x,3)}    box top = {world_z}", fill=(170,170,170), font=fn_font)

    out=out_dir/"axis_comparison.png"; img.save(out)
    print(f"\nSaved {out}")
    print(f"link_6 X  = {link6_x.round(3)}")
    print(f"box top+Z = {world_z}")
    print(f"Angle     = {angle:.1f}°  (need 180° for flush face-to-face contact)")

if __name__=="__main__":
    main()
