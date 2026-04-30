"""Temporary viewer — table only, no arm, no box (Task 2.2 check)."""
import mujoco
import mujoco.viewer

XML = """
<mujoco>
  <option timestep="0.001" gravity="0 0 -9.81"/>
  <asset>
    <material name="table_wood" rgba="0.85 0.78 0.65 1" specular="0.1" shininess="0.15"/>
    <material name="table_leg"  rgba="0.6 0.55 0.45 1"  specular="0.05" shininess="0.1"/>
  </asset>
  <worldbody>
    <light name="key"  pos="0 0 3"    dir="0 -0.3 -1"   directional="true" diffuse="0.8 0.8 0.8"/>
    <light name="fill" pos="-2 2 2"   dir="0.5 -0.5 -1" directional="true" diffuse="0.3 0.3 0.3"/>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.5 0.5 0.5 1"/>
    <body name="table" pos="0 0 0">
      <geom name="tabletop" type="box" size="0.90 0.40 0.025" pos="0 0 0.825" material="table_wood"/>
      <geom name="leg_fl" type="cylinder" fromto=" 0.85  0.35 0  0.85  0.35 0.825" size="0.04" material="table_leg"/>
      <geom name="leg_fr" type="cylinder" fromto=" 0.85 -0.35 0  0.85 -0.35 0.825" size="0.04" material="table_leg"/>
      <geom name="leg_bl" type="cylinder" fromto="-0.85  0.35 0 -0.85  0.35 0.825" size="0.04" material="table_leg"/>
      <geom name="leg_br" type="cylinder" fromto="-0.85 -0.35 0 -0.85 -0.35 0.825" size="0.04" material="table_leg"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(XML)
data  = mujoco.MjData(model)
mujoco.viewer.launch(model, data)
