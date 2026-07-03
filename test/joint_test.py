import mujoco
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path("/home/ayu/Franka_panda/panda_pick_place_scene.xml")   
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        
        # rotate joint 1
        data.ctrl[0] = 0.85 
        mujoco.mj_step(model, data)
        viewer.sync()