import mujoco
import mujoco.viewer
import time

#load scene
model_path = "/home/ayu/Franka_panda/panda_pick_place_scene.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

#define sequence
states = [
    "MOVE_ABOVE_CUBE",
    "LOWER",
    "GRASP",
    "LIFT",
    "MOVE_TO_TARGET",
    "LOWER_TO_TARGET",
    "RELEASE",
    "HOME"
]

current_state = 0

#Helper function
def next_state():
    global current_state

    if current_state < len(states) - 1:
        current_state += 1
        print(f"\n -> {states[current_state]}")


print(f"Starting in {states[current_state]}")

with mujoco.viewer.launch_passive(model, data) as viewer:
    last_change = time.time()

    while viewer.is_running():
        mujoco.mj_step(model, data)

        # change state every 2 seconds
        if time.time() - last_change > 2:
            next_state()
            last_change = time.time()  

        viewer.sync()
        