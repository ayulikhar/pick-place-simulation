import mujoco
import mujoco.viewer
import time
import numpy as np
import cv2

# Load scene
model_path = "/home/ayu/Franka_panda/panda_pick_place_scene.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Keep the runtime cube pose synchronized with panda_pick_place_scene.xml.
CUBE_POSITION = np.array([0.5, 0.0, 0.03])
cube_body_id = model.body("cube").id
cube_joint_id = model.body_jntadr[cube_body_id]
cube_qposadr = model.jnt_qposadr[cube_joint_id]
data.qpos[cube_qposadr:cube_qposadr + 3] = CUBE_POSITION
mujoco.mj_forward(model, data)

# Wrist camera renderer setup
renderer = mujoco.Renderer(model, height=480, width=640)
WRIST_CAM_NAME = "wrist_cam"
WRIST_CAM_WINDOW = "Wrist Camera"

# Joint targets for each phase of the pick-and-place sequence.
# Order: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
# joint4 ctrlrange is (-3.0718, -0.0698) so it must stay negative.
# joint6 ctrlrange is (-0.0175, 3.7525) so it must stay positive.
ARM_TARGETS = {
    "NEUTRAL":          [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
    "MOVE_ABOVE_CUBE":  [0.0,  0.30, 0.0, -2.2, 0.0, 2.5, 0.785],
    # Centers the fingertip pads on the cube at [0.50, 0.00, 0.03].
    "LOWER":            [0.0,  0.5134, 0.0, -2.1911, 0.0, 2.4648, 0.785],
    "GRASP":            [0.0,  0.5134, 0.0, -2.1911, 0.0, 2.4648, 0.785],
    "LIFT":             [0.0,  0.30, 0.0, -2.2, 0.0, 2.5, 0.785],
    "MOVE_TO_TARGET":   [0.5,  0.30, 0.0, -2.2, 0.0, 2.5, 0.785],
    "LOWER_TO_TARGET":  [0.5,  0.65, 0.0, -2.0, 0.0, 2.6, 0.785],
    "RELEASE":          [0.5,  0.65, 0.0, -2.0, 0.0, 2.6, 0.785],
    "LIFT_GRIPPER":     [0.5,  0.45, 0.0, -2.1, 0.0, 2.55, 0.785],
    "LIFT_BEFORE_HOME": [0.5,  0.20, 0.0, -2.2, 0.0, 2.5, 0.785],
    "HOME":             [0.0,  0.0,  0.0, -1.57, 0.0, 1.57, 0.785],
}

# Gripper actuator8 ctrlrange is 0..255. 255 = open, 0 = closed.
GRIPPER_TARGETS = {
    "NEUTRAL":          255,
    "MOVE_ABOVE_CUBE":  255,
    "LOWER":            255,
    "GRASP":              0,
    "LIFT":               0,
    "MOVE_TO_TARGET":     0,
    "LOWER_TO_TARGET":    0,
    "RELEASE":          255,
    "LIFT_GRIPPER":     255,
    "LIFT_BEFORE_HOME": 255,
    "HOME":             255,
}

# How long to hold each phase (seconds)
PHASE_DURATIONS = {
    "NEUTRAL":          5.0,
    "MOVE_ABOVE_CUBE":  5.0,
    "LOWER":            4.0,
    "GRASP":            3.0,
    "LIFT":             4.0,
    "MOVE_TO_TARGET":   5.0,
    "LOWER_TO_TARGET":  4.0,
    "RELEASE":          2.0,
    "LIFT_GRIPPER":     3.0,
    "LIFT_BEFORE_HOME": 4.0,
    "HOME":             6.0,
}

states = [
    "NEUTRAL",
    "MOVE_ABOVE_CUBE",
    "LOWER",
    "GRASP",
    "LIFT",
    "MOVE_TO_TARGET",
    "LOWER_TO_TARGET",
    "RELEASE",
    "LIFT_GRIPPER",
    "LIFT_BEFORE_HOME",
    "HOME",
]

current_state = 0


def apply_targets(state_name):
    """Write the joint + gripper targets for this phase into data.ctrl."""
    arm = ARM_TARGETS[state_name]
    # actuator1..actuator7 drive the 7 arm joints
    for i in range(7):
        data.ctrl[i] = arm[i]
    # actuator8 drives the gripper tendon
    data.ctrl[7] = GRIPPER_TARGETS[state_name]


def next_state():
    global current_state
    if current_state < len(states) - 1:
        current_state += 1
        print(f"\n -> {states[current_state]}")
        apply_targets(states[current_state])


print(f"Starting in {states[current_state]}")
apply_targets(states[current_state])

COOLDOWN_SECS = 1.0  # wait before the first phase begins

with mujoco.viewer.launch_passive(model, data) as viewer:
    print(f"Cooldown: waiting {COOLDOWN_SECS:.0f}s before starting...")
    cooldown_start = time.time()
    while time.time() - cooldown_start < COOLDOWN_SECS:
        mujoco.mj_step(model, data)
        viewer.sync()

    last_change = time.time()
    last_render = time.time()

    while viewer.is_running():
        mujoco.mj_step(model, data)

        phase_duration = PHASE_DURATIONS[states[current_state]]
        if time.time() - last_change > phase_duration:
            next_state()
            last_change = time.time()

        viewer.sync()


        if time.time() - last_render > 0.033:
            renderer.update_scene(data, camera=WRIST_CAM_NAME)
            pixels = renderer.render()
            frame = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
            cv2.imshow(WRIST_CAM_WINDOW, frame)
            cv2.waitKey(1)
            last_render = time.time()

cv2.destroyAllWindows()