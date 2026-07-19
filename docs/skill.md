# Franka Panda Pick-and-Place — Project Seed

Drop this file into an empty directory and ask Drift to reconstruct the project from it.

---

## What this project is

A MuJoCo simulation of the Franka Emika Panda 7-DOF arm performing a pick-and-place task. The arm picks a small red cube from a fixed position on a table and places it on a green target pad. The controller is a plain Python script (`pick_place.py`) that drives the arm through a fixed sequence of joint-space waypoints using MuJoCo's position actuators.

No ROS. No ROS packages. Entry point is `python3 pick_place.py`.

Requires **MuJoCo ≥ 2.3.3**.

---

## Directory layout to recreate

```
/
├── panda.xml                  # Official Franka Panda MJCF (arm + gripper)
├── hand.xml                   # Gripper sub-model included by panda.xml
├── panda_pick_place_scene.xml # Scene: table, cube, target pad
├── scene.xml                  # Alternate scene (textured ground, skybox)
├── pick_place.py              # Pick-and-place controller
├── assets/                    # Mesh files (.obj) referenced by panda.xml
├── docs/
│   ├── DRIFT.md
│   ├── skill.md               # this file
│   └── timing_backup.md
└── archive/                   # Older/experimental XML variants
```

The `assets/` directory must sit next to `panda.xml` — mesh paths inside `panda.xml` are relative to that directory.

---

## Scene file — `panda_pick_place_scene.xml`

Full content to recreate verbatim:

```xml
<mujoco model="panda_pick_place_scene">
  <!-- Place this file next to panda.xml so panda.xml's relative
       meshdir="assets" resolves correctly. Do NOT add a <compiler>
       tag here that overrides meshdir. -->
  <include file="panda.xml"/>

  <option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.81"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="120" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
             rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8"
             width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true"
              texrepeat="5 5" reflectance="0.2"/>
    <material name="table_mat" rgba="0.55 0.4 0.3 1"/>
  </asset>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <light pos="1 -1 2.5" dir="-0.4 0.4 -1" diffuse="0.4 0.4 0.4"/>

    <!-- Floor 0.4m below the table top so the table legs are visible. -->
    <geom name="floor" type="plane" size="0 0 0.05" pos="0 0 -0.4" material="groundplane"/>

    <!-- Table: top surface at z=0 (panda base sits on it). Table centered
         slightly forward so the panda sits near the back edge. -->
    <body name="table" pos="0.25 0 -0.4">
      <geom name="table_top" type="box" size="0.6 0.4 0.02" pos="0 0 0.38" material="table_mat"/>
      <geom name="table_leg_1" type="box" size="0.03 0.03 0.19" pos=" 0.55  0.35 0.19" material="table_mat"/>
      <geom name="table_leg_2" type="box" size="0.03 0.03 0.19" pos=" 0.55 -0.35 0.19" material="table_mat"/>
      <geom name="table_leg_3" type="box" size="0.03 0.03 0.19" pos="-0.55  0.35 0.19" material="table_mat"/>
      <geom name="table_leg_4" type="box" size="0.03 0.03 0.19" pos="-0.55 -0.35 0.19" material="table_mat"/>
    </body>

    <!-- Small red cube in front of the gripper, resting on the table top (z=0). -->
    <body name="cube" pos="0.5 0 0.03">
      <freejoint/>
      <geom name="cube_geom" type="box" size="0.02 0.02 0.02" rgba="1 0 0 1"
            mass="0.05" friction="1.0 0.05 0.001" condim="4"/>
    </body>

    <!-- Green target pad: thin disc on the table, off to the side. -->
    <body name="target_pad" pos="0.5 0.25 0.001">
      <geom name="target_pad_geom" type="cylinder" size="0.05 0.001"
            rgba="0 1 0 0.5" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
```

---

## Controller — `pick_place.py`

Full content to recreate verbatim:

```python
import mujoco
import mujoco.viewer
import time
import numpy as np

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

    while viewer.is_running():
        mujoco.mj_step(model, data)

        phase_duration = PHASE_DURATIONS[states[current_state]]
        if time.time() - last_change > phase_duration:
            next_state()
            last_change = time.time()

        viewer.sync()
```

> **Note:** Update `model_path` to the absolute path on the new machine before running.

---

## How to launch

```bash
python3 pick_place.py
```

The MuJoCo passive viewer opens. The arm starts in NEUTRAL, waits for the cooldown, then steps through the sequence automatically.

---

## Phase sequence and timing

| Phase | Duration (s) | Gripper | Notes |
|---|---|---|---|
| NEUTRAL | 5.0 | open | Initial safe pose |
| MOVE_ABOVE_CUBE | 5.0 | open | Positions end-effector above cube |
| LOWER | 4.0 | open | Descends to cube height |
| GRASP | 3.0 | closed | Closes fingers around cube |
| LIFT | 4.0 | closed | Raises cube off table |
| MOVE_TO_TARGET | 5.0 | closed | Translates laterally to target pad |
| LOWER_TO_TARGET | 4.0 | closed | Descends to place height |
| RELEASE | 2.0 | open | Releases cube onto pad |
| LIFT_GRIPPER | 3.0 | open | Raises gripper clear of cube |
| LIFT_BEFORE_HOME | 4.0 | open | Intermediate lift before homing |
| HOME | 6.0 | open | Returns to home configuration |

Total sequence time (excluding cooldown): **45.0 seconds**

---

## Key values to preserve

**Cube spawn position** (must match between scene XML and `CUBE_POSITION` in the controller):
```
x=0.5, y=0.0, z=0.03
```

**Gripper actuator (`actuator8`) control range:** `0` (fully closed) to `255` (fully open).

**Joint control range constraints:**
- `joint4`: must stay in `(-3.0718, -0.0698)` — always negative
- `joint6`: must stay in `(-0.0175, 3.7525)` — always positive

**Physics settings:**
- `timestep="0.002"`, `integrator="implicitfast"`, `gravity="0 0 -9.81"`

---

## Known issues and fixes

- **Arm shakes on startup** — The NEUTRAL phase runs first to settle the arm into a safe pose before any motion begins. Do not remove it or shorten it below ~3 s.
- **Gripper drops cube during LIFT** — Check that `data.ctrl[7]` is `0` (closed) throughout GRASP, LIFT, MOVE_TO_TARGET, and LOWER_TO_TARGET. Any non-zero value opens the fingers.
- **Cube not at expected position** — `CUBE_POSITION` in `pick_place.py` and the `pos` attribute of the `cube` body in `panda_pick_place_scene.xml` must match exactly.
- **Mesh not found on load** — Keep `assets/` in the same directory as `panda.xml`. Do not add a `<compiler meshdir=...>` override in the scene file.
- **MuJoCo version error** — Requires MuJoCo ≥ 2.3.3.

---

## Wrist Camera — gripper-mounted camera feed

### What was added

A `<camera name="wrist_cam">` element was placed inside the `<body name="hand">` block in `panda.xml`, with `pos` and `euler` tuned to point the camera downward toward the fingers. This gives a first-person view from the gripper during the pick-and-place sequence.

---

### XML snippet — add inside the `hand` body in `panda.xml`

```xml
<camera name="wrist_cam" pos="0 0.04 0.06" euler="180 0 0" fovy="60"/>
```

Place this line anywhere inside the `<body name="hand">` block (before its closing `</body>` tag).

**Exact values used in this project:**
- `pos="0 0.04 0.06"` — offset 4 cm along the hand's local Y axis and 6 cm along local Z from the hand-body origin
- `euler="180 0 0"` — rotates 180° around X so the camera faces downward toward the fingertips
- `fovy="60"` — 60° vertical field of view

---

### Python additions — `pick_place.py`

**New import** (add near the top with the other imports):

python
import cv2
+
**Renderer setup** (add before the `with mujoco.viewer.launch_passive(...)` line):

python
CAM_H, CAM_W = 480, 640
renderer = mujoco.Renderer(model, height=CAM_H, width=CAM_W)
+
**Inside the `while viewer.is_running():` loop**, after `mujoco.mj_step(model, data)`:

python
renderer.update_scene(data, camera="wrist_cam")
pixels = renderer.render()
cv2.imshow("Wrist Camera", cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR))
cv2.waitKey(1)
+
**After the loop ends** (after the `with` block closes):

python
cv2.destroyAllWindows()
+
---

### Dependency

`opencv-python` must be installed if not already present:

bash
pip install opencv-python
+
---

### Tuning the camera pose

Adjust `pos` (offset from the hand-body origin in the hand-local frame) and `euler` (rotation in degrees, applied as extrinsic XYZ) in `panda.xml` to change the viewing angle. `euler="180 0 0"` flips the camera so it faces downward along the hand's −Z axis, looking toward the fingertips. `fovy` controls the vertical field of view in degrees; smaller values zoom in, larger values widen the view.

---

### Known issues

- **OpenCV window does not appear** — Confirm that a display is available (`echo $DISPLAY` should return a non-empty value, e.g. `:0`) and that `opencv-python` is installed (`pip show opencv-python`). On headless servers, use a virtual framebuffer (`Xvfb`) or redirect the feed over SSH with X forwarding.

---

## Prompt-driven modifications

When the user gives a natural-language prompt such as *"add a camera to the hand gripper"* or *"enable the wrist camera feed"*, Drift must **not** invent new values. Instead, Drift should:

1. **Locate the relevant section of `skill.md`** — for example, the *Wrist Camera — gripper-mounted camera feed* section above — and read it in full before touching any project file.
2. **Extract the exact values documented there** — the XML snippet, every Python addition (imports, renderer setup, in-loop code, post-loop teardown), and any listed dependencies — and apply them **verbatim** to the actual project files (`panda.xml`, `pick_place.py`, etc.).
3. **Do not substitute, paraphrase, or improve** the documented values. If `skill.md` says `pos="0 0.04 0.06"` and `euler="180 0 0"`, those exact strings go into `panda.xml`. If `skill.md` says `CAM_H, CAM_W = 480, 640`, that exact line goes into `pick_place.py`.
4. **Apply all sub-parts together.** Each feature section in `skill.md` may document changes across multiple files (XML, Python, shell commands). All of them must be applied as a single coherent change — do not apply only the XML part and skip the Python part, or vice versa.
5. **Install documented dependencies.** If the section lists a `pip install` command, include that step in the instructions returned to the user.

### Why skill.md is the authoritative source of record

`skill.md` was written after the feature was tested and verified to work in this specific project. The values it contains — joint targets, camera offsets, control ranges, physics settings, file paths — were tuned against the actual MuJoCo model and scene. Inventing new values risks breaking constraints (e.g. joint limits, mesh paths, actuator ranges) that are not obvious from the prompt alone.

Any prompt asking to **add**, **modify**, or **re-enable** a feature that already has a section in `skill.md` must be fulfilled by consulting and applying the values already documented here, not by generating new ones from scratch.

### What to do when a feature is NOT documented in skill.md

If the user requests a feature that has no corresponding section in `skill.md`, Drift may generate new values — but should then **append a new section to `skill.md`** documenting the exact values used, following the same format as the existing sections (XML snippet, Python additions, dependency, known issues). This keeps `skill.md` up to date as the project evolves and ensures future prompts for the same feature can be fulfilled from the authoritative record.

---

## How Drift should apply prompts like "add a camera to hand gripper"

When a user gives a prompt that matches something already documented in this file — for example, adding a wrist camera, changing the gripper pose, tuning actuator gains, adjusting phase timings, or modifying control ranges — Drift must **look up the exact values recorded in `skill.md`** and apply those verbatim. Drift must not invent new values, paraphrase the documented ones, or generate a fresh snippet from scratch.

The values stored in this file — `pos`, `euler`, `fovy`, actuator control ranges, joint targets, phase durations, physics settings, file paths, and all associated Python code — are the **ground-truth for this project**. They were verified against the actual MuJoCo model and scene. Substituting different values, even plausible-looking ones, risks violating joint limits, breaking mesh resolution, misaligning the camera, or destabilizing the simulation in ways that are not immediately obvious from the prompt alone.

### Decision rule

1. **Read the prompt carefully** and identify which feature or parameter it refers to.
2. **Search `skill.md` for a matching section.** If a section exists (e.g. *Wrist Camera — gripper-mounted camera feed*), that section is the authoritative source. Read it in full.
3. **Extract every sub-part documented in that section** — XML snippets, Python imports, in-loop code, post-loop teardown, dependency install commands — and apply all of them together as a single coherent change. Do not apply only part of a documented feature.
4. **Copy the recorded values verbatim.** If `skill.md` says `pos="0 0.04 0.06"` and `euler="180 0 0"`, those exact strings go into `panda.xml`. If it says `CAM_H, CAM_W = 480, 640`, that exact line goes into `pick_place.py`. No substitutions, no improvements.
5. **If no matching section exists**, Drift may generate new values — but must then append a new section to `skill.md` in the same format (XML snippet, Python additions, dependency, known issues) so the feature is recorded for future prompts.

### Why this matters

`skill.md` is not a general reference document — it is a project-specific record of what was actually tested and confirmed to work in *this* simulation, with *this* robot model, on *this* scene. A prompt like *"add a camera to the hand gripper"* or *"re-enable the wrist camera feed"* is a request to reproduce a known-good configuration, not an invitation to design a new one. Treating `skill.md` as the authoritative source ensures that repeated or re-applied prompts always produce the same correct result, and that the project remains in a consistent, reproducible state.

### After applying the camera prompt

After all camera-related changes have been applied to `panda.xml` and `pick_place.py`, Drift must also:

1. **Run the simulation** — launch `pick_place.py` so the MuJoCo passive viewer opens and the pick-and-place sequence begins.
2. **Open the camera viewer** — the `cv2.imshow("Wrist Camera", ...)` window should appear automatically once `pick_place.py` is running, because the OpenCV rendering loop is already wired into the main simulation loop. No separate launch step is needed; both windows come up together when `python3 pick_place.py` is executed.

If the OpenCV window does not appear, refer to the *Known issues* entry under the *Wrist Camera* section above.
