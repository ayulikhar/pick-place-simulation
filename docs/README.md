# 🤖 Franka Panda Pick-and-Place Simulation

> **Physics-grounded robotic manipulation using a 7-DOF Franka Panda in MuJoCo.**

<p align="center">
  <b>MuJoCo</b> · <b>Franka Panda</b> · <b>Robotic Manipulation</b> · <b>Trajectory Planning</b> · <b>Motion Control</b> · <b>Agentic Evaluation</b>
</p>

## Overview

A simulation framework for studying **robotic pick-and-place and object manipulation** using the Franka Emika Panda.

The project combines a detailed MuJoCo robot model with joint-space and task-space trajectory execution, gripper control, and physics-based contact dynamics.

```text id="rj2q3m"
Approach → Align → Grasp → Lift → Transport → Place → Release
```

The environment provides a controlled testbed for experimenting with **motion planning, manipulation, control, and reproducibility of agent-generated implementations**.

# Demo
<img width="800" height="450" alt="ezgif com-speed" src="https://github.com/user-attachments/assets/646f177f-a676-4f26-a0f5-74d4e4e8129b" />

## 🔬 Key Components

* **Franka Panda** — 7-DOF manipulator with simulated gripper
* **MuJoCo / MJCF** — rigid-body dynamics and contact simulation
* **Trajectory Control** — joint-space and task-space motion execution
* **Gripper Control** — simulated grasp and release
* **Pick-and-Place** — complete manipulation sequence
* **Cube Stacking** — sequential object placement experiment
* **Modular Scenes** — configurable robot and object environments

## 🧠 Control Architecture

```text id="n7vuj7"
Task Specification
       ↓
Trajectory Generation
       ↓
Panda Controller
       ↓
MuJoCo Physics
       ↓
State / Contact Feedback
       ↓
Manipulation Outcome
```

The separation between task-space objectives, robot control, and physics simulation provides a foundation for extending the system toward learning-based and adaptive manipulation.

## 🤖 Agentic Replication & Evaluation

This repository also serves as a **reproducibility testbed for agent-generated robotics implementations**.

The workflow is:

```text id="t9p8r4"
Reference Implementation
        ↓
Debug & Evaluate
        ↓
Establish Expected Behavior
        ↓
Fresh Repository
        ↓
Agentic Replication
        ↓
Compare Against Reference
```

The original implementation is first debugged and evaluated to establish a working reference. An agent is then asked to **independently reproduce the project in a fresh repository**, allowing the resulting implementation and behavior to be compared against the reworked reference.

This creates a practical benchmark for evaluating whether an agent can reproduce not only the source structure, but also the **functional and behavioral properties of a robotics system**.

## 🚀 Run

```bash id="v0s3ps"
git clone https://github.com/ayulikhar/pick-place-simulation.git
cd pick-place-simulation

pip install mujoco numpy
```

Run pick-and-place:

```bash id="c9z3kq"
python pick_place.py
```

Run cube stacking:

```bash id="n3w4gs"
python stack_cubes.py
```

## 🔭 Research Directions

* Agentic code replication and evaluation
* Collision-aware motion planning
* Trajectory optimization
* Impedance / force control
* Randomized manipulation
* Reinforcement learning
* Vision-based manipulation
* Domain randomization and sim-to-real

## 🛠️ Stack

**Python · MuJoCo · MJCF · NumPy · Franka Panda**

---

### 👤 Author

**Ayush Likhar**
Robotics · Robot Learning · Autonomous Systems · Motion & Control

<p align="center">
  <i>Building the simulation layer before the real robot.</i> 🤖
</p>

