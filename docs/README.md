# Project Overview
This repository features a modular physics simulation environment designed to validate joint-space and task-space trajectory control for the 7-DOF Franka Emika Panda manipulator. The project focuses on executing precision pick-and-place manipulation pipelines, validating URDF joint mechanics, and handling rigid-body contact dynamics in simulation before hardware deployment.
# Demo
<img width="800" height="450" alt="ezgif com-speed" src="https://github.com/user-attachments/assets/646f177f-a676-4f26-a0f5-74d4e4e8129b" />


# Key Technical Features
- Kinematic & Physics Configuration: Configured the 7-DOF manipulator's URDF/SDF files, precisely defining link inertias, joint limits, and friction constraints to ensure realistic rigid-body dynamics in Gazebo/MuJoCo.
- Trajectory & Motion Planning: Implemented smooth joint-space trajectory generation and gripper control sequences to execute reliable, repeatable pick-and-place routines.
- Closed-Loop Telemetry: Integrated sensor and joint-state plugins (joint_state_broadcaster, robot_state_publisher) to stream real-time telemetry data for execution monitoring.

# Workspace Structure
<pre>
├── assets/
│   ├── hand.xml
│   └── panda.xml
├── worlds/
│   ├── scene.xml
│   └── panda_pick_place_scene.xml
├── scripts/
│   ├── pick_place.py
│   └── stack_cubes.py
├── docs/
├── test/
├── archive/
├── .gitignore
├── LICENSE
└── README.md
</pre>

## Agentic Replication & Verification Architecture
This repository is structurally optimized to support automated, agentic verification workflows. The root directory contains dedicated operational frameworks designed for autonomous software agents to interpret, parse, and execute simulation environments dynamically:

**`DRIFT.md`**: Outlines system environment directives, enabling an autonomous agent to initialize the workspace dependencies, handle file paths, and spin up the Franka Panda environment without manual configuration.
**`skill.md`**: Documents precise control action schemas (e.g., visual alignments, path segments, and gripper tracking parameters) used to guide the agent through replicating exact manipulation steps across external test workspaces.
**`timing_backup.md`**: Provides the baseline execution logs and control loop step-time thresholds required to maintain deterministic physics updates during automated test cycles.


# Franka Emika Panda Description (MJCF)

> [!IMPORTANT]
> Requires MuJoCo 2.3.3 or later.

## Changelog

See [CHANGELOG.md](./CHANGELOG.md) for a full history of changes.

## Overview

This package contains a simplified robot description (MJCF) of the [Franka Emika
Panda](https://www.franka.de/) developed by [Franka
Emika](https://www.franka.de/company). It is derived from the [publicly
available URDF
description](https://github.com/frankaemika/franka_ros/tree/develop/franka_description).

<p float="left">
  <img src="panda.png" width="400">
</p>

## URDF → MJCF derivation steps

1. Converted the DAE [mesh
   files](https://github.com/frankaemika/franka_ros/tree/develop/franka_description/meshes/visual)
   to OBJ format using [Blender](https://www.blender.org/).
2. Processed `.obj` files with [`obj2mjcf`](https://github.com/kevinzakka/obj2mjcf).
3. Eliminated the perfectly flat `link0_6` from the resulting submeshes created for `link0`.
4. Created a convex decomposition of the STL collision [mesh
   file](https://github.com/frankaemika/franka_ros/tree/develop/franka_description/meshes/collision)
   for `link5` using [V-HACD](https://github.com/kmammou/v-hacd).
5. Added `<mujoco> <compiler discardvisual="false"/> </mujoco>` to the
   [URDF](https://github.com/frankaemika/franka_ros/tree/develop/franka_description/robots)'s
   `<robot>` clause in order to preserve visual geometries.
6. Loaded the URDF into MuJoCo and saved a corresponding MJCF.
7. Matched inertial parameters with [inertial.yaml](
   https://github.com/frankaemika/franka_ros/blob/develop/franka_description/robots/common/inertial.yaml).
8. Added a tracking light to the base.
9. Manually edited the MJCF to extract common properties into the `<default>` section.
10. Added `<exclude>` clauses to prevent collisions between `link7` and `link8`.
11. Manually designed collision geoms for the fingertips.
12. Added position-controlled actuators for the arm.
13. Added an equality constraint so that the left finger mimics the position of the right finger.
14. Added a tendon to split the force equally between both fingers and a
    position actuator acting on this tendon.
15. Added `scene.xml` which includes the robot, with a textured groundplane, skybox, and haze.

### MJX

A version of the Franka Emika Panda environment was created for MJX. Steps:

1. Added `mjx_panda.xml`, forked from `panda.xml`.
2. Added `mjx_scene.xml` and `mjx_single_cube.xml`, forked from `scene.xml`.
3. Gripper collision geometries were modified to contain less geoms. A capsule collision geom was added to the hand.
4. Solver parameters were tuned for performance.
5. Actuator `kp` and `kv` were reduced for more stable simulation.
6. Added a `site` to the gripper.
7. Removed tendon and added position actuator for the gripper. Changed gripper `ctrlrange`.

## License

This model is released under an [Apache-2.0 License](LICENSE).
