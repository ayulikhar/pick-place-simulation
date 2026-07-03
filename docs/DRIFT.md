# Franka Panda (MJCF)

## Identity

A MuJoCo MJCF description of the Franka Emika Panda 7-DOF arm with parallel-jaw gripper. Derived from the official `franka_ros` URDF. Intended for simulation and learning workflows, including MJX (GPU-accelerated MuJoCo). Not a ROS package — no `package.xml`, no ROS nodes.

Requires **MuJoCo 2.3.3 or later**.

## File Layout

```
panda.xml            # Main robot model (arm + gripper)
scene.xml            # Robot + textured groundplane, skybox, haze
mjx_panda.xml        # MJX-tuned fork of panda.xml
mjx_scene.xml        # MJX scene wrapper
mjx_single_cube.xml  # MJX scene with a single cube object
panda.png            # Preview image
CHANGELOG.md         # Version history
LICENSE              # Apache-2.0
```

Mesh assets live alongside the XML files, converted from the upstream DAE/STL sources.

## Robot Description

The MJCF was derived from the upstream URDF through a documented pipeline:

- DAE visuals → OBJ via Blender → processed with `obj2mjcf`
- `link0_6` submesh eliminated (was perfectly flat, an artifact)
- `link5` collision uses a convex decomposition produced by V-HACD
- Inertial parameters matched to `inertial.yaml` from `franka_ros`
- `<exclude>` clauses prevent self-collision between `link7` and `link8`
- Fingertip collision geoms are manually designed (not from upstream STL)

Key model features:
- Position-controlled actuators on all arm joints
- Left finger mimics right finger via an equality constraint
- Tendon splits force equally between both fingers; a single position actuator drives the tendon
- Tracking light attached to the base

## MJX Variant

`mjx_panda.xml` is a performance-tuned fork for MJX (batched GPU simulation):

- Gripper collision geometry simplified (fewer geoms); a capsule geom added to the hand
- Solver parameters tuned for throughput
- Actuator `kp` and `kv` reduced for simulation stability
- Tendon removed; gripper replaced with a direct position actuator with updated `ctrlrange`
- A `site` added to the gripper (useful for attaching sensors or tracking end-effector pose)

Use `mjx_scene.xml` or `mjx_single_cube.xml` as the entry point for MJX workflows; use `scene.xml` for standard MuJoCo.

## Conventions

- Common joint/actuator/geom properties are factored into the `<default>` section of the MJCF — edit defaults there rather than on individual elements.
- Visual geometries are preserved (`discardvisual="false"` was set before the URDF→MJCF conversion).
- The canonical entry points for loading are `scene.xml` (standard) and `mjx_scene.xml` / `mjx_single_cube.xml` (MJX).

## Debugging

- **MuJoCo version errors at load time** — confirm version ≥ 2.3.3; the MJCF uses features not present in earlier releases.
- **Self-collision artifacts between upper links** — the `<exclude>` between `link7` and `link8` handles the known case; if new artifacts appear, add further `<exclude>` pairs rather than disabling contact globally.
- **Gripper instability in MJX** — the standard `panda.xml` actuator gains are higher than MJX can handle stably; always use `mjx_panda.xml` for batched rollouts.
- **Mesh not found errors** — mesh paths are relative to the XML; keep the directory structure intact when copying files.