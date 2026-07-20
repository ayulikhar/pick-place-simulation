import os
import time

import numpy as np
import mujoco
import mujoco.viewer

try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False

# config
_here = os.path.dirname(os.path.abspath(__file__))
_local_candidate = os.path.join(_here, "panda_pick_place_scene.xml")
MODEL_PATH = _local_candidate if os.path.exists(_local_candidate) \
    else "/home/ayu/Franka_panda/panda_pick_place_scene.xml"

ARM_JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]
ARM_ACTUATOR_NAMES = [f"actuator{i}" for i in range(1, 8)]
GRIPPER_ACTUATOR_NAME = "actuator8"
GRIPPER_OPEN = 255.0
GRIPPER_CLOSED = 0.0

EE_SITE_CANDIDATES = ["attachment_site", "tcp", "ee_site", "gripper_site", "tool_site"]
EE_BODY_FALLBACK = "hand"

# Joint configuration to grasp the cube at [0.5, 0, 0.03].
CALIBRATION_JOINTS = [0.0, 0.5134, 0.0, -2.1911, 0.0, 2.4648, 0.785]
CALIBRATION_FINGERTIP_TARGET = np.array([0.50, 0.00, 0.03])

CUBE_BODY_NAMES = ["cube", "cube2", "cube3"]
TARGET_PAD_BODY = "target_pad"

CUBE_HALF_SIZE = 0.02          # cube geom size
TRANSIT_HEIGHT = 0.25           
SETTLE_TIME = 1.0              # ensure no overshoot


def smoothstep(x):
    """Smooth ease-in/ease-out interpolation factor, x in [0, 1]."""
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


class PandaIKController:
    def __init__(self, model, data):
        self.model = model
        self.data = data

        # resolve arm joints and actuators 
        try:
            self.arm_qposadr = [model.joint(n).qposadr[0] for n in ARM_JOINT_NAMES]
            self.arm_dofadr = [model.joint(n).dofadr[0] for n in ARM_JOINT_NAMES]
            self.arm_jntid = [model.joint(n).id for n in ARM_JOINT_NAMES]
        except KeyError as e:
            raise RuntimeError(
                f"Could not find arm joint {e}. Edit ARM_JOINT_NAMES in the "
                f"CONFIG block at the top of this file to match panda.xml."
            )

        try:
            self.arm_actid = [model.actuator(n).id for n in ARM_ACTUATOR_NAMES]
            self.gripper_actid = model.actuator(GRIPPER_ACTUATOR_NAME).id
        except KeyError as e:
            raise RuntimeError(
                f"Could not find actuator {e}. Edit ARM_ACTUATOR_NAMES / "
                f"GRIPPER_ACTUATOR_NAME in the CONFIG block."
            )

        self.arm_ctrlrange = np.array(
            [model.actuator_ctrlrange[a] for a in self.arm_actid]
        )
        self.arm_jntrange = np.array(
            [model.jnt_range[j] for j in self.arm_jntid]
        )

        # resolve end-effector reference frame 
        self.ee_mode = None
        self.ee_id = None
        for name in EE_SITE_CANDIDATES:
            try:
                self.ee_id = model.site(name).id
                self.ee_mode = "site"
                print(f"[IK] Using site '{name}' as end-effector frame.")
                break
            except KeyError:
                continue
        if self.ee_mode is None:
            try:
                self.ee_id = model.body(EE_BODY_FALLBACK).id
                self.ee_mode = "body"
                print(f"[IK] No known EE site found; using body "
                      f"'{EE_BODY_FALLBACK}' as end-effector frame.")
            except KeyError:
                raise RuntimeError(
                    "Could not find an end-effector site or a body named "
                    f"'{EE_BODY_FALLBACK}'. Edit EE_SITE_CANDIDATES / "
                    "EE_BODY_FALLBACK in the CONFIG block to match panda.xml."
                )

        # scratch MjData used for IK iterations so we never disturb the live simulation state while solving.
        self.data_ik = mujoco.MjData(model)
        self.data_ik.qpos[:] = data.qpos[:]

        self.fingertip_offset_local = np.zeros(3)
        self.target_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self._calibrate()

    def _set_arm_qpos(self, data, q):
        for adr, val in zip(self.arm_qposadr, q):
            data.qpos[adr] = val

    def _get_arm_qpos(self, data):
        return np.array([data.qpos[adr] for adr in self.arm_qposadr])

    def _get_ee_pose(self, data):
        if self.ee_mode == "site":
            pos = data.site_xpos[self.ee_id].copy()
            mat = data.site_xmat[self.ee_id].reshape(3, 3).copy()
        else:
            pos = data.xpos[self.ee_id].copy()
            mat = data.xmat[self.ee_id].reshape(3, 3).copy()
        return pos, mat

    def _jac_ee(self, data):
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        if self.ee_mode == "site":
            mujoco.mj_jacSite(self.model, data, jacp, jacr, self.ee_id)
        else:
            mujoco.mj_jacBody(self.model, data, jacp, jacr, self.ee_id)
        return jacp[:, self.arm_dofadr], jacr[:, self.arm_dofadr]

    def _clip_arm(self, q):
        q = np.array(q, dtype=float)
        for i in range(len(q)):
            lo, hi = self.arm_ctrlrange[i]
            if hi > lo:  # only clip if a real range is defined
                q[i] = np.clip(q[i], lo, hi)
        return q

    # ------------------------- #
    def _calibrate(self):
        """Self-calibrate the world-frame gap between the EE reference
        frame and the point that actually contacts the cube, using the
        known-good GRASP joint configuration from the original script."""
        self._set_arm_qpos(self.data_ik, CALIBRATION_JOINTS)
        # mj_forward populates xpos/xmat/site_xpos so _get_ee_pose reads correct values.
        mujoco.mj_forward(self.model, self.data_ik)
        pos, mat = self._get_ee_pose(self.data_ik)
        self.target_quat = np.zeros(4)
        mujoco.mju_mat2Quat(self.target_quat, mat.flatten())
        # offset expressed in the EE's local frame so it stays valid even
        # if later orientations drift slightly from this calibration pose
        self.fingertip_offset_local = mat.T @ (CALIBRATION_FINGERTIP_TARGET - pos)
        print(f"[IK] Calibrated fingertip offset (local frame): "
              f"{self.fingertip_offset_local}")

    # ------------------------ #
    def solve_ik(self, fingertip_target, q_guess, max_iters=200,
                 tol=1e-4, damping=0.05):
        """Damped least-squares IK. Solves for the 7 arm joint angles that
        place the calibrated fingertip point at `fingertip_target`, while
        holding the end-effector orientation at self.target_quat."""
        q = np.array(q_guess, dtype=float)
        target_mat = np.zeros(9)
        mujoco.mju_quat2Mat(target_mat, self.target_quat)
        target_mat = target_mat.reshape(3, 3)
        # desired EE-frame position such that EE_pos + R @ offset == target
        site_target = fingertip_target - target_mat @ self.fingertip_offset_local

        for _ in range(max_iters):
            self._set_arm_qpos(self.data_ik, q)
            # NOTE: mj_forward is required because mj_jacBody and mj_jacSite read from data structures that only mj_forward populates. Using mj_kinematics alone leaves the Jacobian at zero, so dq collapses to zero and the arm never moves off the neutral pose.
            mujoco.mj_forward(self.model, self.data_ik)
            cur_pos, cur_mat = self._get_ee_pose(self.data_ik)

            pos_err = site_target - cur_pos

            cur_quat = np.zeros(4)
            mujoco.mju_mat2Quat(cur_quat, cur_mat.flatten())
            neg_quat = np.zeros(4)
            mujoco.mju_negQuat(neg_quat, cur_quat)
            err_quat = np.zeros(4)
            mujoco.mju_mulQuat(err_quat, self.target_quat, neg_quat)
            rot_err = np.zeros(3)
            mujoco.mju_quat2Vel(rot_err, err_quat, 1.0)

            if np.linalg.norm(pos_err) < tol and np.linalg.norm(rot_err) < tol * 5:
                break

            jacp, jacr = self._jac_ee(self.data_ik)
            J = np.vstack([jacp, jacr])
            err = np.concatenate([pos_err, rot_err])

            JJt = J @ J.T + damping * np.eye(6)
            dq = J.T @ np.linalg.solve(JJt, err)
            q = self._clip_arm(q + dq)

        return q

    # ----------------------- #
    def current_arm_qpos(self):
        return self._get_arm_qpos(self.data)

    def set_ctrl(self, q_arm, gripper_val):
        for actid, val in zip(self.arm_actid, q_arm):
            self.data.ctrl[actid] = val
        self.data.ctrl[self.gripper_actid] = gripper_val


class Executor:
    """Steps the simulation while smoothly interpolating ctrl targets."""

    def __init__(self, model, data, ik: PandaIKController, viewer,
                 wrist_cam=None, renderer=None, realtime=True):
        self.model = model
        self.data = data
        self.ik = ik
        self.viewer = viewer
        self.wrist_cam = wrist_cam
        self.renderer = renderer
        self.realtime = realtime
        self._last_render = time.time()

    def _maybe_render_wrist_cam(self):
        if not (HAVE_CV2 and self.renderer is not None and self.wrist_cam):
            return
        now = time.time()
        if now - self._last_render < 0.033:
            return
        self._last_render = now
        self.renderer.update_scene(self.data, camera=self.wrist_cam)
        pixels = self.renderer.render()
        frame = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        cv2.imshow("Wrist Camera", frame)
        cv2.waitKey(1)

    def hold(self, duration):
        n = max(1, int(duration / self.model.opt.timestep))
        for _ in range(n):
            t0 = time.time()
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
            self._maybe_render_wrist_cam()
            if self.realtime:
                dt = self.model.opt.timestep - (time.time() - t0)
                if dt > 0:
                    time.sleep(dt)

    def move_to(self, fingertip_target, gripper_val, duration=2.0):
        """Solve IK for fingertip_target and smoothly interpolate the arm
        (and gripper) from the current command to the solution."""
        q_start = self.ik.current_arm_qpos()
        q_target = self.ik.solve_ik(fingertip_target, q_start)

        gripper_start = self.data.ctrl[self.ik.gripper_actid]

        n_steps = max(1, int(duration / self.model.opt.timestep))
        for s in range(n_steps):
            t0 = time.time()
            alpha = smoothstep((s + 1) / n_steps)
            q_cmd = q_start + alpha * (q_target - q_start)
            grip_cmd = gripper_start + alpha * (gripper_val - gripper_start)
            self.ik.set_ctrl(q_cmd, grip_cmd)

            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
            self._maybe_render_wrist_cam()

            if self.realtime:
                dt = self.model.opt.timestep - (time.time() - t0)
                if dt > 0:
                    time.sleep(dt)


def build_stack_sequence(model, data, ik: PandaIKController, ex: Executor):
    # locate cube / pad bodies
    try:
        cube_ids = [model.body(n).id for n in CUBE_BODY_NAMES]
        pad_id = model.body(TARGET_PAD_BODY).id
    except KeyError as e:
        raise RuntimeError(f"Could not find body {e} in the scene.")

    pad_xy = data.xpos[pad_id][:2].copy()
    pad_top_z = data.xpos[pad_id][2] + 0.001  # target_pad half-thickness

    print(f"[SEQ] Target pad at xy={pad_xy}, top z={pad_top_z:.4f}")

    for k, body_id in enumerate(cube_ids):
        cube_pos = data.xpos[body_id].copy()
        place_z = pad_top_z + CUBE_HALF_SIZE + k * (2 * CUBE_HALF_SIZE)
        place_xyz = np.array([pad_xy[0], pad_xy[1], place_z])

        above_cube = np.array([cube_pos[0], cube_pos[1], TRANSIT_HEIGHT])
        above_place = np.array([place_xyz[0], place_xyz[1], TRANSIT_HEIGHT])

        print(f"\n[SEQ] --- Cube {k + 1}/{len(cube_ids)} "
              f"({CUBE_BODY_NAMES[k]}) at {cube_pos} -> stack z={place_z:.4f} ---")

        # move above the cube, gripper open
        ex.move_to(above_cube, GRIPPER_OPEN, duration=2.5)
        # move down onto the cube, still open
        ex.move_to(cube_pos, GRIPPER_OPEN, duration=1.5)
        # close gripper to grasp
        ex.move_to(cube_pos, GRIPPER_CLOSED, duration=1.0)
        ex.hold(0.5)
        # lift up
        ex.move_to(above_cube, GRIPPER_CLOSED, duration=1.5)
        # move above the stack location
        ex.move_to(above_place, GRIPPER_CLOSED, duration=2.5)
        # descend to the stacking height
        ex.move_to(place_xyz, GRIPPER_CLOSED, duration=1.5)
        # release
        ex.move_to(place_xyz, GRIPPER_OPEN, duration=1.0)
        ex.hold(0.5)
        # rise upward
        ex.move_to(above_place, GRIPPER_OPEN, duration=1.5)


def main():
    print(f"[MAIN] Loading model from {MODEL_PATH}")
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    ik = PandaIKController(model, data)

    # Start from a safe, known joint configuration instead of the zero pose.
    neutral = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    ik._set_arm_qpos(data, neutral)
    ik.set_ctrl(neutral, GRIPPER_OPEN)
    mujoco.mj_forward(model, data)

    wrist_cam_name = None
    renderer = None
    # Camera preview disabled since we dont need it,
    print("[MAIN] Camera preview disabled (offscreen renderer skipped).")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        ex = Executor(model, data, ik, viewer,
                      wrist_cam=wrist_cam_name, renderer=renderer)

        print(f"[MAIN] Letting cubes settle for {SETTLE_TIME:.1f}s...")
        ex.hold(SETTLE_TIME)

        try:
            build_stack_sequence(model, data, ik, ex)
        except RuntimeError as e:
            print(f"[ERROR] {e}")
            raise

        print("\n[SEQ] --- Stack complete, returning home ---")
        home = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.785]
        q_start = ik.current_arm_qpos()
        n_steps = int(2.0 / model.opt.timestep)
        for s in range(n_steps):
            alpha = smoothstep((s + 1) / n_steps)
            q_cmd = q_start + alpha * (np.array(home) - q_start)
            ik.set_ctrl(q_cmd, GRIPPER_OPEN)
            mujoco.mj_step(model, data)
            viewer.sync()

        print("[MAIN] Done. Close the viewer window to exit.")
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

    if HAVE_CV2:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
