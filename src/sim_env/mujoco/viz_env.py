from __future__ import annotations

from typing import Callable

import numpy as np
from mujoco import viewer  # type: ignore

from sim_env.mujoco.env import Gen3Env
from common_utils import FloatArray
from common_utils import numpy_util as npu
import common_utils.transforms as tfm
from sim_backend.mujoco.scene_overlay import SceneOverlay


class VizEnv:
    """
    Encapsulates MuJoCo visualization logic for the Gen3 sim.

    Responsibilities:
    - Handle viewer key callbacks (pause/reset toggles and feature flags)
    - Maintain and render frame axes for all published frames
    - Maintain and render planned Cartesian trajectory waypoints (world frame)

    Integration:
    - Construct with a `Gen3Env` instance
    - Provide `on_pause(bool)` and `on_reset()` callbacks injected from node
    - Call `sync(viewer, qpos, qvel, t)` inside the viewer lock each frame
    """


    def __init__(self,
        env: Gen3Env,
        on_pause: Callable[[bool], None] | None = None,
        on_reset: Callable[[], None] | None = None,
    ) -> None:
        self.env = env
        self.on_pause = on_pause
        self.on_reset = on_reset

        self.t = 0.0
        self.q = np.zeros((0,), dtype=npu.dtype)
        self.qd = np.zeros((0,), dtype=npu.dtype)

        # Runtime viz state
        self.paused = True
        self.show_frames = True
        self.show_jacobian = False
        self.show_dynamics = False
        self.show_inertia = False
        self.show_com = False

        # Frames cache: rows are [px, py, pz, qx, qy, qz, qw]
        self.frames_dirty = False
        self.frames_exists = False
        self.frame_poses = np.empty((0, 7), dtype=npu.dtype)
        self._overlay: SceneOverlay | None = None

        # Trajectory cache
        self.traj_dirty = False
        self.traj_exists = False
        self.cartesian_poses = np.empty((0, 7), dtype=npu.dtype)  # shape (N, 3)
        self.has_traj = False


    def set_joint_states(self,
        q: FloatArray,
        qd: FloatArray,
        t: float,
        tau: FloatArray | None = None,
    ) -> None:
        """Set the current joint states and sim time for visualization."""
        self.q = q
        self.qd = qd
        self.t = t
        if tau is not None: 
            self.tau = tau
            self.env.d.ctrl[self.env.act_idx[:len(tau)]] = tau


    def set_frame_states(self, frame_poses: FloatArray) -> None:
        """
        Accepts array of shape (N, 7): (px,py,pz,qx,qy,qz,qw) world-frame.
        """
        self.frame_poses = frame_poses
        self._invalidate_frames()


    def set_planned_traj(self, cartesian_poses: FloatArray | None) -> None:
        """Accepts Nx3 world-frame waypoints for planned Cartesian trajectory."""
        if cartesian_poses is None:
            self.cartesian_poses = np.empty((0, 7), dtype=npu.dtype)
            self.has_traj = False
            self._invalidate_traj()
            return

        cartesian_poses = np.asarray(cartesian_poses, dtype=npu.dtype)

        # Decimate to keep overlay geom count reasonable for interactive rendering
        target_pts = 100
        n = len(cartesian_poses)
        if n > target_pts:
            idx = np.linspace(0, n - 1, target_pts, dtype=int)
            cartesian_poses = cartesian_poses[idx]

        self.cartesian_poses = cartesian_poses
        self.has_traj = True
        self._invalidate_traj()


    def key_callback(self, keycode: int) -> None:
        c = chr(keycode)
        match c:
            case " ":
                self.paused = not self.paused
                if self.on_pause: self.on_pause(self.paused)
            case "r" | "R":
                if self.on_reset: self.on_reset()
                self.paused = True
                if self.on_pause: self.on_pause(True)
            case "1":
                self.show_frames = not self.show_frames
                self._invalidate_frames()
            case "t" | "T":
                self.has_traj = not self.has_traj
                self._invalidate_traj()
            case _:
                return


    def sync(self,
        v: viewer.MjViewer,
    ) -> None:
        """
        Apply sim state and update visualization each render tick.
        Call inside `with v.lock():`.
        """
        if self._overlay is None or self._overlay.scn is not v.user_scn:
            self._overlay = SceneOverlay(v.user_scn)
            self._overlay.reserve("frames", cap=3 * 10)
            self._overlay.reserve("traj",   cap=2 * 4000)

        self.env.set_state(qpos=self.q, qvel=self.qd, t=self.t)

        if self.show_frames:
            self._ensure_frames()
        else:
            self._destroy_frames()

        if self.has_traj:
            self._ensure_traj()
        else:
            self._destroy_traj()


    def _invalidate_frames(self) -> None:
        self.frames_dirty = True


    def _invalidate_traj(self) -> None:
        self.traj_dirty = True


    def _ensure_frames(self) -> None:
        if self.frames_dirty or not self.frames_exists:
            self._destroy_frames()
            self._create_frame_axes()
            self.frames_exists = True
            self.frames_dirty = False


    def _ensure_traj(self) -> None:
        if self.traj_dirty or not self.traj_exists:
            self._destroy_traj()
            self._draw_traj(self.cartesian_poses)
            self.traj_exists = True
            self.traj_dirty = False


    def _destroy_frames(self) -> None:
        if self.frames_exists:
            if self._overlay is not None:
                self._overlay.clear("frames")
            self.frames_exists = False


    def _destroy_traj(self) -> None:
        if self.traj_exists:
            if self._overlay is not None:
                self._overlay.clear("traj")
            self.traj_exists = False


    def _create_frame_axes(self) -> None:
        if self._overlay is None:
            return

        axis_len = 0.3
        rgba_x = np.array([1.0, 0.2, 0.2, 1.0], dtype=np.float32)
        rgba_y = np.array([0.2, 1.0, 0.2, 1.0], dtype=np.float32)
        rgba_z = np.array([0.2, 0.2, 1.0, 1.0], dtype=np.float32)

        with self._overlay.layer("frames") as L:
            for pose in self.frame_poses:
                origin = pose[:3]
                quat = pose[3:7]
                R = tfm.quat_to_rotation_matrix(quat)
                L.arrow(origin, origin + axis_len * R[:, 0], rgba_x, width=0.005)
                L.arrow(origin, origin + axis_len * R[:, 1], rgba_y, width=0.005)
                L.arrow(origin, origin + axis_len * R[:, 2], rgba_z, width=0.005)


    def _draw_traj(self, pts: FloatArray) -> None:
        rgba_pt = np.array([1.0, 0.7, 0.2, 1.0])
        rgba_ln = np.array([1.0, 0.8, 0.2, 0.8])
        if self._overlay is None:
            return
        with self._overlay.layer("traj") as L:
            for i in range(len(pts)):
                p = pts[i][:3] if pts.shape[1] >= 3 else pts[i]
                L.sphere(p, radius=0.01, rgba=rgba_pt)
                if i > 0:
                    p_prev = pts[i - 1][:3] if pts.shape[1] >= 3 else pts[i - 1]
                    L.line(p_prev, p, rgba_ln, width=0.002)
