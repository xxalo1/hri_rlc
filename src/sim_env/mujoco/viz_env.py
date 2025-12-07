from __future__ import annotations

from typing import Callable, Iterable, Optional

import numpy as np
import mujoco as mj
from mujoco import viewer  # type: ignore

from sim_env.mujoco.env import Gen3Env
from common_utils import FloatArray
from common_utils import numpy_util as npu
import common_utils.transforms as trf

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
        on_pause: Optional[Callable[[bool], None]] = None,
        on_reset: Optional[Callable[[], None]] = None,
    ) -> None:
        self.env = env
        self.on_pause = on_pause
        self.on_reset = on_reset

        self.t: float = 0.0
        self.q: FloatArray = np.zeros((0,), dtype=npu.dtype)
        self.qd: FloatArray = np.zeros((0,), dtype=npu.dtype)

        # Runtime viz state
        self.paused: bool = True
        self.show_frames: bool = False
        self.show_jacobian: bool = False
        self.show_dynamics: bool = False
        self.show_inertia: bool = False
        self.show_com: bool = False

        # Frames cache: rows are [px, py, pz, qx, qy, qz, qw]
        self.frames_dirty: bool = False
        self.frames_exists: bool = False
        self.frame_poses: FloatArray = np.empty((0, 7), dtype=npu.dtype)

        # Trajectory cache
        self.traj_dirty: bool = False
        self.cartesian_poses: FloatArray = np.empty((0, 7), dtype=npu.dtype)  # shape (N, 3)


    def set_joint_states(self,
        q: FloatArray,
        qd: FloatArray,
        t: float,
    ) -> None:
        """Set the current joint states and sim time for visualization."""
        self.q = q
        self.qd = qd
        self.t = t


    def set_frame_states(self, frame_poses: FloatArray) -> None:
        """
        Accepts array of shape (N, 7): (px,py,pz,qx,qy,qz,qw) world-frame.
        """
        self.frame_poses = frame_poses
        self._invalidate_frames()


    def set_planned_traj(self, cartesian_poses: FloatArray) -> None:
        """Accepts Nx3 world-frame waypoints for planned Cartesian trajectory."""
        self.cartesian_poses = cartesian_poses
        self._invalidate_traj()


    def key_callback(self, keycode: int) -> None:
        c = chr(keycode)
        match c:
            case " ":
                # Toggle pause; notify node via injected callback
                self.paused = not self.paused
                if self.on_pause:
                    self.on_pause(self.paused)
            case "r" | "R":
                # Reset simulation and pause after reset
                if self.on_reset:
                    self.on_reset()
                self.paused = True
                if self.on_pause:
                    self.on_pause(True)
            case "1":
                self.show_frames = not self.show_frames
                self._invalidate_frames()
            case "j" | "J":
                self.show_jacobian = not self.show_jacobian
            case "d" | "D":
                self.show_dynamics = not self.show_dynamics
            case "i" | "I":
                self.show_inertia = not self.show_inertia
            case "'":
                self.show_com = not self.show_com
            case _:
                # Unhandled keys are ignored
                return


    def sync(self,
        v: viewer.MjViewer,
    ) -> None:
        """
        Apply sim state and update visualization each render tick.
        Call inside `with v.lock():`.
        """
        # Mirror sim state into env
        self.env.set_state(qpos=self.q, qvel=self.qd, t=self.t)

        # Update frames visualization on demand
        if self.show_frames:
            self._ensure_frames(v)
        else:
            self._destroy_frames(v)

        # Update trajectory visualization on demand
        self._refresh_trajectory(v)

    # ---------- Internal helpers ----------

    def _invalidate_frames(self) -> None:
        self.frames_dirty = True


    def _invalidate_traj(self) -> None:
        self.traj_dirty = True


    def _ensure_frames(self, v: viewer.MjViewer) -> None:
        if self.frames_dirty or not self.frames_exists:
            # Recreate frame axes visuals based on latest poses
            self._destroy_frames(v)
            self._create_frame_axes(v)
            self.frames_exists = True
            self.frames_dirty = False


    def _destroy_frames(self, v: viewer.MjViewer) -> None:
        if self.frames_exists:
            # Clearing user geoms/markers if previously created
            self._clear_user_markers(v)
            self.frames_exists = False


    def _create_frame_axes(self, v: viewer.MjViewer) -> None:
        # Draw axes for each pose using simple markers (line segments)
        # Axes length and thickness defaults
        axis_len = 0.08
        rgba_x = np.array([1.0, 0.2, 0.2, 1.0])
        rgba_y = np.array([0.2, 1.0, 0.2, 1.0])
        rgba_z = np.array([0.2, 0.2, 1.0, 1.0])

        for pose in self.frame_poses:
            if pose.shape != (7,):
                continue
            origin = pose[:3]
            quat = pose[3:7]
            R = trf.quat_to_rotation_matrix(quat)
            Rx = R[:, 0]
            Ry = R[:, 1]
            Rz = R[:, 2]
            self._add_line(v, origin, origin + axis_len * Rx, rgba_x)
            self._add_line(v, origin, origin + axis_len * Ry, rgba_y)
            self._add_line(v, origin, origin + axis_len * Rz, rgba_z)


    def _refresh_trajectory(self, v: viewer.MjViewer) -> None:
        if not self.traj_dirty:
            return
        # Clear previous traj markers and redraw if a traj is present
        self._clear_traj_markers(v)
        if self.cartesian_poses is not None and len(self.cartesian_poses) > 0:
            pts = self.cartesian_poses
            # Draw small spheres at waypoints and line segments between them
            rgba_pt = np.array([1.0, 0.7, 0.2, 1.0])
            rgba_ln = np.array([1.0, 0.8, 0.2, 0.8])
            for i in range(len(pts)):
                p = pts[i][:3] if pts.shape[1] >= 3 else pts[i]
                self._add_sphere(v, p, radius=0.01, rgba=rgba_pt)
                if i > 0:
                    p_prev = pts[i - 1][:3] if pts.shape[1] >= 3 else pts[i - 1]
                    self._add_line(v, p_prev, p, rgba_ln)
        self.traj_dirty = False

    # ---------- Marker primitives (viewer helper wrappers) ----------

    def _clear_user_markers(self, v: viewer.MjViewer) -> None:
        # Best-effort clear: reset the user scene if available
        try:
            scn = v.user_scn
            if hasattr(scn, "ngeom"):
                scn.ngeom = 0
        except Exception:
            pass


    def _clear_traj_markers(self, v: viewer.MjViewer) -> None:
        # For simplicity, clear all and let frames be re-added if needed
        self._clear_user_markers(v)
        # Mark frames dirty so they are re-created after clear if showing
        if self.show_frames:
            self.frames_dirty = True


    def _add_line(self,
        v: viewer.MjViewer,
        p0: np.ndarray,
        p1: np.ndarray,
        rgba: np.ndarray,
    ) -> None:
        # Use mjv function to append a thin capsule/segment into user scene if exposed
        try:
            scn = v.user_scn
            geom_id = scn.ngeom
            if geom_id < scn.maxgeom:
                mj.mjv_initGeom(
                    scn.geoms[geom_id],
                    mj.mjtGeom.mjGEOM_LINE,
                    size=np.zeros(3),
                    rgba=rgba,
                    pos=np.zeros(3),
                    mat=np.eye(3),
                )
                scn.geoms[geom_id].data = np.concatenate([p0, p1]).astype(float)
                scn.ngeom += 1
        except Exception:
            # Fallback: no-op if viewer doesn't expose user_scn
            pass

        
    def _add_sphere(self,
        v: viewer.MjViewer,
        center: np.ndarray,
        radius: float,
        rgba: np.ndarray,
    ) -> None:
        try:
            scn = v.user_scn
            geom_id = scn.ngeom
            if geom_id < scn.maxgeom:
                mj.mjv_initGeom(
                    scn.geoms[geom_id],
                    mj.mjtGeom.mjGEOM_SPHERE,
                    size=np.array([radius, 0.0, 0.0]),
                    rgba=rgba,
                    pos=center.astype(float),
                    mat=np.eye(3),
                )
                scn.ngeom += 1
        except Exception:
            pass