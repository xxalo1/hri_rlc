from __future__ import annotations
from pathlib import Path
import numpy as np
from dataclasses import dataclass
from enum import Enum, auto

from common_utils import numpy_util as npu
from common_utils import FloatArray
from common_utils import transforms as tfm

from .kin import PinocchioDynamics
from .controller import Controller
from .planning import TrajPlanner, JointTraj, CartesianTraj

class CtrlMode(Enum):
    CT = auto()
    PID = auto()
    IM = auto()
    GC = auto()

class TraceMode(Enum):
    HOLD = auto()
    TRAJ = auto()


@dataclass
class RobotSpec:
    name: str
    urdf: Path
    tcp_frame: str


@dataclass
class Robot:
    spec: RobotSpec
    dyn: PinocchioDynamics
    ctrl: Controller
    planner: TrajPlanner


    def __post_init__(self) -> None:

        n = self.n
        self.g_w = np.array([0.0, 0.0, -9.807], dtype=npu.dtype)
        self._pose_wb = np.eye(4, dtype=npu.dtype)

        # Current state
        self._q = np.zeros(n, dtype=npu.dtype)
        self._qd = np.zeros(n, dtype=npu.dtype)
        self._qdd = np.zeros(n, dtype=npu.dtype)
        self.t = 0.0
        self.t_prev = 0.0

        # Desired state
        self._q_des = np.zeros(n, dtype=npu.dtype)
        self._qd_des = np.zeros(n, dtype=npu.dtype)
        self._qdd_des = np.zeros(n, dtype=npu.dtype)

        # Trajectory storage
        self.ti = 0.0
        self.tf = 0.0
        self.joint_traj = JointTraj()
        self.cart_traj = CartesianTraj()

        # Control modes
        self.controller_mode = CtrlMode.CT
        self.TraceMode = TraceMode.HOLD


    @classmethod
    def from_spec(cls, 
        spec: RobotSpec
    ) -> Robot:
        dyn = PinocchioDynamics.from_urdf(spec.urdf, tcp_frame=spec.tcp_frame)
        ctrl = Controller(dyn)
        planner = TrajPlanner()
        return cls(spec, dyn, ctrl, planner)


    @property
    def n(self) -> int: return self.dyn.n

    @property
    def pose_wb(self) -> FloatArray: return self._pose_wb

    @property
    def q(self) -> FloatArray: return self._q
    @q.setter
    def q(self, v: FloatArray) -> None:
        self._q[:] = v
        self.dyn.step(q=v)

    @property
    def qd(self) -> FloatArray: return self._qd
    @qd.setter
    def qd(self, v: FloatArray) -> None:
        self._qd[:] = v
        self.dyn.step(qd=v)

    @property
    def qdd(self) -> FloatArray: return self._qdd
    @qdd.setter
    def qdd(self, v: FloatArray) -> None:
        self._qdd[:] = v
        self.dyn.step(qdd=v)

    @property
    def q_des(self) -> FloatArray: return self._q_des
    @q_des.setter
    def q_des(self, v: FloatArray) -> None:
        self._q_des[:] = v

    @property
    def qd_des(self) -> FloatArray: return self._qd_des
    @qd_des.setter
    def qd_des(self, v: FloatArray) -> None:
        self._qd_des[:] = v

    @property
    def qdd_des(self) -> FloatArray: return self._qdd_des
    @qdd_des.setter
    def qdd_des(self, v: FloatArray) -> None:
        self._qdd_des[:] = v

    @property
    def joint_names(self) -> list[str]:
        return self.dyn.joint_names


    def set_base_pose(self, pose_wb: FloatArray) -> None:
        """
        Set the base pose of the robot expressed in the world frame (world_from_base),
        and update the gravity vector for the dynamics model (expressed in base/model frame).

        Parameters
        -----------
        pose_wb : ndarray, shape (7,)
            The base pose as [x, y, z, qw, qx, qy, qz].
        """
        if pose_wb.shape != (7,):
            raise ValueError(f"pose_wb must have shape (7,), got {pose_wb.shape}")

        self._pose_wb[:] = pose_wb
        R_wb = tfm.quat_to_rotation_matrix(
            self.pose_wb[3:]
        )
        g_b = R_wb.T @ self.g_w
        self.dyn.set_gravity(g_b)


    def set_ctrl_mode(self, mode: str | CtrlMode) -> None:
        """
        Set the current controller mode.

        Parameters
        -----------
        mode : str
            The desired controller mode.
            options: "pid", "ct", "im".

        Raises
        -------
        ValueError
            If an invalid mode is provided.
        """

        if isinstance(mode, CtrlMode):
            self.controller_mode = mode
            return
        
        match mode:
            case "pid":
                ctrl_mode = CtrlMode.PID
            case "ct":
                ctrl_mode = CtrlMode.CT
            case "im":
                ctrl_mode = CtrlMode.IM
            case _:
                raise ValueError(f"Unknown control mode '{mode}'")

        self.controller_mode = ctrl_mode


    def set_joint_state(self,
        q: FloatArray | None = None,
        qd: FloatArray | None = None,
        t: float | None = None,
        qdd: FloatArray | None = None,
    ) -> None:
        """
        Update current joint state and refresh dynematics.

        Parameters
        -----------
        q : ndarray, (n,)
            Current joint positions.
        qd : ndarray, (n,)
            Current joint velocities.
        qdd : ndarray, (n,)
            Current joint accelerations.
        t : float
            Current time in seconds.
        """
        if q is not None: self.q = q
        if qd is not None: self.qd = qd
        if qdd is not None: self.qdd = qdd
        if t is not None: self.t_prev = self.t; self.t = t


    def has_traj(self) -> bool:
        """
        Check if a trajectory is available at time t.

        Parameters
        -----------
        t: float
            Time in seconds.

        Returns
        ----------
        available: bool
            True if a trajectory is available, False otherwise.
        """
        return self.t >= self.ti and self.t <= self.tf
    

    def clear_traj(self) -> None:
        """
        Clear the currently set trajectory.
        """
        self.ti = 0.0
        self.tf = 0.0
        self.joint_traj = JointTraj()


    def set_joint_des(self,
        q_des: FloatArray | None = None,
        qd_des: FloatArray | None = None,
        qdd_des: FloatArray | None = None,
    ) -> None:
        """
        Set desired target state to `q_des`, `qd_des`, and `qdd_des`.

        Parameters
        -----------
        q_des : ndarray, (n,), optional
            Desired joint positions.
        qd_des : ndarray, (n,), optional
            Desired joint velocities. If None, set to zero.
        qdd_des : ndarray, (n,), optional
            Desired joint accelerations. If None, set to zero.
        """
        if q_des is not None: self.q_des = q_des
        if qd_des is not None: self.qd_des = qd_des
        if qdd_des is not None: self.qdd_des = qdd_des


    def set_joint_traj(self,
        joint_traj: JointTraj,
        duration: float,
        ti: float | None = None,
    ) -> None:
        """
        Set the joint trajectory to follow.
        
        Parameters
        -----------
        joint_traj : JointTraj
            Joint trajectory (times, positions, velocities, accelerations).
        duration : float
            Duration of the trajectory.
        ti : float
            initial time offset of the trajectory.
        
        -------
        Notes
            -Stores `T` as an ndarray in `self.T`.
            -Computes and stores the trajectory frequency in `self.freq`.
            -Sets `self.ti` and `self.tf`.
        """
        if ti is None: ti = self.t
        self.joint_traj = joint_traj
        self.ti = ti
        tf = ti + duration
        self.tf = tf

        self.TraceMode = TraceMode.TRAJ


    def update_joint_des(self):
        """
        Update the desired joint state based on the current time and active trajectory.

        This method updates self.q_des, self.qd_des, and self.qdd_des according to
        the tracdyng mode and the internally stored joint trajectory.

        Behavior
        --------
        1) If `self.tracdyng_mode` is `TracdyngMode.HOLD`, the method returns without changes.
        2) If a trajectory is active at the current time,
        the method samples the trajectory and uses
        the returned values as the new desired joint state.
        3) If no trajectory is active, the method keeps the last desired position,
        sets desired velocities and accelerations to zero, and switches
        `self.tracdyng_mode` to `TracdyngMode.HOLD`.

        """
        if self.TraceMode is TraceMode.HOLD:
            return
        
        if self.has_traj():
            q_des, qd_des, qdd_des = self.sample_joint_traj()
        else: 
            self.clear_traj()
            q_des = self.q_des
            qd_des = np.zeros_like(self.q_des)
            qdd_des = np.zeros_like(self.q_des)
            self.TraceMode = TraceMode.HOLD

        self.set_joint_des(q_des, qd_des, qdd_des)


    def sample_joint_traj(self) -> tuple[FloatArray, FloatArray, FloatArray]:
        """
        Sample the desired joint state from the internally stored trajectory
        at the current time `self.t`.

        Uses the time stamps in `self.joint_traj.t` and performs linear
        interpolation between the two neighboring samples. If `self.t` is
        before the first sample or after the last sample, the boundary
        sample is returned.
        """
        traj = self.joint_traj  # type: ignore[attr-defined]
        t = self.t

        t_arr = traj.t      # (N,)
        q_arr = traj.q      # (N, n)
        qd_arr = traj.qd    # (N, n)
        qdd_arr = traj.qdd  # (N, n)

        # Clamp to first / last sample if outside range
        if t <= t_arr[0]:
            return q_arr[0].copy(), qd_arr[0].copy(), qdd_arr[0].copy()

        if t >= t_arr[-1]:
            return q_arr[-1].copy(), qd_arr[-1].copy(), qdd_arr[-1].copy()

        # Find indices i0, i1 such that t_arr[i0] <= t_now <= t_arr[i1]
        i1 = int(np.searchsorted(t_arr, t, side="right"))
        i0 = i1 - 1

        t0 = float(t_arr[i0])
        t1 = float(t_arr[i1])
        dt = t1 - t0

        # Guard against degenerate dt
        if dt <= 0.0: alpha = 0.0
        else: alpha = (t - t0) / dt

        # Linear interpolation
        q_des   = (1.0 - alpha) * q_arr[i0]   + alpha * q_arr[i1]
        qd_des  = (1.0 - alpha) * qd_arr[i0]  + alpha * qd_arr[i1]
        qdd_des = (1.0 - alpha) * qdd_arr[i0] + alpha * qdd_arr[i1]

        return q_des, qd_des, qdd_des


    def compute_ctrl_effort(self) -> FloatArray:
        """
        Compute control efforts using the current controller.

        Returns
        -------
        tau: ndarray, (n,)
            Control efforts.
        """

        q, qd, qdd = self.q, self.qd, self.qdd
        q_des, qd_des, qdd_des = self.q_des, self.qd_des, self.qdd_des

        match self.controller_mode:
            case CtrlMode.CT:
                tau = self.ctrl.computed_torque(q, qd, q_des, qd_des, qdd_des)
            case CtrlMode.PID:
                dt = self.t - self.t_prev
                tau = self.ctrl.pid(q, qd, q_des, qd_des, dt)
            case CtrlMode.GC:
                tau = self.dyn.gravity_vector()
            case _:
                tau = np.zeros(self.n, dtype=npu.dtype)

        return tau
    

    def setup_quintic_traj(self,
        q_des: FloatArray,
        duration: float  = 10.0,
        q: FloatArray | None = None,
        ti: float | None = None,
        freq: float = 100.0,
    ) -> JointTraj:

        if q is None: q = self.q
        if ti is None: ti = self.t

        joint_traj = self.planner.quintic_trajs(q, q_des, duration, freq)
        self.set_joint_traj(joint_traj, duration)
        return joint_traj


    def get_cartesian_traj(self) -> CartesianTraj:
        """
        Get end-effector pose trajectory as position and quaternion.
        from the active joint trajectory.

        Returns
        -------
        CartesianTraj : CartesianTraj
            Cartesian trajectory of end-effector.
        """
        joint_traj = self.joint_traj
        T_w_ee_traj = self.dyn.batch_frame_T(joint_traj.q)
        poses = tfm.transform_to_pos_quat(T_w_ee_traj)
        self.cart_traj = CartesianTraj(t=joint_traj.t, pose=poses)
        return self.cart_traj
