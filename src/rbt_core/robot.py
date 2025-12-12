from __future__ import annotations
import math
import numpy as np
from dataclasses import dataclass
from enum import Enum, auto

from common_utils import numpy_util as npu
from common_utils import FloatArray
from common_utils import transforms as tfm

from .kin.kinematics import Kinematics
from .dynamics import Dynamics
from .controller import Controller
from .planning import TrajPlanner, JointTraj, CartesianTraj

class ControllerMode(Enum):
    CT = auto()
    PID = auto()
    IM = auto()

class TrackingMode(Enum):
    HOLD = auto()
    TRAJ = auto()


@dataclass
class RobotSpec:
    name: str
    dh: dict[str, FloatArray]
    inertia: dict[str, FloatArray]
    T_base: FloatArray
    joint_names: list[str]


@dataclass
class Robot:
    spec: RobotSpec
    kin: Kinematics[FloatArray]
    dyn: Dynamics
    ctrl: Controller
    planner: TrajPlanner


    def __post_init__(self) -> None:

        n = self.n

        # Current state
        self._q = np.zeros(n, dtype=npu.dtype)
        self._qd = np.zeros(n, dtype=npu.dtype)
        self._qdd = np.zeros(n, dtype=npu.dtype)
        self.t = 0.0
        self.t_prev = 0.0

        # Desired state
        self.q_des = np.zeros(n, dtype=npu.dtype)
        self.qd_des = np.zeros(n, dtype=npu.dtype)
        self.qdd_des = np.zeros(n, dtype=npu.dtype)

        # Trajectory storage
        self.ti = 0.0
        self.tf = 0.0
        self.freq = 0.0
        self.joint_traj = JointTraj()
        self.cart_traj = CartesianTraj()

        # Control modes
        self.controller_mode = ControllerMode.CT
        self.tracking_mode = TrackingMode.HOLD


    @classmethod
    def from_spec(cls, 
        spec: RobotSpec
    ) -> Robot:
        kin = Kinematics(dh=spec.dh, T_wb=spec.T_base, inertia=spec.inertia)
        dyn = Dynamics(kin)
        ctrl = Controller(dyn)
        planner = TrajPlanner()
        return cls(spec, kin, dyn, ctrl, planner)

    @property
    def n(self) -> int: return self.kin.n

    @property
    def q(self) -> FloatArray: return self._q
    @q.setter
    def q(self, v: FloatArray) -> None:
        self._q = v
        self.kin.step(q=v)

    @property
    def qd(self) -> FloatArray: return self._qd
    @qd.setter
    def qd(self, v: FloatArray) -> None:
        self._qd = v
        self.kin.step(qd=v)

    @property
    def qdd(self) -> FloatArray: return self._qdd
    @qdd.setter
    def qdd(self, v: FloatArray) -> None:
        self._qdd = v
        self.kin.step(qdd=v)

    @property
    def joint_names(self) -> list[str]:
        return self.spec.joint_names
    
    def set_ctrl_mode(self, mode: str) -> None:
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

        match mode:
            case "pid":
                ctrl_mode = ControllerMode.PID
            case "ct":
                ctrl_mode = ControllerMode.CT
            case "im":
                ctrl_mode = ControllerMode.IM
            case _:
                raise ValueError(f"Unknown control mode '{mode}'")

        self.controller_mode = ctrl_mode


    def set_joint_state(
        self,
        q: FloatArray | None = None,
        qd: FloatArray | None = None,
        t: float | None = None,
        qdd: FloatArray | None = None,
    ) -> None:
        """
        Update current joint state and refresh kinematics.

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
        self.freq = 0.0
        self.ti = 0.0
        self.tf = 0.0
        self.joint_traj = JointTraj()

        self.qd_des = np.zeros_like(self.qd_des)
        self.qdd_des = np.zeros_like(self.qdd_des)


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

        self.tracking_mode = TrackingMode.TRAJ


    def update_joint_des(self):
        """
        Update the desired joint state based on the current time and active trajectory.

        This method updates self.q_des, self.qd_des, and self.qdd_des according to
        the tracking mode and the internally stored joint trajectory.

        Behavior
        --------
        1) If `self.tracking_mode` is `TrackingMode.HOLD`, the method returns without changes.
        2) If a trajectory is active at the current time,
        the method samples the trajectory and uses
        the returned values as the new desired joint state.
        3) If no trajectory is active, the method keeps the last desired position,
        sets desired velocities and accelerations to zero, and switches
        `self.tracking_mode` to `TrackingMode.HOLD`.

        """
        if self.tracking_mode is TrackingMode.HOLD:
            return
        
        if self.has_traj():
            q_des, qd_des, qdd_des = self.sample_joint_traj()
        else: 
            q_des = self.q_des
            qd_des = np.zeros_like(self.q_des)
            qdd_des = np.zeros_like(self.q_des)
            self.tracking_mode = TrackingMode.HOLD

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
            case ControllerMode.CT:
                tau = self.ctrl.computed_torque(q, qd, q_des, qd_des, qdd_des)
            case ControllerMode.PID:
                dt = self.t - self.t_prev
                tau = self.ctrl.pid(q, qd, q_des, qd_des, dt)
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
        T_wf_traj = self.kin.batch_forward_kinematics(joint_traj.q)
        T_w_ee_traj = T_wf_traj[:, -1]
        poses = tfm.transform_to_pos_quat(T_w_ee_traj)
        self.cart_traj = CartesianTraj(t=joint_traj.t, pose=poses)
        return self.cart_traj
