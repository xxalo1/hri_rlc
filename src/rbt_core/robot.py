from __future__ import annotations
import math
import numpy as np
from dataclasses import dataclass
from enum import Enum, auto

from .kin.kinematics import Kinematics
from .dynamics import Dynamics
from .controller import Controller
from .planning import TrajPlanner
from common_utils import numpy_util as npu
from common_utils import FloatArray


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
        self.traj = np.zeros((3, 100, n), dtype=npu.dtype)

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
        q: FloatArray | None,
        qd: FloatArray | None,
        qdd: FloatArray | None,
        t: float | None,
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
        if q is not None: self._q = q
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
        self.traj = np.zeros((3, 0, self.kin.n), dtype=npu.dtype)

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
        traj: FloatArray,
        tf: float,
        ti: float | None = None,
    ) -> None:
        """
        Set the joint trajectory to follow.
        
        Parameters
        -----------
        traj : ndarray, (3, N, n)
            traj[0]: desired positions.
            traj[1]: desired velocities.
            traj[2]: desired accelerations.
        tf : float:
            final time of the trajectory.
        ti : float
            initial time offset of the trajectory.
        
        -------
        Notes
            -Stores `T` as an ndarray in `self.T`.
            -Computes and stores the trajectory frequency in `self.freq`.
            -Sets `self.ti` and `self.tf`.
        """
        if ti is None: ti = self.t
        self.traj = traj
        self.ti = ti
        self.tf = tf
        self.freq = 1.0 / ((tf - ti) / (traj.shape[1] - 1))

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
        Get desired state from the internally saved trajectory at time `self.t`.

        Notes
        -----
        - Assumes self.T has shape (3, N, n) and self.freq is the sampling frequency (Hz) of the trajectory.
        - Times outside [self.ti, self.ti + (N-1)/self.freq] are clamped to the first/last sample.
        """
        T = self.traj

        s = (self.t - self.ti) * self.freq
        N = self.traj.shape[1]

        if s <= 0.0:
            i0 = i1 = 0
            alpha = 0.0
        elif s >= N - 1:
            i0 = i1 = N - 1
            alpha = 0.0
        else:
            i0 = int(math.floor(s))      # lower index
            i1 = i0 + 1                  # upper index
            alpha = s - i0               # interpolation factor in [0,1)

        # linear interpolation between i0 and i1
        q_des   = (1.0 - alpha) * T[0, i0] + alpha * T[0, i1]
        qd_des  = (1.0 - alpha) * T[1, i0] + alpha * T[1, i1]
        qdd_des = (1.0 - alpha) * T[2, i0] + alpha * T[2, i1]

        return q_des, qd_des, qdd_des
    

    def compute_ctrl_effort(self) -> FloatArray:
        """
        Compute control efforts using the current controller.

        Returns
        -------
        tau: ndarray, (n,)
            Control efforts.
        """

        q = self.q; qd = self.qd; qdd = self.qdd
        q_des = self.q_des; qd_des = self.qd_des; qdd_des = self.qdd_des

        match self.controller_mode:
            case ControllerMode.CT:
                tau = self.ctrl.computed_torque(q, qd, q_des, qd_des, qdd_des)
            case ControllerMode.PID:
                dt = self.t - self.t_prev
                tau = self.ctrl.pid(q, qd, q_des, qd_des, dt)
            case _:
                tau = np.zeros(self.kin.n, dtype=npu.dtype)

        return tau
    

    def setup_quintic_traj(self,
        q_des: FloatArray,
        q: FloatArray | None = None,
        ti: float = 0.0,
        tf: float  = 10.0,
        freq: float = 100.0,
    ) -> None:

        if q is None: q = self.q
        T = self.planner.quintic_trajs(q, q_des, ti, tf, freq)
        self.set_joint_traj(T, tf=tf, ti=ti)


    def get_ee_traj(self) -> FloatArray:
        T = self.traj
        T_wf_traj = self.kin.batch_forward_kinematics(T[0])
        Q_EF = T_wf_traj[:, -1, :3, 3]
        self.Q_EF_np = Q_EF

        return self.Q_EF_np
