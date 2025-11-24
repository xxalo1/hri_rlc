from __future__ import annotations
import math

import numpy as np
import torch


# robot/robot.py
from dataclasses import dataclass
from .kin import Kinematics
from .dyn import Dynamics
from .control import Controller
from .planning import TrajPlanner
from ..utils import pytorch_util as ptu
from ..utils import numpy_util as npu
FloatArray = npu.FloatArray


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
        n = self.kin.n
        self.q_des = np.zeros(n, dtype=npu.dtype)


    @classmethod
    def from_spec(cls, 
        spec: RobotSpec
    ) -> Robot:
        kin = Kinematics(dh=spec.dh, T_wb=spec.T_base, inertia=spec.inertia)
        dyn = Dynamics(kin)
        ctrl = Controller(dyn)
        planner = TrajPlanner()
        return cls(spec, kin, dyn, ctrl, planner)


    def set_target(self,
        q_des: FloatArray
    ) -> None:
        self.q_des = q_des
        self.qd_des = np.zeros_like(q_des)
        self.qdd_des = np.zeros_like(q_des)


    def setup_quintic_traj(self, 
        q: FloatArray | None = None,
        q_des: FloatArray | None = None,
        freq: float = 100.0,
        ti: float = 0.0,
        tf: float  = 10.0,
    ) -> None:
        
        if q is None: q = self.kin.q
        if q_des is None: q_des = self.q_des
        else: self.set_target(q_des)
        
        T = self.planner.quintic_trajs(q, q_des, ti, tf, freq)

        self.set_trajectory(T, freq=freq, ti=ti)


    def get_ee_traj(self) -> FloatArray:
        T = self.T
        T_wf_traj = self.kin.batch_forward_kinematics(T[0])
        Q_EF = T_wf_traj[:, -1, :3, 3]
        self.Q_EF_np = Q_EF

        return self.Q_EF_np
    

    def set_trajectory(self, 
        T: FloatArray,
        freq: float,
        ti: float = 0.0
    ) -> None:
        """
        Set the trajectory to follow.
        
        Parameters
        -----------
        T: ndarray, (3, N, n)
            T[0]: desired positions.
            T[1]: desired velocities.
            T[2]: desired accelerations.
        freq: float:
            frequency at which the trajectory is sampled.
        ti: float
            initial time offset of the trajectory.
        
        --------
        Notes:
        Stores T as an ndarray in self.T.
        """
        self.T = T
        self.freq = freq
        self.ti = ti


    def get_desired_state(self,
        t: float
    ) -> tuple[FloatArray, FloatArray, FloatArray]:
        """
        Get desired state at a given time.

        Parameters
        -----------
        t: float
            Time in seconds.

        Returns
        ----------
        q_des: ndarray, (n,)
            Desired joint positions at time step.
        qd_des: ndarray, (n,)
            Desired joint velocities at time step.
        qdd_des: ndarray, (n,)
            Desired joint accelerations at time step.

        ----------
        Notes
        --
            1- Assumes self.T has shape (3, N, n) and self.freq is the
            sampling frequency (Hz) of the trajectory.
            2- Times outside [self.ti, self.ti + (N-1)/self.freq] are clamped to the first/last sample.
        """

        s = (t - self.ti) * self.freq  # continuous index
        N = self.T.shape[1]

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
        q_des   = (1.0 - alpha) * self.T[0, i0] + alpha * self.T[0, i1]
        qd_des  = (1.0 - alpha) * self.T[1, i0] + alpha * self.T[1, i1]
        qdd_des = (1.0 - alpha) * self.T[2, i0] + alpha * self.T[2, i1]

        return q_des, qd_des, qdd_des