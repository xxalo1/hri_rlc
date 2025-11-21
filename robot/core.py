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
    kin: Kinematics[torch.Tensor]
    dyn: Dynamics
    ctrl: Controller
    planner: TrajPlanner

    def __post_init__(self) -> None:
        n = self.kin.n
        device = self.kin.d.device

        self.qt = torch.zeros(n, device=device)

    @classmethod
    def from_spec(cls, 
        spec: RobotSpec
    ) -> Robot:
        T_base_t = ptu.from_numpy(spec.T_base)
        dh_t = ptu.from_numpy_dict(spec.dh)
        inertia_t = ptu.from_numpy_dict(spec.inertia)

        kin = Kinematics(dh=dh_t, T_wb=T_base_t, inertia=inertia_t)
        dyn = Dynamics(kin)
        ctrl = Controller(dyn)
        planner = TrajPlanner()
        return cls(spec, kin, dyn, ctrl, planner)
    

    def set_target(self, 
        qt: torch.Tensor
    ) -> None:
        self.qt = qt
        self.dqt = torch.zeros_like(qt)
        self.ddqt = torch.zeros_like(qt)


    def setup_quintic_traj(self, 
        q: torch.Tensor | None = None,
        qt: torch.Tensor | None = None,
        freq: float = 100.0,
        ti: float = 0.0,
        tf: float  = 10.0,
    ) -> None:
        
        if q is None: q = self.kin.q
        if qt is None: qt = self.qt
        else: self.set_target(qt)
        

        q = ptu.to_numpy(q)
        qt = ptu.to_numpy(qt)
        T = self.planner.quintic_trajs(q, qt, ti, tf, freq)

        self.set_trajectory(T, freq=freq, ti=ti)


    def get_ee_traj(self) -> FloatArray:
        T = self.T
        T_wf_traj = self.kin.batch_forward_kinematics(T[0])
        Q_EF = T_wf_traj[:, -1, :3, 3]
        self.Q_EF_np = ptu.to_numpy(Q_EF)

        return self.Q_EF_np
    

    def set_trajectory(self, 
        T: torch.Tensor | FloatArray,
        freq: float,
        ti: float = 0.0
    ) -> None:
        """
        Set the trajectory to follow.
        
        Parameters
        -----------
        T: ndarray | Tensor, (3, N, n)
            T[0]: desired positions.
            T[1]: desired velocities.
            T[2]: desired accelerations.
        freq: float:
            frequency at which the trajectory is sampled.
        ti: float
            initial time offset of the trajectory.
        
        --------
        Notes:
        Stores T as a Tensor in self.T.
        """
        if isinstance(T, np.ndarray):
            T = ptu.from_numpy(T)
        self.T = T
        self.freq = freq
        self.ti = ti


    def get_desired_state(self,
        t: float
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Get desired state at a given time.

        Parameters
        -----------
        t: float
            Time in seconds.

        Returns
        ----------
        qt: Tensor, (n,)
            Desired joint positions at time step.
        qdt: Tensor, (n,)
            Desired joint velocities at time step.
        ddqt: Tensor, (n,)
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
        qt   = (1.0 - alpha) * self.T[0, i0] + alpha * self.T[0, i1]
        qdt  = (1.0 - alpha) * self.T[1, i0] + alpha * self.T[1, i1]
        ddqt = (1.0 - alpha) * self.T[2, i0] + alpha * self.T[2, i1]

        return qt, qdt, ddqt