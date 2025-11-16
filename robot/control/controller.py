import torch
import numpy as np
import math

from ..dyn import Dynamics
from ..traj import TrajPlanner
from ...utils import pytorch_util as ptu
from ...utils import numpy_util as npu
FloatArray = npu.FloatArray

class Controller:
    """Robot controller base class."""
    def __init__(self, 
        dynamics: Dynamics, 
        Kv: torch.Tensor | None = None, 
        Kp: torch.Tensor | None = None,
    ) -> None:
        
        self.dyn = dynamics
        if Kv is None:
            Kv = torch.eye(dynamics.kin.n, dtype=ptu.dtype) * 2.0
        if Kp is None:
            Kp = torch.eye(dynamics.kin.n, dtype=ptu.dtype) * 5.0
        self.Kv = Kv
        self.Kp = Kp


    def computed_torque(self, 
        q: torch.Tensor, 
        qd: torch.Tensor, 
        t: float,
    ) -> torch.Tensor:
        """Compute torque using computed torque control."""
        qt, qdt, ddqt = self.get_desired_state(t)
        e = qt - q
        de = qdt - qd
        v = ddqt + self.Kv @ de + self.Kp @ e
        M, C, tau_g = self.dyn.Dynamics_matrices(q, qd)
        tau = M @ v + C @ qd + tau_g
        return tau


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
            Current time in seconds.

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
            1- Assumes self.T has shape (3, N, n) and self.f is the
            sampling frequency (Hz) of the trajectory.
            2- Times outside [0, (N-1)/f] are clamped to the first/last sample.
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
