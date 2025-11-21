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
        Kv: float | None = None, 
        Kp: float | None = None,
    ) -> None:
        
        self.dyn = dynamics
        I = torch.eye(dynamics.kin.n, device=ptu.device)
        if Kv is None:
            Kv = 2.0
        if Kp is None:
            Kp = 5.0
        self.Kv : torch.Tensor = Kv * I
        self.Kp : torch.Tensor = Kp * I
        # lightweight in-memory logger (append-only, fast)


    def computed_torque(self, 
        q: torch.Tensor, 
        qd: torch.Tensor, 
        t: float,
        mjd: dict[str, torch.Tensor]
    ) -> torch.Tensor:
        """Compute torque using computed torque control."""
        qt, qdt, ddqt = self.get_desired_state(t)
        e = qt - q
        de = qdt - qd
        v = ddqt + self.Kv @ de + self.Kp @ e

        Mjd = mjd["M"] @ v
        taumj = Mjd + mjd["b"]

        tau_rnea = self.dyn.inverse_dynamics_rnea(q, qd, v)

        dict_out = {
                    "qt": qt, "qdt": qdt, "ddqt": ddqt, "q": q, "qd": qd, "qdd": mjd["qdd"], "t": t, 
                    "e": e, "de": de, "v": v, "Mjd": Mjd, "bjd": mjd["b"], "taumj": taumj, "tau_rnea": tau_rnea
                    }

        return taumj, tau_rnea, dict_out


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

