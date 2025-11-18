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
        self._log: dict[str, list] = {
            "t":   [],
            "qt":   [],
            "qdt": [],
            "ddqt":[],
            "q":   [],
            "qd":  [],
            "tau": [],
            "e":   [],
            "de":  [],
            "v":   [],
            "inertia_com": [],
            "h":   [],
            "tau_g": [],
            "Mjd": [],
            "bjd": [],
            "b":   [],
        }


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
        M, h, tau_g = self.dyn.Dynamics_matrices(q, qd)
        inertia_com = M @ v
        tau = inertia_com + h + tau_g
        b = h + tau_g
        Mjd = mjd["M"] @ -v[1:]
        taumj = -Mjd - mjd["b"]
        taumj = torch.cat([torch.zeros(1, device=taumj.device, dtype=taumj.dtype), taumj], dim=0)
        
        self.record_controller(qt, qdt, ddqt, q, qd, t, 
                               tau, e, de, v, 
                               inertia_com, h, tau_g, 
                               Mjd, mjd["b"], b)
        return taumj


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


    def record_controller(self,
        qt: torch.Tensor,
        qdt: torch.Tensor,
        ddqt: torch.Tensor,
        q: torch.Tensor,
        qd: torch.Tensor,
        t: float,
        tau: torch.Tensor,
        e: torch.Tensor,
        de: torch.Tensor,
        v: torch.Tensor,
        inertia_com: torch.Tensor,
        h: torch.Tensor,
        tau_g: torch.Tensor,
        Mjd: torch.Tensor,
        bjd: torch.Tensor,
        b: torch.Tensor,
    ) -> None:
        """Record controller data for analysis (fast append, in-memory)."""
        log = self._log
        log["t"].append(float(t))
        # detach tensors to drop graph, avoid copies/moves
        log["qt"].append(qt.detach())
        log["qdt"].append(qdt.detach())
        log["ddqt"].append(ddqt.detach())
        log["q"].append(q.detach())
        log["qd"].append(qd.detach())
        log["tau"].append(tau.detach())
        log["e"].append(e.detach())
        log["de"].append(de.detach())
        log["v"].append(v.detach())
        log["inertia_com"].append(inertia_com.detach())
        log["h"].append(h.detach())
        log["tau_g"].append(tau_g.detach())
        log["Mjd"].append(Mjd.detach())
        log["bjd"].append(bjd.detach())
        log["b"].append(b.detach())


    def get_log(self, as_numpy: bool = True) -> dict[str, np.ndarray | torch.Tensor]:
        """
        Return stacked logs.
        - as_numpy=True: returns numpy arrays (t: (N,), others: (N, n))
        - as_numpy=False: returns torch tensors on current device
        """
        out: dict[str, np.ndarray | torch.Tensor] = {}
        out["t"] = np.asarray(self._log["t"], dtype=float)
        # stack vector entries; handle empty log
        keys = ["qt","qdt","ddqt","q","qd","tau","e","de","v", "inertia_com", "h", "tau_g", "Mjd", "bjd", "b"]
        for k in keys:
            if len(self._log[k]) == 0:
                out[k] = np.empty((0, self.dyn.kin.n), dtype=float) if as_numpy else torch.empty((0, self.dyn.kin.n), device=ptu.device)
                continue
            ts = torch.stack(self._log[k], dim=0)  # (N, n)
            out[k] = ptu.to_numpy(ts) if as_numpy else ts
        return out


    def clear_log(self) -> None:
        """Clear all recorded data."""
        for k in self._log:
            self._log[k].clear()