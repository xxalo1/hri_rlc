import torch
import numpy as np
import math

from ..dyn import Dynamics
from ...utils import pytorch_util as ptu
from ...utils import numpy_util as npu
FloatArray = npu.FloatArray

class Controller:
    """Robot controller base class."""
    def __init__(self, 
        dynamics: Dynamics, 
        Kv: float | None = None, 
        Kp: float | None = None,
        Ki: float | None = None,
    ) -> None:
        
        self.dyn = dynamics
        I = torch.eye(dynamics.kin.n, device=ptu.device)
        if Kv is None:
            Kv = 2.0
        if Kp is None:
            Kp = 5.0
        if Ki is None:
            Ki = 0.0
        self.Kv : torch.Tensor = Kv * I
        self.Kp : torch.Tensor = Kp * I
        self.Ki : torch.Tensor = Ki * I

        self.e_int = torch.zeros((dynamics.kin.n,), device=ptu.device)
        # lightweight in-memory logger (append-only, fast)


    def set_gains(self,
        Kp: float | torch.Tensor,
        Kv: float | torch.Tensor,
        Ki: float | torch.Tensor = 0.0,
    ) -> None:
        """
        Set proportional, derivative, and integral gains.
        
        Parameters
        -----------
        Kp: float | Tensor, (n, n)
            Proportional gain.
        Kv: float | Tensor, (n, n)
            Derivative gain.
        Ki: float | Tensor, (n, n)
            Integral gain.
        
        --------
        Notes:
        Stores gains as Tensors in self.Kp, self.Kv, self.Ki.
        1- If a float is provided, it is multiplied by identity.
        2- otherwise, the provided Tensor is used directly.
        """
        I = torch.eye(self.dyn.kin.n, device=ptu.device)
        if isinstance(Kp, float):
            Kp = Kp * I
        if isinstance(Kv, float):
            Kv = Kv * I
        if isinstance(Ki, float):
            Ki = Ki * I
        
        self.Kp = Kp
        self.Kv = Kv
        self.Ki = Ki


    def computed_torque(self, 
        q: torch.Tensor, 
        qd: torch.Tensor, 
        qt: torch.Tensor, 
        qdt: torch.Tensor,
        ddqt: torch.Tensor,
        mjd: dict[str, torch.Tensor]
    ) -> torch.Tensor:
        """Compute torque using computed torque control."""
        e = qt - q
        de = qdt - qd
        v = ddqt + self.Kv @ de + self.Kp @ e

        Mjd = mjd["M"] @ v
        taumj = Mjd + mjd["b"]

        tau_rnea = self.dyn.inverse_dynamics_rnea(q, qd, v)

        dict_out = {"e": e, "de": de, "v": v}

        return taumj, tau_rnea, dict_out


    def pid(self,
        q: torch.Tensor,
        qd: torch.Tensor,
        qt: torch.Tensor,
        qdt: torch.Tensor,
        dt: float,
    ) -> torch.Tensor:
        """
        Joint-space PID controller.

        Parameters
        ----------
        q  : (n,)  current joint positions
        qd : (n,)  current joint velocities
        t  : float current time (s)
        dt : float time step since last call (s)

        Returns
        -------
        tau : (n,) torque command
        """

        e  = qt  - q
        de = qdt - qd

        self.e_int = self.e_int + e * dt

        tau_p = self.Kp @ e
        tau_d = self.Kv @ de
        tau_i = self.Ki @ self.e_int

        tau = tau_p + tau_d + tau_i

        return tau
