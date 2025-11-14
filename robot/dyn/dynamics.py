from __future__ import annotations

import numpy as np
import torch
from typing import Any, Generic, Optional, Sequence, Callable, TypeVar


from ...utils import numpy_util as npu
from ...utils import pytorch_util as ptu
from ..kin import Kinematics

from ...utils import(numpy_util as npu, array_compat as xp, ArrayT, FloatArray, dtype)

class Dynamics():

    def __init__(self, kin: Kinematics[torch.Tensor],
                 g_vec: torch.Tensor):
        self.kin = kin
        self.g_vec = g_vec


    def inverse_dynamics(self, 
        q: torch.Tensor, 
        qd: torch.Tensor, 
        qdd: torch.Tensor
        ) -> torch.Tensor:
        M, tau_g, C = self.Dynamics_matrices(q, qd)
        return M @ qdd + C @ qd + tau_g


    def Dynamics_matrices(self, 
        q: torch.Tensor, 
        qd: torch.Tensor
        ) -> Sequence[torch.Tensor]:
        """Jacobian/energy-form M(q)."""
        self.kin.step(q=q)
        m  = self.kin.mass[1:]           # (n,)
        g = self.g_vec
        J_com = self.kin.jac_com()       # (n, 6, n)
        Ic_wl = self.kin.Ic_wl[1:]       # (n, 3, 3)
        M = self.inertia_matrix(J_com, m, Ic_wl)
        tau_g = self.gravity_vector(J_com, m, g)
        C = self.coriolis_matrix(q, qd)
        return M, tau_g, C


    def inertia_matrix(self, 
        J_com: torch.Tensor,
        m: torch.Tensor,
        Ic_wl: torch.Tensor,
        ) -> torch.Tensor:
        """Jacobian/energy-form M(q)."""
        Jv_com = J_com[:, :3, :]         # (n, 3, n)
        Jw     = J_com[:, 3:, :]         # (n, 3, n)
        Mv = torch.einsum('ikn, i, ikm -> nm', Jv_com, m, Jv_com)  # (n,n)
        IwJw = torch.einsum('ijk, ikn -> ijn', Ic_wl, Jw)          # (n, 3, n)
        Mw   = torch.einsum('ikn, ikm -> nm', Jw, IwJw)           # (n,n)
        
        M = Mv + Mw
        M = 0.5 * (M + M.T)  # tidy symmetry
        return M


    def gravity_vector(self,
        J_com: torch.Tensor,
        m: torch.Tensor,
        g: torch.Tensor
        ) -> torch.Tensor:
        """Gravity vector tau_g(q)."""
        Jv_com = J_com[:, :3, :]         # (n, 3, n)
        tau_g = torch.einsum('i, ikn , k -> n', m, Jv_com, g)  # (n,)
        return tau_g


    def coriolis_matrix(self,
        q: torch.Tensor,
        qd: torch.Tensor,
        ) -> torch.Tensor:
        """
        C(q, qd) such that tau_c = C(q, qd) @ qd.
        """
        q_req = q.detach().clone().requires_grad_(True)


        def M_fn(q_in: torch.Tensor) -> torch.Tensor:
            self.kin.step(q=q_in)
            m  = self.kin.mass[1:]
            J_com = self.kin.jac_com()
            Ic_wl = self.kin.Ic_wl[1:]
            return self.inertia_matrix(J_com, m, Ic_wl)  # (n, n)

        dM_dq = torch.autograd.functional.jacobian(M_fn, q_req, create_graph=False)

        term1 = dM_dq
        term2 = dM_dq.permute(0, 2, 1)  # (i, k, j)
        term3 = dM_dq.permute(1, 2, 0)  # (j, k, i)

        c = 0.5 * (term1 + term2 - term3)   # (n, n, n)

        C = torch.einsum('ijk,k->ij', c, qd)  # (n, n)
        return C


