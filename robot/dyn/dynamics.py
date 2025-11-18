from __future__ import annotations

import torch
from typing import Sequence

from ..kin import Kinematics
from ...utils import pytorch_util as ptu
class Dynamics():

    def __init__(self, kin: Kinematics[torch.Tensor],
                 g_vec: torch.Tensor | None = None) -> None:
        self.kin = kin
        if g_vec is None:
            g_vec = torch.tensor([0.0, 0.0, -9.81], device=ptu.device)
        self.g_vec = g_vec


    def Dynamics_matrices(self, 
        q: torch.Tensor, 
        qd: torch.Tensor
        ) -> Sequence[torch.Tensor]:
        """Jacobian/energy-form M(q)."""
        self.kin.step(q=q)
        m  = self.kin.mass           # (n,)
        g = self.g_vec
        J_com = self.kin.jac_com()       # (n, 6, n)
        Ic_wl = self.kin.Ic_wl       # (n, 3, 3)
        M = self.inertia_matrix(J_com, m, Ic_wl)
        tau_g = self.gravity_vector(J_com, m, g)
        # C = self.coriolis_matrix(q, qd)
        h = self.coriolis_vector(q, qd)
        return M, h, tau_g


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
            m  = self.kin.mass
            J_com = self.kin.jac_com()
            Ic_wl = self.kin.Ic_wl
            return self.inertia_matrix(J_com, m, Ic_wl)  # (n, n)

        dM_dq = torch.autograd.functional.jacobian(M_fn, q_req, create_graph=False)

        term1 = dM_dq
        term2 = dM_dq.permute(0, 2, 1)  # (i, k, j)
        term3 = dM_dq.permute(1, 2, 0)  # (j, k, i)

        c = 0.5 * (term1 + term2 - term3)   # (n, n, n)

        C = torch.einsum('ijk,k->ij', c, qd)  # (n, n)
        return C


    def coriolis_vector(self, q: torch.Tensor, qd: torch.Tensor) -> torch.Tensor:
        """
        Compute h(q, qd) = C(q, qd) @ qd directly.
        """

        q_req = q.detach().clone().requires_grad_(True)

        def M_fn(q_in: torch.Tensor) -> torch.Tensor:
            self.kin.step(q=q_in)
            m  = self.kin.mass
            J_com = self.kin.jac_com()
            Ic_wl = self.kin.Ic_wl
            return self.inertia_matrix(J_com, m, Ic_wl)  # (n, n)

        # Jacobian of M wrt q: shape (n, n, n)
        dM_dq = torch.autograd.functional.jacobian(
            M_fn,
            q_req,
            create_graph=False,
            vectorize=True,        # important for speed
        )

        term1 = dM_dq
        term2 = dM_dq.permute(0, 2, 1)
        term3 = dM_dq.permute(1, 2, 0)

        c = 0.5 * (term1 + term2 - term3)      # (n, n, n)

        h = torch.einsum("ijk,j,k->i", c, qd, qd)   # (n,)
        return h
