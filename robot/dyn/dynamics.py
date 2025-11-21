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

        Mv = torch.einsum('ian, iam, i -> nm', Jv_com, Jv_com, m)
        Mw = torch.einsum('ian, iab, ibm -> nm', Jw, Ic_wl, Jw)
        
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
        return -tau_g


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
        Compute h(q, qd) so that tau = M(q) qdd + h(q, qd) + tau_g(q).

        Uses Christoffel symbols from the gradient of M(q):
            c_ijk = 0.5 * ( dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i )
        and
            h_i   = sum_{j,k} c_ijk * qd_j * qd_k
        """
        q_req = q.detach().clone().requires_grad_(True)

        def M_fn(q_in: torch.Tensor) -> torch.Tensor:
            self.kin.step(q=q_in)
            m     = self.kin.mass
            J_com = self.kin.jac_com()
            Ic_wl = self.kin.Ic_wl
            return self.inertia_matrix(J_com, m, Ic_wl)  # (n, n)

        # dM_dq[k, i, j] = ∂M[i, j] / ∂q[k]
        dM_dq = torch.autograd.functional.jacobian(
            M_fn,
            q_req,
            create_graph=False,
            vectorize=True,
        )  # shape (n, n, n) = (k, i, j)

        A = dM_dq  # just to match the math notation

        # term1[i,j,k] = ∂M_ij / ∂q_k = A[k, i, j]
        term1 = A.permute(1, 2, 0)

        # term2[i,j,k] = ∂M_ik / ∂q_j = A[j, i, k]
        term2 = A.permute(1, 0, 2)

        # term3[i,j,k] = ∂M_jk / ∂q_i = A[i, j, k]
        term3 = A

        c = 0.5 * (term1 + term2 - term3)   # (n, n, n) with indices (i, j, k)

        # h_i = sum_{j,k} c_ijk qd_j qd_k
        h = torch.einsum("ijk,j,k->i", c, qd, qd)

        return h


    @staticmethod
    def _cross(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
        """Small wrapper: cross product for (3,) vectors."""
        return torch.cross(a, b, dim=-1)


    def inverse_dynamics_rnea(
        self,
        q: torch.Tensor,
        qd: torch.Tensor,
        qdd: torch.Tensor,
    ) -> torch.Tensor:
        """
        Recursive Newton–Euler inverse dynamics.

        Computes joint torques tau(q, qd, qdd) for a fixed-base, all-revolute
        manipulator using link quantities expressed in the WORLD frame.

        Assumes:
            - joint i rotates about z_{i-1} (z-axis of frame i-1)
            - forward_kinematics() returns T_wf[k] for frames k = 0..n
              (0: base, k: joint k origin)
            - kin.mass, kin.com_wl, kin.Ic_wl correspond to links 1..n.

        Parameters
        ----------
        q   : (n,) joint positions
        qd  : (n,) joint velocities
        qdd : (n,) joint accelerations

        Returns
        -------
        tau : (n,) joint torques
        """
        device = q.device
        n = self.kin.n

        assert q.shape == (n,)
        assert qd.shape == (n,)
        assert qdd.shape == (n,)

        # --- update kinematics cache ---
        self.kin.step(q=q)

        # world-to-frame transforms (0..n), positions and joint z-axes
        T_wf = self.kin.forward_kinematics()            # (n+1, 4, 4)
        p = T_wf[:, :3, 3]                              # (n+1, 3) origins
        z = T_wf[:, :3, 2]                              # (n+1, 3) z-axes

        # link data in WORLD frame
        m = self.kin.mass                               # (n,)
        com_wl = self.kin.com_wl                        # (n, 3)
        Ic_wl = self.kin.Ic_wl                          # (n, 3, 3)

        # --- forward recursion: velocities & accelerations ---

        w   = torch.zeros((n, 3), device=device)        # angular velocity
        dw  = torch.zeros((n, 3), device=device)        # angular accel
        dvJ = torch.zeros((n, 3), device=device)        # linear accel at joint origin
        a_c = torch.zeros((n, 3), device=device)        # linear accel at COM

        # base (frame 0) state
        w_prev  = torch.zeros(3, device=device)
        dw_prev = torch.zeros(3, device=device)
        dv_prev = -self.g_vec.to(device)                # a_0 = -g

        for i in range(n):
            # joint i+1: parent frame index = i, joint frame index = i+1
            idx_parent = i
            idx_joint = i + 1

            z_im1 = z[idx_parent]                       # axis of joint i+1 in world
            qdi   = qd[i]
            qddi  = qdd[i]

            # angular velocity & acceleration
            w_i = w_prev + qdi * z_im1
            dw_i = dw_prev + qddi * z_im1 + self._cross(w_prev, qdi * z_im1)

            # linear accel of joint origin i+1
            r_prev_to_joint = p[idx_joint] - p[idx_parent]
            dv_i = (
                dv_prev
                + self._cross(dw_prev, r_prev_to_joint)
                + self._cross(w_prev, self._cross(w_prev, r_prev_to_joint))
            )

            # COM accel for link i+1
            r_joint_to_com = com_wl[i] - p[idx_joint]
            a_c_i = (
                dv_i
                + self._cross(dw_i, r_joint_to_com)
                + self._cross(w_i, self._cross(w_i, r_joint_to_com))
            )

            w[i]   = w_i
            dw[i]  = dw_i
            dvJ[i] = dv_i
            a_c[i] = a_c_i

            # propagate for next link
            w_prev, dw_prev, dv_prev = w_i, dw_i, dv_i

        # --- link wrenches in world frame ---

        F = torch.zeros((n, 3), device=device)          # linear force at COM
        N_c = torch.zeros((n, 3), device=device)        # moment about COM

        for i in range(n):
            I_i = Ic_wl[i]                              # (3, 3)
            w_i = w[i]
            dw_i = dw[i]
            a_c_i = a_c[i]

            F_i = m[i] * a_c_i
            N_ci = I_i @ dw_i + self._cross(w_i, I_i @ w_i)

            F[i] = F_i
            N_c[i] = N_ci

        # --- backward recursion: joint torques ---

        f = torch.zeros((n, 3), device=device)          # net force at joint origin
        n_m = torch.zeros((n, 3), device=device)        # net moment at joint origin
        tau = torch.zeros(n, device=device)

        for i in reversed(range(n)):
            idx_joint = i + 1    # origin of joint i+1
            # vector from joint origin to COM
            r_joint_to_com = com_wl[i] - p[idx_joint]

            # moment about joint origin from this link alone
            N0_i = N_c[i] + self._cross(r_joint_to_com, F[i])

            if i == n - 1:
                # last link: no child
                f_i = F[i]
                n_i = N0_i
            else:
                # include child wrench transported to this joint origin
                f_child = f[i + 1]
                n_child = n_m[i + 1]

                r_joint_to_next = p[idx_joint + 1] - p[idx_joint]

                f_i = F[i] + f_child
                n_i = N0_i + n_child + self._cross(r_joint_to_next, f_child)

            f[i] = f_i
            n_m[i] = n_i

            # joint axis for joint i+1 is z_{i} (parent frame)
            axis = z[i]
            tau[i] = torch.dot(axis, n_i)

        return tau
