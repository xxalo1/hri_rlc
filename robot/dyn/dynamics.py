from __future__ import annotations

import numpy as np
import torch
from typing import Any, Optional, Sequence, Callable
from numpy.typing import ArrayLike, NDArray

from ...utils import numpy_util as npu
from ...utils import pytorch_util as ptu
from ..kin import Kinematics

FloatArray = npu.FloatArray
dtype = npu.dtype

def skew(v: FloatArray) -> FloatArray:

    x, y, z = v[..., 0], v[..., 1], v[..., 2]
    S = np.zeros((*v.shape[:-1], 3, 3), dtype=v.dtype)
    S[..., 0, 1] = -z
    S[..., 0, 2] =  y
    S[..., 1, 0] =  z
    S[..., 1, 2] = -x
    S[..., 2, 0] = -y
    S[..., 2, 1] =  x
    return S

class Dynamics:
    """
    Minimal NumPy dynamics that *depends on* your Kinematics instance.
    Implements:
      - M(q) = Σ_i [ Jv_i^T m_i Jv_i + Jw_i^T I_Ci^w Jw_i ]
      - g(q) = Σ_i Jv_i^T (m_i * g_w)
    Notes:
      - No inertial duplication: reads kin.mass, kin.com_fl, kin.Ic_fl directly.
      - Per-link Jacobian is built via kin.jac(q, link_idx=i) (6 × (i+1)), then padded.
      - Linear Jacobian is shifted to COM using Jv_com = Jv_origin + Jw @ [r]_x.
      - I_C is rotated to world per link: Iw = R_wi @ Ic_link @ R_wi^T.
    """

    def __init__(self, kin: Kinematics, n: int, joint_to_link: np.ndarray | None = None,
                 g_vec: np.ndarray = np.array([0.0, 0.0, -9.81], dtype=float)):
        self.kin = kin
        self.n = int(n)
        self.joint_to_link = np.arange(self.n, dtype=int) if joint_to_link is None \
                             else np.asarray(joint_to_link, dtype=int)
        self.g_vec = np.asarray(g_vec, dtype=float)
        self.lib = np 


    def J_full(self, q: FloatArray) -> FloatArray:
        """
        Full (n*6*n) Jacobian for all links.
        """
        n = len(q)
        J = np.zeros((n, 6, n), dtype=q.dtype)
        for i in range(n):
            J[i, :, : i + 1] = self.kin.jac(q, i=i)  # (6, i+1)
        return J


    def shift_J_and_Ic(
        self, 
        J: FloatArray, 
        T_wl: FloatArray, 
        com_l: FloatArray, 
        Ic: FloatArray
        ) -> tuple[FloatArray, FloatArray]:
        """
        updates J to be at links com instead of frame origin
        updates Ic to be in world frame

        Parameters
        ----------
        J : FloatArray, shape (n, 6, n)
            Full Jacobian for all links.
        T_wl : FloatArray, shape (n, 4, 4)
            World to link transforms for all links.
            [T_w_1, T_w_2, ..., T_w_n]
        com_l : FloatArray, shape (n, 3)
            Link frame origins to COM in link frame.
            [com_l1, com_l2, ..., com_ln]
        Ic : FloatArray, shape (n, 3, 3)
            Link inertia matrices in link frame.
            [I_1, I_2, ..., I_n]   
        Returns
        -------
        J : FloatArray, shape (n, 6, n)
            full jacobian with linear part updated to COM

        Ic_w : FloatArray, shape (n, 3, 3)
            Link inertia matrices in world frame.
        """

        Jv_o = J[:, 0:3, :]
        Jw   = J[:, 3:6, :]

        R_wl = T_wl[:, :3, :3]   # (n, 3, 3)
        r_w  = R_wl @ com_l      # (n, 3)

        S = skew(r_w)            # (n, 3, 3)
        Jv_com = Jv_o - (S @ Jw) # (n, 3, n)

        Ic_w = R_wl @ Ic @ np.transpose(R_wl, (0, 2, 1))

        return Jv_com, Ic_w


    def Dynamics_matrices(self, q: FloatArray) -> FloatArray:
        """Jacobian/energy-form M(q)."""
        m  = self.kin.mass[1:]      # skip base
        com_fl = self.kin.com_fl[1:]
        Ic_fl  = self.kin.Ic_fl[1:]
        g_w = self.g_w
        n = len(q)

        J = self.J_full(q)         # (n, 6, n)
        T_wl = self.kin.fk(q)      # (n, 4, 4)

        Jv_com, Ic_w = self.shift_J_and_Ic(J, T_wl, com_fl, Ic_fl)
        Jw = J[:, 3:6, :]

        Mv = np.einsum('ikn, i, ikm -> nm', Jv_com, m, Jv_com)  # (n,n)

        IwJw = np.einsum('ijk, ikn -> ijn', Ic_w, Jw)          # (n, 3, n)
        Mw   = np.einsum('ikn, ikm -> nm', Jw, IwJw)           # (n,n)
        
        M = Mv + Mw
        M = 0.5 * (M + M.T)  # tidy symmetry

        tau_g = np.einsum('i, ikn , k -> n', m, Jv_com, g_w)  # (n,)

        return M, tau_g