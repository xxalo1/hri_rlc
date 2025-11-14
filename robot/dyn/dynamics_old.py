from __future__ import annotations

import numpy as np
import torch
from typing import Any, Generic, Optional, Sequence, Callable, TypeVar


from ...utils import numpy_util as npu
from ...utils import pytorch_util as ptu
from ..kin import Kinematics

from ...utils import(numpy_util as npu, array_compat as xp, ArrayT, FloatArray, dtype)

class Dynamics(Generic[ArrayT]):

    def __init__(self, kin: Kinematics[ArrayT],
                 g_vec: ArrayT):
        self.kin = kin
        self.g_vec = g_vec
        self.g_w = self.g_vec

    def Dynamics_matrices(self, q: ArrayT) -> Sequence[ArrayT]:
        """Jacobian/energy-form M(q)."""
        m  = self.kin.mass[1:]           # (n,)
        g_w = self.g_w
        n = self.kin.n
        self.kin.step(q=q)
        J_com = self.kin.jac_com()       # (n, 6, n)
        Ic_wl = self.kin.Ic_wl[1:]       # (n, 3, 3)
        Jv_com = J_com[:, :3, :]         # (n, 3, n)
        Jw     = J_com[:, 3:, :]         # (n, 3, n)

        Mv = xp.einsum('ikn, i, ikm -> nm', Jv_com, m, Jv_com)  # (n,n)
        IwJw = xp.einsum('ijk, ikn -> ijn', Ic_wl, Jw)          # (n, 3, n)
        Mw   = xp.einsum('ikn, ikm -> nm', Jw, IwJw)           # (n,n)
        
        M = Mv + Mw
        M = 0.5 * (M + M.T)  # tidy symmetry

        tau_g = xp.einsum('i, ikn , k -> n', m, Jv_com, g_w)  # (n,)

        return M, tau_g