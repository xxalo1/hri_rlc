import numpy as np
from typing import Any, Optional, Sequence
from numpy.typing import ArrayLike, NDArray

from ...utils import util as ut

FloatArray = ut.FloatArray
dtype = ut.dtype

def jacobian(T_0: FloatArray, Ts: Sequence[FloatArray]) -> FloatArray:
    """
    6*n Jacobian for revolute joints:
        Jv_i = z_{i-1} * (o_n - o_{i-1})
        Jw_i = z_{i-1}
    args:
        T_0: base frame transform (4,4) T_0_0
        Ts: list of n transforms (4,4) with respect to base frame T_0_1, T_0_2, ..., T_0_n at current theta
    Returns J
    """
    n = len(Ts)
    origins = [T_0[:3, 3]]   # o_0
    axes_z  = [T_0[:3, 2]]   # z axis from rotation matrix
    for i in range(n):
        origins.append(Ts[i][:3, 3].copy())  # o_i
        axes_z.append(Ts[i][:3, 2].copy())   # z_i

    o_n = origins[-1]
    Jv = np.zeros((3, n), dtype=dtype)
    Jw = np.zeros((3, n), dtype=dtype)
    for i in range(n):
        z_im1 = axes_z[i]
        o_im1 = origins[i]
        Jv[:, i] = np.cross(z_im1, o_n - o_im1)
        Jw[:, i] = z_im1
    J = np.vstack([Jv, Jw])  # 6*n
    return J
