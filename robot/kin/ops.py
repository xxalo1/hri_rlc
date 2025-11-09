import numpy as np
from typing import Any, Optional, Sequence
from numpy.typing import ArrayLike, NDArray

from ...utils import numpy_util as npu

FloatArray = npu.FloatArray
dtype = npu.dtype

def transform_matrix(theta: float, d: float,
                    a: float, alpha: float, b: float = 0.0
                    ) -> FloatArray:
    """Standard DH T_i (Craig)
    b: optional translation along the new z axis"""
    cT, sT = np.cos(theta), np.sin(theta)
    cA, sA = np.cos(alpha), np.sin(alpha)
    A = np.array([[cT, -sT * cA,  sT * sA, a * cT],
                    [sT,  cT * cA, -cT * sA, a * sT],
                    [0 ,      sA ,      cA ,     d  ],
                    [0 ,      0  ,      0  ,     1  ]], dtype=dtype)
    A[:3, 3] += b * A[:3, 2]  # translate along the new z axis
    return A


def transform_chain(theta: FloatArray, d: FloatArray, a: FloatArray, alpha: FloatArray, b: FloatArray
                    ) -> Sequence[FloatArray]:
    """
    Transform of joints with respect to each other using stored DH params.
    theta: 1D array (length n)
    d: 1D array (length n)
    a: 1D array (length n)
    alpha: 1D array (length n)
    b: optional 1D array of translations along the new z axis (length n)

    returns: [A0, …, A_{n-1}]
    """
    A = []
    for j in range(len(d)):
        A_i = transform_matrix(theta[j], d[j], a[j], alpha[j], b[j])
        A.append(A_i)
    return A


def cumulative_transforms(theta: FloatArray, d: FloatArray, a: FloatArray, alpha: FloatArray, b: FloatArray
                          ) -> Sequence[FloatArray]:
    """
    Transform of joints with respect to base frame using stored DH params.
    theta: 1D array (length n)
    d: 1D array (length n)
    a: 1D array (length n)
    alpha: 1D array (length n)
    b: optional 1D array of translations along the new z axis (length n)

    returns: [T_0_1, …, T_0_{n-1}]
    """
    T = []
    A = transform_chain(theta, d, a, alpha, b)
    T_i = np.eye(4, dtype=dtype)
    for j in range(len(d)):
        T_i = T_i @ A[j]
        T.append(T_i)
    return T


def jacobian(T_0: FloatArray, Ts: Sequence[FloatArray]
             ) -> FloatArray:
    n = len(Ts)
    origins = [T_0[:3, 3]]   # o_0
    axes_z  = [T_0[:3, 2]]   # z axis from rotation matrix
    for i in range(n):
        origins.append(Ts[i][:3, 3].copy())  # o_i
        axes_z.append(Ts[i][:3, 2].copy())   # z_i

    o_n = origins[-1]
    Jv = np.zeros((3, n), dtype=T_0.dtype)
    Jw = np.zeros((3, n), dtype=T_0.dtype)
    for i in range(n):
        z_im1 = axes_z[i]
        o_im1 = origins[i]
        Jv[:, i] = np.cross(z_im1, o_n - o_im1)
        Jw[:, i] = z_im1
    J = np.vstack([Jv, Jw])  # 6*n
    return J
