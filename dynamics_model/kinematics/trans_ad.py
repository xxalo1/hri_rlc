import numpy as np
from typing import Any, Optional, Sequence
from numpy.typing import ArrayLike, NDArray

import torch  # add torch for AD
from ...utils import util as ut

FloatArray = ut.FloatArray
dtype = ut.dtype

def transform_matrix(theta: float, d: float,
                    a: float, alpha: float, b: float = 0.0) -> FloatArray:
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

def transform_chain(theta: FloatArray, d: FloatArray, a: FloatArray, alpha: FloatArray, b: FloatArray) -> Sequence[FloatArray]:
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

def cumulative_transforms(theta: FloatArray, d: FloatArray, a: FloatArray, alpha: FloatArray, b: FloatArray) -> Sequence[FloatArray]:
    """
    Transform of joints with respect to base frame using stored DH params.
    theta: 1D array (length n)
    d: 1D array (length n)
    a: 1D array (length n)
    alpha: 1D array (length n)
    b: optional 1D array of translations along the new z axis (length n)

    returns: [T_0_0, …, T_0_{n-1}]
    """
    T = []
    A = transform_chain(theta, d, a, alpha, b)
    T_i = np.eye(4, dtype=dtype)
    for j in range(len(d)):
        T_i = T_i @ A[j]
        T.append(T_i)
    return T