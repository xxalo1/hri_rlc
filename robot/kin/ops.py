import numpy as np
from typing import Any, Optional, Sequence
from numpy.typing import ArrayLike, NDArray

from ...utils import numpy_util as npu

FloatArray = npu.FloatArray
dtype = npu.dtype

def transform_matrices( 
    q: FloatArray,
    d: FloatArray,
    a: FloatArray,
    alpha: FloatArray,
    b: FloatArray | None = None,
    ) -> FloatArray:
    """
    Vectorized Standard DH transforms (Craig's convention).

    For each joint i, builds the homogeneous transform A_i(theta_i, d_i, a_i, alpha_i).
    
    Optionally applies an extra translation b_i along the *new* z-axis of A_i.

    Parameters
    ----------
    q : ndarray, shape (n,)
        Joint positions.
    d : ndarray, shape (n,)
        Offsets along previous z.
    a : ndarray, shape (n,)
        Lengths along current x.
    alpha : ndarray, shape (n,)
        Twists about current x (rad).
    b : ndarray, shape (n,), optional
        Extra translation along the new z-axis after forming A_i.
        Defaults to zeros if None.

    Returns
    -------
    A : ndarray, shape (n, 4, 4)
        Stack of per-link homogeneous transforms.

    Notes
    -----
    Standard DH (not modified): 
        A_i = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)
    The `b` shift is applied by adding b_i * z_new to the translation column.
    """

    if b is None: b = np.zeros_like(q)

    assert q.ndim == d.ndim == a.ndim == alpha.ndim == b.ndim == 1, "All inputs must be 1-D"
    assert q.shape == d.shape == a.shape == alpha.shape == b.shape, "All inputs must have the same length"
    n = q.size

    cT, sT = np.cos(q), np.sin(q)
    cA, sA = np.cos(alpha), np.sin(alpha)

    A = np.empty((n, 4, 4), dtype=q.dtype)

    A[:, 0, 0] = cT
    A[:, 0, 1] = -sT * cA
    A[:, 0, 2] =  sT * sA
    A[:, 0, 3] =  a * cT

    A[:, 1, 0] = sT
    A[:, 1, 1] =  cT * cA
    A[:, 1, 2] = -cT * sA
    A[:, 1, 3] =  a * sT

    A[:, 2, 0] = 0.0
    A[:, 2, 1] = sA
    A[:, 2, 2] = cA
    A[:, 2, 3] = d

    A[:, 3, 0] = 0.0
    A[:, 3, 1] = 0.0
    A[:, 3, 2] = 0.0
    A[:, 3, 3] = 1.0

    A[:, :3, 3] += b[:, None] * A[:, :3, 2]

    return A


def cumulative_transforms(
    q: FloatArray,
    d: FloatArray,
    a: FloatArray,
    alpha: FloatArray,
    b: FloatArray | None = None,
    ) -> FloatArray:
    """
    Cumulative DH transforms with respect to the base frame.

    For i = 0..n-1, computes T_0i = A_0 A_1 ... A_i where
    A_k = Rot_z(q_k) * Trans_z(d_k) * Trans_x(a_k) * Rot_x(alpha_k),
    then applies an extra shift b_k along the new z-axis of A_k.

    Parameters
    ----------
    q, d, a, alpha : ndarray, shape (n,)
        Standard DH parameters as 1D arrays of equal length.
    b : ndarray, shape (n,), optional
        Extra translation along each link's new z-axis. Defaults to zeros.

    Returns
    -------
    T_bl : ndarray, shape (n, 4, 4)
        Stack of base-to-link transforms.
    """
    n = q.size

    A = transform_matrices(q, d, a, alpha, b)  # (n,4,4)
    T_bl = np.empty_like(A)                      # (n,4,4)

    T_b = np.eye(4, dtype=A.dtype)
    for i in range(n):
        T_b = T_b @ A[i]
        T_bl[i] = T_b

    return T_bl


def jacobian(
    T_wf: FloatArray,
    ) -> FloatArray:
    """
    Compute the geometric Jacobian J(q) in the world frame.

    Assumed revolute about z_{i-1}.

    Parameters
    ----------
    T_wf : ndarray, shape (n+1, 4, 4)
        World-to-frame transforms for all frames including base (frame 0).
    
    Returns
    -------
    J : ndarray, shape (6, n)
        Geometric Jacobian expressed in the world frame.
        Rows 0..2 are the linear part Jv, rows 3..5 are the angular part Jw.
    """
    # shape checks
    assert T_wf.ndim == 3 and T_wf.shape[1:] == (4, 4), "T_wf must be (n, 4, 4)"
    n = T_wf.shape[0] - 1

    o_wf = T_wf[:, :3, 3]   # (n+1, 3) origins in world
    z_wf = T_wf[:, :3, 2]   # (n+1, 3) z-axes in world

    o_n = o_wf[-1]           # end-effector origin

    # revolute joints: Jv_i = z_{i-1} x (o_n - o_{i-1}), Jw_i = z_{i-1}
    Jv = np.cross(z_wf[:-1], (o_n - o_wf[:-1]), axis=1).T   # (3, n)
    Jw = z_wf[:-1].T                                          # (3, n)

    J = np.vstack([Jv, Jw])  # (6, n)
    return J

def com_world(
    T_wf: FloatArray,
    com_f: FloatArray,
    ) -> FloatArray:
    """
    Compute centers of mass in the world frame.

    Parameters
    ----------
    T_wf : ndarray, shape (n+1, 4, 4)
        World-to-frame transforms for base (0) and all links.
    com_local : ndarray, shape (n+1, 3)
        COM positions expressed in each frame's local coordinates.

    Returns
    -------
    com_w : ndarray, shape (n+1, 3)
        COM positions for base and links, expressed in the world frame.
    """
    R = T_wf[:, :3, :3]            # (n+1, 3, 3)
    p = T_wf[:, :3, 3]             # (n+1, 3)
    com_f = com_f[..., None]      # (n+1, 3, 1)
    com_w = R @ com_f + p   # (n+1, 3)
    return com_w
