import torch
from torch.func import jvp
from typing import Callable

JacFn = Callable[[torch.Tensor], torch.Tensor]  # q -> J(q) (m×n)

from ...utils import pytorch_util as ptu


def transform_matrices(
    q: torch.Tensor, 
    d: torch.Tensor, 
    a: torch.Tensor, 
    alpha: torch.Tensor, 
    b: torch.Tensor | None = None
    ) -> torch.Tensor:
    """
    Vectorized Standard DH transforms (Craig's convention).
    pytorch version for automatic differentiation.


    For each joint i, builds the homogeneous transform A_i(theta_i, d_i, a_i, alpha_i).
    
    Optionally applies an extra translation b_i along the *new* z-axis of A_i.

    Parameters
    ----------
    q : Tensor, shape (n,)
        Joint positions.
    d : Tensor, shape (n,)
        Offsets along previous z.
    a : Tensor, shape (n,)
        Lengths along current x.
    alpha : Tensor, shape (n,)
        Twists about current x (rad).
    b : Tensor, shape (n,), optional
        Extra translation along the new z-axis after forming A_i.
        Defaults to zeros if None.

    Returns
    -------
    A : Tensor, shape (n, 4, 4)
        Stack of per-link homogeneous transforms.

    Notes
    -----
    Standard DH (not modified): 
        A_i = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)
    The `b` shift is applied by adding b_i * z_new to the translation column.
    """
    
    if b is None: b = torch.zeros_like(q)

    assert q.ndim == d.ndim == a.ndim == alpha.ndim == b.ndim == 1, "All inputs must be 1-D"
    assert q.shape == d.shape == a.shape == alpha.shape == b.shape, "All inputs must have the same length"
    n = q.shape[0]

    cT, sT = torch.cos(q),  torch.sin(q)
    cA, sA = torch.cos(alpha), torch.sin(alpha)

    A = torch.empty(n, 4, 4, dtype=q.dtype, device=q.device)

    A[:, 0, 0] = cT;   A[:, 0, 1] = -sT * cA; A[:, 0, 2] = sT * sA; A[:, 0, 3] = a * cT
    A[:, 1, 0] = sT;   A[:, 1, 1] = cT * cA; A[:, 1, 2] = -cT * sA; A[:, 1, 3] = a * sT
    A[:, 2, 0] = 0.0;  A[:, 2, 1] = sA;    A[:, 2, 2] = cA;    A[:, 2, 3] = d
    A[:, 3, 0] = 0.0;  A[:, 3, 1] = 0.0;    A[:, 3, 2] = 0.0;    A[:, 3, 3] = 1.0

    A[:, :3, 3] += b[..., None] * A[:, :3, 2]

    return A


def cumulative_transforms(
    q: torch.Tensor, 
    d: torch.Tensor, 
    a: torch.Tensor, 
    alpha: torch.Tensor, 
    b: torch.Tensor | None = None
    ) -> torch.Tensor:
    """
    Cumulative DH transforms with respect to the base frame. 
    pytorch version for automatic differentiation.


    For i = 0..n-1, computes T_0i = A_0 A_1 ... A_i where
    A_k = Rot_z(q_k) * Trans_z(d_k) * Trans_x(a_k) * Rot_x(alpha_k),
    then applies an extra shift b_k along the new z-axis of A_k.

    Parameters
    ----------
    q, d, a, alpha : Tensor, shape (n,)
        Standard DH parameters as 1D arrays of equal length.
    b : Tensor, shape (n,), optional
        Extra translation along each link's new z-axis. Defaults to zeros.

    Returns
    -------
    T_bl : Tensor, shape (n, 4, 4)
        Stack of base-to-link transforms.
    """
    n = q.shape[0]

    A = transform_matrices(q, d, a, alpha, b)
    T_bl = torch.empty_like(A)                      # (n,4,4)

    T_b = torch.eye(4, dtype=A.dtype, device=A.device)
    for i in range(n):
        T_b = T_b @ A[i]
        T_bl[i] = T_b

    return T_bl


def jacobian(
    T_wf: torch.Tensor,
    ) -> torch.Tensor:
    """
    Compute the geometric Jacobian J(q) in the world frame. 
    pytorch version for automatic differentiation.

    Assumed revolute about z_{i-1}.

    Parameters
    ----------
    T_wf : Tensor, shape (n+1, 4, 4)
        World-to-frame transforms for all frames including base (frame 0).
    
    Returns
    -------
    J : Tensor, shape (6, n)
        Geometric Jacobian expressed in the world frame.
        Rows 0..2 are the linear part Jv, rows 3..5 are the angular part Jw.
    """
    # shape checks
    assert T_wf.ndim == 3 and T_wf.shape[1:] == (4, 4), "T_wf must be (n, 4, 4)"

    o_wf = T_wf[:, :3, 3]   # (n+1, 3) origins in world
    z_wf = T_wf[:, :3, 2]   # (n+1, 3) z-axes in world

    o_n = o_wf[-1]           # end-effector origin

    Jv = torch.cross(z_wf[:-1], (o_n - o_wf[:-1]), dim=1).T   # (3, n)
    Jw = z_wf[:-1].T                                          # (3, n)

    J = torch.cat([Jv, Jw])  # (6, n)
    return J


def spatial_vel(
    q: torch.Tensor, 
    qd: torch.Tensor, 
    jac: JacFn
    ) -> torch.Tensor:
    J = jac(q)
    return J @ qd


def spatial_acc(
    q: torch.Tensor, 
    qd: torch.Tensor, 
    qdd: torch.Tensor, 
    jac: JacFn
    ) -> torch.Tensor:
    J = jac(q)
    f = lambda x: jac(x) @ qd
    Jdot_qd = jvp(f, (q,), (qd,))[1]
    return J @ qdd + Jdot_qd

