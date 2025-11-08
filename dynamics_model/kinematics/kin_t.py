import torch
from torch.func import jvp
from typing import Callable

JacFn = Callable[[torch.Tensor], torch.Tensor]  # q -> J(q) (m×n)

from ...utils import pytorch_util as ptu

dtype = ptu.dtype


def transform_matrices(theta: torch.Tensor, 
                       d: torch.Tensor, 
                       a: torch.Tensor, 
                       alpha: torch.Tensor, 
                       b: torch.Tensor | None = None
                       ) -> torch.Tensor:

    n = theta.shape[0]
    cT, sT = torch.cos(theta),  torch.sin(theta)
    cA, sA = torch.cos(alpha), torch.sin(alpha)

    A = torch.empty(n, 4, 4, dtype=dtype, device=theta.device)
    A[:, 0, 0] = cT;   A[:, 0, 1] = -sT * cA; A[:, 0, 2] = sT * sA; A[:, 0, 3] = a * cT
    A[:, 1, 0] = sT;   A[:, 1, 1] = cT * cA; A[:, 1, 2] = -cT * sA; A[:, 1, 3] = a * sT
    A[:, 2, 0] = 0.0;  A[:, 2, 1] = sA;    A[:, 2, 2] = cA;    A[:, 2, 3] = d
    A[:, 3, 0] = 0.0;  A[:, 3, 1] = 0.0;    A[:, 3, 2] = 0.0;    A[:, 3, 3] = 1.0
    if b is not None:
        A[:, 3, 3] = A[:, 3, 3] + b.unsqueeze(-1) * A[:, 3, 2]
    return A


def cumulative_transforms(theta: torch.Tensor, 
                          d: torch.Tensor, 
                          a: torch.Tensor, 
                          alpha: torch.Tensor, 
                          b: torch.Tensor | None = None
                          ) -> torch.Tensor:
    """
    Transform of joints with respect to base frame using stored DH params.
    theta: 1D tensor (length n)
    d: 1D tensor (length n)
    a: 1D tensor (length n)
    alpha: 1D tensor (length n)
    b: optional 1D tensor of translations along the new z axis (length n)

    returns: 3d tensor [T_0_0, …, T_0_{n-1}]
    """
    if b is None:
        b = torch.zeros_like(d)

    T = []
    A = transform_matrices(theta, d, a, alpha, b)
    T_i = torch.eye(4, dtype=d.dtype, device=theta.device)
    for j in range(len(d)):
        T_i = T_i @ A[j]
        T.append(T_i)
    return torch.stack(T)


def jacobian(T0: torch.Tensor, Ts: torch.Tensor) -> torch.Tensor:

    Tall = torch.cat([T0.unsqueeze(0), Ts], dim=0)   # (n+1,4,4)
    O = Tall[:, :3, 3]                               # (n+1,3)
    Z = Tall[:, :3, 2]                               # (n+1,3)

    on = O[-1]                                       # (3,)
    z  = Z[:-1]                                      # (n,3)
    o  = O[:-1]                                      # (n,3)

    Jv = torch.cross(z, on - o, dim=1).T             # (3,n)
    Jw = z.T                                         # (3,n)
    return torch.cat([Jv, Jw], dim=0)                # (6,n)


def spatial_vel(q: torch.Tensor, qd: torch.Tensor, jac: JacFn) -> torch.Tensor:
    J = jac(q)
    return J @ qd


def spatial_acc(q: torch.Tensor, qd: torch.Tensor, qdd: torch.Tensor, jac: JacFn) -> torch.Tensor:
    J = jac(q)
    f = lambda x: jac(x) @ qd
    Jdot_qd = jvp(f, (q,), (qd,))[1]
    return J @ qdd + Jdot_qd

