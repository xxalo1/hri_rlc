from ...utils import(numpy_util as npu, array_compat as xp, ArrayT, FloatArray, dtype)


def transform_matrices(
    q: ArrayT, 
    d: ArrayT, 
    a: ArrayT, 
    alpha: ArrayT, 
    b: ArrayT | None = None
    ):
    """
    Vectorized Standard DH transforms (Craig's convention).

    For each joint i, builds the homogeneous transform A_i(theta_i, d_i, a_i, alpha_i).
    
    Optionally applies an extra translation b_i along the *new* z-axis of A_i.

    Parameters
    ----------
    q : ndarray or Tensor, shape (n,)
        Joint positions.
    d : ndarray or Tensor, shape (n,)
        Offsets along previous z.
    a : ndarray or Tensor, shape (n,)
        Lengths along current x.
    alpha : ndarray or Tensor, shape (n,)
        Twists about current x (rad).
    b : ndarray or Tensor, shape (n,), optional
        Extra translation along the new z-axis after forming A_i.
        Defaults to zeros if None.

    Returns
    -------
    A : ndarray or Tensor, shape (n, 4, 4)
        Stack of per-link homogeneous transforms.

    Notes
    -----
    Standard DH (not modified): 
        A_i = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)
    The `b` shift is applied by adding b_i * z_new to the translation column.
    """

    if b is None: b = xp.zeros_like(q)

    assert q.ndim == d.ndim == a.ndim == alpha.ndim == b.ndim == 1, "All inputs must be 1-D"
    assert q.shape == d.shape == a.shape == alpha.shape == b.shape, "All inputs must have the same length"
    n: int = q.shape[0]


    cT, sT = xp.cos(q), xp.sin(q)
    cA, sA = xp.cos(alpha), xp.sin(alpha)

    A = xp.zeros_like(q, (n, 4, 4))

    A[:, 0, 0] = cT;   A[:, 0, 1] = -sT * cA; A[:, 0, 2] = sT * sA; A[:, 0, 3] = a * cT
    A[:, 1, 0] = sT;   A[:, 1, 1] = cT * cA; A[:, 1, 2] = -cT * sA; A[:, 1, 3] = a * sT
    A[:, 2, 0] = 0.0;  A[:, 2, 1] = sA;    A[:, 2, 2] = cA;    A[:, 2, 3] = d
    A[:, 3, 0] = 0.0;  A[:, 3, 1] = 0.0;    A[:, 3, 2] = 0.0;    A[:, 3, 3] = 1.0

    A[:, :3, 3] += b[:, None] * A[:, :3, 2]

    return A


def cumulative_transforms(
    q: ArrayT, 
    d: ArrayT, 
    a: ArrayT, 
    alpha: ArrayT, 
    b: ArrayT | None = None
    ):
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
    n: int = q.shape[0]

    A = transform_matrices(q, d, a, alpha, b)  # (n,4,4)
    T_bl = xp.empty_like(A)                      # (n,4,4)

    T_b = xp.eye_like(q, 4)

    for i in range(n):
        T_b = T_b @ A[i]
        T_bl[i] = T_b

    return T_bl


def jacobian(T_wf: ArrayT) -> ArrayT:
    """
    Compute the geometric Jacobian J(q) in the world frame. 
    pytorch version for automatic differentiation.

    Assumed revolute about z_{i-1}.

    Parameters
    ----------
    T_wf : ndarray | Tensor, shape (n+1, 4, 4)
        World-to-frame transforms for all frames including base (frame 0).
    
    Returns
    -------
    J : ndarray | Tensor, shape (6, n)
        Geometric Jacobian expressed in the world frame.
        Rows 0..2 are the linear part Jv, rows 3..5 are the angular part Jw.
    """
    # shape checks

    assert T_wf.ndim == 3 and T_wf.shape[1:] == (4, 4), "T_wf must be (n, 4, 4)"

    o_wf = T_wf[:, :3, 3]   # (n+1, 3) origins in world
    z_wf = T_wf[:, :3, 2]   # (n+1, 3) z-axes in world

    o_n = o_wf[-1]           # end-effector origin

    Jv = xp.cross(z_wf[:-1], (o_n - o_wf[:-1]), dim=1).T   # (3, n)
    Jw = z_wf[:-1].T                                          # (3, n)

    J = xp.concatenate(Jv, Jw) # (6, n)
    return J


def com_world(
    T_wf: ArrayT,
    com_fl: ArrayT,
    ) -> ArrayT:
    """
    Compute centers of mass in the world frame.

    Parameters
    ----------
    T_wf : ndarray | Tensor, shape (n+1, 4, 4)
        World-to-frame transforms for base (0) and all links.
    com_fl : ndarray | Tensor, shape (n+1, 3)
        COM positions expressed in each frame's local coordinates.

    Returns
    -------
    com_w : ndarray | Tensor, shape (n+1, 3)
        COM positions for base and links, expressed in the world frame.
    """
    R = T_wf[:, :3, :3]            # (n+1, 3, 3)
    p = T_wf[:, :3, 3]             # (n+1, 3)
    com_w = R @ com_fl + p          # (n+1, 3)
    return com_w


def skew(v: ArrayT) -> ArrayT:
    """
    Batched skew operator.

    Parameters
    ----------
    r_w : (n, 3)
        Vectors in world frame.

    Returns
    -------
    S : (n, 3, 3)
        where for each row vector [rx, ry, rz],
        S = [[  0, -rz,  ry],
            [ rz,   0, -rx],
            [-ry,  rx,   0]]

    """
    x, y, z = v[..., 0], v[..., 1], v[..., 2]
    S = xp.zeros_like(v, shape=(*v.shape[:-1], 3, 3))
    S[..., 0, 1] = -z
    S[..., 0, 2] =  y
    S[..., 1, 0] =  z
    S[..., 1, 2] = -x
    S[..., 2, 0] = -y
    S[..., 2, 1] =  x
    return S

