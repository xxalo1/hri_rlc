import numpy as np
from typing import Any, Optional, Sequence
from numpy.typing import ArrayLike, NDArray

from ...utils import numpy_util as npu

FloatArray = npu.FloatArray
dtype = npu.dtype

def quintic_trajs(q0: FloatArray, qf: FloatArray, t0: float, tf: float, freq: float,
                      v0: FloatArray, a0: FloatArray, vf: FloatArray, af: FloatArray
                  ) -> tuple[FloatArray, FloatArray, FloatArray]:
        """
        Compute quintic polynomial coefficients and evaluate trajectories.  
        """
        A = quintic_coeffs(q0, qf, t0, tf, v0, a0, vf, af)
        
        dt = 1.0 / freq
        n_steps = int(np.round((tf - t0) * freq)) + 1
        T = t0 + np.arange(n_steps, dtype=dtype) * dt
        Q, Qd, Qdd = eval_quintic(A, T)
        return Q, Qd, Qdd

def quintic_coeffs(q0: FloatArray, qf: FloatArray, t0: float, tf: float,
                    v0: FloatArray, a0: FloatArray, vf: FloatArray, af: FloatArray
                  ) -> FloatArray:
    """
    Solve for coefficients a s.t.
    q(t0)=q0, q'(t0)=v0, q''(t0)=a0, q(tf)=q1, q'(tf)=v1, q''(tf)=a1
    q0, q1: (n,)   -> start/goal for n joints
    v0,a0,v1,a1: 0 or (n,)
    Returns a with shape (6, n), columns are per-joint coeffs [a0..a5].
    """

    M = np.array([
        [1, t0, t0**2,   t0**3,     t0**4,      t0**5],
        [0,  1,  2*t0,   3*t0**2,   4*t0**3,    5*t0**4],
        [0,  0,     2,   6*t0,     12*t0**2,   20*t0**3],
        [1, tf, tf**2,   tf**3,     tf**4,      tf**5],
        [0,  1,  2*tf,   3*tf**2,   4*tf**3,    5*tf**4],
        [0,  0,     2,   6*tf,     12*tf**2,   20*tf**3],
    ], dtype=dtype)

    B = np.vstack([q0, v0, a0, qf, vf, af,]) # (6, n)

    A = np.linalg.solve(M, B)        # (6, n)
    return A

def eval_quintic(a: FloatArray, t: FloatArray) -> tuple[FloatArray, FloatArray, FloatArray]:
    """
    Evaluate position/velocity/acceleration for coefficients a at times t.
    a: (6, n)  (output of quintic_coeffs)
    t: scalar or (K,)
    Returns:
    q:   (K, n)
    qd:  (K, n)
    qdd: (K, n)
    If t is scalar, K=1.
    """
    tcol  = t[:, None]

    # Horner form; broadcasting handles all DoFs without branching
    q   = (((((a[5]*tcol + a[4])*tcol + a[3])*tcol + a[2])*tcol + a[1])*tcol + a[0])
    qd  = ((((5*a[5]*tcol + 4*a[4])*tcol + 3*a[3])*tcol + 2*a[2])*tcol + a[1])
    qdd = (((20*a[5]*tcol + 12*a[4])*tcol + 6*a[3])*tcol + 2*a[2])

    return q, qd, qdd