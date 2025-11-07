from __future__ import annotations

import numpy as np
import scipy as sp
from typing import Optional, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

FloatArray = NDArray[np.float32]

class DH:
    """Minimal standard DH (Craig) helper."""
    def __init__(self, d: FloatArray, a: FloatArray, alpha: FloatArray, theta0: FloatArray = None, 
                 b: FloatArray = None, o_0: FloatArray = None, z_0: FloatArray = None):
        """
        d, a, alpha: 1D arrays (length n)
        theta0: optional 1D array of fixed offsets (length n), defaults to zeros
        b: optional 1D array of translations along the new z axis (length n)

        """
        self.d     = d
        self.a     = a
        self.alpha = alpha
        self.theta0 = theta0
        self.theta = self.theta0.copy()
        self.dtype = np.float32
        self.o_0 = o_0  # origins in base frame
        self.z_0 = z_0  # z axes in base frame

        if b is None:
            self.b = np.zeros_like(d)
        else:
            self.b = b

    def dh_transform(self, theta: float, d: float,
                      a: float, alpha: float, b: float = None) -> FloatArray:
        """Standard DH T_i (Craig)
        b: optional translation along the new z axis"""
        cT, sT = np.cos(theta), np.sin(theta)
        cA, sA = np.cos(alpha), np.sin(alpha)
        A = np.array([[cT, -sT * cA,  sT * sA, a * cT],
                      [sT,  cT * cA, -cT * sA, a * sT],
                      [0 ,      sA ,      cA ,     d  ],
                      [0 ,      0  ,      0  ,     1  ]], dtype=self.dtype)
        A[:3, 3] += b * A[:3, 2]  # translate along the new z axis
        return A

    def T_i(self, i: int) -> FloatArray:
        """
        Transform of joints i with respect to i-1 using stored DH params.
        """
        T = []
        for j in range(i+1):
            T_i = self.dh_transform(self.theta[j], self.d[j], self.a[j], self.alpha[j], self.b[j])
            T.append(T_i)
        return T[i]

    def T(self) -> Sequence[FloatArray]:
        """
        Transform of joints with respect to each other using stored DH params.
        """
        T = []
        for j in range(len(self.d)):
            T_i = self.dh_transform(self.theta[j], self.d[j], self.a[j], self.alpha[j], self.b[j])
            T.append(T_i)
        return T

    def fk(self, i: int) -> FloatArray:
        """
        Transform of joint i with respect to ground using stored DH params.
        """
        T = [np.eye(4)]
        for j in range(i+1):
            T_i = self.dh_transform(self.theta[j], self.d[j], self.a[j], self.alpha[j], self.b[j])
            T_i = T[j] @ T_i
            T.append(T_i)
        # strip off T0
        T = T[1:]
        return T

    def update(self, theta: Optional[FloatArray] = None, d: Optional[FloatArray] = None,
                a: Optional[FloatArray] = None, alpha: Optional[FloatArray] = None):
        """Update attributes theta, d, a, alpha."""
        if theta is not None:
            self.theta = theta
        if d is not None:
            self.d = d
        if a is not None:
            self.a = a
        if alpha is not None:
            self.alpha = alpha
    
    def jacobian(self):
        """
        6*n Jacobian for revolute joints:
          Jv_i = z_{i-1} * (o_n - o_{i-1})
          Jw_i = z_{i-1}
        Returns J
        """
        n = len(self.d)
        origins = self.o_0   # o_0
        axes_z  = self.z_0   # z_0
        Ts = self.T0_i()
        for i in range(n):
            origins.append(Ts[i][:3, 3].copy())  # o_i
            axes_z.append(Ts[i][:3, 2].copy())   # z_i

        o_n = origins[-1]
        Jv = np.zeros((3, n), dtype=self.dtype)
        Jw = np.zeros((3, n), dtype=self.dtype)
        for i in range(n):
            z_im1 = axes_z[i]
            o_im1 = origins[i]
            Jv[:, i] = np.cross(z_im1, o_n - o_im1)
            Jw[:, i] = z_im1
        J = np.vstack([Jv, Jw])  # 6*n
        return J

    def ik_7dof(self, target_T: FloatArray, max_iters: int = 1000, 
                tol: float = 1e-4, alpha: float = 0.1) -> FloatArray:
        """
        Inverse kinematics for 7-DOF arm using iterative Jacobian pseudo-inverse method.
        target_T: desired end-effector transform (4x4)
        q_init: initial joint angles (7,)
        Returns q_sol: joint angles achieving target_T within tolerance.
        """
        q = self.theta.copy()  # initial guess
        for iter in range(max_iters):
            self.update(theta=q)
            current_T = self.fk(len(q)-1)[-1]
            pos_err = target_T[:3, 3] - current_T[:3, 3]
            rot_err_matrix = current_T[:3, :3].T @ target_T[:3, :3]
            rot_err_axis, rot_err_angle = self._rotation_matrix_to_axis_angle(rot_err_matrix)
            err = np.hstack((pos_err, rot_err_axis * rot_err_angle))

            if np.linalg.norm(err) < tol:
                break

            J = self.jacobian()
            J_pinv = np.linalg.pinv(J)
            delta_q = alpha * (J_pinv @ err)
            q += delta_q

        return q

    def quintic_trajs(self, q0: FloatArray, qf: FloatArray, t0: float, tf: float, dt: float,
                      v0: FloatArray = 0.0, a0: FloatArray = 0.0, vf: FloatArray = 0.0,
                      af: FloatArray = 0.0) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray]:
        """
        Compute quintic polynomial coefficients and evaluate trajectories.  
        """
        A = self._quintic_coeffs(q0, qf, t0, tf, v0, a0, vf, af)
        T = np.linspace(t0, tf, int((tf - t0) / dt) + 1, dtype=self.dtype)
        Q, Qd, Qdd = self._eval_quintic(A, T)
        return Q, Qd, Qdd, T

    def _quintic_coeffs(self, q0: FloatArray, qf: FloatArray, t0: float, tf: float,
                       v0: FloatArray = 0.0, a0: FloatArray = 0.0, vf: FloatArray = 0.0,
                       af: FloatArray = 0.0) -> FloatArray:
        """
        Solve for coefficients a s.t.
        q(t0)=q0, q'(t0)=v0, q''(t0)=a0, q(tf)=q1, q'(tf)=v1, q''(tf)=a1
        q0, q1: (n,)   -> start/goal for n joints
        v0,a0,v1,a1: scalar or (n,)
        Returns a with shape (6, n), columns are per-joint coeffs [a0..a5].
        """
        n = len(q0)
        dtype = self.dtype
        v0 = np.full(n, v0, dtype=dtype)
        a0 = np.full(n, a0, dtype=dtype)
        vf = np.full(n, vf, dtype=dtype)
        af = np.full(n, af, dtype=dtype)

        M = np.array([
            [1, t0, t0**2,   t0**3,     t0**4,      t0**5],
            [0,  1,  2*t0,   3*t0**2,   4*t0**3,    5*t0**4],
            [0,  0,     2,   6*t0,     12*t0**2,   20*t0**3],
            [1, tf, tf**2,   tf**3,     tf**4,      tf**5],
            [0,  1,  2*tf,   3*tf**2,   4*tf**3,    5*tf**4],
            [0,  0,     2,   6*tf,     12*tf**2,   20*tf**3],
        ], dtype=self.dtype)

        B = np.vstack([q0, v0, a0, qf, vf, af,]) # (6, n)

        A = np.linalg.solve(M, B)        # (6, n)
        return A

    def _eval_quintic(self, a: FloatArray, t: FloatArray) -> tuple[FloatArray, FloatArray, FloatArray]:
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