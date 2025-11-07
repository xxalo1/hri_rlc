from __future__ import annotations


import numpy as np
from typing import Any, Optional, Sequence
from numpy.typing import ArrayLike, NDArray

from . import kinematics as kin
from ..utils import util as ut
from . import traj as tr
FloatArray = ut.FloatArray
dtype = ut.dtype

class model:
    def __init__(self, d: FloatArray, a: FloatArray, alpha: FloatArray,
                 theta0: FloatArray | None = None,
                 b: FloatArray | None = None,
                 o_0: FloatArray | None = None,
                 axes_o: FloatArray | None = None,
                 inertia: dict[Any, Any] | None = None,
                 dtype = dtype) -> None:
        """
        d, a, alpha: 1D array (length n)
        theta0: optional 1D array of fixed offsets (length n), defaults to zeros
        b: optional 1D array of translations along the new z axis (length n)
        axes_o: optional 2D array (3x3), axes of base frame in world coords, defaults to identity
        inertia: optional dict of manipulator iertias coms masses 
        o_0: optional 1d array, origin of base frame in world coords, defaults to [0,0,0]
        """
        if o_0 is None:
            o_0 = np.array([0, 0, 0], dtype=dtype)
        if axes_o is None:
            axes_o = np.array([[1, 0, 0],
                               [0, 1, 0],
                               [0, 0, 1]], dtype=dtype)
        if b is None:
            b = np.zeros_like(d)
        if theta0 is None:
            theta0 = np.zeros_like(d)
        
        self.inertia = inertia
        self.d     = d
        self.a     = a
        self.alpha = alpha
        self.theta0 = theta0
        self.theta: FloatArray = self.theta0.copy()
        self.dtype = dtype

        self.b = b
        self.o_0 = o_0  # origins in base frame
        self.axes_o = axes_o  # z axes in base frame

        self.T_0 = np.eye(4, dtype=self.dtype)
        self.T_0[:3, :3] = axes_o
        self.T_0[:3, 3] = o_0

        if inertia:
            updated_com = self._COMs(inertia)
            for i, name in enumerate(inertia.keys()):
                inertia[name]['COM'] = updated_com[i]
        else:
            inertia = {}
        self.inertia = inertia

    def fk(self, n: Optional[int] = None) -> Sequence[FloatArray]:
        """
        Compute cumulative forward-kinematics transforms from base to each joint.

        Args:
        n: Number of joints to include. If None, includes all joints.

        Returns:
        list[np.ndarray]: List of 4x4 transforms
        [T_0_1, T_0_2, ..., T_0_n] (length n), each mapping the base
        frame (0) to joint frames 1..n.

        Notes:
        The identity T_0_0 is not included; prepend np.eye(4) if needed.
        """
        if n is None:
            n = len(self.d)
        T = [self.T_0]
        A = kin.transform_chain(self.theta, self.d, self.a, self.alpha, self.b)
        for j in range(len(self.d)):
            T_i = T[j] @ A[j]
            T.append(T_i)
        return T[1:]

    def update(self, theta: Optional[FloatArray] = None, d: Optional[FloatArray] = None,
                a: Optional[FloatArray] = None, alpha: Optional[FloatArray] = None, o_0: Optional[FloatArray] = None,
                axes_o: Optional[FloatArray] = None):
        """Update attributes theta, d, a, alpha, o_0, axes_o."""
        if theta:
            self.theta = theta
        if d:
            self.d = d
        if a:
            self.a = a
        if alpha:
            self.alpha = alpha
        if o_0:
            self.o_0 = o_0
            self.T_0[:3, 3] = o_0
        if axes_o:
            self.axes_o = axes_o
            self.T_0[:3, :3] = axes_o
    
    def jacobian(self) -> FloatArray:
        """
        6*n Jacobian for revolute joints:
          Jv_i = z_{i-1} * (o_n - o_{i-1})
          Jw_i = z_{i-1}
        Returns J
        """
        n = len(self.d)
        origins = [self.T_0[:3, 3]]   # o_0
        axes_z  = [self.T_0[:3, 2]]   # z axis from rotation matrix
        Ts = self.fk()
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

    def _COMs(self, inertia):
        """
        Compute centers of mass for each link in world frame.
        Returns list of COM positions (3,) for each link.
        """
        n = len(inertia)
        Ts = self.fk()  # Convert Sequence to list so we can modify it
        T_0 = self.T_0
        COMs = []
        for i in range(n):
            local_COM = inertia[i]['COM']  # (3,)
            T_i = T_0 if i==0 else Ts[i-1]
            COM_i_homog = T_i @ np.hstack((local_COM, 1.0))  # (4,)
            COMs.append(COM_i_homog[:3])  # (3,)
        return COMs
    
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
                      v0: FloatArray | None = None,
                      a0: FloatArray | None = None,
                      vf: FloatArray | None = None,
                      af: FloatArray | None = None
                      ) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray]:
        """
        Compute quintic polynomial coefficients and evaluate trajectories.  
        """
        n = len(q0)

        if not v0:
            v0 = ut.to_n_array(0.0, n)
        if not a0:
            a0 = ut.to_n_array(0.0, n)
        if not vf:
            vf = ut.to_n_array(0.0, n)
        if not af:
            af = ut.to_n_array(0.0, n)

        Q, Qd, Qdd, T = tr.quintic_trajs(q0, qf, t0, tf, dt, v0, a0, vf, af)
        return Q, Qd, Qdd, T