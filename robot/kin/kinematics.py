from __future__ import annotations

import numpy as np
import torch
from typing import Any, Optional, Sequence, Callable
from numpy.typing import ArrayLike, NDArray


from . import ops, ops_t
from ...utils import numpy_util as npu
from ...utils import pytorch_util as ptu

FloatArray = npu.FloatArray
dtype = npu.dtype

class Kinematics:
    """
    Kinematics class for representing and computing robot kinematics.
    """
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

        self.q: FloatArray = self.theta0.copy()
        self.qd: FloatArray = np.zeros_like(self.q)
        self.qdd: FloatArray = np.zeros_like(self.q)

        self._jac_fn: Callable[[torch.Tensor], torch.Tensor] | None = None
        self.update_tensors()


    def fk(self, q: FloatArray | None = None, n: int | None = None) -> Sequence[FloatArray]:
        """
        Compute cumulative forward kinematics in the world frame.

        Parameters
        ----------
        q : ndarray, shape (n,), optional
            Joint positions. Defaults to the internal state `self.q` if None.
        n : int, optional
            Number of joints to include. Defaults to all joints.

        Returns
        -------
        Ts : list of ndarray, each (4, 4), length n
            World-frame transforms [T_W_1, ..., T_W_n].
        """
        if n is None:
            n = len(self.d)
        if q is None:
            q = self.q

        # Base-frame cumulative transforms (A0, A0@A1, …)
        Ts_base = ops.cumulative_transforms(q, self.d, self.a, self.alpha, self.b)

        # World-frame transforms: pre-multiply by base/world pose
        T_WB = self.T_0
        Ts_world = [T_WB @ Ti for Ti in Ts_base[:n]]
        return Ts_world


    def fk_t(self, q: torch.Tensor | None = None, n: int | None = None) -> torch.Tensor:
        """
        Compute cumulative forward kinematics in the world frame (PyTorch).

        Parameters
        ----------
        q : Tensor, shape (n,), optional
            Joint positions. Defaults to the internal state `self.q_t` if None.
        n : int, optional
            Number of joints to include. Defaults to all joints.

        Returns
        -------
        Ts : Tensor, shape (n, 4, 4)
            World-frame transforms stacked as [T_W_1, ..., T_W_n].
        """
        if q is None:
            q = self.q_t
        # Base-frame cumulative transforms from DH as torch (N,4,4)
        Ts_base = ops_t.cumulative_transforms(q, self.d_t, self.a_t, self.alpha_t, self.b_t)
        if n is None:
            n = Ts_base.shape[0]
        Ts_base = Ts_base[:n]

        Ts_world = self.T_0_t @ Ts_base  # (n,4,4)

        return Ts_world


    def update_config(self,
                d: FloatArray | None = None,
                a: FloatArray | None = None, 
                alpha: FloatArray | None = None, 
                o_0: FloatArray | None = None,
                axes_o: FloatArray | None = None
                ) -> None:
        """
        Update attributes d, a, alpha, o_0, axes_o.

        Parameters
        ----------
        d : FloatArray | None
            Link offsets.
        a : FloatArray | None
            Link lengths.
        alpha : FloatArray | None
            Link twists.
        o_0 : FloatArray | None
            Base frame origin.
        axes_o : FloatArray | None
            Base frame axes.
        """
        if d is not None:
            self.d = d
        if a is not None:
            self.a = a
        if alpha is not None:
            self.alpha = alpha
        if o_0 is not None:
            self.o_0 = o_0
            self.T_0[:3, 3] = o_0
        if axes_o is not None:
            self.axes_o = axes_o
            self.T_0[:3, :3] = axes_o
            
        if any(x is not None for x in (d, a, alpha, o_0, axes_o)):
            self._jac_fn = None
            self.update_tensors()


    def step(self, 
             q: FloatArray | None = None,
              qd: FloatArray | None = None, 
              qdd: FloatArray | None = None
              ) -> None:
        """
        Update current joint states `self.q`, `self.qd`, `self.qdd`.
        and corresponding torch tensors. `self.q_t`, `self.qd_t`, `self.qdd_t`.
        updated if not None.

        Parameters
        ----------
        q : ndarray, shape (n,), optional
            Joint positions.
        qd : ndarray, shape (n,), optional
            Joint velocities.
        qdd : ndarray, shape (n,), optional
            Joint accelerations.
        """


        if q is not None: 
            self.q = q
            self.q_t = ptu.from_numpy(q, dtype=ptu.dtype)
        if qd is not None:
            self.qd = qd
            self.qd_t = ptu.from_numpy(qd, dtype=ptu.dtype)
        if qdd is not None:
            self.qdd = qdd
            self.qdd_t = ptu.from_numpy(qdd, dtype=ptu.dtype)


    def update_tensors(self):
        """Update all attributes to PyTorch tensors for autograd."""
        self.d_t = ptu.from_numpy(self.d, dtype=ptu.dtype)
        self.a_t = ptu.from_numpy(self.a, dtype=ptu.dtype)
        self.alpha_t = ptu.from_numpy(self.alpha, dtype=ptu.dtype)
        self.q_t = ptu.from_numpy(self.q, dtype=ptu.dtype)
        self.b_t = ptu.from_numpy(self.b, dtype=ptu.dtype) if self.b is not None else None
        self.T_0_t = ptu.from_numpy(self.T_0, dtype=ptu.dtype)


    def jac(self, q: FloatArray | None = None) -> FloatArray:
        """
        Compute the geometric Jacobian matrix using NumPy operations.

        This method computes the manipulator Jacobian at the given joint
        configuration. For revolute joints, the Jacobian columns are defined as:

            Jv_i = z_{i-1} * (o_n - o_{i-1})
            Jw_i = z_{i-1}

        where `Jv_i` is the linear velocity component and `Jw_i` is the angular
        velocity component of the i-th joint.

        Parameters
        ----------
        q : ndarray, shape (n,), optional
            Joint positions.
            If None, defaults to `self.q`.

        Returns
        -------
        J : ndarray, shape (6, n)
            The geometric Jacobian matrix evaluated at the specified configuration.
            The top three rows correspond to linear velocity terms, and the bottom
            three correspond to angular velocity terms.
        """
        if q is None: q = self.q
        Ts = self.fk(q)
        J = ops.jacobian(self.T_0, Ts)
        return J


    def jac_t(self, q: torch.Tensor | None = None) -> torch.Tensor:
        """
        Compute the Jacobian matrix using a differentiable PyTorch implementation.

        This method evaluates the autograd-compatible Jacobian function directly
        using a provided `Tensor` joint configuration. If no configuration
        is supplied, the internally stored tensor `self.q_t` is used.

        Parameters
        ----------
        q : Tensor, shape (n,), optional
            Joint configuration vector. If None, defaults to the internal joint
            positions `self.q_t`.

        Returns
        -------
        J : Tensor, shape (6, n)
            The geometric Jacobian matrix computed at the given configuration,
            suitable for PyTorch autograd differentiation.

        Notes
        -----
        This function calls `self.jac_fn` if available; otherwise it initializes
        a callable via `self._init_jac_callable()`.
        """
        if q is None: q = self.q_t
        jac = self.jac_fn or self._init_jac_callable()
        return jac(q)


    def _init_jac_callable(self) -> Callable[[torch.Tensor], torch.Tensor]:
        """
        Initialize jacobian function for autograd.        
        """
        def jac(q: torch.Tensor) -> torch.Tensor:
            Ts = self.fk_t(q)
            return ops_t.jacobian(self.T_0_t, Ts)   # (6,n)

        self.jac_fn = jac
        return jac


    def spatial_vel(self, 
                    q: torch.Tensor | None = None, 
                    qd: torch.Tensor | None = None
                    ) -> torch.Tensor:
        """
        Compute the spatial velocity of all joints in the manipulator.

        Each joint's 6D spatial velocity is given as:
        [vx, vy, vz, wx, wy, wz].

        Parameters
        ----------
        q : Tensor, shape (n,), optional
            Joint positions. Defaults to `self.q_t` if None.
        qd : Tensor, shape (n,), optional
            Joint velocities. Defaults to `self.qd_t` if None.

        Returns
        -------
        v : Tensor, shape (n, 6)
            Spatial velocities for all joints, stacked row-wise.
            Each row corresponds to one joint's [linear, angular] velocity.
        """

        if q is None: q = self.q_t
        if qd is None: qd = self.qd_t

        n = q.shape[0]
        v = []
        for i in range(n):
            v_i = ops_t.spatial_vel(q[: i + 1], qd[: i + 1], self.jac_fn)
            v.append(v_i.unsqueeze(0))   # (1, 6)
        v = torch.cat(v, dim=0)          # (n, 6)
        return v        


    def spatial_acc(self, q: torch.Tensor, qd: torch.Tensor, qdd: torch.Tensor) -> torch.Tensor:
        """
        Compute the spatial acceleration of all joints in the manipulator.

        Each joint's 6D spatial acceleration is represented as:
        [ax, ay, az, alpha_x, alpha_y, alpha_z].

        Parameters
        ----------        
        q : Tensor, shape (n,), optional
            Joint positions. Defaults to the internal state `self.q_t` if None.
        qd : Tensor, shape (n,), optional
            Joint velocities. Defaults to `self.qd_t` if None.
        qdd : Tensor, shape (n,), optional
            Joint accelerations. Defaults to `self.qdd_t` if None.

        Returns
        -------
        a : Tensor, shape (n, 6)
            Spatial accelerations for all joints, shape (n, 6).
        """

        if q is None: q = self.q_t
        if qd is None: qd = self.qd_t
        if qdd is None: qdd = self.qdd_t

        n = q.shape[0]
        a = []
        for i in range(n):
            a_i = ops_t.spatial_acc(q[: i + 1], qd[: i + 1], qdd[: i + 1], self.jac_fn)
            a.append(a_i.unsqueeze(0))   # (1, 6)
        a = torch.cat(a, dim=0)       # (n, 6)
        return a


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
        q = self.q.copy()  # initial guess
        for iter in range(max_iters):
            self.update(theta=q)
            current_T = self.fk(len(q)-1)[-1]
            pos_err = target_T[:3, 3] - current_T[:3, 3]
            rot_err_matrix = current_T[:3, :3].T @ target_T[:3, :3]
            rot_err_axis, rot_err_angle = self._rotation_matrix_to_axis_angle(rot_err_matrix)
            err = np.hstack((pos_err, rot_err_axis * rot_err_angle))

            if np.linalg.norm(err) < tol:
                break

            J = self.jac()
            J_pinv = np.linalg.pinv(J)
            delta_q = alpha * (J_pinv @ err)
            q += delta_q

        return q