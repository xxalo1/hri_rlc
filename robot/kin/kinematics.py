from __future__ import annotations

import numpy as np
import torch
from typing import Any, Optional, Sequence, Callable
from numpy.typing import ArrayLike, NDArray


from . import ops, ops_t, core_ops as cops
from ...utils import numpy_util as npu
from ...utils import pytorch_util as ptu

FloatArray = npu.FloatArray
dtype = npu.dtype

class Kinematics:
    """
    Kinematics class for representing and computing robot kinematics.

    initializes with DH parameters and provides methods for forward kinematics,
    Jacobian computation, spatial velocities, and accelerations.

    Parameters
    ----------
    dh : dict[str, FloatArray]
        Dictionary containing DH parameters:
            - 'd': link offsets
            - 'a': link lengths
            - 'alpha': link twists
            - 'q0': initial joint positions
            - 'b': optional translations along new z axes
    o_wb : FloatArray | None, optional
        Origin of the base frame in world coordinates. Defaults to [0, 0, 0].
    axes_wb : FloatArray | None, optional
        Axes of the base frame in world coordinates. Defaults to identity.
    inertia : dict[str, FloatArray] | None, optional
        Dictionary containing inertia properties:
            - 'com': centers of mass
            - 'mass': masses
            - 'Ic': inertia tensors
    dtype : data-type, optional
        Data type for numerical arrays. Defaults to npu.dtype.

    Attributes
    ----------
    q : FloatArray
        Current joint positions.
    qd : FloatArray
        Current joint velocities.
    qdd : FloatArray
        Current joint accelerations.
    d : FloatArray
        Link offsets.
    a : FloatArray
        Link lengths.
    alpha : FloatArray
        Link twists.
    q0 : FloatArray
        Initial joint positions.
    b : FloatArray
        Translations along new z axes.
    o_wb : FloatArray
        Origin of the base frame in world coordinates.
    axes_wb : FloatArray
        Axes of the base frame in world coordinates.
    T_wb : FloatArray
        World-to-base homogeneous transform.
    com_fl : FloatArray | None
        Centers of mass in the link's frame.
    mass : FloatArray | None
        Masses of the links.
    Ic_fl : FloatArray | None
        Inertia tensors in the link's frame.
    q_t : Tensor
        Current joint positions as a PyTorch tensor.
    qd_t : Tensor
        Current joint velocities as a PyTorch tensor.
    qdd_t : Tensor
        Current joint accelerations as a PyTorch tensor.
    d_t : Tensor
        Link offsets as a PyTorch tensor.
    a_t : Tensor
        Link lengths as a PyTorch tensor.
    alpha_t : Tensor
        Link twists as a PyTorch tensor.
    q0_t : Tensor
        Initial joint positions as a PyTorch tensor.
    b_t : Tensor
        Translations along new z axes as a PyTorch tensor.
    T_wb_t : Tensor
        World-to-base homogeneous transform as a PyTorch tensor.

    Methods
    -------
    """
    def __init__(self, 
        dh: dict[str, FloatArray],
        o_wb: FloatArray | None = None,
        axes_wb: FloatArray | None = None,
        inertia: dict[str, FloatArray] | None = None,
        dtype = dtype,

        ) -> None:
        """
        d, a, alpha: 1D array (length n)
        theta0: optional 1D array of fixed offsets (length n), defaults to zeros
        b: optional 1D array of translations along the new z axis (length n)
        axes_o: optional 2D array (3x3), axes of base frame in world coords, defaults to identity
        inertia: optional dict of manipulator iertias coms masses 
        o_0: optional 1d array, origin of base frame in world coords, defaults to [0,0,0]
        """

        self.dtype = dtype

        self.d = dh["d"]
        self.a = dh["a"]
        self.alpha = dh["alpha"]
        self.q0 = dh["q0"]
        self.b = dh.get("b")

        if o_wb is None: o_wb = np.array([0, 0, 0], dtype=dtype)
        if axes_wb is None: axes_wb = np.eye(3, dtype=dtype)
        if self.b is None: self.b = np.zeros_like(self.d)

        if inertia is None: inertia = {}

        self.com_fl = inertia.get("com", None)
        self.mass = inertia.get("mass", None)
        self.Ic_fl = inertia.get("Ic", None)

        self.o_wb = o_wb  # origins in base frame
        self.axes_wb = axes_wb  # z axes in base frame

        self.T_wb = np.eye(4, dtype=self.dtype)
        self.T_wb[:3, :3] = axes_wb
        self.T_wb[:3, 3] = o_wb

        self.q: FloatArray = self.q0.copy()
        self.qd: FloatArray = np.zeros_like(self.q)
        self.qdd: FloatArray = np.zeros_like(self.q)

        self._jac_fn: Callable[[torch.Tensor], torch.Tensor] | None = None
        self.update_tensors()

    # done
    def prepend_base(self,
        T_wl: FloatArray | torch.Tensor
        ) -> FloatArray | torch.Tensor:
        """Stack link transforms `T_wl` with base transform `T_wb`.
        
        Parameters
        ----------
        T_wl : ndarray | Tensor, shape (n, 4, 4)
            3D array of world-frame transforms [T_w1, ..., T_wn].
        Returns
        -------
        T_w : ndarray | Tensor, shape (n+1, 4, 4)
            3D array of world-frame transforms including the base [T_w0, T_w1, ..., T_wn].
        """

        if isinstance(T_wl, torch.Tensor):
            T_w = torch.empty(T_wl.shape[0] + 1, 4, 4, dtype=T_wl.dtype, device=T_wl.device)
        elif isinstance(T_wl, np.ndarray):
            T_w = np.empty((T_wl.shape[0] + 1, 4, 4), dtype=T_wl.dtype)
        else: 
            raise TypeError("T_wl must be a numpy array or torch tensor.")
        
        T_w[0]  = self.T_wb # type: ignore
        T_w[1:] = T_wl # type: ignore
        return T_w

    # done
    def fk(self, 
        q: FloatArray | None = None, 
        i: int | None = None
        ) -> FloatArray:
        """
        Compute cumulative forward kinematics in the world frame.

        Parameters
        ----------
        q : ndarray, shape (n,), optional
            Joint positions. Defaults to the internal state `self.q` if `None`.
        i : int, optional
            joint index up to which to compute the forward kinematics. 
            Defaults to all joints if `None`.

        Returns
        -------
        T_wl : ndarray, shape (i+1, 4, 4)
            World-frame transforms [T_W_1, ..., T_W_(i+1)].
        """
        if q is None: q = self.q
        if i is None: i = len(q) -1
        i+=1

        T_bl = cops.transform_matrices(q, self.d, self.a, self.alpha, self.b)
        T_bl = T_bl[:i]
        T_wl = self.T_wb @ T_bl
        return T_wl

    # done
    def fk_t(self, 
        q: torch.Tensor | None = None, 
        i: int | None = None
        ) -> torch.Tensor:
        """
        Compute cumulative forward kinematics in the world frame (PyTorch).

        Parameters
        ----------
        q : Tensor, shape (n,), optional
            Joint positions. Defaults to the internal state `self.q_t` if None.
        i : int, optional
            joint index up to which to compute the forward kinematics. 
            Defaults to all joints if `None`.

        Returns
        -------
        T_wl : Tensor, shape (i+1, 4, 4)
            World-frame transforms stacked as [T_W_1, ..., T_W_(i+1)].
        """
        if q is None: q = self.q_t
        if i is None: i = len(q) -1
        i+=1

        T_bl = ops_t.cumulative_transforms(q, self.d_t, self.a_t, self.alpha_t, self.b_t)
        T_bl = T_bl[:i]
        T_wl = self.T_wb_t @ T_bl
        return T_wl

    # done
    def update_com(self) -> None:
        """
        Update the centers of mass (COM) positions 
        from link frame to world frame.
        """
        if self.com_fl is None: return
        T_wl = self.fk()
        T_w = self.prepend_base(T_wl)
        self.com_wl = ops.com_world(T_w, self.com_fl) # type: ignore

    # done
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

        assert any(x is not None and not np.ndarray for x in (d, a, alpha, o_0, axes_o)),\
              "any input provided must be a numpy array."

        if d is not None: self.d = d
        if a is not None: self.a = a
        if alpha is not None: self.alpha = alpha
        if o_0 is not None: 
            self.o_wb = o_0; 
            self.T_wb[:3, 3] = o_0
        if axes_o is not None: 
            self.axes_wb = axes_o
            self.T_wb[:3, :3] = axes_o

        if any(x is not None for x in (d, a, alpha, o_0, axes_o)):
            self._jac_fn = None
            self.update_tensors()

    # done
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
        assert any(x is not None and not np.ndarray for x in (q, qd, qdd)),\
              "any input provided must be a numpy array."

        if q is not None: 
            self.q = q
            self.q_t = ptu.from_numpy(q, dtype=ptu.dtype)
        if qd is not None:
            self.qd = qd
            self.qd_t = ptu.from_numpy(qd, dtype=ptu.dtype)
        if qdd is not None:
            self.qdd = qdd
            self.qdd_t = ptu.from_numpy(qdd, dtype=ptu.dtype)

    # done
    def update_tensors(self):
        """Update all attributes to PyTorch tensors for autograd."""
        self.d_t = ptu.from_numpy(self.d, dtype=ptu.dtype)
        self.a_t = ptu.from_numpy(self.a, dtype=ptu.dtype)
        self.alpha_t = ptu.from_numpy(self.alpha, dtype=ptu.dtype)
        self.b_t = ptu.from_numpy(self.b, dtype=ptu.dtype) if self.b is not None else None
        self.T_wb_t = ptu.from_numpy(self.T_wb, dtype=ptu.dtype)

    # done
    def jac(self, 
        q: FloatArray | None = None,
        i: int | None = None
        ) -> FloatArray:
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
        i : int, optional
            joint index up to which to compute the Jacobian.
            If None, defaults to end effector.

        Returns
        -------
        J : ndarray, shape (6, n)
            The geometric Jacobian matrix evaluated at the specified configuration.
            The top three rows correspond to linear velocity terms, and the bottom
            three correspond to angular velocity terms.
        """
        if q is None: q = self.q
        T_wl = self.fk(q, i=i)
        T_wf = self.prepend_base(T_wl)
        J = ops.jacobian(T_wf) # type: ignore
        return J

    # add way to 
    def jac_t(self, 
        q: torch.Tensor | None = None,
        i: int | None = None
        ) -> torch.Tensor:
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
        i : int, optional
            Joint index to which compute the Jacobian.
            If None, defaults to end effector.

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
        return jac(q, i=i)

    # done
    def _init_jac_callable(self) -> Callable[[torch.Tensor, int | None], torch.Tensor]:
        """
        Initialize jacobian function callable `self.jac_fn` for autograd.
        """
        def jac(q: torch.Tensor, i: int | None = None) -> torch.Tensor:
            T_bl = self.fk_t(q, i=i)
            T_wf = self.prepend_base(T_bl)
            return ops_t.jacobian(T_wf)   # type: ignore # (6,n)

        self.jac_fn = jac
        return jac

    # done
    def spatial_vel_t(self, 
        q: torch.Tensor | None = None, 
        qd: torch.Tensor | None = None
        ) -> torch.Tensor:
        """
        Compute the spatial velocity of all joints in the manipulator. 
        pytorch version for automatic differentiation.

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

        n = len(q)
        v = []
        J_all = torch.zeros((n, 6, n), dtype=q.dtype, device=q.device)
        for i in range(n):
            J_all[i, :, : i + 1] = self.jac_t(q, i=i)  # (6, i+1)
        v = J_all @ qd
        return v

    # done
    def spatial_vel(self, 
        q: FloatArray | None = None, 
        qd: FloatArray | None = None
        ) -> FloatArray:
        """
        Compute the spatial velocity of all joints in the manipulator.

        Each joint's 6D spatial velocity is given as:
        [vx, vy, vz, wx, wy, wz].

        Parameters
        ----------
        q : ndarray, shape (n,), optional
            Joint positions. Defaults to `self.q_t` if None.
        qd : ndarray, shape (n,), optional
            Joint velocities. Defaults to `self.qd_t` if None.

        Returns
        -------
        v : ndarray, shape (n, 6)
            Spatial velocities for all joints, stacked row-wise.
            Each row corresponds to one joint's [linear, angular] velocity.
        """

        if q is None: q = self.q
        if qd is None: qd = self.qd

        n = len(q)
        v = []
        J_all = np.zeros((n, 6, n), dtype=q.dtype)
        for i in range(n):
            J_all[i, :, : i + 1] = self.jac(q, i=i)  # (6, i+1)
        v = J_all @ qd
        return v

    # broken
    def spatial_acc(self, 
        q: torch.Tensor, 
        qd: torch.Tensor, 
        qdd: torch.Tensor
        ) -> torch.Tensor:
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

    # broken
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
            self.step(q=q)
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