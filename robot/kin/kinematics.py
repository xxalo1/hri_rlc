from __future__ import annotations

import numpy as np
import torch
from typing import Generic, TypeVar

from . import ops, ops_t
from . import core_ops as cops
from ...utils import(numpy_util as npu, array_compat as xp, ArrayT, FloatArray, dtype)

ArrayT = TypeVar("ArrayT", FloatArray, torch.Tensor)

class Kinematics(Generic[ArrayT]):
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
        dh: dict[str, ArrayT],
        o_wb: ArrayT | None = None,
        axes_wb: ArrayT | None = None,
        inertia: dict[str, ArrayT] | None = None,
        ) -> None:
        """
        d, a, alpha: 1D array (length n)
        theta0: optional 1D array of fixed offsets (length n), defaults to zeros
        b: optional 1D array of translations along the new z axis (length n)
        axes_o: optional 2D array (3x3), axes of base frame in world coords, defaults to identity
        inertia: optional dict of manipulator iertias coms masses 
        o_0: optional 1d array, origin of base frame in world coords, defaults to [0,0,0]
        """
        

        self.d = dh["d"]
        self.a = dh["a"]
        self.alpha = dh["alpha"]
        self.q0 = dh["q0"]
        self.b = dh.get("b")

        if o_wb is None: o_wb = xp.array_like(self.d, [0, 0, 0])
        if axes_wb is None: axes_wb = xp.eye_like(self.d, 3)
        if self.b is None: self.b = xp.zeros_like(self.d)

        if inertia is None: inertia = {}

        self.o_wb = o_wb  # origins in base frame
        self.axes_wb = axes_wb  # z axes in base frame

        self.T_wb = xp.zeros_like(self.d, shape=(4, 4))
        self.T_wb[:3, :3] = axes_wb
        self.T_wb[:3, 3] = o_wb

        self.q = xp.zeros_like(self.d)
        self.qd = xp.zeros_like(self.d)
        self.qdd = xp.zeros_like(self.d)

        n = self.n

        self.com_fl = inertia.get("com", xp.zeros_like(self.d, shape=(n+1, 3)))
        self.mass = inertia.get("mass", xp.zeros_like(self.d, shape=n+1))
        self.Ic_fl = inertia.get("Ic", xp.zeros_like(self.d, shape=(n+1, 3, 3)))

        self.T_wl: ArrayT = xp.empty_like(self.d, shape=(n, 4, 4))
        self.T_wf: ArrayT = xp.empty_like(self.d, shape=(n + 1, 4, 4))
        self.J: ArrayT = xp.empty_like(self.d, shape=(n, 6, n))
        self._fk_valid: bool = False
        self._J_valid: bool = False
        self._array_cls = np.ndarray if isinstance(self.d, np.ndarray) else torch.Tensor

    @property
    def n(self) -> int:
        return len(self.d)

    @property
    def d(self): return self._d
    @d.setter
    def d(self, v):
        self._validate_array(v, self._d, "d")
        self._d = v
        self._invalidate_kinematics()

    @property
    def alpha(self): return self._alpha
    @alpha.setter
    def alpha(self, v):
        self._validate_array(v, self._alpha, "alpha")
        self._alpha = v
        self._invalidate_kinematics()

    @property
    def q0(self): return self._q0
    @q0.setter
    def q0(self, v):
        self._validate_array(v, self._q0, "q0")
        self._q0 = v
        self._invalidate_kinematics()

    @property
    def a(self): return self._a
    @a.setter
    def a(self, v):
        self._validate_array(v, self._a, "a")
        self._a = v
        self._invalidate_kinematics()

    @property
    def o_wb(self): return self._o_wb
    @o_wb.setter
    def o_wb(self, v):
        self._validate_array(v, self._o_wb, "o_wb")
        self._o_wb = v
        # keep T_wb derived and consistent:
        self.T_wb[:3, 3] = v
        self._invalidate_kinematics()

    @property
    def axes_wb(self): return self._axes_wb
    @axes_wb.setter
    def axes_wb(self, v):
        self._validate_array(v, self._axes_wb, "axes_wb")
        self._axes_wb = v
        self.T_wb[:3, :3] = v
        self._invalidate_kinematics()

    @property
    def q(self): return self._q
    @q.setter
    def q(self, v):
        self._validate_array(v, self._q, "q")
        self._q = v
        self._invalidate_kinematics()

    @property
    def qd(self): return self._qd
    @qd.setter
    def qd(self, v):
        self._validate_array(v, self._qd, "qd")
        self._qd = v

    @property
    def qdd(self): return self._qdd
    @qdd.setter
    def qdd(self, v):
        self._validate_array(v, self._qdd, "qdd")
        self._qdd = v

    @property
    def com_wl(self) -> ArrayT:
        return self._compute_com_wl()

    @property
    def Ic_wl(self) -> ArrayT:
        return self._compute_Ic_wl()


    def _invalidate_kinematics(self) -> None:
        self._fk_valid = False
        self._J_valid = False
        self._com_valid = False
        self._Ic_valid  = False


    def _validate_array(self, a, b, name: str | None = None) -> None:
        """
        Validate that `x` is of the correct type and shape.

        Parameters
        ----------
        x : ndarray | Tensor
            Array to validate.
        b : ndarray | Tensor
            Reference array for shape checking.
        name : str, optional
            Name of the array for error messages.
        """
        ArrayT = self._array_cls
        name = name or "array"
        if not isinstance(a, ArrayT):
            raise TypeError(f"{name} must be {ArrayT.__name__}")
        if a.shape != b.shape:
            raise ValueError(f"{name}.shape {a.shape} != expected {b.shape}")
        if isinstance(a, torch.Tensor) and (a.dtype != b.dtype or a.device != b.device):
            raise TypeError(f"{name} dtype/device must match instance")


    def _validate_index_range(self, i: int, max_value: int | None = None) -> None:
        """
        Validate that index `i` is within the valid range.

        Parameters
        ----------
        i : int
            Index to validate.

        max_value : int, optional
            Maximum valid value for the index. Defaults to `self.n - 1`.
        """
        if max_value is None:
            max_value = self.n - 1
        if not (0 <= i < max_value):
            raise ValueError(f"i must be in [0, {max_value - 1}]")


    def _compute_com_wl(self) -> ArrayT | None:
        """
        Compute centers of mass (COM) positions in the world frame.
        includes base at index 0.
        """
        if self._com_valid:
            return self._com_wl

        T_wf = self.forward_kinematics()
        com_wl = cops.com_world(T_wf, self.com_fl)

        self._com_wl = com_wl
        self._com_valid = True
        return com_wl


    def _compute_Ic_wl(self) -> ArrayT | None:
        """
        Inertia tensors rotated to world frame, shape (n+1, 3, 3).
        includes base at index 0.
        """
        if self._Ic_valid:
            return self._Ic_wl

        T_wf = self.forward_kinematics()   # (n+1, 4, 4)
        R = T_wf[:, :3, :3]                             # (n+1, 3, 3)
        Ic_fl = self.Ic_fl                                  # (n+1, 3, 3)

        Ic_wl = xp.einsum("kia, kab, kjb -> kij", R, Ic_fl, R)

        self._Ic_wl = Ic_wl
        self._Ic_valid = True
        return Ic_wl


    def _prepend_base(self,
        T_wl: ArrayT
        ) -> ArrayT:
        """Stack link transforms `T_wl` with base transform `T_wb`.
        
        Parameters
        ----------
        T_wl : ndarray | Tensor, shape (n, 4, 4)
            3D array of world-frame transforms [T_w1, ..., T_wn].
        Returns
        -------
        T_wf : ndarray | Tensor, shape (n+1, 4, 4)
            3D array of world-frame transforms including the base [T_w0, T_w1, ..., T_wn].
        """

        T_wf = xp.empty_like(T_wl, shape=(T_wl.shape[0] + 1, 4, 4))
        
        T_wf[0]  = self.T_wb
        T_wf[1:] = T_wl
        return T_wf


    def forward_kinematics(self, 
        q: ArrayT | None = None, 
        i: int | None = None,
        use_cache: bool = True
        ) -> ArrayT:
        """
        Compute cumulative forward kinematics in the world frame.

        Stores results in 'self.T_wf' and 'self.T_wl'.

        Parameters
        ----------
        q : ndarray | Tensor, shape (n,), optional
            Joint positions. Defaults to the internal state `self.q` if `None`.
        i : int, optional
            joint index up to which to compute the forward kinematics. 
            Defaults to all joints if `None`.

        Returns
        -------
        T_wf : ndarray | Tensor, shape (i+2, 4, 4)
            World-frame transforms [T_W_0, ..., T_W_(i+1)].
        """
        if q is not None:
            self._validate_array(q, self.q, name ="q")
            use_cache = False
        else:
            q = self.q
        n = self.n

        if i is not None: 
            self._validate_index_range(i)
        else:
            i = n -1

        k = i + 2

        if use_cache and self._fk_valid:
            return self.T_wf[:k]

        T_bl = cops.cumulative_transforms(q, self.d, self.a, self.alpha, self.b)
        T_wl = self.T_wb @ T_bl
        T_wf = self._prepend_base(T_wl)

        if not use_cache: 
            return T_wf[:k]

        self.T_wl = T_wl
        self.T_wf = T_wf
        self._fk_valid = True
        return self.T_wf[:k]


    def update_config(self,
        d: ArrayT | None = None,
        a: ArrayT | None = None, 
        alpha: ArrayT | None = None, 
        o_0: ArrayT | None = None,
        axes_o: ArrayT | None = None
        ) -> None:
        """
        Update attributes d, a, alpha, o_0, axes_o.

        Parameters
        ----------
        d : ndarray | Tensor | None
            Link offsets.
        a : ndarray | Tensor | None
            Link lengths.
        alpha : ndarray | Tensor | None
            Link twists.
        o_0 : ndarray | Tensor | None
            Base frame origin.
        axes_o : ndarray | Tensor | None
            Base frame axes.
        """

        if d is not None: self.d = d
        if a is not None: self.a = a
        if alpha is not None: self.alpha = alpha
        if o_0 is not None: 
            self.o_wb = o_0; 
            self.T_wb[:3, 3] = o_0
        if axes_o is not None: 
            self.axes_wb = axes_o
            self.T_wb[:3, :3] = axes_o


    def step(self, 
        q: ArrayT | None = None,
        qd: ArrayT | None = None, 
        qdd: ArrayT | None = None
        ) -> None:
        """
        Update current joint states `self.q`, `self.qd`, `self.qdd`.
        updated if not None.

        Parameters
        ----------
        q : ndarray | Tensor, shape (n,), optional
            Joint positions.
        qd : ndarray | Tensor, shape (n,), optional
            Joint velocities.
        qdd : ndarray | Tensor, shape (n,), optional
            Joint accelerations.
        """
        if q is not None: self.q = q
        if qd is not None: self.qd = qd
        if qdd is not None: self.qdd = qdd


    def full_jac(self,
        q: ArrayT | None = None,
        i: int | None = None,
        use_cache: bool = True
        ) -> ArrayT:
        """
        Compute the full geometric Jacobian matrix for all joints
        and store it in `self.J`.

        This method computes the manipulator Jacobian at the current joint
        configuration `self.q`.

        Parameters
        ----------
        q : ndarray | Tensor, shape (n,), optional
            Joint positions.
        i : int, optional
            Joint index to compute the Jacobian for.
        use_cache : bool, optional
            Whether to use cached results if available.

        Returns
        -------
        J : ndarray | Tensor, shape (n, 6, n)
            The geometric Jacobian matrix evaluated at the current configuration.
            The top three rows correspond to linear velocity terms, and the bottom
            three correspond to angular velocity terms.
        """

        if q is not None: 
            self._validate_array(q, self.q, name ="q")
            use_cache = False

        if i is not None: 
            self._validate_index_range(i)

        if use_cache and self._J_valid:
            return self.J if i is None else self.J[i, ...]

        n = self.n
        T_wf = self.forward_kinematics(q)
        J = xp.zeros_like(T_wf, shape=(n, 6, n))
        for j in range(n):
            J[j, :, :j+1] = cops.jacobian(T_wf[:j+2])  # pyright: ignore[reportArgumentType]

        if not use_cache: 
            return J if i is None else J[i, ...]

        self.J = J
        self._J_valid = True
        return J


    def jac(self, 
        q: ArrayT | None = None,
        i: int | None = None
        ) -> ArrayT:
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
        q : ndarray | Tensor, shape (n,), optional
            Joint positions.
            If None, defaults to `self.q`.
        i : int, optional
            joint index up to which to compute the Jacobian.
            If None, defaults to end effector.

        Returns
        -------
        J : ndarray | Tensor, shape (6, n)
            The geometric Jacobian matrix evaluated at the specified configuration.
            The top three rows correspond to linear velocity terms, and the bottom
            three correspond to angular velocity terms.
        """
        if q is not None: 
            self._validate_array(q, "q")
        else:
            q = self.q
        if i is not None: 
            self._validate_index_range(i)
        else:
            i = self.n -1

        T_wf = self.forward_kinematics(q, i=i)
        J = cops.jacobian(T_wf)
        return J


    def jac_com(self, use_cache: bool = True) -> ArrayT:
        """
        Full-body Jacobians about each link COM instead of the link frame origin.

        Uses the current state `self.q` and the stored COMs in link frame `self.com_fl`.

        J_orig : about link frame origins
            Jv_o = J[ :, 0:3, : ]
            Jw   = J[ :, 3:6, : ]

        J_com : about link COMs
            Jv_com = Jv_o - [r_w] @ Jw
            where r_w is the vector from link origin to COM in world frame.

        Parameters
        ----------
        use_cache : bool, optional
            Whether to use cached FK/J if valid. Defaults to True.

        Returns
        -------
        J_com : ndarray | Tensor, shape (n, 6, n)
            Full Jacobian for all links, linear part shifted to COMs.
        """
        
        T_wf = self.forward_kinematics(use_cache=use_cache)
        J = self.full_jac(use_cache=use_cache)
        T_wl = T_wf[1:]
        R_wl = T_wl[:, :3, :3]   # (n, 3, 3)
        o_w  = T_wl[:, :3, 3]    # (n, 3)

        com_l = self.com_fl
        Rc = R_wl @ com_l                                # (n, 3)
        com_w = Rc + o_w                                 # (n, 3)

        r_w = com_w - o_w                                # (n, 3)

        S = cops.skew(r_w)                               # (n, 3, 3)
        Jv_o = J[:, 0:3, :]                              # (n, 3, n)
        Jw   = J[:, 3:6, :]                              # (n, 3, n)
        Jv_com = Jv_o - (S @ Jw)
        J_com = xp.zeros_like(J)
        J_com[:, 0:3, :] = Jv_com # pyright: ignore[reportArgumentType]
        J_com[:, 3:6, :] = Jw # pyright: ignore[reportArgumentType]
        return J_com


    def spatial_vel(self, 
        q: ArrayT | None = None, 
        qd: ArrayT | None = None
        ) -> ArrayT:
        """
        Compute the spatial velocity of all joints in the manipulator.

        Each joint's 6D spatial velocity is given as:
        [vx, vy, vz, wx, wy, wz].

        Parameters
        ----------
        q : ndarray | Tensor, shape (n,), optional
            Joint positions. Defaults to `self.q_t` if None.
        qd : ndarray | Tensor, shape (n,), optional
            Joint velocities. Defaults to `self.qd_t` if None.

        Returns
        -------
        v : ndarray | Tensor, shape (n, 6)
            Spatial velocities for all joints, stacked row-wise.
            Each row corresponds to one joint's [linear, angular] velocity.
        """

        if q is None: q = self.q
        if qd is None: qd = self.qd

        J = self.full_jac(q)  # (n, 6, n)
        v = J @ qd            # (n, 6, n) @ (n,) -> (n, 6)
        
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
            current_T = self.forward_kinematics(len(q)-1)[-1]
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