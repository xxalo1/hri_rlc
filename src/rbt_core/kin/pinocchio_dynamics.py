from __future__ import annotations

from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
import pinocchio as pin
from common_utils import FloatArray
from common_utils import numpy_util as npu


class PinocchioDynamics:
    """Minimal Pinocchio wrapper for kinematics + dynamics.

    Assumes 1-DoF joints (model.nv == model.njoints - 1) for per-joint stacks.
    """

    def __init__(self,
        model: pin.Model,
        data: pin.Data | None = None,
        *,
        tcp_frame: str | None = None,
    ) -> None:
        model.nv
        self._model = model
        self._data = data if data is not None else model.createData()
        self._q = np.array(pin.neutral(model), dtype=npu.dtype)
        self._qd = np.zeros(model.nv, dtype=npu.dtype)
        self._qdd = np.zeros(model.nv, dtype=npu.dtype)

        self._fk_valid = False
        self._J_valid = False

        self._T_wf: FloatArray = np.eye(4, dtype=npu.dtype).reshape(4,4)

        if tcp_frame is None: 
            self._tcp_frame_id = self.nq
        else: 
            self._tcp_frame_id = self.get_frame_id(tcp_frame)

        self._mass = np.array(
            [self._model.inertias[i].mass for i in range(1, self._model.njoints)],
            dtype=npu.dtype,
        )

    @classmethod
    def from_urdf(cls,
        urdf_path: str | Path,
        *,
        package_dirs: Iterable[str] | None = None,
        root_joint: pin.JointModel | None = None,
        tcp_frame: str | None = None,
    ) -> "PinocchioDynamics":
        urdf_path = Path(urdf_path)
        kwargs: dict[str, object] = {}
        if root_joint is not None:
            kwargs["root_joint"] = root_joint
        if package_dirs is not None:
            kwargs["package_dirs"] = [str(p) for p in package_dirs]
        model = pin.buildModelFromUrdf(str(urdf_path), **kwargs)
        return cls(model, tcp_frame=tcp_frame)


    def get_frame_id(self, frame: str) -> int:
        """Convert frame name to frame ID, or return frame ID if already int."""
        return self._model.getFrameId(frame)


    @staticmethod
    def _validate_array(a, b, name: str | None = None) -> None:
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
        name = name or "array"
        if not isinstance(a, np.ndarray):
            raise TypeError(f"{name} must be {FloatArray.__name__}")
        if a.shape != b.shape:
            raise ValueError(f"{name}.shape {a.shape} != expected {b.shape}")


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
        if not (0 <= i <= max_value):
            raise ValueError(f"i must be in [0, {max_value - 1}]")


    def _invalidate_kinematics(self) -> None:
        self._fk_valid = False
        self._J_valid = False
        self._com_valid = False
        self._Ic_valid  = False


    @property
    def q(self) -> FloatArray: return self._q
    @q.setter
    def q(self, v: FloatArray) -> None:
        self._validate_array(v, self._q, "q")
        self._q[:] = v
        self._invalidate_kinematics()

    @property
    def qd(self) -> FloatArray: return self._qd
    @qd.setter
    def qd(self, v: FloatArray) -> None:
        self._validate_array(v, self._qd, "qd")
        self._qd[:] = v

    @property
    def qdd(self) -> FloatArray: return self._qdd
    @qdd.setter
    def qdd(self, v: FloatArray) -> None:
        self._validate_array(v, self._qdd, "qdd")
        self._qdd[:] = v

    @property
    def T_wj(self) -> FloatArray: return self._T_wf
    @T_wj.setter
    def T_wj(self, v: FloatArray) -> None:
        self._validate_array(v, self._T_wf, "T_wf")
        self._T_wf[:] = v

    @property
    def n(self) -> int:
        return self._model.nv

    @property
    def nq(self) -> int:
        return self._model.nq

    @property
    def joint_names(self) -> list[str]:
        return list(self._model.names[1:])

    @property
    def mass(self) -> FloatArray:
        return self._mass

    @property
    def oMi(self) -> Sequence[pin.SE3]:
        """Joint placements in world, list-like of SE3."""
        return self._data.oMi

    @property
    def oMf(self) -> Iterable[pin.SE3]:
        """Frame placements in world, list-like of SE3."""
        return self._data.oMf

    @property
    def M(self) -> FloatArray:
        """Mass matrix (nv x nv)."""
        M = self._data.M
        return 0.5 * (M + M.T)

    @property
    def nle(self) -> FloatArray:
        """Nonlinear effects (C*qd + g)."""
        return self._data.nle

    @property
    def g(self) -> FloatArray:
        """Generalized gravity vector."""
        return self._data.g

    @property
    def com(self) -> FloatArray:
        """Center of mass vectors"""
        return self._data.com


    def set_gravity(self, g_vec: FloatArray) -> None:
        self._model.gravity.linear[:] = g_vec


    def step(self,
        q: FloatArray | None = None,
        qd: FloatArray | None = None,
        qdd: FloatArray | None = None,
    ) -> None:
        if q is not None: self.q = q
        if qd is not None: self.qd = qd
        if qdd is not None: self.qdd = qdd


    def joint_se3(self,
        i: int
    ) -> pin.SE3:
        self._validate_index_range(i, self._model.njoints)
        self._ensure_fk()
        return self.oMi[i]


    def frame_se3(self, 
        frame_id: int | None = None
    ) -> pin.SE3:
        if frame_id is None: 
            frame_id = self._model.nframes - 1
        else: 
            self._validate_index_range(frame_id, self._model.nframes - 1)
        self._ensure_fk()
        return self._data.oMf[frame_id]


    def joint_T(self, 
        i: int
    ) -> FloatArray:
        self._validate_index_range(i, self._model.njoints)
        self._ensure_fk()
        return self._data.oMi[i].homogeneous


    def frame_T(self, 
        i: int | None = None
    ) -> FloatArray:
        if i is None: 
            i = self._tcp_frame_id
        else: 
            self._validate_index_range(i, self._model.nframes - 1)
        self._ensure_fk()
        return self._data.oMf[i].homogeneous


    def frame_jac(self, 
        frame_id: int | None = None
    ) -> FloatArray:
        if frame_id is None: 
            frame_id = self._tcp_frame_id
        else: 
            self._validate_index_range(frame_id, self._model.nframes - 1)
        self._ensure_jac()
        self._data.
        return J
    

    def joint_Rp(self, 
        i: int
    ) -> tuple[FloatArray, FloatArray]:
        self._validate_index_range(i, self._model.njoints)
        self._ensure_fk()
        oM = self._data.oMi[i]
        return oM.rotation, oM.translation


    def joint_T_stack(self
    ) -> FloatArray:
        self._ensure_fk()
        self.T_wj = np.asarray(self._data.oMi)
        return self.T_wj


    def frame_T_stack(self
    ) -> FloatArray:
        self._ensure_fk()
        self.T_wf = np.asarray(self._data.oMf)
        return self.T_wf


    def dof_joint_id(self, 
        dof: int
    ) -> int:
        return dof + 1


    def inertia_matrix(self) -> FloatArray:
        pin.crba(self._model, self._data, self.q)
        return self.M


    def batch_frame_T(self, 
        Q: FloatArray, 
        frame: str | None = None
    ) -> FloatArray:
        """
        World -> one frame transform for each q in Q.

        Returns: (B, 4, 4) float64
        """
        model = self._model
        if frame is None:
            frame_id = self._tcp_frame_id
        else: 
            frame_id = self.get_frame_id(frame)

        data = model.createData()
        out = np.empty((Q.shape[0], 4, 4), dtype=np.float64)

        for i in range(Q.shape[0]):
            q = Q[i]
            pin.framesForwardKinematics(model, data, q)
            out[i] = data.oMf[frame_id].homogeneous
        return out


    def gravity_vector(self) -> FloatArray:
        return pin.computeGeneralizedGravity(self._model, self._data, self.q)


    def rnea(self, 
        q: FloatArray,
        qd: FloatArray,
        qdd: FloatArray
    ) -> FloatArray:
        return pin.rnea(self._model, self._data, q, qd, qdd)


    def _ensure_fk(self) -> None:
        if self._fk_valid:
            return
        self.compute_fk()
        self._fk_valid = True


    def compute_fk(self) -> None:
        pin.forwardKinematics(self._model, self._data, self.q)
        pin.updateFramePlacements(self._model, self._data)


    def _ensure_jac(self) -> None:
        if self._J_valid:
            return
        self.compute_jac()
        self._J_valid = True


    def compute_jac(self) -> None:
        pin.computeJointJacobians(self._model, self._data, self.q)
        pin.updateFramePlacements(self._model, self._data)


    def dyn_pinv(self, J_task: FloatArray) -> tuple[FloatArray, FloatArray]:
        M = self.inertia_matrix()
        X = np.linalg.solve(M, J_task.T)
        Lambda_inv = J_task @ X
        Lambda = np.linalg.inv(Lambda_inv)
        J_dyn = X @ Lambda
        return J_dyn, Lambda


    def task_to_joint(
        self,
        xdd: FloatArray,
        J_task: FloatArray,
        xdd_sec: FloatArray | None = None,
        J_sec: FloatArray | None = None,
    ) -> FloatArray:
        n = J_task.shape[1]
        I = np.eye(n, dtype=xdd.dtype)

        J1_dyn, _ = self.dyn_pinv(J_task)
        qdd_main = J1_dyn @ xdd

        qdd_sec = np.zeros(n, dtype=xdd.dtype)
        if xdd_sec is not None and J_sec is not None:
            J2 = J_sec
            JJt = J2 @ J2.T
            JJt_inv = np.linalg.inv(JJt)
            J2_pinv = J2.T @ JJt_inv

            qdd_sec_candidate = J2_pinv @ xdd_sec
            N1 = I - J1_dyn @ J_task
            qdd_sec = qdd_sec + N1 @ qdd_sec_candidate

        return qdd_main + qdd_sec


    @staticmethod
    def _pin_to_linear_angular(J: FloatArray) -> FloatArray:
        return np.vstack((J[3:, :], J[:3, :]))