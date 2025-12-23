from __future__ import annotations

from pathlib import Path
from typing import Iterable

import numpy as np
import pinocchio as pin

from common_utils import FloatArray
from common_utils import numpy_util as npu


class PinocchioDynamics:
    """Minimal Pinocchio wrapper for kinematics + dynamics.

    Assumes 1-DoF joints (model.nv == model.njoints - 1) for per-joint stacks.
    """

    def __init__(
        self,
        model: pin.Model,
        data: pin.Data | None = None,
        *,
        ee_frame: str | int | None = None,
        g_vec: FloatArray | None = None,
    ) -> None:
        self.model = model
        self.data = data if data is not None else model.createData()

        self._q = np.array(pin.neutral(model), dtype=npu.dtype)
        self._qd = np.zeros(model.nv, dtype=npu.dtype)
        self._qdd = np.zeros(model.nv, dtype=npu.dtype)

        self._fk_valid = False
        self._J_valid = False
        self._com_valid = False
        self._Ic_valid = False
        self._J_com_valid = False

        self._T_wf: FloatArray | None = None
        self._J_full: FloatArray | None = None
        self._com_wl: FloatArray | None = None
        self._Ic_wl: FloatArray | None = None
        self._J_com: FloatArray | None = None

        self._ee_frame_id: int | None = None
        if ee_frame is not None:
            self._ee_frame_id = self._resolve_frame_id(ee_frame)

        if g_vec is not None:
            g = np.asarray(g_vec, dtype=npu.dtype).reshape(3)
            self.model.gravity.linear = g
        self.g_vec = np.asarray(self.model.gravity.linear, dtype=npu.dtype)

        self._mass = np.array(
            [self.model.inertias[i].mass for i in range(1, self.model.njoints)],
            dtype=npu.dtype,
        )

    @classmethod
    def from_urdf(
        cls,
        urdf_path: str | Path,
        *,
        package_dirs: Iterable[str] | None = None,
        root_joint: pin.JointModel | None = None,
        ee_frame: str | int | None = None,
        g_vec: FloatArray | None = None,
    ) -> "PinocchioKinematics":
        urdf_path = Path(urdf_path)
        kwargs: dict[str, object] = {}
        if root_joint is not None:
            kwargs["root_joint"] = root_joint
        if package_dirs is not None:
            kwargs["package_dirs"] = [str(p) for p in package_dirs]
        model = pin.buildModelFromUrdf(str(urdf_path), **kwargs)
        return cls(model, ee_frame=ee_frame, g_vec=g_vec)


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
        if np.array_equal(v, self._q):
            return
        self._q[:] = v
        self._invalidate_kinematics()

    @property
    def qd(self) -> FloatArray: return self._qd
    @qd.setter
    def qd(self, v: FloatArray) -> None:
        self._validate_array(v, self._qd, "qd")
        if np.array_equal(v, self._qd):
            return
        self._qd[:] = v

    @property
    def qdd(self) -> FloatArray: return self._qdd
    @qdd.setter
    def qdd(self, v: FloatArray) -> None:
        self._validate_array(v, self._qdd, "qdd")
        if np.array_equal(v, self._qdd):
            return
        self._qdd[:] = v

    @property
    def n(self) -> int:
        return self.model.nv

    @property
    def nq(self) -> int:
        return self.model.nq

    @property
    def joint_names(self) -> list[str]:
        return list(self.model.names[1:])

    @property
    def mass(self) -> FloatArray:
        return self._mass


    def step(
        self,
        q: FloatArray | None = None,
        qd: FloatArray | None = None,
        qdd: FloatArray | None = None,
    ) -> None:
        if q is not None:
            self.q = q
        if qd is not None:
            self.qd = qd
        if qdd is not None:
            self.qdd = qdd


    def forward_kinematics(
        self,
        q: FloatArray | None = None,
        i: int | None = None,
        use_cache: bool = True,
    ) -> FloatArray:
        if q is not None:
            self.q = self._as_q(q)
            self._invalidate_cache()
            use_cache = False

        if not use_cache or not self._fk_valid or self._T_wf is None:
            self._ensure_fk()

        T_wf = self._T_wf if self._T_wf is not None else self._stack_joint_placements()
        if i is not None:
            self._validate_index(i)
            return T_wf[: i + 2]
        return T_wf


    def batch_forward_kinematics(self, Q: FloatArray) -> FloatArray:
        Q_arr = np.asarray(Q, dtype=npu.dtype)
        if Q_arr.ndim != 2 or Q_arr.shape[1] != self.nq:
            raise ValueError(f"Q must be (N, {self.nq}), got {Q_arr.shape}.")
        T_wf = np.empty((Q_arr.shape[0], self.model.njoints, 4, 4), dtype=npu.dtype)
        for i in range(Q_arr.shape[0]):
            self.step(q=Q_arr[i])
            T_wf[i] = self.forward_kinematics(use_cache=False)
        return T_wf


    def frame_transform(self, frame: str | int) -> FloatArray:
        frame_id = self._resolve_frame_id(frame)
        self._ensure_fk()
        return self._se3_to_matrix(self.data.oMf[frame_id])


    def ee_transform(self) -> FloatArray:
        if self._ee_frame_id is None:
            raise ValueError("ee_frame is not configured for this model.")
        return self.frame_transform(self._ee_frame_id)


    def inertia_matrix(self, q: FloatArray | None = None) -> FloatArray:
        if q is not None:
            self.step(q=q)
        M = pin.crba(self.model, self.data, self.q)
        return 0.5 * (M + M.T)


    def gravity_vector(self, q: FloatArray | None = None) -> FloatArray:
        if q is not None:
            self.step(q=q)
        return pin.computeGeneralizedGravity(self.model, self.data, self.q)


    def rnea(self) -> FloatArray:
        return pin.rnea(self.model, self.data, self.q, self.qd, self.qdd)


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


    def _ensure_fk(self) -> None:
        if self._fk_valid:
            return
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        self._T_wf = self._stack_joint_placements()
        self._fk_valid = True


    def _invalidate_cache(self) -> None:
        self._fk_valid = False
        self._J_valid = False
        self._com_valid = False
        self._Ic_valid = False
        self._J_com_valid = False


    def _stack_joint_placements(self) -> FloatArray:
        T_wf = np.empty((self.model.njoints, 4, 4), dtype=npu.dtype)
        for joint_id in range(self.model.njoints):
            T_wf[joint_id] = self._se3_to_matrix(self.data.oMi[joint_id])
        return T_wf


    def _resolve_frame_id(self, frame: str | int) -> int:
        if isinstance(frame, int):
            return frame
        if not self.model.existFrame(frame):
            raise ValueError(f"Unknown frame '{frame}'.")
        return self.model.getFrameId(frame)


    def _validate_index(self, i: int) -> None:
        if i < 0 or i >= self.n:
            raise ValueError(f"Index {i} out of range for n={self.n}.")

    @staticmethod
    def _pin_to_linear_angular(J: FloatArray) -> FloatArray:
        return np.vstack((J[3:, :], J[:3, :]))

    @staticmethod
    def _se3_to_matrix(se3: pin.SE3) -> FloatArray:
        if hasattr(se3, "homogeneous"):
            return np.array(se3.homogeneous, dtype=npu.dtype)
        return np.array(se3.toHomogeneousMatrix(), dtype=npu.dtype)
