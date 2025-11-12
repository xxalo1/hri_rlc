from __future__ import annotations
from typing import Any, Literal, Optional, Sequence, TypeVar, Generic

import numpy as np
import torch

from numpy import ndarray
from torch import Tensor

from typing import overload
class BackendError(AssertionError):
    """Raised when backend/dtype/device assertions fail."""
    pass


def _is_numpy(x: Any) -> bool:
    return isinstance(x, np.ndarray)


def _is_torch(x: Any) -> bool:
    return isinstance(x, torch.Tensor)

Arr = TypeVar("Arr", np.ndarray, torch.Tensor)
TArr = TypeVar("TArr", ndarray, Tensor)

class XP(Generic[Arr]):
    """
    Minimal NumPy↔PyTorch backend (single class, readable, extensible).

    • Infer backend/dtype/device from inputs (see `from_inputs`).
    • Keep math code free of np/torch branches — call XP methods instead.
    • Catch mistakes early with `assert_consistent`, `assert_backend`, etc.

    Default fallback (when no arrays provided) is NumPy float64.

    Typical use:
        xp = XP.from_inputs(q, qd)
        xp.assert_consistent(q, qd)
        M  = xp.einsum('ikn,i,ikm->nm', Jc, m, Jc)
    """


    def __init__(self,
                 kind: Literal["np", "torch"],
                 dtype: Any,
                 device: str | None = None):
        self.kind: Literal["np", "torch"] = kind
        self.dtype: Any = dtype
        self.device = device


    @overload
    @classmethod
    def from_inputs(cls, *xs: ndarray) -> "XP[ndarray]": ...
    @overload
    @classmethod
    def from_inputs(cls, *xs: Tensor) -> "XP[Tensor]": ...
    @classmethod
    def from_inputs(cls, *xs: ndarray | Tensor) -> "XP":
        for x in xs:
            if _is_torch(x):
                return cls("torch", dtype=x.dtype, device=str(x.device))
            if _is_numpy(x):
                return cls("np", dtype=x.dtype)
        raise ValueError("Xs kind must be 'np' or 'torch'")


    def __repr__(self) -> str:
        return f"XP(kind={self.kind}, dtype={self.dtype}, device={self.device})"

    @overload
    def zeros(self: "XP[ndarray]", shape: Sequence[int]) -> ndarray: ...
    @overload
    def zeros(self: "XP[Tensor]",  shape: Sequence[int]) -> Tensor: ...
    def zeros(self, shape):
        """All-zeros array/tensor with configured dtype/device."""
        if self.kind == "np":
            return np.zeros(shape, dtype=self.dtype)
        return torch.zeros(shape, dtype=self.dtype, device=self.device)


    def ones(self, shape: Sequence[int]) -> Any:
        """All-ones array/tensor with configured dtype/device."""
        if self.kind == "np":
            return np.ones(shape, dtype=self.dtype)
        return torch.ones(shape, dtype=self.dtype, device=self.device)


    def eye(self, n: int) -> Any:
        """Identity matrix with configured dtype/device."""
        if self.kind == "np":
            return np.eye(n, dtype=self.dtype)
        return torch.eye(n, dtype=self.dtype, device=self.device)


    def zeros_like(self, x: Arr) -> Arr:
        """
        Zeros with `x`'s shape, returned in THIS backend.

        If `x` is same backend, preserves its dtype/device;
        otherwise uses this XP's dtype/device.
        """
        if self.kind == "np":
            return np.zeros_like(x)
        return torch.zeros_like(x)


    def empty_like(self, x: Any) -> Any:
        """Uninitialized like `x`, returned in THIS backend."""
        if self.kind == "np":
            return np.empty_like(x)
        return torch.empty_like(x)


    def transpose(self, x: Any, axes: Optional[Sequence[int]] = None) -> Any:
        """
        Transpose/permute axes.
        NumPy: np.transpose(x, axes) or x.T if axes is None (2D).
        Torch: x.permute(*axes); if axes is None, mT for 2D or reversed axes.
        """
        if self.kind == "np":
            return np.transpose(x, axes) if axes is not None else x.T
        if axes is None:
            return x.mT if x.ndim == 2 else x.permute(*range(x.ndim - 1, -1, -1))
        return x.permute(*axes)


    def swapaxes(self, x: Any, a: int, b: int) -> Any:
        """Swap two axes."""
        if self.kind == "np":
            return np.swapaxes(x, a, b)
        return x.swapaxes(a, b)


    def stack(self, xs: Sequence[Any], axis: int = 0) -> Any:
        """Stack sequence along a new axis (backend-checked)."""
        if self.kind == "np":
            return np.stack(xs, axis=axis)
        return torch.stack(xs, dim=axis)


    def concat(self, xs: Sequence[Any], axis: int = 0) -> Any:
        """Concatenate sequence along an existing axis (backend-checked)."""
        if self.kind == "np":
            return np.concatenate(xs, axis=axis)
        return torch.cat(xs, dim=axis)


    def einsum(self, eq: str, *ops: Any) -> Any:
        """Einstein summation (backend-checked)."""
        if self.kind == "np":
            return np.einsum(eq, *ops)
        return torch.einsum(eq, *ops)


    def sin(self, x: Any) -> Any:
        """Elementwise sine."""
        return np.sin(x) if self.kind == "np" else torch.sin(x)


    def cos(self, x: Any) -> Any:
        """Elementwise cosine."""
        return np.cos(x) if self.kind == "np" else torch.cos(x)


    def exp(self, x: Any) -> Any:
        """Elementwise exponential."""
        return np.exp(x) if self.kind == "np" else torch.exp(x)


    def log(self, x: Any) -> Any:
        """Elementwise natural log."""
        return np.log(x) if self.kind == "np" else torch.log(x)


    def sqrt(self, x: Any) -> Any:
        """Elementwise square root."""
        return np.sqrt(x) if self.kind == "np" else torch.sqrt(x)


    def norm(self, x: Any, axis: Optional[int] = None) -> Any:
        """Vector/matrix norm along axis."""
        if self.kind == "np":
            return np.linalg.norm(x, axis=axis)
        return torch.linalg.norm(x, dim=axis)


    def cross(self, a: Any, b: Any, axis: int = -1) -> Any:
        """3D cross product along axis/dim."""
        if self.kind == "np":
            return np.cross(a, b, axis=axis)
        return torch.cross(a, b, dim=axis)


    def clip(self, x: Any, x_min: float, x_max: float) -> Any:
        """Elementwise clamp to [x_min, x_max]."""
        if self.kind == "np":
            return np.clip(x, x_min, x_max)
        return torch.clamp(x, min=x_min, max=x_max)


    def assert_backend(self, *xs: Any) -> None:
        """
        Ensure each array/tensor belongs to THIS backend.
        Non-array inputs (scalars/lists) are ignored.
        """
        for x in xs:
            if self.kind == "np" and _is_torch(x):
                raise BackendError("Torch tensor passed to NumPy backend path.")
            if self.kind == "torch" and _is_numpy(x):
                raise BackendError("NumPy array passed to Torch backend path.")
    

    def assert_consistent(self, *xs: Any) -> None:
        """
        Ensure all arrays/tensors belong to the SAME backend/dtype/device.
        Non-array inputs (scalars/lists) are ignored.
        """
        backend = None
        dtype = None
        device = None
        for x in xs:
            if _is_numpy(x):
                if backend is None:
                    backend = "np"
                    dtype = x.dtype
                elif backend != "np":
                    raise BackendError("Inconsistent backends among inputs.")
                elif x.dtype != dtype:
                    raise BackendError("Inconsistent dtypes among inputs.")
            elif _is_torch(x):
                if backend is None:
                    backend = "torch"
                    dtype = x.dtype
                    device = str(x.device)
                elif backend == "torch":
                    raise BackendError("Inconsistent backends among inputs.")
                elif x.dtype != dtype:
                    raise BackendError("Inconsistent dtypes among inputs.")
                elif str(x.device) != device:
                    raise BackendError("Inconsistent devices among inputs.")