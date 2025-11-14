from __future__ import annotations

import numpy as np
import torch

from functools import singledispatch
from typing import Any, TypeVar, overload

from . import numpy_util as npu
from . import pytorch_util as ptu
FloatArray = npu.FloatArray
Shape = int | tuple[int, ...]
Arr = TypeVar("Arr", FloatArray, torch.Tensor)
# ----------------------------- trigonometry -----------------------------

@overload
def cos(x: FloatArray) -> FloatArray: ...
@overload
def cos(x: torch.Tensor) -> torch.Tensor: ...
@singledispatch
def cos(x: Any):
    raise NotImplementedError(f'cos not implemented for type {type(x)}')

@cos.register(np.ndarray)  # type: ignore[attr-defined]
def _(x: FloatArray) -> FloatArray:
    return np.cos(x)

@cos.register(torch.Tensor)  # type: ignore[attr-defined]
def _(x: torch.Tensor) -> torch.Tensor:
    return torch.cos(x)


@overload
def sin(x: FloatArray) -> FloatArray: ...
@overload
def sin(x: torch.Tensor) -> torch.Tensor: ...
@singledispatch
def sin(x: Any):
    raise NotImplementedError(f'sin not implemented for type {type(x)}')

@sin.register(np.ndarray)  # type: ignore[attr-defined]
def _(x: FloatArray) -> FloatArray:
    return np.sin(x)

@sin.register(torch.Tensor)  # type: ignore[attr-defined]
def _(x: torch.Tensor) -> torch.Tensor:
    return torch.sin(x)


@overload
def tan(x: FloatArray) -> FloatArray: ...
@overload
def tan(x: torch.Tensor) -> torch.Tensor: ...
@singledispatch
def tan(x: Any):
    raise NotImplementedError(f'tan not implemented for type {type(x)}')

@tan.register(np.ndarray)  # type: ignore[attr-defined]
def _(x: FloatArray) -> FloatArray:
    return np.tan(x)

@tan.register(torch.Tensor)  # type: ignore[attr-defined]
def _(x: torch.Tensor) -> torch.Tensor:
    return torch.tan(x)


# ----------------------------- elementwise ------------------------------

@overload
def exp(x: FloatArray) -> FloatArray: ...
@overload
def exp(x: torch.Tensor) -> torch.Tensor: ...
@singledispatch
def exp(x: Any):
    raise NotImplementedError(f'exp not implemented for type {type(x)}')

@exp.register(np.ndarray)  # type: ignore[attr-defined]
def _(x: FloatArray) -> FloatArray:
    return np.exp(x)

@exp.register(torch.Tensor)  # type: ignore[attr-defined]
def _(x: torch.Tensor) -> torch.Tensor:
    return torch.exp(x)


@overload
def log(x: FloatArray) -> FloatArray: ...
@overload
def log(x: torch.Tensor) -> torch.Tensor: ...
@singledispatch
def log(x: Any):
    raise NotImplementedError(f'log not implemented for type {type(x)}')

@log.register(np.ndarray)  # type: ignore[attr-defined]
def _(x: FloatArray) -> FloatArray:
    return np.log(x)

@log.register(torch.Tensor)  # type: ignore[attr-defined]
def _(x: torch.Tensor) -> torch.Tensor:
    return torch.log(x)


@overload
def sqrt(x: FloatArray) -> FloatArray: ...
@overload
def sqrt(x: torch.Tensor) -> torch.Tensor: ...
@singledispatch
def sqrt(x: Any):
    raise NotImplementedError(f'sqrt not implemented for type {type(x)}')

@sqrt.register(np.ndarray)  # type: ignore[attr-defined]
def _(x: FloatArray) -> FloatArray:
    return np.sqrt(x)

@sqrt.register(torch.Tensor)  # type: ignore[attr-defined]
def _(x: torch.Tensor) -> torch.Tensor:
    return torch.sqrt(x)


@overload
def abs_(x: FloatArray) -> FloatArray: ...
@overload
def abs_(x: torch.Tensor) -> torch.Tensor: ...
@singledispatch
def abs_(x: Any):
    raise NotImplementedError(f'abs not implemented for type {type(x)}')

@abs_.register(np.ndarray)  # type: ignore[attr-defined]
def _(x: FloatArray) -> FloatArray:
    return np.abs(x)

@abs_.register(torch.Tensor)  # type: ignore[attr-defined]
def _(x: torch.Tensor) -> torch.Tensor:
    return torch.abs(x)


# ------------------------------ reductions ------------------------------

@overload
def matmul(a: FloatArray, b: FloatArray) -> FloatArray: ...
@overload
def matmul(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor: ...
@singledispatch
def matmul(a: Any, b: Any):
    raise NotImplementedError(f'matmul not implemented for type {type(a)}')

@matmul.register(np.ndarray)  # type: ignore[attr-defined]
def _(a: FloatArray, b: FloatArray) -> FloatArray:
    return np.matmul(a, b)

@matmul.register(torch.Tensor)  # type: ignore[attr-defined]
def _(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    return torch.matmul(a, b)


# ---------------------------- shape utilities ---------------------------

@overload
def concatenate(a: FloatArray, *rest: FloatArray, axis: int | None = None) -> FloatArray: ...
@overload
def concatenate(a: torch.Tensor, *rest: torch.Tensor, axis: int | None = None) -> torch.Tensor: ...
@singledispatch
def concatenate(a: Any, *rest: Any, axis: int | None = None):
    raise NotImplementedError(f'concatenate not implemented for type {type(a)}')

@concatenate.register(np.ndarray)  # type: ignore[attr-defined]
def _(a: FloatArray, *rest: FloatArray, axis: int | None = None) -> FloatArray:
    return np.concatenate((a, *rest), axis=axis)

@concatenate.register(torch.Tensor)  # type: ignore[attr-defined]
def _(a: torch.Tensor, *rest: torch.Tensor, axis: int | None = None) -> torch.Tensor:
    return torch.cat((a, *rest), dim=axis)


@overload
def stack(a: FloatArray, *rest: FloatArray, axis: int = 0) -> FloatArray: ...
@overload
def stack(a: torch.Tensor, *rest: torch.Tensor, axis: int = 0) -> torch.Tensor: ...
@singledispatch
def stack(a: Any, *rest: Any, axis: int = 0):
    raise NotImplementedError(f'stack not implemented for type {type(a)}')

@stack.register(np.ndarray)  # type: ignore[attr-defined]
def _(a: FloatArray, *rest: FloatArray, axis: int = 0) -> FloatArray:
    return np.stack((a, *rest), axis=axis)

@stack.register(torch.Tensor)  # type: ignore[attr-defined]
def _(a: torch.Tensor, *rest: torch.Tensor, axis: int = 0) -> torch.Tensor:
    return torch.stack((a, *rest), dim=axis)

# ------------------------------- einsum ---------------------------------

@overload
def einsum(subscripts: str, *operands: FloatArray) -> FloatArray: ...
@overload
def einsum(subscripts: str, *operands: torch.Tensor) -> torch.Tensor: ...
def einsum(subscripts: str, *operands):
    """Dispatch by inspecting the first array/tensor among operands."""
    for op in operands:
        if isinstance(op, np.ndarray):
            return np.einsum(subscripts, *operands)
        if isinstance(op, torch.Tensor):
            return torch.einsum(subscripts, *operands)
    raise TypeError("einsum expects at least one ndarray or torch.Tensor")

# ------------------------------- zeros_like --------------------------------

@overload
def zeros_like(x: FloatArray, shape: Shape | None = None) -> FloatArray: ...
@overload
def zeros_like(x: torch.Tensor, shape: Shape | None = None) -> torch.Tensor: ...
@singledispatch
def zeros_like(x: Any, shape: Shape | None = None):
    raise NotImplementedError(f'zeros_like not implemented for type {type(x)}')

@zeros_like.register(np.ndarray)  # type: ignore[attr-defined]
def _(x: FloatArray, shape: Shape | None = None) -> FloatArray:
    return np.zeros_like(x, shape=shape)

@zeros_like.register(torch.Tensor)  # type: ignore[attr-defined]
def _(x: torch.Tensor, shape: Shape | None = None) -> torch.Tensor:
    if shape is None: return torch.zeros_like(x)
    else: return torch.zeros(shape, dtype=x.dtype, device=x.device)

# --------------------------------- zeros -----------------------------------

@overload
def zeros(like: FloatArray, shape: Shape) -> FloatArray: ...
@overload
def zeros(like: torch.Tensor, shape: Shape) -> torch.Tensor: ...
@singledispatch
def zeros(like: Any, shape: Shape):
    raise NotImplementedError(f'zeros not implemented for type {type(like)}')

@zeros.register(np.ndarray)  # type: ignore[attr-defined]
def _(like: FloatArray, shape: Shape) -> FloatArray:
    return np.zeros(shape, dtype=like.dtype)

@zeros.register(torch.Tensor)  # type: ignore[attr-defined]
def _(like: torch.Tensor, shape: Shape) -> torch.Tensor:
    return torch.zeros(shape, dtype=like.dtype, device=like.device)


# ------------------------------- empty_like --------------------------------

@overload
def empty_like(x: FloatArray) -> FloatArray: ...
@overload
def empty_like(x: torch.Tensor) -> torch.Tensor: ...
@singledispatch
def empty_like(x: Any):
    raise NotImplementedError(f'empty_like not implemented for type {type(x)}')

@empty_like.register(np.ndarray)  # type: ignore[attr-defined]
def _(x: FloatArray) -> FloatArray:
    return np.empty_like(x)

@empty_like.register(torch.Tensor)  # type: ignore[attr-defined]
def _(x: torch.Tensor) -> torch.Tensor:
    return torch.empty_like(x)


# ---------------------------------- empty -----------------------------------

@overload
def empty(like: FloatArray, shape: Shape) -> FloatArray: ...
@overload
def empty(like: torch.Tensor, shape: Shape) -> torch.Tensor: ...
@singledispatch
def empty(like: Any, shape: Shape):
    raise NotImplementedError(f'empty not implemented for type {type(like)}')

@empty.register(np.ndarray)  # type: ignore[attr-defined]
def _(like: FloatArray, shape: Shape) -> FloatArray:
    return np.empty(shape, dtype=like.dtype)

@empty.register(torch.Tensor)  # type: ignore[attr-defined]
def _(like: torch.Tensor, shape: Shape) -> torch.Tensor:
    return torch.empty(shape, dtype=like.dtype, device=like.device)


# ----------------------------------- eye ------------------------------------

@overload
def eye(like: FloatArray, n: int, m: int | None = None) -> FloatArray: ...
@overload
def eye(like: torch.Tensor, n: int, m: int | None = None) -> torch.Tensor: ...
@singledispatch
def eye(like: Any, n: int, m: int | None = None):
    raise NotImplementedError(f'eye not implemented for type {type(like)}')

@eye.register(np.ndarray)  # type: ignore[attr-defined]
def _(like: FloatArray, n: int, m: int | None = None) -> FloatArray:
    return np.eye(n, m, dtype=like.dtype)

@eye.register(torch.Tensor)  # type: ignore[attr-defined]
def _(like: torch.Tensor, n: int, m: int | None = None) -> torch.Tensor:
    return torch.eye(n, m if m is not None else n, dtype=like.dtype, device=like.device)

@overload
def eye_like(like: FloatArray, n: int, m: int | None = None) -> FloatArray: ...
@overload
def eye_like(like: torch.Tensor, n: int, m: int | None = None) -> torch.Tensor: ...
@singledispatch
def eye_like(like: Any, n: int, m: int | None = None):
    raise NotImplementedError(f'eye_like not implemented for type {type(like)}')

@eye_like.register(np.ndarray)  # type: ignore[attr-defined]
def _(like: FloatArray, n: int, m: int | None = None) -> FloatArray:
    return np.eye(n, m, dtype=like.dtype)

@eye_like.register(torch.Tensor)  # type: ignore[attr-defined]
def _(like: torch.Tensor, n: int, m: int | None = None) -> torch.Tensor:
    return torch.eye(n, m if m is not None else n, dtype=like.dtype, device=like.device)

# ------------------------------------ cross -----------------------------------

@overload
def cross(a: FloatArray, b: FloatArray, dim: int | None = None) -> FloatArray: ...
@overload
def cross(a: torch.Tensor, b: torch.Tensor, dim: int | None = None) -> torch.Tensor: ...
@singledispatch
def cross(a: Any, b: Any, dim: int | None = None):
    raise NotImplementedError(f'cross not implemented for type {type(a)}')
@cross.register(np.ndarray)  # type: ignore[attr-defined]
def _(a: FloatArray, b: FloatArray, dim: int | None = None) -> FloatArray:
    return np.cross(a, b, axis=dim)
@cross.register(torch.Tensor)  # type: ignore[attr-defined]
def _(a: torch.Tensor, b: torch.Tensor, dim: int | None = None) -> torch.Tensor:
    return torch.cross(a, b, dim=dim)