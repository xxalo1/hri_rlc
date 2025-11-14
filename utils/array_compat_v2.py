from __future__ import annotations
from re import A

import numpy as np
import torch

from typing import Any, Sequence, TypeVar, overload

from . import numpy_util as npu
FloatArray = npu.FloatArray
Shape = int | tuple[int, ...]
ArrayT = TypeVar("ArrayT", FloatArray, torch.Tensor)

def cos(x: ArrayT) -> ArrayT: 
    """Cosine function compatible with both numpy arrays and torch tensors."""
    if isinstance(x, np.ndarray):
        return np.cos(x)
    elif isinstance(x, torch.Tensor):
        return torch.cos(x)
    else:
        raise NotImplementedError(f'cos not implemented for type {type(x)}')


def sin(x: ArrayT) -> ArrayT: 
    """Sine function compatible with both numpy arrays and torch tensors."""
    if isinstance(x, np.ndarray):
        return np.sin(x)
    elif isinstance(x, torch.Tensor):
        return torch.sin(x)
    else:
        raise NotImplementedError(f'sin not implemented for type {type(x)}')


def tan(x: ArrayT) -> ArrayT: 
    """Tangent function compatible with both numpy arrays and torch tensors."""
    if isinstance(x, np.ndarray):
        return np.tan(x)
    elif isinstance(x, torch.Tensor):
        return torch.tan(x)
    else:
        raise NotImplementedError(f'tan not implemented for type {type(x)}')


def exp(x: ArrayT) -> ArrayT: 
    """Exponential function compatible with both numpy arrays and torch tensors."""
    if isinstance(x, np.ndarray):
        return np.exp(x)
    elif isinstance(x, torch.Tensor):
        return torch.exp(x)
    else:
        raise NotImplementedError(f'exp not implemented for type {type(x)}')


def log(x: ArrayT) -> ArrayT: 
    """Natural logarithm function compatible with both numpy arrays and torch tensors."""
    if isinstance(x, np.ndarray):
        return np.log(x)
    elif isinstance(x, torch.Tensor):
        return torch.log(x)
    else:
        raise NotImplementedError(f'log not implemented for type {type(x)}')


def sqrt(x: ArrayT) -> ArrayT: 
    """Square root function compatible with both numpy arrays and torch tensors."""
    if isinstance(x, np.ndarray):
        return np.sqrt(x)
    elif isinstance(x, torch.Tensor):
        return torch.sqrt(x)
    else:
        raise NotImplementedError(f'sqrt not implemented for type {type(x)}')


def abs(x: ArrayT) -> ArrayT: 
    """Absolute value function compatible with both numpy arrays and torch tensors."""
    if isinstance(x, np.ndarray):
        return np.abs(x)
    elif isinstance(x, torch.Tensor):
        return torch.abs(x)
    else:
        raise NotImplementedError(f'abs not implemented for type {type(x)}')


def matmul(a: ArrayT, b: ArrayT) -> ArrayT:
    """Matrix multiplication compatible with both numpy arrays and torch tensors."""
    if isinstance(a, np.ndarray) and isinstance(b, np.ndarray):
        return np.matmul(a, b)
    elif isinstance(a, torch.Tensor) and isinstance(b, torch.Tensor):
        return torch.matmul(a, b)
    else:
        raise NotImplementedError(f'matmul not implemented for types {type(a)} and {type(b)}')


def einsum(subscripts: str, *operands: ArrayT) -> ArrayT:
    """Dispatch by inspecting the first array/tensor among operands."""
    for op in operands:
        if isinstance(op, np.ndarray):
            return np.einsum(subscripts, *operands)
        if isinstance(op, torch.Tensor):
            return torch.einsum(subscripts, *operands)
    raise TypeError("einsum expects at least one ndarray or torch.Tensor")


def concatenate(arrays: Sequence[ArrayT], axis: int = 0) -> ArrayT:
    """Concatenate a list of arrays/tensors along the specified axis."""
    if all(isinstance(x, np.ndarray) for x in arrays):
        return np.concatenate(arrays, axis=axis)
    elif all(isinstance(x, torch.Tensor) for x in arrays):
        return torch.cat(arrays, dim=axis)
    else:
        raise TypeError("all inputs must be either numpy arrays or torch tensors")


def stack(arrays: Sequence[ArrayT], axis: int = 0) -> ArrayT:
    if all(isinstance(x, np.ndarray) for x in arrays):
        return np.stack(arrays, axis=axis)
    elif all(isinstance(x, torch.Tensor) for x in arrays):
        return torch.stack(arrays, dim=axis)
    else:
        raise TypeError("all inputs must be either numpy arrays or torch tensors")


def zeros_like(x: ArrayT, shape: Shape | None = None) -> ArrayT:
    """Create an array/tensor of zeros with the same type as x."""
    if isinstance(x, np.ndarray):
        return np.zeros_like(x, shape=shape)
    elif isinstance(x, torch.Tensor):
        if shape is None:
            return torch.zeros_like(x)
        else:
            return torch.zeros(shape, dtype=x.dtype, device=x.device)
    else:
        raise NotImplementedError(f'zeros_like not implemented for type {type(x)}')


def empty_like(x: ArrayT, shape: Shape | None = None) -> ArrayT:
    """Create an array/tensor of uninitialized values with the same type as x."""
    if isinstance(x, np.ndarray):
        return np.empty_like(x, shape=shape)
    elif isinstance(x, torch.Tensor):
        if shape is None:
            return torch.empty_like(x)
        else:
            return torch.empty(shape, dtype=x.dtype, device=x.device)
    raise NotImplementedError(f'empty_like not implemented for type {type(x)}')


def eye_like(x: ArrayT, n: int, m: int | None = None) -> ArrayT:
    """Create an identity matrix with the same type as like."""
    if isinstance(x, np.ndarray):
        return np.eye(n, M=m, dtype=x.dtype)
    elif isinstance(x, torch.Tensor):
        return torch.eye(n, m=m if m is not None else n, dtype=x.dtype, device=x.device)
    raise NotImplementedError(f'eye_like not implemented for type {type(x)}')


def array_like(x: ArrayT, data: Sequence[float]) -> ArrayT:
    """Create an array/tensor from data with the same type as x."""
    if isinstance(x, np.ndarray):
        return np.array(data, dtype=x.dtype)
    elif isinstance(x, torch.Tensor):
        return torch.tensor(data, dtype=x.dtype, device=x.device)
    raise NotImplementedError(f'array_like not implemented for type {type(x)}')


def cross(a: ArrayT, b: ArrayT, dim: int | None = None):
    """Cross product compatible with both numpy arrays and torch tensors."""
    if isinstance(a, np.ndarray) and isinstance(b, np.ndarray):
        return np.cross(a, b, axis=dim)
    elif isinstance(a, torch.Tensor) and isinstance(b, torch.Tensor):
        return torch.cross(a, b, dim=dim)
    raise NotImplementedError(f'cross not implemented for type {type(a)}')
