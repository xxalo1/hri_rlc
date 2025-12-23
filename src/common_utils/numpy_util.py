from __future__ import annotations

from typing import Sequence, TypeVar
import torch
import numpy as np
from numpy.typing import NDArray
from typing import Mapping, Sequence

FloatArray = NDArray[np.floating]
dtype = np.float64

ArrayT = TypeVar("ArrayT", FloatArray, torch.Tensor)

def to_array(a: Sequence) -> np.ndarray:
    """Convert input to a NumPy array if it is not already one."""
    return np.asarray(a, dtype=dtype)


def to_n_array(x, n: int) -> FloatArray:
    """Convert input to a NumPy array of length n.
    If input is a scalar or size 1, repeat to length n.
    """
    arr = to_array(x)
    if arr.size == n:
        return arr.reshape(n)
    return np.full(n, arr.item(), dtype=dtype)


def dict_to_arrays(data: Mapping[str, Sequence], keys: Sequence[str] | None = None) -> dict[str, np.ndarray]:
    """
    Convert a dict of lists to a dict of ndarrays using `to_array`.

    Parameters
    ----------
    data : dict[str, Sequence]
        Each value is a list/sequence of values.
    keys : sequence[str] | None
        Keys to convert. If None, convert all keys in `data`.

    Returns
    -------
    out : dict[str, np.ndarray]
        Same keys (or the selected subset), with values converted by `to_array`.
    """
    if keys is None:
        keys = list(data.keys())
    return {k: to_array(data[k]) for k in keys}


def check_array_shapes(
    arrays: Mapping[str, FloatArray],
    *,
    check_first_dim: bool = False,
    check_second_dim: bool = False,
    check_full_shape: bool = False,
    expected_ndim: int | None = None,
) -> None:
    """
    Validate that a set of arrays are shape compatible.

    Parameters
    ----------
    arrays : mapping str -> array_like
        Dict-like mapping from a descriptive name to an array.
        None values should be filtered out by the caller.
    check_first_dim : bool, default False
        If True, require that all arrays have the same size along axis 0.
    check_second_dim : bool, default False
        If True, require that all arrays have the same size along axis 1.
        Raises if any array has ndim < 2.
    check_full_shape : bool, default False
        If True, require that all arrays have exactly the same shape.
    expected_ndim : int, optional
        If not None, require that all arrays have this number of dimensions.

    Raises
    ------
    ValueError
        If shapes or dimensions are incompatible.
    """

    items = [(name, np.asarray(arr)) for name, arr in arrays.items()]
    if not items:
        return  # nothing to check

    # Check ndim if requested
    if expected_ndim is not None:
        for name, arr in items:
            if arr.ndim != expected_ndim:
                raise ValueError(
                    f"{name} must have ndim={expected_ndim}, "
                    f"got ndim={arr.ndim} with shape {arr.shape}."
                )

    ref_name, ref = items[0]
    ref_shape = ref.shape

    # Full shape equality
    if check_full_shape:
        for name, arr in items[1:]:
            if arr.shape != ref_shape:
                raise ValueError(
                    f"{name} shape {arr.shape} does not match "
                    f"{ref_name} shape {ref_shape}."
                )

    # First dimension
    if check_first_dim:
        if ref.ndim == 0:
            raise ValueError(
                f"Cannot check first dimension: {ref_name} is scalar with shape {ref_shape}."
            )
        ref_n0 = ref_shape[0]
        for name, arr in items[1:]:
            if arr.ndim == 0:
                raise ValueError(
                    f"Cannot check first dimension: {name} is scalar with shape {arr.shape}."
                )
            if arr.shape[0] != ref_n0:
                raise ValueError(
                    f"{name} first dimension ({arr.shape[0]}) does not match "
                    f"{ref_name} first dimension ({ref_n0})."
                )

    # Second dimension
    if check_second_dim:
        if ref.ndim < 2:
            raise ValueError(
                f"Cannot check second dimension: {ref_name} has ndim={ref.ndim} "
                f"with shape {ref_shape}."
            )
        ref_n1 = ref_shape[1]
        for name, arr in items[1:]:
            if arr.ndim < 2:
                raise ValueError(
                    f"Cannot check second dimension: {name} has ndim={arr.ndim} "
                    f"with shape {arr.shape}."
                )
            if arr.shape[1] != ref_n1:
                raise ValueError(
                    f"{name} second dimension ({arr.shape[1]}) does not match "
                    f"{ref_name} second dimension ({ref_n1})."
                )


def validate_array_shape(
    arr: FloatArray,
    expected_shape: Sequence[int],
    arr_name: str = "array",
) -> None:
    """
    Validate that an array has the expected shape.

    Parameters
    ----------
    arr : ndarray
        Array to check.
    expected_shape : sequence[int]
        Expected shape.
    arr_name : str, default "array"
        Name of the array for error messages.

    Raises
    ------
    ValueError
        If the array does not have the expected shape.
    """
    arr_shape = arr.shape
    if arr_shape != tuple(expected_shape):
        raise ValueError(
            f"{arr_name} has shape {arr_shape}, expected {expected_shape}."
        )