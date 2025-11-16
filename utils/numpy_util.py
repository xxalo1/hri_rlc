from __future__ import annotations

from typing import Any, Optional, Sequence, TypeVar
import torch
import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.floating]
dtype = np.float64

ArrayT = TypeVar("ArrayT", FloatArray, torch.Tensor)

def to_array(a: Sequence[float]) -> np.ndarray:
    """Convert input to a NumPy array if it is not already one."""
    return np.asarray(a, dtype=dtype).reshape(-1)


def to_n_array(x, n: int) -> FloatArray:
    """Convert input to a NumPy array of length n.
    If input is a scalar or size 1, repeat to length n.
    """
    arr = to_array(x)
    if arr.size == n:
        return arr.reshape(n)
    return np.full(n, arr.item(), dtype=dtype)

from typing import Mapping, Sequence
import numpy as np  # only for typing/context; `to_array` does the conversion

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
