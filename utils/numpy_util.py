from __future__ import annotations

from typing import Any, Optional, Sequence

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float32]
dtype = np.float32

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