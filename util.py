import numpy as np
from typing import Optional, Sequence
def to_array(a: Sequence[float]) -> np.ndarray:
    """Convert input to a NumPy array if it is not already one."""
    return np.asarray(a, dtype=np.float32).reshape(-1)