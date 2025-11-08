
import numpy as np
from typing import Any, Optional, Sequence
from numpy.typing import ArrayLike, NDArray

from . import quintic as qt
from ...utils import numpy_util as npu
FloatArray = npu.FloatArray
dtype = npu.dtype


class TrajPlanner:
    """Trajectory class for robot joint trajectories."""
    def __init__(self) -> None:
        """
        Initialize the trajectory with given waypoints and duration.

        Args:
        """
        
    def quintic_trajs(self, q0: FloatArray, qf: FloatArray, t0: float, tf: float, dt: float,
                      v0: FloatArray | None = None,
                      a0: FloatArray | None = None,
                      vf: FloatArray | None = None,
                      af: FloatArray | None = None
                      ) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray]:
        """
        Compute quintic polynomial coefficients and evaluate trajectories.  
        """
        n = len(q0)

        if not v0:
            v0 = npu.to_n_array(0.0, n)
        if not a0:
            a0 = npu.to_n_array(0.0, n)
        if not vf:
            vf = npu.to_n_array(0.0, n)
        if not af:
            af = npu.to_n_array(0.0, n)

        Q, Qd, Qdd, T = qt.quintic_trajs(q0, qf, t0, tf, dt, v0, a0, vf, af)
        return Q, Qd, Qdd, T