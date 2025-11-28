import numpy as np

from . import quintic as qt

from common_utils import numpy_util as npu
from common_utils import FloatArray


class TrajPlanner:
    """Trajectory class for robot joint trajectories."""
    def __init__(self) -> None:
        """
        Initialize the trajectory with given waypoints and duration.

        Args:
        """
        
    def quintic_trajs(self, 
        q0: FloatArray, 
        qf: FloatArray, 
        t0: float, 
        tf: float, 
        freq: float,
        v0: FloatArray | None = None,
        a0: FloatArray | None = None,
        vf: FloatArray | None = None,
        af: FloatArray | None = None
    ) -> FloatArray:
        """
        Compute quintic polynomial trajectories between q0 and qf. 

        Parameters
        ----------
        q0: ndarray. shape (n,)
            Initial joint positions.
        qf: ndarray. shape (n,)
            Final joint positions.
        t0: float
            Start time.
        tf: float
            End time.
        freq: float
            Frequency of trajectory points.
        v0: ndarray or None. shape (n,), optional
            Initial joint velocities. Defaults to zero if None.
        a0: ndarray or None. shape (n,), optional
            Initial joint accelerations. Defaults to zero if None.
        vf: ndarray or None. shape (n,), optional
            Final joint velocities. Defaults to zero if None.
        af: ndarray or None. shape (n,), optional
            Final joint accelerations. Defaults to zero if None.

        Returns
        ----------
        T: ndarray, shape (3, N, n)
            quintic_trajectory sampled at frequency `freq` 
            where q is at index 0 qd at index 1 and qdd at
            index 2.
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

        Q, Qd, Qdd = qt.quintic_trajs(q0, qf, t0, tf, freq, v0, a0, vf, af)
        T = np.stack([Q, Qd, Qdd], axis=0)
        return T