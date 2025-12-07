from dataclasses import dataclass, field
import numpy as np

from . import quintic as qt

from common_utils import numpy_util as npu
from common_utils import FloatArray


@dataclass(slots=True)
class JointTraj:
    """
    Joint-space trajectory container.

    Attributes
    ----------
    t : ndarray, shape (N,)
        1D array of sample times in seconds, shape (N,).
        Can be absolute time (e.g. simulation clock) or time-from-start,
        depending on the planner that produced it. Samples are assumed to be
        in increasing order.
    q : ndarray, shape (N, n) 
        Joint positions. Each row q[k] is the n-dimensional
        joint position vector at time t[k].
    qd : ndarray, shape (N, n)
        Joint velocities. Each row qd[k] is the joint velocity
        vector at time t[k].
    qdd : ndarray, shape (N, n)
        Joint accelerations. Each row qdd[k] is the joint
        acceleration vector at time t[k].

    Notes
    -----
    All arrays share the same number of samples N and joint dimension n.
    For the default-constructed object, these arrays are empty.
    """
    t: FloatArray = field(
        default_factory=lambda: np.zeros(0, dtype=npu.dtype)
    )  # (N,)

    q: FloatArray = field(
        default_factory=lambda: np.zeros((0, 0), dtype=npu.dtype)
    )  # (N, n)

    qd: FloatArray = field(
        default_factory=lambda: np.zeros((0, 0), dtype=npu.dtype)
    )  # (N, n)

    qdd: FloatArray = field(
        default_factory=lambda: np.zeros((0, 0), dtype=npu.dtype)
    )  # (N, n)



@dataclass(slots=True)
class CartesianTraj:
    """
    Cartesian trajectory container.

    Attributes
    ----------
    t : ndarray, shape (N,)
        1D array of sample times in seconds.
        Can be absolute time (e.g. simulation clock) or time-from-start,
        depending on the planner that produced it. Samples are assumed to be
        in increasing order.

    pose : ndarray, shape (N, 7)
        pose in the world frame. Each row pose[k] is the
        [x, y, z, qw, qx, qy, qz] pose at time t[k].

    twist : ndarray, shape (N, 6)
        Spatial velocity of the pose at time t[k], stored as
        [vx, vy, vz, wx, wy, wz] where v is linear velocity and w is angular
        velocity in the world frame.

    acc : ndarray, shape (N, 6)
        Spatial acceleration of the pose at time t[k], stored as
        [ax, ay, az, awx, awy, awz], with a linear part and an angular part.

    Notes
    -----
    All arrays share the same number of samples N. For the default-constructed
    object, these arrays are empty.
    """

    t: FloatArray = field(
        default_factory=lambda: np.zeros(0, dtype=npu.dtype)
    )  # (N,)

    pose: FloatArray = field(
        default_factory=lambda: np.zeros((0, 7), dtype=npu.dtype)
    )  # (N, 7)


    twist: FloatArray = field(
        default_factory=lambda: np.zeros((0, 6), dtype=npu.dtype)
    )  # (N, 6)

    acc: FloatArray = field(
        default_factory=lambda: np.zeros((0, 6), dtype=npu.dtype)
    )  # (N, 6)


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
        duration: float, 
        freq: float,
        v0: FloatArray | None = None,
        a0: FloatArray | None = None,
        vf: FloatArray | None = None,
        af: FloatArray | None = None
    ) -> JointTraj:
        """
        Compute quintic polynomial trajectories between q0 and qf. 

        Parameters
        ----------
        q0: ndarray. shape (n,)
            Initial joint positions.
        qf: ndarray. shape (n,)
            Final joint positions.
        duration: float
            Duration of the trajectory.
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
        t0 = 0.0

        if not v0:
            v0 = npu.to_n_array(0.0, n)
        if not a0:
            a0 = npu.to_n_array(0.0, n)
        if not vf:
            vf = npu.to_n_array(0.0, n)
        if not af:
            af = npu.to_n_array(0.0, n)

        Q, Qd, Qdd, T = qt.quintic_trajs(q0, qf, t0, duration, freq, v0, a0, vf, af)
        joint_traj = JointTraj(t=T, q=Q, qd=Qd, qdd=Qdd)
        return joint_traj