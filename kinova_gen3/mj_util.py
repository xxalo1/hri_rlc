import numpy as np
import torch
from typing import Any, Optional, Sequence
from numpy.typing import ArrayLike, NDArray
from ..utils import numpy_util as npu
from ..utils import pytorch_util as ptu
FloatArray = npu.FloatArray


def from_mj_numpy(
    d: dict[str, FloatArray | float],
    skip_last: bool = True
) -> tuple[FloatArray, FloatArray, float]:
    """
    Convert MuJoCo readings to the defined kinova_gen3 frames.
    For Kinova Gen3, this involves negating the joints.
    
    Parameters
    ----------
    d : dict[str, FloatArray]
        Dictionary containing MuJoCo readings with keys "qpos", "qvel", and "time".
    skip_last : bool, optional
        Whether to skip the last joint, by default True
    
    Returns
    -------
    q : FloatArray
        Processed joint positions.
    dq : FloatArray
        Processed joint velocities.
    t : float
        Timestamp.
    """
    
    q = d["qpos"].copy()
    dq = d["qvel"].copy()
    t = d["time"]
    if skip_last:
        q = -q[:-1]
        dq = -dq[:-1]
    q = np.concatenate(([0.0], -q))
    dq = np.concatenate(([0.0], -dq))
    return q, dq, t
    

def from_mj_torch(
    d: dict[str, FloatArray | float],
    skip_last: bool = True
) -> tuple[torch.Tensor, torch.Tensor, float]:
    """
    Convert MuJoCo readings to the defined kinova_gen3 frames using torch.
    For Kinova Gen3, this involves negating the joints.
    
    Parameters
    ----------
    d : dict[str, FloatArray | float]
        Dictionary containing MuJoCo readings with keys "qpos", "qvel", and "time".
    skip_last : bool, optional
        Whether to skip the last joint, by default True
    
    Returns
    -------
    q : torch.Tensor
        Processed joint positions.
    dq : torch.Tensor
        Processed joint velocities.
    t : float
        Timestamp.
    """
    
    q = d["qpos"].copy()
    dq = d["qvel"].copy()
    t = d["time"]
    if skip_last:
        q = -q[:-1]
        dq = -dq[:-1]
    q = np.concatenate(([0.0], -q))
    dq = np.concatenate(([0.0], -dq))
    q_t = ptu.from_numpy(q)
    dq_t = ptu.from_numpy(dq)
    return q_t, dq_t, t


def to_mj_torque(
    tau: FloatArray | torch.Tensor,
    skip_last: bool = True
)-> FloatArray:
    """
    Convert torques from kinova_gen3 frames to MuJoCo frames.
    For Kinova Gen3, this involves negating the joints.
    
    Parameters
    ----------
    tau : FloatArray | torch.Tensor
        Torques in kinova_gen3 frames.
    skip_last : bool, optional
        Whether to skip the last joint, by default True
    
    Returns
    -------
    tau_mj : FloatArray
        Torques in MuJoCo frames.
    """
    if isinstance(tau, torch.Tensor):
        tau = ptu.to_numpy(tau)
    tau_mj = tau.copy()
    if skip_last:
        tau_mj = np.concatenate((tau_mj, [0.0]))
    tau_mj = -tau_mj[1:]
    return tau_mj

def to_mj_q(
    q: FloatArray | torch.Tensor,
    skip_last: bool = True
) -> FloatArray:
    """
    Convert joint positions from kinova_gen3 frames to MuJoCo frames.
    For Kinova Gen3, this involves negating the joints.
    
    Parameters
    ----------
    q : FloatArray | torch.Tensor
        Joint positions in kinova_gen3 frames.
    skip_last : bool, optional
        Whether to skip the last joint, by default True
    
    Returns
    -------
    q_mj : FloatArray
        Joint positions in MuJoCo frames.
    """
    if isinstance(q, torch.Tensor):
        q = ptu.to_numpy(q)
    q_mj = q.copy()
    if skip_last:
        q_mj = np.concatenate((q_mj, [0.0]))
    q_mj = -q_mj[1:]
    return q_mj
