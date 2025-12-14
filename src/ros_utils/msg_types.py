from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field


from rbt_core.planning import CartesianTraj, JointTraj
from common_utils import FloatArray
from common_utils import numpy_util as npu 


@dataclass(slots=True)
class JointStateActionData():
    positions: FloatArray
    velocities: FloatArray
    actions: FloatArray
    actions_baseline: FloatArray
    stamp: float
    joint_names: list[str]
    
@dataclass(slots=True)
class JointStateData():
    positions: FloatArray
    velocities: FloatArray
    efforts: FloatArray | None
    stamp: float
    joint_names: list[str]

@dataclass(slots=True)
class JointEffortCommandData():
    efforts: FloatArray
    stamp: float
    joint_names: list[str]

@dataclass(slots=True)
class JointControllerStateData():
    """
    Parsed representation of a JointTrajectoryControllerState message.
    """
    stamp: float
    joint_names: list[str]
    tau: FloatArray
    q: FloatArray
    q_des: FloatArray
    qd: FloatArray | None
    qd_des: FloatArray | None
    qdd: FloatArray | None
    qdd_des: FloatArray | None
    e: FloatArray | None
    ed: FloatArray | None

@dataclass(slots=True)
class JointTrajectoryData:
    """
    Parsed representation of a JointTrajectory message.

    All arrays are NumPy arrays (dtype=npu.dtype). If velocities or
    accelerations are not present in the message, they are filled with
    zeros of shape positions.shape.
    """
    stamp: float
    joint_names: list[str]
    time_from_start: FloatArray        # (N,)
    positions: FloatArray              # (N, n)
    velocities: FloatArray | None      # (N, n) or None
    accelerations: FloatArray | None   # (N, n) or None
    _traj: JointTraj | None = field(init=False, default=None)

    def __post_init__(self) -> None:
        if self.velocities is None:
            self.velocities = np.zeros(self.shape, dtype=self.dtype)

        if self.accelerations is None:
            self.accelerations = np.zeros(self.shape, dtype=self.dtype)

        self._traj = JointTraj(
            t=self.time_from_start,
            q=self.positions,
            qd=self.velocities,
            qdd=self.accelerations,
        )

    @property
    def traj(self) -> JointTraj:
        return self._traj # type: ignore

    @property
    def shape(self) -> tuple[int, ...]:
        return self.positions.shape

    @property
    def dtype(self) -> np.dtype:
        return self.positions.dtype

@dataclass(slots=True)
class PoseArrayData():
    """
    Parsed representation of a PoseArray message.

    All arrays are NumPy arrays (dtype=npu.dtype).
    """
    stamp: float
    frame_id: str
    poses: FloatArray  # (N, 7) [x, y, z, qw, qx, qy, qz]

