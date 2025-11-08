from __future__ import annotations

import numpy as np
import torch
from typing import Any, Optional, Sequence, Callable
from numpy.typing import ArrayLike, NDArray


from .kin import kin, kin_t
from ..utils import numpy_util as npu
from ..utils import pytorch_util as ptu
from .traj import quintic as tr
from dynamics_model import kinematics

FloatArray = npu.FloatArray
dtype = npu.dtype

class robot:
    def __init__(self, kinematics_model: kinematics, trajectory_model: traj) -> None:
        """
        d, a, alpha: 1D array (length n)
        theta0: optional 1D array of fixed offsets (length n), defaults to zeros
        b: optional 1D array of translations along the new z axis (length n)
        axes_o: optional 2D array (3x3), axes of base frame in world coords, defaults to identity
        inertia: optional dict of manipulator iertias coms masses 
        o_0: optional 1d array, origin of base frame in world coords, defaults to [0,0,0]
        """
        self.kinematics = kinematics_model