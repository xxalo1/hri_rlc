from __future__ import annotations
import numpy as np

from ...sim_backend.mujoco.mujoco_base import BaseMujocoEnv
from ...common_utils import numpy_util as npu
from ...common_utils import FloatArray

class Gen3Env(BaseMujocoEnv):
    def __init__(self, xml_path, nsubsteps=1, seed: int | None = 0):
        super().__init__(xml_path, nsubsteps, seed)

        self.joint_names = ["joint_1", "joint_2", "joint_3",
                            "joint_4", "joint_5", "joint_6", "joint_7", "grip_right_driver_joint"]


        self.joint_names = ["gen3_" + name for name in self.joint_names]


        self.active_joints = {name: self.joint_ids[name] for name in self.joint_names if name in self.joint_ids}


        self.active_joints_idx = np.array(list(self.active_joints.values()), dtype=int)
        self.act_idx = np.array([self.act_ids[n] for n in self.act_ids], dtype=int)


        # self.display()