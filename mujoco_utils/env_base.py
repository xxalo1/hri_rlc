# gen3_env.py
from __future__ import annotations

import numpy as np
import mujoco as mj
import pandas as pd
from IPython.display import display

from ..utils import numpy_util as npu
FloatArray = npu.FloatArray

class BaseMujocoEnv:
    def __init__(self, xml_path, nsubsteps=10, seed: int | None = 0):
        self.m = mj.MjModel.from_xml_path(xml_path)
        self.d = mj.MjData(self.m)
        self.nsub = nsubsteps
        self.rng = np.random.default_rng(seed)


        # ---- cache useful IDs (scales well)
        self.joint_ids  = {mj.mj_id2name(self.m, mj.mjtObj.mjOBJ_JOINT, i): i for i in range(self.m.njnt)}
        self.site_ids   = {mj.mj_id2name(self.m, mj.mjtObj.mjOBJ_SITE, i):  i for i in range(self.m.nsite)}
        self.act_ids    = {mj.mj_id2name(self.m, mj.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(self.m.nu)}


        self.joint_names = ["joint_1", "joint_2", "joint_3",
                            "joint_4", "joint_5", "joint_6", "joint_7", "grip_right_driver_joint"]


        self.joint_names = ["gen3_" + name for name in self.joint_names]


        self.active_joints = {name: self.joint_ids[name] for name in self.joint_names if name in self.joint_ids}


        self.joint_idx = np.array(list(self.active_joints.values()), dtype=int)
        self.act_idx = np.array([self.act_ids[n] for n in self.act_ids], dtype=int)


        # ranges (for safe scaling / clipping)
        self.ctrl_min = self.m.actuator_ctrlrange[self.act_idx, 0]
        self.ctrl_max = self.m.actuator_ctrlrange[self.act_idx, 1]


        # optional: remember a keyframe to reset to (uses Menagerie keys if present)
        self.key_home = mj.mj_name2id(self.m, mj.mjtObj.mjOBJ_KEY, "home") if self.m.nkey > 0 else -1


        # self.display()


    def display(self):
        """display attributes for debugging as a panda table"""
        data = [
            ("nsite", self.m.nsite),
            ("njnt", self.m.njnt),
            ("nactuator", self.m.nu),
            ("key_home", self.key_home),
            ("site_ids", self.site_ids),
            ("act_ids", self.act_ids),
            ("ctrl_min", self.ctrl_min),
            ("ctrl_max", self.ctrl_max),
            ("act_idx", self.act_idx),
            ("limited_act_idx", self.m.actuator_ctrllimited[self.act_idx]),
            ("active_joints", self.active_joints),
            ("actuator_gear:", self.m.actuator_gear[self.act_idx]),
            ("actuator_ctrllimited:", self.m.actuator_ctrllimited[self.act_idx]),
            ("actuator_ctrlrange:", self.m.actuator_ctrlrange[self.act_idx]),
            ("dof_damping    =", self.m.dof_damping),
            ("dof_friction   =", self.m.dof_frictionloss),
        ]

        pd.set_option('display.max_colwidth', None)   # show full cell contents
        pd.set_option('display.max_rows', None)       # optional: show all rows

        df = pd.DataFrame(data, columns=["Attribute", "Value"])
        display(df)


    def reset(self):
        """Reset to 'home' keyframe if it exists; otherwise to default qpos=0."""
        if self.key_home >= 0:
            mj.mj_resetDataKeyframe(self.m, self.d, self.key_home)
        else:
            mj.mj_resetData(self.m, self.d)

        mj.mj_forward(self.m, self.d)
        return self.observe()


    def step(self, 
        action: FloatArray, 
        mode: str ="torque"
    ) -> dict[str, FloatArray]:
        
        """Apply action then step physics."""
        if mode == "torque":
            self.d.ctrl[self.act_idx[:len(action)]] = action
        mj.mj_step(self.m, self.d, self.nsub)

        return self.observe()


    def observe(self
    ) -> dict[str, FloatArray]:
        """Return a simple state dict; extend as you need."""
        obs = {
            "qpos": self.d.qpos.copy()[self.joint_idx],
            "qvel": self.d.qvel.copy()[self.joint_idx],
            "qacc":  self.d.qacc.copy()[self.joint_idx],
            "time": self.d.time,
        }

        for x in self.site_ids:
            sid = self.site_ids[x]
            obs[f"{x}_pos"] = self.d.site_xpos[sid].copy()
            obs[f"{x}_rotmat"] = self.d.site_xmat[sid].reshape(3,3).copy()

        return obs


    def set_state(self, 
        qpos: FloatArray,
        qvel: FloatArray,
        t: float
    ) -> None:
        aji = self.joint_idx
        self.d.qpos[aji], self.d.qvel[aji], self.d.time = qpos, qvel, t
        mj.mj_forward(self.m, self.d)


