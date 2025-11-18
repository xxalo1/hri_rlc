# gen3_env.py
from __future__ import annotations
import os, numpy as np
import mujoco as mj
import pandas as pd
from IPython.display import display

from .utils import numpy_util as npu
FloatArray = npu.FloatArray
class Gen3Env:
    def __init__(self, xml_path="world/world.xml", nsubsteps=10, seed: int | None = 0):
        self.m = mj.MjModel.from_xml_path(xml_path)
        self.d = mj.MjData(self.m)
        self.nsub = nsubsteps
        self.rng = np.random.default_rng(seed)

        # ---- cache useful IDs (scales well)
        self.joint_ids  = {mj.mj_id2name(self.m, mj.mjtObj.mjOBJ_JOINT, i): i for i in range(self.m.njnt)}
        self.site_ids   = {mj.mj_id2name(self.m, mj.mjtObj.mjOBJ_SITE, i):  i for i in range(self.m.nsite)}
        self.act_ids    = {mj.mj_id2name(self.m, mj.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(self.m.nu)}

        self.joint_names = ["gen3_joint_1", "gen3_joint_2", "gen3_joint_3",
                            "gen3_joint_4", "gen3_joint_5", "gen3_joint_6", "gen3_joint_7", "gen3_grip_right_driver_joint"]
        self.active_joints = {name: self.joint_ids[name] for name in self.joint_names if name in self.joint_ids}

        self.active_joints_idx = np.array(list(self.active_joints.values()), dtype=int)
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


    def step(self, action: FloatArray, mode: str ="torque"):
        """Apply action then step physics."""
        if mode == "torque":
            self.apply_torque(action)

        for _ in range(self.nsub):
            mj.mj_step(self.m, self.d)

        return self.observe()

    def apply_torque(self, tao: FloatArray):
        """
        Apply torque control to active actuators.

        Parameters
        ----------
        tao : ndarray, shape (n,)
              the torque values to apply at each actuator.
              where `n` is number of active joints
        
        returns:
        -----------
        None

        -----------
        Note: 
        this method directly sets the torque values in the simulator.
        it sets them to the corresponding indices in `self.d.ctrl`.
        """

        self.d.ctrl[self.act_idx[:len(tao)]] = tao
        return

    def observe(self):
        """Return a simple state dict; extend as you need."""
        obs = {
            "qpos": self.d.qpos.copy()[self.active_joints_idx],
            "qvel": self.d.qvel.copy()[self.active_joints_idx],
            "qacc":  self.d.qacc.copy()[self.active_joints_idx],
            "time": self.d.time,
        }
        # example: end-effector pose (replace name to match yours)
        ee_name = "gen3_pinch_site"
        sid = self.site_ids[ee_name]
        obs["ee_pos"] = self.d.site_xpos[sid].copy()
        obs["ee_rotmat"] = self.d.site_xmat[sid].reshape(3,3).copy()

        return obs

    def set_state(self, qpos, qvel, t):
        aji = self.active_joints_idx
        self.d.qpos[aji], self.d.qvel[aji], self.d.time = qpos, qvel, t
        mj.mj_forward(self.m, self.d)
