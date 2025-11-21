# gen3_env.py
from __future__ import annotations
import math
from matplotlib import pyplot as plt
import os, numpy as np
import mujoco as mj
import pandas as pd
from IPython.display import display
import torch

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


        self.joint_names = ["joint_1", "joint_2", "joint_3",
                            "joint_4", "joint_5", "joint_6", "joint_7", "grip_right_driver_joint"]


        self.joint_names = ["gen3_" + name for name in self.joint_names]


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
        if ee_name in self.site_ids:
            sid = self.site_ids[ee_name]
            obs["ee_pos"] = self.d.site_xpos[sid].copy()
            obs["ee_rotmat"] = self.d.site_xmat[sid].reshape(3,3).copy()

        return obs


    def set_state(self, qpos, qvel, t):
        aji = self.active_joints_idx
        self.d.qpos[aji], self.d.qvel[aji], self.d.time = qpos, qvel, t
        mj.mj_forward(self.m, self.d)


class Plotter():
    
    def __init__(self):
        self._log = {}


    def log(self, key: str, value: torch.Tensor | np.ndarray):
        """Log a value under the specified key."""
        if key in self._log:
            if isinstance(value, np.ndarray):
                self._log[key].append(value.copy())
            elif isinstance(value, torch.Tensor):
                self._log[key].append(value.detach())
            else: 
                self._log[key].append(value)
        else:
            if isinstance(value, np.ndarray):
                self._log[key] = [value.copy()]
            elif isinstance(value, torch.Tensor):
                self._log[key] = [value.detach()]
            else: 
                self._log[key] = [value]


    def get_log(self) -> dict[str, np.ndarray]:
        """Retrieve the logged data as numpy arrays."""
        return {key: np.array(values) for key, values in self._log.items()}


    def clear_log(self) -> None:
        """Clear all recorded data."""
        for k in self._log:
            self._log[k].clear()


    def draw_plots(self):
        log = self.get_log()
        t    = log["t"]
        qt   = log["qt"]
        q    = log["q"]
        qdt  = log["qdt"]
        qd   = log["qd"]
        e    = log["e"]
        de   = log["de"]
        v = log["v"]
        Mjd = log["Mjd"]
        bjd = log["bjd"]
        taumj = log["taumj"]
        ddqt = log["ddqt"]
        qdd = log["qdd"]
        tau_rnea = log.get("tau_rnea", None)

        N, n = q.shape

        plots = [
            ("Joint Positions",       [(qt, "qt"), (q, "q"), (qd, "qd"), (qdt, "qdt")]),
            ("tarking error e and dt with v", [(e, "e"), (de, "de"), (v, "v"), (ddqt, "ddqt")]),
            ("Computed MuJoCo Torque", [(taumj, "taumj"), (tau_rnea, "tau_rnea")]),
            ("Joint Accelerations", [(qdd, "qdd"), (ddqt, "ddqt"), (v, "v")]),
        ]

        cols = 3                                # change to 2 if you want
        rows = math.ceil(n / cols)              # auto rows based on n

        for title, series in plots:
            fig, axes = plt.subplots(
                rows, cols,
                figsize=(cols*5, rows*3),
                sharex=True
            )

            axes = axes.flatten()               # make indexing simple
            fig.suptitle(title)

            for j in range(n):
                ax = axes[j]
                for arr, label in series:

                    if label:
                        ax.plot(t, arr[:, j], label=label)
                    else:
                        ax.plot(t, arr[:, j])
                ax.set_ylabel(f"J{j+1}")
                if any(label is not None for _, label in series):
                    ax.legend()

            # turn off any unused subplot
            for k in range(n, len(axes)):
                axes[k].axis("off")

            axes[-1].set_xlabel("time [s]")

        plt.show()

