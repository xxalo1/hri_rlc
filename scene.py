from pathlib import Path
import numpy as np
import torch
import time

import mujoco as mj
from mujoco import viewer


from .env import Gen3Env
from .robot import Kinematics, Dynamics, Controller, TrajPlanner
from .kinova_gen3 import load_kinova_gen3
from .utils import pytorch_util as ptu
from .utils import numpy_util as npu

FloatArray = npu.FloatArray
ptu.init_gpu()

HERE = Path(__file__).parent
XML_PATH = HERE / "world/world.xml"


def _draw_all_frames(scn, T_wf: FloatArray, axis_len: float=0.2, width: float=0.004):
    D = npu.dtype  # your preferred numeric dtype (e.g., np.float64 or np.float32)
    scn.ngeom = 0

    for i in range(T_wf.shape[0]):
        T = T_wf[i]
        o = T[:3, 3]
        R = T[:3, :3]

        cols = (
            np.array([1, 0, 0, 1], dtype=D),  # X
            np.array([0, 1, 0, 1], dtype=D),  # Y
            np.array([0, 0, 1, 1], dtype=D),  # Z
        )

        for a, rgba in zip(R.T, cols):
            p1 = o
            p2 = o + axis_len * a

            g = scn.geoms[scn.ngeom]

            mj.mjv_connector(
                g,
                mj.mjtGeom.mjGEOM_ARROW,
                float(width),
                np.asarray(p1, dtype=np.float64),
                np.asarray(p2, dtype=np.float64),
            )
            # Set color separately (must be float32):
            g.rgba[:] = rgba.astype(np.float32, copy=False)

            scn.ngeom += 1


def key_callback(keycode):
  if chr(keycode) == ' ':
    global paused
    paused = not paused


def wait_if_paused():
    while paused and v.is_running():
        time.sleep(0.1)


def init_system():
    dh, inertia, dh_t, inertia_t = load_kinova_gen3()
    o_wb = np.array([1., 0., 0.75], dtype=npu.dtype)
    axes_wb=np.eye(3, dtype=npu.dtype)
    R_z_pi = np.array([[-1., 0., 0.],
                        [0., -1., 0.],
                        [0., 0., 1.]], dtype=npu.dtype)
    axes_wb = R_z_pi @ axes_wb

    o_wb_t = ptu.from_numpy(o_wb)
    axes_wb_t = ptu.from_numpy(axes_wb)

    kin = Kinematics(dh=dh_t, o_wb=o_wb_t, axes_wb=axes_wb_t, inertia=inertia_t)
    dyn = Dynamics(kin)
    ctrl = Controller(dyn)
    planner = TrajPlanner()


    return kin, dyn, ctrl, planner


def main():
    global paused, v
    paused = False

    kin, dyn, ctrl, planner = init_system()

    freq = 100.0  # trajectory frequency
    q0 = np.zeros(7, dtype=npu.dtype)
    qf = np.array([0.0, -np.pi/4, 0.0, -3*np.pi/4, 0.0, np.pi/2, 0.0], dtype=npu.dtype)
    t0 = 30.0
    tf = 40.0
    T = planner.quintic_trajs(q0, qf, t0, tf, freq)

    ctrl.set_trajectory(T, freq=freq, ti=t0)
    
    env = Gen3Env(xml_path=str(XML_PATH), nsubsteps=10, seed=0)
    env.reset()

    with viewer.launch_passive(env.m, env.d, key_callback=key_callback) as v:
        q0 = env.d.qpos[env.active_joints_idx].copy()
        q_kin = np.concatenate(([0.0], -q0[:-1]))
        kin.step(q=q_kin)
        with v.lock():
            T_wf = kin.forward_kinematics()
            _draw_all_frames(v.user_scn, T_wf)
            v.sync()

        while v.is_running():
            wait_if_paused()

            with v.lock():
                mj.mj_step(env.m, env.d)

                q = env.d.qpos[env.active_joints_idx].copy()
                q_kin = np.concatenate(([0.0], -q[:-1]))
                kin.step(q=q_kin)

                T_wf = kin.forward_kinematics()
                _draw_all_frames(v.user_scn, T_wf)

                v.sync()
            time.sleep(0.00001)
main()