from pathlib import Path
import numpy as np
import torch
import time
import logging


from .robot import Kinematics, Dynamics, Controller, TrajPlanner
from .kinova_gen3 import load_kinova_gen3
from .utils import pytorch_util as ptu
from .utils import numpy_util as npu
FloatArray = npu.FloatArray

HERE = Path(__file__).parent
XML_PATH = HERE / "world/world.xml"
LOG_DIR = HERE / "logs"
LOG_DIR.mkdir(exist_ok=True)

# setup .log file logging
LOG_DIR.mkdir(exist_ok=True)
log_path = LOG_DIR / f"env_run_{time.strftime('%Y%m%d-%H%M%S')}.log"
logger = logging.getLogger("env_run")
logger.setLevel(logging.INFO)
# avoid duplicate handlers if main() re-runs
logger.handlers.clear()
fh = logging.FileHandler(log_path, mode="w")
fh.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
logger.addHandler(fh)
logger.propagate = False


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
    ptu.init_gpu(use_gpu=False)

    kin, dyn, ctrl, planner = init_system()

    freq = 100.0  # trajectory frequency
    q0 = np.zeros(8, dtype=npu.dtype)
    qf = np.array([0.0, -np.pi/4, 0.0, -3*np.pi/4, 0.0, np.pi/2, 0.0, 0.0], dtype=npu.dtype)
    t0 = 30.0
    tf = 40.0
    T = planner.quintic_trajs(q0, qf, t0, tf, freq)
    print(f"T shape: {T.shape}")
    ctrl.set_trajectory(T, freq=freq, ti=t0)
    
    time_cl = []
    time_fk = []
    time_j = []
    time_jcom = []
    try: 
        for i in range(0, 500):
            t = i / freq
            # Generate random q and qd in [-pi, pi]
            q_np = np.random.uniform(-np.pi, np.pi, size=8).astype(npu.dtype)
            qd_np = np.random.uniform(-np.pi, np.pi, size=8).astype(npu.dtype)
            q = ptu.from_numpy(q_np)
            qd = ptu.from_numpy(qd_np)

            t1 = time.time_ns()
            kin.forward_kinematics(q, use_cache=False)
            t2 = time.time_ns()
            time_fk.append((t2 - t1) / 1e9)  # convert to seconds

            t1 = time.time_ns()
            kin.full_jac(q, use_cache=False)
            t2 = time.time_ns()
            time_j.append((t2 - t1) / 1e9)  # convert to seconds

            kin.step(q=q, qd=qd)
            t1 = time.time_ns()
            kin.jac_com()
            t2 = time.time_ns()
            time_jcom.append((t2 - t1) / 1e9)  # convert to seconds

            kin.step(q=q, qd=qd)
            t1 = time.time_ns()
            ctrl.computed_torque(q, qd, t)
            t2 = time.time_ns()
            time_cl.append((t2 - t1) / 1e9)  # convert to seconds   
    finally:
        print(f"time computed_torque: {np.mean(time_cl):.9f}")
        print(f"time forward_kinematics: {np.mean(time_fk):.9f}")
        print(f"time full_jac: {np.mean(time_j):.9f}")
        print(f"time jac_com: {np.mean(time_jcom):.9f}")

main()