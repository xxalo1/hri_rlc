from pathlib import Path
import numpy as np
import math
import time
import logging
import matplotlib.pyplot as plt
import mujoco as mj
from mujoco import viewer

from .mujoco_utils import draw_all_frames, draw_ee_traj
from .env import Gen3Env
from .robot import Kinematics, Dynamics, Controller, TrajPlanner
from .kinova_gen3 import load_kinova_gen3
from .utils import pytorch_util as ptu
from .utils import numpy_util as npu
from .kinova_gen3 import mj_util as mju
FloatArray = npu.FloatArray
ptu.init_gpu(use_gpu=False)

HERE = Path(__file__).parent
XML_PATH = HERE / "world/world.xml"

# setup .log file logging
LOG_DIR = HERE / "logs"
LOG_DIR.mkdir(exist_ok=True)
log_path = LOG_DIR / f"env_run.log"
logger = logging.getLogger("env_run")
logger.setLevel(logging.INFO)
# avoid duplicate handlers if main() re-runs
logger.handlers.clear()
fh = logging.FileHandler(log_path, mode="w")
fh.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
logger.addHandler(fh)
logger.propagate = False


def key_callback(keycode):
    global paused, want_plot, reset, show_frames
    c = chr(keycode)
    if c == ' ':
        paused = not paused
    elif c in ('r', 'R'):
        reset = True
        paused = True
    elif c in ('p', 'P'):
        want_plot = True
        paused = True
    elif c in ("f", "F"):
        show_frames = not show_frames


def wait_if_paused():
    while paused and v.is_running():
        time.sleep(0.1)


def draw_plots(log):
    t    = log["t"]
    qt   = log["qt"]
    q    = log["q"]
    qdt  = log["qdt"]
    qd   = log["qd"]
    tau  = log["tau"]
    e    = log["e"]
    de   = log["de"]
    inertia_com = log["inertia_com"]
    h = log["h"]
    tau_g = log["tau_g"]
    v = log["v"]

    N, n = q.shape

    plots = [
        ("Joint Positions",       [(qt, "qt"), (q, "q"), (qd, "qd"), (qdt, "qdt")]),
        ("tarking error e and dt with v", [(e, "e"), (de, "de"), (v, "v")]),
        ("Dynamics Components", [(inertia_com, "inertia_com"), (h, "h"), (tau_g, "tau_g"), (tau, "tau")]),
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


def init_system(kv, kp):
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
    ctrl = Controller(dyn, Kv=kv, Kp=kp)
    planner = TrajPlanner()


    return kin, dyn, ctrl, planner


def main():
    global paused, v, reset, want_plot, show_frames
    paused = True
    want_plot = False
    reset = False
    show_frames = True

    kin, dyn, ctrl, planner = init_system(kv = 20.0, kp = 20.0)

    freq = 100.0  # trajectory frequency
    q0 = np.array([0.0, -np.pi/2, 0.0, np.pi/4, np.pi/2, 0.0, 0.0, 0.0], dtype=npu.dtype)
    qf = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=npu.dtype)
    t0 = 0.0
    tf = 10.0
    T = planner.quintic_trajs(q0, qf, t0, tf, freq)
    ctrl.set_trajectory(T, freq=freq, ti=t0)

    # end position curve
    T_n = ptu.from_numpy(T)
    T_wf_traj = kin.batch_forward_kinematics(T_n[0])
    Q_EF = T_wf_traj[:, -1, :3, 3] # (N, 3)
    Q_EF_np = ptu.to_numpy(Q_EF)
    
    env = Gen3Env(xml_path=str(XML_PATH))
    env.display()

    dt_sim = env.m.opt.timestep          # MuJoCo integration step
    dt_ctrl = 1.0 / freq                 # 0.01 s for 100 Hz
    nsub = max(1, int(round(dt_ctrl / dt_sim)))
    env.nsub = nsub

    t_end = tf + 2.0

    q0_mj = mju.to_mj_q(q0, skip_last=False)
    dq0_mj = np.zeros_like(q0_mj)
    env.set_state(q0_mj, dq0_mj, t0)
    obs = env.observe()

    with viewer.launch_passive(env.m, env.d, key_callback=key_callback) as v:
        draw_ee_traj(v.user_scn, Q_EF_np, radius=0.002, stride=20)
        v.sync()
        while v.is_running():

            if want_plot:
                want_plot = False
                draw_plots(ctrl.get_log())

            if reset:
                reset = False
                with v.lock():
                    ctrl.clear_log()
                    env.set_state(q0_mj, dq0_mj, t0)
                    obs = env.observe()
                    v.sync()

            wait_if_paused()
            with v.lock():
                q, qd, t = mju.from_mj_torch(obs, skip_last=False)
                kin.step(q=q, qd=qd)

                if show_frames:
                    T_wf = kin.forward_kinematics()
                    draw_all_frames(v.user_scn, ptu.to_numpy(T_wf), axis_len=0.2, width=0.004)

                mj.mj_step(env.m, env.d)
                v.sync()

                if t >= t_end:
                    break

            time.sleep(0.001)


main()