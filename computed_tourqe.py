from pathlib import Path
import numpy as np
import math
import time
import logging
import matplotlib.pyplot as plt
import mujoco as mj
from mujoco import viewer

from .mujoco_utils import draw_all_frames, draw_ee_traj, draw_all_frames_at_com
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
    global paused, want_plot, reset, show_frames, show_jacobian
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
    elif c in ("j", "J"):
        show_jacobian = True


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
    Mjd = log["Mjd"]
    bjd = log["bjd"]
    b = log["b"]

    Mjd = np.insert(Mjd, 0, 0.0, axis=1)
    bjd = np.insert(bjd, 0, 0.0, axis=1)
    N, n = q.shape

    plots = [
        ("Joint Positions",       [(qt, "qt"), (q, "q"), (qd, "qd"), (qdt, "qdt")]),
        ("tarking error e and dt with v", [(e, "e"), (de, "de"), (v, "v")]),
        ("Dynamics Components", [(inertia_com, "inertia_com"), (h, "h"), (tau_g, "tau_g"), (tau, "tau")]),
        ("Mass Matrix Entries", [(Mjd, "Mjd"), (inertia_com, "inertia_com")]),
        ("Bias Forces", [(bjd, "bjd"), (b, "b")]),
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


def compare_jacs(kin, env):
    model, data = env.m, env.d
    link_i = 2   # for example

    J_com = kin.jac_com()                     # xp backend
    J_my = ptu.to_numpy(-J_com[link_i+1, :, 1:])        # (6, n)

    joint_name = env.joint_names[link_i]
    jid = env.joint_ids[joint_name]           # joint id
    bid = model.jnt_bodyid[jid]               # body attached to that joint

    nv = model.nv
    jacp = np.zeros((3, nv), dtype=npu.dtype)
    jacr = np.zeros((3, nv), dtype=npu.dtype)

    mj.mj_jacBodyCom(model, data, jacp, jacr, bid)
    J_mj_full = np.vstack([jacp, jacr])       # (6, nv)

    dof_idx = model.jnt_dofadr[env.active_joints_idx]   # (n,)
    J_mj = J_mj_full[:, dof_idx]                        # (6, n)

    print(f"\n=== My J_com for link {link_i} (kin.jac_com) ===")
    print(J_my)

    print(f"\n=== MuJoCo J_com for body of {joint_name} (mj_jacBodyCom) ===")
    print(J_mj)

    diff = J_my - J_mj
    print("\n=== Jacobian difference at COM (my J - MuJoCo J) ===")
    print(diff)
    print("max |diff| =", np.max(np.abs(diff)))



def main():
    global paused, v, reset, want_plot, show_frames, show_jacobian
    paused = True
    want_plot = False
    reset = False
    show_frames = True
    show_jacobian = True

    kin, dyn, ctrl, planner = init_system(kv = 20.0, kp = 20.0)

    freq = 100.0  # trajectory frequency
    q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=npu.dtype)
    qf = np.array([0.0, -np.pi/2, 0.0, np.pi/4, np.pi/2, 0.0, 0.0, 0.0], dtype=npu.dtype)
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

                if show_jacobian:
                    compare_jacs(kin, env)
                    show_jacobian = False
                    paused = True

                if show_frames:
                    T_wf = kin.forward_kinematics()
                    com_wl = ptu.to_numpy(kin.com_wl)
                    draw_all_frames_at_com(v.user_scn, ptu.to_numpy(T_wf), com_wl, axis_len=0.4, width=0.001)
                    
                
                nv = env.m.nv
                M_mj = np.empty((nv, nv), dtype=npu.dtype)
                mj.mj_fullM(env.m, M_mj, env.d.qM)
                b_mj = env.d.qfrc_bias.copy()
                mjd = {
                    "M": ptu.from_numpy(M_mj),
                    "b": ptu.from_numpy(b_mj)
                }
                tau = ctrl.computed_torque(q, qd, t, mjd)
                tau_mj = mju.to_mj_torque(tau, skip_last=False)
                obs = env.step(tau_mj)
                v.sync()

            time.sleep(0.001)


main()