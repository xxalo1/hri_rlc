from pathlib import Path
import numpy as np
import math
import time
import logging
import matplotlib.pyplot as plt
import mujoco as mj
from mujoco import viewer
import mujoco_viewer

from ..mujoco_utils import draw_all_frames, draw_ee_traj, Plotter
from .env import Gen3Env
from ..robot import Kinematics, Dynamics, Controller, TrajPlanner
from ..kinova_gen3 import load_kinova_gen3_v2
from ..utils import pytorch_util as ptu
from ..utils import numpy_util as npu

FloatArray = npu.FloatArray
ptu.init_gpu(use_gpu=False)

HERE = Path(__file__).parent
XML_PATH = HERE / "world/world.xml"
XML_PATH_2 = HERE / "world/scene.xml"

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
    global pid, show_dynamics, show_inertia, show_com, use_rnea
    c = chr(keycode)
    if c == ' ':
        paused = not paused
    elif c in ('r', 'R'):
        reset = True
        paused = True
    elif c in ('p', 'P'):
        want_plot = True
        paused = True
    elif c in ("1"):
        show_frames = not show_frames
    elif c in ("j", "J"):
        show_jacobian = True
    elif c in ("d", "D"):
        show_dynamics = True
    elif c in ("i", "I"):
        show_inertia = True
    elif c in ("'"):
        show_com = not show_com
    elif c in ("e", "E"):
        use_rnea = not use_rnea
    elif c in ("t", "T"):
        pid = not pid


def wait_if_paused():
    while paused and v.is_running():
        time.sleep(0.1)


def init_system(kv, kp):
    dh, inertia, T_base, dh_t, inertia_t, T_base_t = load_kinova_gen3_v2()

    o_wb = np.array([1., 0., 0.75], dtype=npu.dtype)
    axes_wb=np.eye(3, dtype=npu.dtype)

    T_world_base = np.eye(4, dtype=npu.dtype)
    T_world_base[:3, :3] = axes_wb
    T_world_base[:3, 3] = o_wb
    T_wb = T_world_base @ T_base

    T_wb_t = ptu.from_numpy(T_wb)
    kin = Kinematics(dh=dh_t, T_wb= T_wb_t, inertia=inertia_t)
    logger.info(f"mass: ")
    dyn = Dynamics(kin)
    ctrl = Controller(dyn, Kv=kv, Kp=kp)
    planner = TrajPlanner()


    return kin, dyn, ctrl, planner


def compare_jacs(kin, env):
    model, data = env.m, env.d
    link_i = 2   # for example

    J_com = kin.jac_com()                     # xp backend
    J_my = ptu.to_numpy(J_com[link_i])        # (6, n)

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

    diff = J_my - J_mj
    max_diff = np.max(np.abs(diff))


    msg = (
        f"\n\n"
        f"==============================\n"
        f" Jacobian Comparison (Link {link_i}: {joint_name})\n"
        f"==============================\n"
        f"\n"
        f"[My J_com]\n{J_my}\n\n"
        f"[MuJoCo J_com]\n{J_mj}\n\n"
        f"[Difference  (my - mujoco)]\n{diff}\n"
        f"Max |diff| = {max_diff:.6e}\n"
        f"------------------------------\n"
    )

    return msg


def compare_dynamics(q, dq, dyn, env):
    model = env.m

    nv = model.nv
    n = dyn.kin.n

    M_mj = np.empty((nv, nv), dtype=npu.dtype)
    mj.mj_fullM(env.m, M_mj, env.d.qM)
    b_mj = env.d.qfrc_bias.copy()
    m, h, tau_g = dyn.Dynamics_matrices(q, dq)

    M_dyn = ptu.to_numpy(m)      # (n, n)
    b_dyn = ptu.to_numpy(h + tau_g)  # (n,)
    M_dyn = M_dyn  # skip first row/col for fixed joint
    b_dyn = b_dyn      # skip first element for fixed joint

    diff_M = M_dyn - M_mj
    diff_b = b_dyn - b_mj

    max_M = np.max(np.abs(diff_M))
    max_b = np.max(np.abs(diff_b))

    msg = (
        "\n\n"
        "==============================\n"
        "   Dynamics Comparison (M, b)\n"
        "==============================\n\n"
        "[My Mass Matrix]\n"
        f"{M_dyn}\n\n"
        "[MuJoCo Mass Matrix]\n"
        f"{M_mj}\n\n"
        "[Mass Matrix Difference (my - mujoco)]\n"
        f"{diff_M}\n"
        f"Max |diff_M| = {max_M:.6e}\n"
        "------------------------------\n\n"
        "[My Bias Forces]\n"
        f"{b_dyn}\n\n"
        "[MuJoCo Bias Forces]\n"
        f"{b_mj}\n\n"
        "[Bias Forces Difference (my - mujoco)]\n"
        f"{diff_b}\n"
        f"Max |diff_b| = {max_b:.6e}\n"
        "------------------------------\n"
    )
    return msg


def log_ic_mass_compare(logger, kin, env):
    """
    Compare inertial parameters between your model and MuJoCo, both in local
    link frames and in the world frame.

    Your model
    ----------
    - mass          : kin.mass     (n,)
    - inertia local : kin.Ic_fl    (n, 3, 3)
        about COM, expressed in your link local frame (for example DH)
    - inertia world : kin.Ic_wl    (n, 3, 3)
        about COM, expressed in world frame
    - COM world     : kin.com_wl   (n, 3)
        COM position expressed in world frame

    MuJoCo
    ------
    - mass              : m.body_mass      (nbodies,)
    - inertia diag      : m.body_inertia   (nbodies, 3)
        about COM, expressed in the inertial frame (principal axes)
        given by (body_ipos, body_iquat)
    - world orientation of inertia frame : d.ximat (nbodies x 9)
        rotation of inertia principal axes into world frame
    - COM world         :
        computed from body frame pose d.xipos and body_ipos
    """

    m = env.m        # mjModel
    d = env.d        # mjData

    your_m_local  = ptu.to_numpy(kin.mass)      # (n,)
    your_I_local  = ptu.to_numpy(kin.Ic_fl)     # (n, 3, 3)
    your_I_world  = ptu.to_numpy(kin.Ic_wl)     # (n, 3, 3)
    your_com_world = ptu.to_numpy(kin.com_wl)   # (n, 3)

    mj_m      = m.body_mass                     # (nbodies,)
    mj_I_diag = m.body_inertia                  # (nbodies, 3)

    # Orientation of body inertia frame in world (flattened 3x3 per body)
    ximat = d.ximat.reshape(m.nbody, 3, 3)      # (nbodies, 3, 3)

    # Orientation and position of body frame in world
    xmat  = d.xmat.reshape(m.nbody, 3, 3)       # (nbodies, 3, 3)
    xipos = d.xipos.reshape(m.nbody, 3)
                             # (nbodies, 3)

    # COM position in body frame (ipos) and inertial orientation (iquat)
    body_ipos  = m.body_ipos                    # (nbodies, 3)
    # body_iquat = m.body_iquat                 # (nbodies, 4) not needed for COM position

    # Only compare active joints (your links)
    body_ids = m.jnt_bodyid[env.active_joints_idx]  # length n

    msg = []
    msg.append("\n" + "="*70)
    msg.append("   Inertial Parameter Comparison (Your Model vs MuJoCo)")
    msg.append("="*70)
    msg.append(
        "\nFrame conventions:\n"
        "  • Yours local  : Ic_fl about COM, in your link local frame.\n"
        "  • Yours world  : Ic_wl about COM, in world frame.\n"
        "  • MuJoCo diag  : body_inertia = diag(Ixx,Iyy,Izz) about COM,\n"
        "                   in the inertial principal axes frame.\n"
        "  • MuJoCo world : same inertia rotated into world frame using d.ximat.\n"
        "  • COM world    : both models compared as positions in world frame."
    )

    for i, bid in enumerate(body_ids):
        body_name = m.body(bid).name
        msg.append(f"\n-- Link {i} / Body '{body_name}' --")

        # ---------- Mass ----------
        m_yours = float(your_m_local[i])
        m_mj    = float(mj_m[bid])

        msg.append("  Mass:")
        msg.append(f"    • Yours   : {m_yours: .6f} kg")
        msg.append(f"    • MuJoCo  : {m_mj: .6f} kg")
        msg.append(f"    • Diff    : {m_yours - m_mj: .6f}\n")

        # ---------- COM in world ----------
        com_yours_w = your_com_world[i]    # (3,)
        # COM in MuJoCo: body origin in world + R_wb * ipos
        com_mj_w    = xipos[bid]
        msg.append("  COM position in WORLD frame (m):")
        msg.append(f"    • Yours (com_wl) : {com_yours_w}")
        msg.append(f"    • MuJoCo         : {com_mj_w}")
        msg.append(f"    • Diff (Yours - MuJoCo): {com_yours_w - com_mj_w}\n")

        # ---------- Local inertia ----------
        I_yours_local   = your_I_local[i]         # (3,3)
        I_mj_diag_vec   = mj_I_diag[bid]          # (3,)
        I_mj_inertial   = np.diag(I_mj_diag_vec)  # (3,3) in inertial principal frame

        msg.append("  Inertia about COM (local and inertial frames, kg·m²):")

        msg.append("    • Yours local (your link frame):")
        msg.append("      " + np.array2string(
            I_yours_local,
            precision=6,
            floatmode='fixed'
        ).replace("\n", "\n      "))

        msg.append("    • MuJoCo inertial frame (principal axes):")
        msg.append(f"      diag([{I_mj_diag_vec[0]:.6f}, "
                          f"{I_mj_diag_vec[1]:.6f}, "
                          f"{I_mj_diag_vec[2]:.6f}])")

        # ---------- World inertia ----------
        R_wi = ximat[bid]                     # (3,3) world orientation of inertia frame
        I_mj_world = R_wi @ I_mj_inertial @ R_wi.T

        I_yours_world = your_I_world[i]

        msg.append("  Inertia in WORLD frame (about COM, expressed in world axes):")

        msg.append("    • Yours world (Ic_wl):")
        msg.append("      " + np.array2string(
            I_yours_world,
            precision=6,
            floatmode='fixed'
        ).replace("\n", "\n      "))

        msg.append("    • MuJoCo world (from body_inertia and d.ximat):")
        msg.append("      " + np.array2string(
            I_mj_world,
            precision=6,
            floatmode='fixed'
        ).replace("\n", "\n      "))

        diff_world = I_yours_world - I_mj_world
        msg.append("    • Diff world (Yours - MuJoCo):")
        msg.append("      " + np.array2string(
            diff_world,
            precision=6,
            floatmode='fixed'
        ).replace("\n", "\n      "))

    msg.append("\n" + "="*70 + "\n")

    logger.info("\n".join(msg))


def draw_overlay(viewer):
    # top-left corner
    viewer.add_overlay(
        mj.mjtGridPos.mjGRID_TOPLEFT,
        "Sim",
        f"paused={paused}  reset={reset}  want_plot={want_plot}"
    )
    viewer.add_overlay(
        mj.mjtGridPos.mjGRID_TOPLEFT,
        "Vis",
        f"frames={show_frames}  jacobian={show_jacobian}"
    )
    viewer.add_overlay(
        mj.mjtGridPos.mjGRID_TOPLEFT,
        "Ctrl",
        f"pid={pid}  use_rnea={use_rnea}  show_dyn={show_dynamics}  "
        f"show_M={show_inertia}  show_com={show_com}"
    )


def main():
    global paused, v, reset, want_plot, show_frames, show_jacobian
    global pid, show_dynamics, show_inertia, show_com, use_rnea

    paused = True
    want_plot = False
    reset = False
    show_frames = False
    show_jacobian = True
    show_dynamics = True
    show_inertia = True
    show_com = False
    use_rnea = True
    pid = True

    kin, dyn, ctrl, planner = init_system(kv = 0.10, kp = 0.10)
    plotter = Plotter()

    freq = 1000.0  # trajectory frequency
    q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=npu.dtype)
    qf = np.array([np.pi/4, -np.pi/2, np.pi/3, -np.pi/3, 0.0, np.pi/6, 0.0], dtype=npu.dtype)
    t0 = 0.0
    tf = 5.0
    T = planner.quintic_trajs(q0, qf, t0, tf, freq)
    ctrl.set_trajectory(T, freq=freq, ti=t0)

    # end position curve
    T_n = ptu.from_numpy(T)
    T_wf_traj = kin.batch_forward_kinematics(T_n[0])
    Q_EF = T_wf_traj[:, -1, :3, 3] # (N, 3)
    Q_EF_np = ptu.to_numpy(Q_EF)

    env = Gen3Env(xml_path=str(XML_PATH))

    dt_sim = env.m.opt.timestep          # MuJoCo integration step
    dt_ctrl = 1.0 / freq                 # 0.01 s for 100 Hz
    nsub = max(1, int(round(dt_ctrl / dt_sim)))
    env.nsub = nsub

    dq0 = np.zeros_like(q0)
    env.set_state(q0, dq0, t0)
    env.display()
    t_pref = t0

    log_ic_mass_compare(logger, kin, env)

    with viewer.launch_passive(env.m, env.d, key_callback=key_callback) as v:
        draw_ee_traj(v.user_scn, Q_EF_np, radius=0.002, stride=20)
        v.sync()
        while v.is_running():

            if want_plot:
                want_plot = False
                plotter.draw_plots()

            if reset:
                reset = False
                with v.lock():
                    plotter.clear_log()
                    env.set_state(q0, dq0, t0)
                    v.sync()

            with v.lock():
                obs = env.observe()
                q = ptu.from_numpy(obs["qpos"])    # skip fixed joint
                qd = ptu.from_numpy(obs["qvel"])
                qdd = ptu.from_numpy(obs["qacc"])
                t = obs["time"]
                kin.step(q=q, qd=qd)

                if show_jacobian:
                    msg = compare_jacs(kin, env)
                    logger.info(msg)
                    show_jacobian = False
                    paused = True

                if show_frames:
                    T_wf = kin.forward_kinematics()
                    com_wl = ptu.to_numpy(kin.com_wl)
                    draw_all_frames(v.user_scn, ptu.to_numpy(T_wf), com_wl, show_com, axis_len=0.4, width=0.004)

                if show_dynamics:
                    msg = compare_dynamics(q, qd, dyn, env)
                    logger.info(msg)
                    show_dynamics = False
                    paused = True

                if show_inertia:
                    log_ic_mass_compare(logger, kin, env)
                    show_inertia = False

                nv = env.m.nv
                M_mj = np.empty((nv, nv), dtype=npu.dtype)
                mj.mj_fullM(env.m, M_mj, env.d.qM)
                b_mj = env.d.qfrc_bias.copy()
                mjd = {
                    "M": ptu.from_numpy(M_mj),
                    "b": ptu.from_numpy(b_mj),
                    "qdd": qdd
                }
                tau, tau_rnea, dict_out = ctrl.computed_torque(q, qd, t, mjd)
                for key, value in dict_out.items():
                    plotter.log(key, value)

                if use_rnea:
                    tau = tau_rnea

                if pid:
                    dt = t - t_pref
                    tau_pid = ctrl.pid(q, qd, t, dt)
                    tau = tau + tau_pid

                tau_n = ptu.to_numpy(tau)
                obs = env.step(tau_n)
                t_pref = t
                draw_overlay(v)
                v.sync()
            wait_if_paused()

            time.sleep(0.00001)


main()