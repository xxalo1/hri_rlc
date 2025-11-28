import numpy as np

from robots.kinova_gen3 import init_kinova_robot
from rbt_core import Robot, RobotSpec, Kinematics, Dynamics

from rbt_core import Robot
from common_utils import numpy_util as npu
from .....src.sim_env.mujoco.env import Gen3Env


FloatArray = npu.FloatArray


def init_kinova_robot() -> Robot:
    """
    initialize and return a Kinova Gen3 robot instance
    """
    robot_spec: RobotSpec = kinova_gen3_spec()

    o_wb = np.array([1., 0., 0.75], dtype=npu.dtype)
    axes_wb = np.eye(3, dtype=npu.dtype)

    T_world_base = np.eye(4, dtype=npu.dtype)
    T_world_base[:3, :3] = axes_wb
    T_world_base[:3, 3] = o_wb
    T_wb = T_world_base @ robot_spec.T_base
    robot_spec.T_base = T_wb

    robot = Robot.from_spec(robot_spec)
    return robot


def compare_jacs(
    kin: Kinematics, 
    env: Gen3Env
) -> str:
    model, data = env.m, env.d
    link_i = 2   # for example

    J_com = kin.jac_com()             
    J_my = J_com[link_i]              # (6, n)

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


def compare_dynamics(
    dyn: Dynamics, 
    env: Gen3Env
) -> str:
    model = env.m

    nv = model.nv
    n = dyn.kin.n

    M_mj = np.empty((nv, nv), dtype=npu.dtype)
    mj.mj_fullM(env.m, M_mj, env.d.qM)
    b_mj = env.d.qfrc_bias.copy()
    m, tau_g = dyn.Dynamics_matrices()

    M_dyn = m      # (n, n)
    b_dyn = tau_g  # (n,)
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


def log_ic_mass_compare(
    logger, 
    kin: Kinematics, 
    env: Gen3Env
) -> None:
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

    your_m_local  = kin.mass      # (n,)
    your_I_local  = kin.Ic_fl     # (n, 3, 3)
    your_I_world  = kin.Ic_wl     # (n, 3, 3)
    your_com_world = kin.com_wl   # (n, 3)

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

