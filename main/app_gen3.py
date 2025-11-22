from __future__ import annotations
from pathlib import Path
import time
import numpy as np
import mujoco as mj
from mujoco import viewer
import torch
from enum import Enum, auto

from .ops import init_kinova_robot, compare_jacs, compare_dynamics, log_ic_mass_compare

from .env import Gen3Env
from ..mujoco_utils import draw_all_frames, draw_ee_traj, Plotter
from ..utils import numpy_util as npu, pytorch_util as ptu

FloatArray = npu.FloatArray



class ControlMode(Enum):
    CT = auto()
    RNEA = auto()
    PID = auto()
    GC = auto()

class TrackingMode(Enum):
    PT = auto()
    TRAJ = auto()
    PTC = auto()


class Gen3App:
    def __init__(self, xml_path: Path, logger):
        self.logger = logger
        self.xml_path = xml_path

        # flags that used to be global
        self.paused = True
        self.want_plot = False
        self.reset = False
        self.show_frames = False
        self.show_jacobian = False
        self.show_dynamics = False
        self.show_inertia = False
        self.show_com = False
        self.use_rnea = True
        self.use_pid = True

        # build robot classes
        self.robot = init_kinova_robot()
        self.robot.ctrl.set_gains(Kp=2.0, Kv=2.0, Ki=1.0)
        self.plotter = Plotter()
        self.env = Gen3Env(xml_path=str(self.xml_path))

        # basic trajectory
        self.q0 = torch.zeros(7, device=ptu.device)
        self.q0_np = ptu.to_numpy(self.q0)
        self.dq0_np = np.zeros_like(self.q0_np)
        self.qf = torch.tensor([np.pi/4, -np.pi/2, np.pi/3, -np.pi/3, 0.0, np.pi/6, 0.0], device=ptu.device)

        self.robot.setup_quintic_traj(freq=1000.0, ti=0.0, tf=5.0, qt=self.qf)
        self.Q_EF_np = self.robot.get_ee_traj()

        dt_sim = self.env.m.opt.timestep          # MuJoCo integration step
        dt_ctrl = 1.0 / self.robot.freq                 # 0.01 s for 100 Hz
        nsub = max(1, int(round(dt_ctrl / dt_sim)))
        self.env.nsub = nsub

        self.t_prev = 0.0
        self.control_mode = ControlMode.RNEA
        self.tracking_mode = TrackingMode.TRAJ
        self.track_current = False
        self.pt_set = False


    def key_callback(self, keycode):
        c = chr(keycode)
        match c:
            case " ":
                self.paused = not self.paused
            case "r" | "R":
                self.reset = True
                self.paused = True
            case "p" | "P":
                self.want_plot = True
                self.paused = True
            case "1":
                self.show_frames = not self.show_frames
            case "j" | "J":
                self.show_jacobian = True
            case "d" | "D":
                self.show_dynamics = True
            case "i" | "I":
                self.show_inertia = True
            case "'":
                self.show_com = not self.show_com
            case "e" | "E":
                self.control_mode = ControlMode.RNEA
            case "t" | "T":  
                self.control_mode = ControlMode.PID
            case "c" | "C":
                self.control_mode = ControlMode.CT
            case "g" | "G":
                self.control_mode = ControlMode.GC
            case "3" :
                self.tracking_mode = TrackingMode.TRAJ
            case "4" :
                self.tracking_mode = TrackingMode.PT
            case "5" :
                self.track_current = not self.track_current
                self.pt_set = False


    def compute_tau(self, q, dq, qt, dqt, ddqt, t):
        nv = self.env.m.nv
        M_mj = np.empty((nv, nv), dtype=npu.dtype)
        mj.mj_fullM(self.env.m, M_mj, self.env.d.qM)
        b_mj = self.env.d.qfrc_bias.copy()

        self.plotter.log("M", M_mj)
        self.plotter.log("b", b_mj)
        
        mjd = {
            "M": ptu.from_numpy(M_mj),
            "b": ptu.from_numpy(b_mj),
        }

        taumj, tau_rnea, dict_out = self.robot.ctrl.computed_torque(q, dq, qt, dqt, ddqt, mjd)
        self.plotter.log("tau_rnea", tau_rnea)
        self.plotter.log("taumj", taumj)
        diff = (tau_rnea - taumj) 
        diff_per = diff / (taumj + 1e-8)
        self.plotter.log("tau_diff", diff)
        self.plotter.log("tau_diff_per", diff_per)

        match self.control_mode:
            case ControlMode.RNEA:
                self.robot.ctrl.set_gains(Kp=2.0, Kv=2.0)
                tau = tau_rnea
            case ControlMode.CT:
                self.robot.ctrl.set_gains(Kp=2.0, Kv=2.0)
                tau = taumj
                tau = taumj + 0.05 * torch.sign(taumj)
            case ControlMode.PID:
                self.robot.ctrl.set_gains(Kp=0.1, Kv=0.1, Ki=1.0)
                dt = t - self.t_prev
                tau = self.robot.ctrl.pid(q, dq, qt, dqt, dt)
            case ControlMode.GC:
                M, h, tau_g = self.robot.dyn.Dynamics_matrices(q, dq)
                tau = tau_g

        for k, v in dict_out.items():
            self.log_data(**{k: v})
        
        return tau


    def logging(self, v, q, dq):

        if self.show_jacobian:
            msg = compare_jacs(self.robot.kin, self.env)
            self.logger.info(msg)
            self.show_jacobian = False
            self.paused = True

        if self.show_frames:
            T_wf = self.robot.kin.forward_kinematics()
            com_wl = ptu.to_numpy(self.robot.kin.com_wl)
            draw_all_frames(
                v.user_scn,
                ptu.to_numpy(T_wf),
                com_wl,
                self.show_com,
                axis_len=0.4,
                width=0.004
            )

        if self.show_dynamics:
            msg = compare_dynamics(q, dq, self.robot.dyn, self.env)
            self.logger.info(msg)
            self.show_dynamics = False
            self.paused = True

        if self.show_inertia:
            log_ic_mass_compare(self.logger, self.robot.kin, self.env)
            self.show_inertia = False


    def step_once(self, v):
        
        if self.reset:
            self.reset = False
            with v.lock():
                self.plotter.clear_log()
                self.env.set_state(self.q0_np, self.dq0_np, 0.0)
                v.sync()


        if self.want_plot:
            self.want_plot = False
            self.plotter.draw_plots()
            self.reset = True


        obs = self.env.observe()
        q = ptu.from_numpy(obs["qpos"])
        dq = ptu.from_numpy(obs["qvel"])
        ddq = ptu.from_numpy(obs["qacc"])

        t = obs["time"]

        if self.tracking_mode == TrackingMode.PT:
            if self.track_current and not self.pt_set:
                qt = q
                self.robot.set_target(qt)
                self.pt_set = True
            elif not self.track_current:
                qt = self.qf
                self.robot.set_target(qt)

        if self.tracking_mode == TrackingMode.PT:
            qt = self.robot.qt
            dqt = self.robot.dqt
            ddqt = self.robot.ddqt
        else:
            qt, dqt, ddqt = self.robot.get_desired_state(t)


        self.robot.kin.step(q=q, qd=dq)

        self.logging(v, q, dq)

        tau = self.compute_tau(q, dq, qt, dqt, ddqt, t)
        tau_n = ptu.to_numpy(tau)
        self.env.step(tau_n)
        self.t_prev = t

        self.log_data(qt=qt, dqt=dqt, ddqt=ddqt)
        self.log_data(t=t, q=q, dq=dq, ddq=ddq)
 
        v.sync()


    def log_data(self, **kwargs):
        for k, v in kwargs.items():
            self.plotter.log(k, v)


    def run(self):
        self.q0_np = ptu.to_numpy(self.q0)
        self.dq0_np = np.zeros_like(self.q0_np)
        self.env.set_state(self.q0_np, self.dq0_np, 0.0)
        self.env.display()

        # optional inertia log once
        log_ic_mass_compare(self.logger, self.robot.kin, self.env)

        with viewer.launch_passive(self.env.m, self.env.d, key_callback=self.key_callback) as v:
            draw_ee_traj(v.user_scn, self.Q_EF_np, radius=0.002, stride=20)
            v.sync()
            while v.is_running():
                if not self.paused:
                    with v.lock():
                        self.step_once(v)
                else:
                    time.sleep(0.1)
                time.sleep(0.00001)

