from __future__ import annotations
import time
import numpy as np
from mujoco import viewer

from rbt_core.robot import CtrlMode

from .ops import compare_jacs, compare_dynamics, log_ic_mass_compare
from robots.kinova_gen3 import init_kinova_robot
from sim_env.mujoco.env import Gen3Env
from sim_backend.mujoco.util import draw_all_frames, draw_ee_traj
from sim_env.mujoco.viz_env import VizEnv
from common_utils import numpy_util as npu

FloatArray = npu.FloatArray


class Gen3App:
    def __init__(self, logger):
        self.logger = logger

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
        self.robot.ctrl.set_joint_gains(Kp=2.0, Kv=2.0, Ki=1.0)
        self.env = Gen3Env.from_default_scene()
        self.viz = VizEnv(Gen3Env.from_default_scene(), 
                          on_pause=self.on_pause, 
                          on_reset=self.on_reset)

        # basic trajectory
        self.q0 = np.zeros(self.robot.kin.n, dtype=npu.dtype)
        self.qd0 = np.zeros_like(self.q0)
        qf = np.array([np.pi/4, -np.pi/2, np.pi/3, -np.pi/3, 0.0, np.pi/6, 0.0], dtype=npu.dtype)

        self.robot.setup_quintic_traj(qf, freq=1000.0, duration=5.0)
        self.cart_traj = self.robot.get_cartesian_traj()


    def key_callback(self, keycode):
        c = chr(keycode)
        match c:
            case " ":
                self.paused = not self.paused

            case "r" | "R":
                self.reset = True
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

            case "P" | "p":  
                self.robot.set_ctrl_mode(CtrlMode.PID)
                self.robot.ctrl.set_joint_gains(Kp=0.1, Kv=0.1, Ki=1.0)

            case "c" | "C":
                self.robot.set_ctrl_mode(CtrlMode.CT)
                self.robot.ctrl.set_joint_gains(Kp=2.0, Kv=2.0)

            case "m" | "M":
                self.robot.set_ctrl_mode(CtrlMode.IM)
                self.robot.ctrl.set_task_gains(Kx=2.0, Dx=2.0, Kix=1.0)

            case "x" | "X":
                self.robot.clear_traj()


    def on_reset(self):
        self.env.set_state(self.q0, self.qd0, 0.0)
        obs = self.env.observe()
        self.robot.set_joint_state(q=obs.q, qd=obs.qd, t=obs.t)


    def on_pause(self, paused: bool):
        pass


    def logging(self, v):
        if self.show_jacobian:
            msg = compare_jacs(self.robot.kin, self.env)
            self.logger.info(msg)
            self.show_jacobian = False
            self.paused = True

        if self.show_frames:
            T_wf = self.robot.kin.forward_kinematics()
            com_wl = self.robot.kin.com_wl
            draw_all_frames(
                v.user_scn,
                T_wf,
                com_wl,
                self.show_com,
                axis_len=0.4,
                width=0.004
            )

        if self.show_dynamics:
            msg = compare_dynamics(self.robot.dyn, self.env)
            self.logger.info(msg)
            self.show_dynamics = False
            self.paused = True

        if self.show_inertia:
            log_ic_mass_compare(self.logger, self.robot.kin, self.env)
            self.show_inertia = False


    def step_once(self, v):
        if self.reset:
            self.reset = False
            self.env.set_state(self.q0, self.qd0, 0.0)
            v.sync()

        obs = self.env.observe()

        self.robot.set_joint_state(q=obs.q, qd=obs.qd, t=obs.t)


        self.robot.update_joint_des()
        tau = self.robot.compute_ctrl_effort()
        self.env.step(tau, nsub=1)
        self.viz.set_joint_states(q=obs.q, qd=obs.qd, t=obs.t)
        self.viz.sync(v)

        v.sync()


    def run(self):
        self.env.set_state(self.q0, self.qd0, 0.0)
        self.env.display()

        log_ic_mass_compare(self.logger, self.robot.kin, self.env)

        with viewer.launch_passive(self.viz.env.m, self.viz.env.d, key_callback=self.viz.key_callback) as v:
            while v.is_running():
                if not self.paused:
                    with v.lock():
                        self.step_once(v)
                else:
                    time.sleep(0.1)
                time.sleep(0.0001)


if __name__ == "__main__":
    import logging

    logger = logging.getLogger("Gen3App")
    logger.setLevel(logging.INFO)
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    ch.setFormatter(formatter)
    logger.addHandler(ch)
    app = Gen3App(logger=logger)
    app.run()


