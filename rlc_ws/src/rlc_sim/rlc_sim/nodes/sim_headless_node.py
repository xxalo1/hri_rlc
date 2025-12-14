from __future__ import annotations

import threading
import time

from netplan import State
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rosgraph_msgs.msg import Clock
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sim_backend.mujoco.mujoco_base import Observation, StateAction
from sim_env.mujoco.env import Gen3Env
from robots.kinova_gen3 import init_kinova_robot
from common_utils import numpy_util as npu, FloatArray
from ros_utils import msg_conv as rmsg
from ros_utils import time_util as rtime
from ros_utils.config import qos_latest
from rbt_core.robot import CtrlMode

from rlc_common.endpoints import (
    TOPICS, SERVICES, ACTIONS, 
    JointEffortCmdMsg,
    ResetSimSrv, PauseSimSrv,
)

class Gen3MujocoSimNode(Node):
    def __init__(self, 
        joint_names: list[str]
    ) -> None:
        super().__init__("gen3_mujoco_sim")
        self.robot = init_kinova_robot()
        self.robot.ctrl.set_joint_gains(Kp=1, Kv=1, Ki=1)
        self.robot.set_ctrl_mode(CtrlMode.CT)

        self.joint_names = joint_names
        self.n = len(joint_names)

        # ---------- Command ----------
        self._cmd_lock = threading.Lock()
        self._cmd_pending = np.zeros(self.n, dtype=npu.dtype)
        self.cmd = np.zeros(self.n, dtype=npu.dtype)
        self._cmd_fresh = threading.Event()

        # ---------- State ----------
        self._state_lock = threading.Lock()
        self.state = Observation(
            t = 0.0,
            q = np.zeros(self.n, dtype=npu.dtype),
            qd = np.zeros(self.n, dtype=npu.dtype),
            qdd = np.zeros(self.n, dtype=npu.dtype),
        )
        self._state_back = Observation(
            t = 0.0,
            q = np.zeros(self.n, dtype=npu.dtype),
            qd = np.zeros(self.n, dtype=npu.dtype),
            qdd = np.zeros(self.n, dtype=npu.dtype),
        )
        self.state_action = StateAction(
            t = 0.0,
            q = np.zeros(self.n, dtype=npu.dtype),
            qd = np.zeros(self.n, dtype=npu.dtype),
            qdd = np.zeros(self.n, dtype=npu.dtype),
            effort_applied = np.zeros(self.n, dtype=npu.dtype),
            effort_bl = np.zeros(self.n, dtype=npu.dtype),
        )
        self._state_action_back = StateAction(
            t = 0.0,
            q = np.zeros(self.n, dtype=npu.dtype),
            qd = np.zeros(self.n, dtype=npu.dtype),
            qdd = np.zeros(self.n, dtype=npu.dtype),
            effort_applied = np.zeros(self.n, dtype=npu.dtype),
            effort_bl = np.zeros(self.n, dtype=npu.dtype),
        )

        # ---------- Control flags ----------
        self._pause_req = threading.Event()
        self._reset_req = threading.Event()

        # ---------- ROS parameters ----------
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("realtime_factor", 1.0)
        
        self.publish_rate = self.get_parameter(
            "publish_rate_hz"
            ).get_parameter_value().double_value
        
        self.realtime_factor = self.get_parameter(
            "realtime_factor"
            ).get_parameter_value().double_value

        # ---------- Callback Groups ----------
        self._cb_cmd = MutuallyExclusiveCallbackGroup()
        self._cb_pub = MutuallyExclusiveCallbackGroup()
        self._cb_srv = MutuallyExclusiveCallbackGroup()

        # ---------- Publishers ----------
        self.joint_state_pub = self.create_publisher(
            TOPICS.joint_state.type, 
            TOPICS.joint_state.name, 
            qos_latest,
            callback_group=self._cb_pub,
        )

        self.joint_state_action_pub = self.create_publisher(
            TOPICS.joint_state_action.type, 
            TOPICS.joint_state_action.name, 
            qos_latest,
            callback_group=self._cb_pub,
        )

        self.publish_period = 1.0 / self.publish_rate
        self.publish_timer = self.create_timer(
            self.publish_period, 
            self.publish_joint_state,
            callback_group=self._cb_pub,
        )

        # ---------- Subscribers ----------
        self.torque_sub = self.create_subscription(
            TOPICS.effort_cmd.type,
            TOPICS.effort_cmd.name,
            self.torque_cmd_callback,
            qos_latest,
            callback_group=self._cb_cmd,
        )

        # ---------- Services ----------
        self.reset_srv = self.create_service(
            SERVICES.reset_sim.type,
            SERVICES.reset_sim.name,
            self.reset_service_callback,
            callback_group=self._cb_srv,
        )
        self.pause_srv = self.create_service(
            SERVICES.pause_sim.type,
            SERVICES.pause_sim.name,
            self.pause_service_callback,
            callback_group=self._cb_srv,
        )

        # ---------- Info log ----------
        self.get_logger().info(
            f"Headless Gen3 MuJoCo sim ready with {self.n} joints. "
            f"publish_rate={self.publish_rate} Hz, realtime_factor={self.realtime_factor}"
        )


    def reset_service_callback(self, 
        request: ResetSimSrv.Request, 
        response: ResetSimSrv.Response
    ) -> ResetSimSrv.Response:
        """
        Reset the simulation.
        """
        self._reset_req.set()
        response.success = True
        response.message = "Reset requested."
        return response


    def pause_service_callback(self, 
        request: PauseSimSrv.Request, 
        response: PauseSimSrv.Response
    ) -> PauseSimSrv.Response:
        """
        Pause/unpause physics stepping.
        """

        if request.data: self._pause_req.set()
        else: self._pause_req.clear()

        response.success = True
        state = "pause requested" if self._pause_req.is_set() else "run requested"
        response.message = f"Simulation is now {state}."
        return response


    def torque_cmd_callback(self, msg: JointEffortCmdMsg) -> None:
        state = rmsg.from_joint_effort_cmd_msg(msg)
        tau = state.efforts
        if tau.size != self.n:
            self.get_logger().warn(
                f"Received tau size {tau.size}, expected {self.n}"
            )
            return
        with self._cmd_lock:
            self._cmd_pending[:] = tau
            self._cmd_fresh.set()


    def compute_tau(self, obs: Observation) -> FloatArray:
        robot = self.robot
        t = obs.t
        q = obs.q
        qd = obs.qd

        self.robot.set_joint_state(q=q, qd=qd, t=t)
        self.robot.update_joint_des()
        tau = robot.compute_ctrl_effort()
        return tau


    def publish_joint_state(self) -> None:
        with self._state_lock:
            t_state = self.state.t
            q_state = self.state.q.copy()
            qd_state = self.state.qd.copy()

            t_st_act = self.state_action.t
            q_st_act = self.state_action.q.copy()
            qd_st_act = self.state_action.qd.copy()
            effort = self.state_action.effort_applied.copy()
            effort_bl = self.state_action.effort_bl.copy()

        msg_st_act = rmsg.to_joint_state_action_msg(
            stamp=t_st_act,
            joint_names=self.joint_names,
            position=q_st_act,
            velocity=qd_st_act,
            action=effort,
            action_baseline=effort_bl,
        )
        msg_state = rmsg.to_joint_state_msg(
            stamp=t_state,
            joint_names=self.joint_names,
            positions=q_state,
            velocities=qd_state,
        )
        self.joint_state_action_pub.publish(msg_st_act)
        self.joint_state_pub.publish(msg_state)

        # If you want /clock and use_sim_time, publish it here too
        # clock_msg = Clock()
        # clock_msg.clock = rtime.to_ros_time(t)
        # self.clock_pub.publish(clock_msg)


    def step_cmd(self) -> FloatArray:
        if self._cmd_fresh.is_set():
            with self._cmd_lock:
                self.cmd[:] = self._cmd_pending
                self._cmd_fresh.clear()
        return self.cmd


    def reset_cmd(self) -> FloatArray:
        with self._cmd_lock:
            self._cmd_pending[:] = 0.0
            self.cmd[:] = 0.0
            self._cmd_fresh.clear()
        return self.cmd


    def step_state(self) -> None:
        with self._state_lock:
            self.state, self._state_back = self._state_back, self.state
            self.state_action, self._state_action_back \
                = self._state_action_back, self.state_action
        self._cmd_fresh.clear()


def physics_loop(
    env: Gen3Env, 
    node: Gen3MujocoSimNode,
    stop_evt: threading.Event,
    rt : float = 1.0
) -> None:
    env.reset()
    dt_phy = env.m.opt.timestep

    t0_wall = time.perf_counter()
    k = 0

    obs = env.observe()
    while rclpy.ok() and not stop_evt.is_set():
        if node._reset_req.is_set():
            env.reset()
            cmd = node.reset_cmd()
            obs = env.observe_into(node._state_back)
            node.step_state()
            k = 0
            t0_wall = time.perf_counter()
            node._reset_req.clear()

        if node._pause_req.is_set():
            time.sleep(0.01)
            continue
        
        node._cmd_fresh.wait()
        cmd = node.step_cmd()
        effort_bl = node.compute_tau(obs)
        obs = env.step(cmd, mode="torque", nsub=1)

        env.observe_into(node._state_back)
        env.state_action_into(node._state_action_back, effort_bl=effort_bl)
        node.step_state()

        k += 1
        time.sleep(0.001)
        continue  # TEMP DISABLE REALTIME SLEEP
        target_wall = t0_wall + (k * dt_phy) / rt
        now = time.perf_counter()
        sleep_s = target_wall - now
        if sleep_s > 0:
            time.sleep(sleep_s)


def main(args=None) -> None:
    rclpy.init(args=args)
    env = Gen3Env.from_default_scene()
    node = Gen3MujocoSimNode(env.joint_names)
    rt_factor = 0.1

    stop_evt = threading.Event()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=False)

    phys_thread = threading.Thread(
        target=physics_loop,
        args=(env, node, stop_evt),
        kwargs={"rt": rt_factor},
        daemon=False,
    )

    spin_thread.start()
    phys_thread.start()

    try:
        while rclpy.ok() and not stop_evt.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop_evt.set()
        executor.shutdown()
        phys_thread.join(timeout=2.0)
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
