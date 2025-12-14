from __future__ import annotations
from dataclasses import dataclass
from enum import Enum, auto

import numpy as np
import time

import threading
from rbt_core.planning.trajectory import JointTraj
from rbt_core.robot import CtrlMode
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, MultiDOFJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle

from robots import kinova_gen3
from common_utils import numpy_util as npu
from common_utils import FloatArray
from ros_utils import msg_conv as rmsg
from ros_utils import time_util as rtime
from ros_utils.config import qos_latest

from rlc_common.endpoints import TOPICS, ACTIONS, SERVICES
from rlc_common.endpoints import (
    JointStateMsg, 
    SetControllerGainsSrv, SetControllerModeSrv,
    FollowTrajAction,
)

class TrajectoryType(Enum):
    JOINT = auto()
    CARTESIAN = auto()

class GainType(Enum):
    JOINT = auto()
    CARTESIAN = auto()

@dataclass(slots=True)
class TrajCommand:
    traj: JointTraj
    ti: float
    duration: float
    type: TrajectoryType
    seq: int = 0

@dataclass(slots=True)
class GainsCommand:
    kp: float
    kv: float
    ki: float
    type: GainType

@dataclass(slots=True)
class CtrlSnapshot:
    t: float
    q: FloatArray
    qd: FloatArray
    q_des: FloatArray
    qd_des: FloatArray

class Gen3ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("gen3_controller")

        # ---------- Robot Model ----------
        self.robot = kinova_gen3.init_kinova_robot()
        self.robot.ctrl.set_joint_gains(Kp=1, Kv=1, Ki=1)
        self.robot.set_ctrl_mode(CtrlMode.CT)
        self.robot.set_joint_des()

        # ---------- State ----------
        self._state_lock = threading.Lock()
        self.state = rmsg.JointStateData(
            positions=np.zeros(self.robot.n, dtype=npu.dtype),
            velocities=np.zeros(self.robot.n, dtype=npu.dtype),
            efforts=None,
            stamp=0.0,
            joint_names=self.robot.joint_names,
        )
        self.q = np.zeros(self.robot.n, dtype=npu.dtype)
        self.qd = np.zeros(self.robot.n, dtype=npu.dtype)
        self.t = 0.0

        # ---------- Action State ----------
        self._goal_lock = threading.Lock()
        self._active_goal: ServerGoalHandle | None = None
        self._preempt_evt_by_goal: dict[bytes, threading.Event] = {}

        # ---------- Controller State ----------
        self._snap_lock = threading.Lock()
        self._snap  = CtrlSnapshot(
            t=0.0,
            q=np.zeros(self.robot.n, dtype=npu.dtype),
            qd=np.zeros(self.robot.n, dtype=npu.dtype),
            q_des=np.zeros(self.robot.n, dtype=npu.dtype),
            qd_des=np.zeros(self.robot.n, dtype=npu.dtype),
        )

        # ---------- Trajectory State ----------
        self._traj_lock = threading.Lock()
        self._traj_pending : TrajCommand | None = None
        self._traj_clear_req = threading.Event()

        # ---------- Gains Srv ----------
        self._gains_lock = threading.Lock()
        self._gains_pending : GainsCommand | None = None

        # ---------- Control Mode srv ----------
        self._ctrl_mode_lock = threading.Lock()
        self._ctrl_mode_pending : CtrlMode | None = None


        # ---------- Callback Groups ----------
        self._cb_ctrl = MutuallyExclusiveCallbackGroup()
        self._cb_state = MutuallyExclusiveCallbackGroup()
        self._cb_action = MutuallyExclusiveCallbackGroup()
        self._cb_srv = MutuallyExclusiveCallbackGroup()

        # ---------- Parameters ----------
        self.declare_parameter("controller_rate_hz", 1000.0)
        self.controller_rate = (
            self.get_parameter("controller_rate_hz").get_parameter_value().double_value
        )

        # ---------- Publishers ----------
        self.effort_pub = self.create_publisher(
            TOPICS.effort_cmd.type,
            TOPICS.effort_cmd.name,
            qos_latest,
            callback_group=self._cb_state
        )
        self.ctrl_state_pub = self.create_publisher(
            TOPICS.controller_state.type,
            TOPICS.controller_state.name,
            qos_latest,
            callback_group=self._cb_state
        )
        self.publish_period = 1.0 / self.controller_rate
        # self.publish_timer = self.create_timer(
        #    self.publish_period,
        #    self.control_step,
        #    callback_group=self._cb_ctrl
        #)

        # ---------- Subscribers ----------
        self.joint_state_sub = self.create_subscription(
            TOPICS.joint_state.type,
            TOPICS.joint_state.name,
            self.joint_state_callback,
            qos_latest,
            callback_group=self._cb_state,
        )

        # ---------- Action Servers ----------
        self._traj_action_server = ActionServer(
            self,
            ACTIONS.follow_traj.type,
            ACTIONS.follow_traj.name,
            execute_callback=self._execute_follow_traj,
            handle_accepted_callback=self._accepted_cb,
            callback_group=self._cb_action,
        )

        # ---------- Services ----------
        self._set_ctrl_gains = self.create_service(
            SERVICES.set_controller_gains.type,
            SERVICES.set_controller_gains.name,
            self.set_ctrl_gains_callback,
            callback_group=self._cb_srv,
        )
        self._set_ctrl_mode = self.create_service(
            SERVICES.set_controller_mode.type,
            SERVICES.set_controller_mode.name,
            self.set_ctrl_mode_callback,
            callback_group=self._cb_srv,
        )

        # ---------- Logging ----------
        self.get_logger().info(
            f"Gen3 controller ready with {self.robot.n} joints. "
            f"controller_rate={self.controller_rate} Hz"
        )


    def set_ctrl_gains_callback(self, 
        request: SetControllerGainsSrv.Request, 
        response: SetControllerGainsSrv.Response
    ) -> SetControllerGainsSrv.Response:
        kp = request.kp
        kv = request.kv
        ki = request.ki

        if request.mode is not None: 
            mode = request.mode.strip().lower()
        else: mode = None

        match mode:
            case "joint":
                with self._gains_lock:
                    self._gains_pending = GainsCommand(kp, kv, ki, GainType.JOINT)
            case "impedance":
                with self._gains_lock:
                    self._gains_pending = GainsCommand(kp, kv, ki, GainType.CARTESIAN)
            case _:
                response.success = False
                response.message = (
                    f"Unknown gain mode '{request.mode}'. Expected 'joint' or 'impedance'."
                )
                self.get_logger().error(response.message)
                return response

        response.success = True
        response.message = f"Successfully requested to set {mode} gains: Kp={kp}, Kv={kv}, Ki={ki}"
        self.get_logger().info(response.message)
        return response


    def set_ctrl_mode_callback(self,
        request: SetControllerModeSrv.Request, 
        response: SetControllerModeSrv.Response
    ) -> SetControllerModeSrv.Response:
        if request.mode is not None:
            mode = request.mode.strip().lower()
        else: mode = None

        match mode:
            case "pid":
                with self._ctrl_mode_lock:
                    self._ctrl_mode_pending = CtrlMode.PID
            case "ct":
                with self._ctrl_mode_lock:
                    self._ctrl_mode_pending = CtrlMode.CT
            case "im":
                with self._ctrl_mode_lock:
                    self._ctrl_mode_pending = CtrlMode.IM
            case _:
                response.success = False 
                response.message = (
                    f"Unknown control mode '{request.mode}'. Expected 'pid', 'ct', or 'im'."
                )
                self.get_logger().error(response.message)
                return response
    
        response.success = True
        response.message = f"Successfully requested to set control mode to '{mode}'"
        self.get_logger().info(response.message)
        return response


    def joint_state_callback(self, msg: JointStateMsg) -> None:
        """Callback for joint state messages."""
        s = rmsg.from_joint_state_msg(msg)
        with self._state_lock:
            self.state.joint_names = s.joint_names
            self.state.positions[:] = s.positions
            self.state.velocities[:] = s.velocities
            self.state.stamp = s.stamp

        self.control_step()


    def _consume_traj_requests(self) -> None:
        """Consume trajectory requests. Must be called only from the control loop thread."""
        robot = self.robot

        if self._traj_clear_req.is_set():
            self._traj_clear_req.clear()
            with self._traj_lock: # return to this later. should be ok
                self._traj_pending = None
            robot.clear_traj()
            return

        with self._traj_lock:
            cmd = self._traj_pending
            self._traj_pending = None

        if cmd is None:
            return
        
        if cmd.type == TrajectoryType.JOINT:
            robot.set_joint_traj(cmd.traj, cmd.duration, ti=cmd.ti)
        else:
            self.get_logger().warn(
                f"Received unsupported trajectory type: {cmd.type}"
            )
            return


    def _consume_gains_request(self) -> None:
        """Consume gains request. Must be called only from the control loop thread."""
        robot = self.robot

        with self._gains_lock:
            cmd = self._gains_pending
            self._gains_pending = None

        if cmd is None:
            return
        
        match cmd.type:
            case GainType.JOINT:
                robot.ctrl.set_joint_gains(Kp=cmd.kp, Kv=cmd.kv, Ki=cmd.ki)
                self.get_logger().info(
                    f"Set joint gains: Kp={cmd.kp}, Kv={cmd.kv}, Ki={cmd.ki}"
                )
            case GainType.CARTESIAN:
                robot.ctrl.set_task_gains(Kx=cmd.kp, Dx=cmd.kv, Kix=cmd.ki)
                self.get_logger().info(
                    f"Set Cartesian gains: Kx={cmd.kp}, Dx={cmd.kv}, Kix={cmd.ki}"
                )
            case _:
                self.get_logger().warn(
                    f"Received unsupported gain type: {cmd.type}"
                )
        
        return


    def _consume_mode_request(self) -> None:
        """Consume control mode request. Must be called only from the control loop thread."""
        robot = self.robot

        with self._ctrl_mode_lock:
            mode = self._ctrl_mode_pending
            self._ctrl_mode_pending = None

        if mode is None:
            return

        robot.set_ctrl_mode(mode)
        self.get_logger().info(
            f"Set control mode to '{mode}'"
        )
        return


    def _update_snapshot(self) -> None:
        robot = self.robot
        with self._snap_lock:
            self._snap.t = float(robot.t)
            self._snap.q[:] = robot.q
            self._snap.qd[:] = robot.qd
            self._snap.q_des[:] = robot.q_des
            self._snap.qd_des[:] = robot.qd_des


    def _read_snapshot(self
    ) -> tuple[float, FloatArray, FloatArray, FloatArray, FloatArray]:
        with self._snap_lock:
            t = self._snap.t
            q = self._snap.q.copy()
            qd = self._snap.qd.copy()
            q_des = self._snap.q_des.copy()
            qd_des = self._snap.qd_des.copy()
        return t, q, qd, q_des, qd_des


    def control_step(self) -> None:
        robot = self.robot

        # Update state from latest joint state message
        with self._state_lock:
            self.q[:] = self.state.positions
            self.qd[:] = self.state.velocities
            self.t = self.state.stamp

        # Consume any pending requests
        self._consume_mode_request()
        self._consume_gains_request()

        # Process any trajectory requests
        self._consume_traj_requests()

        # Compute control effort
        robot.set_joint_state(q=self.q, qd=self.qd, t=self.t)
        robot.update_joint_des()
        tau = robot.compute_ctrl_effort()

        # Update snapshot
        self._update_snapshot()

        # Build messages
        cmd_msg = rmsg.to_joint_effort_cmd_msg(
            robot.t,
            robot.joint_names,
            tau,
        )
        state_msg = rmsg.to_joint_ctrl_state_msg(
            robot.t,
            robot.joint_names,
            tau,
            robot.q,
            robot.q_des,
            qd=robot.qd,
            qd_des=robot.qd_des,
            qdd=robot.qdd,
            qdd_des=robot.qdd_des,
        )

        # Publish messages
        self.effort_pub.publish(cmd_msg)
        self.ctrl_state_pub.publish(state_msg)


    def _build_traj_command(self, 
        msg: JointTrajectory
    ) -> TrajCommand | None:
        if len(msg.points) == 0:
            return None

        traj_data = rmsg.from_joint_traj_msg(msg)

        traj = traj_data.traj
        duration = traj.t[-1]

        t_now, *_ = self._read_snapshot()
        ti = t_now

        return TrajCommand(
            traj=traj, 
            ti=ti, 
            duration=duration, 
            type=TrajectoryType.JOINT
        )


    def _accepted_cb(self, 
        goal_handle: ServerGoalHandle
    ) -> None:
        gid = bytes(goal_handle.goal_id.uuid)

        with self._goal_lock:
            if self._active_goal is not None:
                old_gid = bytes(self._active_goal.goal_id.uuid)
                old_evt = self._preempt_evt_by_goal.get(old_gid)
                if old_evt is not None:
                    old_evt.set()

            self._active_goal = goal_handle
            self._preempt_evt_by_goal[gid] = threading.Event()

        t = threading.Thread(target=self._run_goal_worker, args=(goal_handle,), daemon=False)
        t.start()


    def _run_goal_worker(self, 
        goal_handle: ServerGoalHandle
    ) -> None:
        try:
            goal_handle.execute()
        finally:
            gid = bytes(goal_handle.goal_id.uuid)
            with self._goal_lock:
                self._preempt_evt_by_goal.pop(gid, None)
                if self._active_goal is goal_handle:
                    self._active_goal = None


    def _execute_follow_traj(self, 
        goal_handle: ServerGoalHandle
    ) -> FollowTrajAction.Result:
        goal: FollowTrajAction.Goal = goal_handle.request
        result = FollowTrajAction.Result()

        cmd = self._build_traj_command(goal.trajectory)
        if cmd is None:
            result.error_code = FollowTrajAction.Result.INVALID_GOAL
            result.error_string = "Empty or invalid trajectory"
            goal_handle.abort()
            return result

        gid = bytes(goal_handle.goal_id.uuid)
        with self._goal_lock:
            preempt_evt = self._preempt_evt_by_goal[gid]

        self._traj_clear_req.clear()
        with self._traj_lock:
            self._traj_pending = cmd

        tf = cmd.ti + cmd.duration
        feedback = FollowTrajAction.Feedback()
        feedback_dt = 0.02  # 50 Hz

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._traj_clear_req.set()
                goal_handle.canceled()
                result.error_code = FollowTrajAction.Result.SUCCESSFUL
                result.error_string = "Trajectory cancelled"
                return result

            if preempt_evt.is_set():
                # Preempted by a newer goal
                self._traj_clear_req.set()
                goal_handle.abort()
                result.error_code = FollowTrajAction.Result.SUCCESSFUL
                result.error_string = "Preempted by a newer goal"
                return result

            t, q, qd, q_des, qd_des = self._read_snapshot()
            feedback.header.stamp = rtime.to_ros_time(t)
            feedback.actual.positions = q.tolist()
            feedback.actual.velocities = qd.tolist()
            feedback.desired.positions = q_des.tolist()
            feedback.desired.velocities = qd_des.tolist()
            feedback.error.positions = (q_des - q).tolist()
            goal_handle.publish_feedback(feedback)

            if t >= tf:
                break

            time.sleep(feedback_dt)

        goal_handle.succeed()
        result.error_code = FollowTrajAction.Result.SUCCESSFUL
        result.error_string = ""
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Gen3ControllerNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
