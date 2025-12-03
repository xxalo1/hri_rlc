from __future__ import annotations
from unittest import case

import numpy as np
from enum import Enum, auto
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory

from rlc_interfaces.msg import JointEffortCmd, JointStateSim
from rlc_interfaces.srv import SetControllerGains, SetControllerMode
from rlc_common import endpoints
from robots.kinova_gen3 import init_kinova_robot
from common_utils import numpy_util as npu
from common_utils import FloatArray


class ControlMode(Enum):
    CT = auto()
    PID = auto()
    IM = auto()


class TrackingMode(Enum):
    PT = auto()
    TRAJ = auto()


class Gen3ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("gen3_controller")
        self.robot = init_kinova_robot()
        self.n = self.robot.kin.n
        self.joint_names = list(self.robot.spec.joint_names)
        self.robot.ctrl.set_joint_gains(Kp=2.0, Kv=2.0, Ki=0.0)


        self.q = np.zeros(self.n, dtype=npu.dtype)
        self.qd = np.zeros_like(self.q)
        q_des = np.array([np.pi/4, -np.pi/2, np.pi/3, -np.pi/3, 0.0, np.pi/6, 0.0], dtype=npu.dtype)
        self.robot.setup_quintic_traj(freq=1000.0, ti=0.0, tf=5.0, q_des=q_des)
        self.t = 0.0
        self.t_pref = 0.0
        self.tau = np.zeros(self.n, dtype=npu.dtype)

        self.traj_duration = 0.0
        self.traj_start_t = 0.0
        self._active_goal_handle = None

        self.declare_parameter("controller_rate_hz", 200.0)
        self.controller_rate = (
            self.get_parameter("controller_rate_hz").get_parameter_value().double_value
        )

        self.joint_state_sub = self.create_subscription(
            JointStateSim,
            endpoints.JOINT_STATE_TOPIC,
            self.joint_state_callback,
            10,
        )

        self.effort_pub = self.create_publisher(
            JointEffortCmd,
            endpoints.EFFORT_COMMAND_TOPIC,
            10,
        )

        self.publish_period = 1.0 / self.controller_rate
        self.publish_timer = self.create_timer(
            self.publish_period, self.publish_effort_cmd
        )

        self._traj_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            endpoints.FOLLOW_TRAJECTORY_ACTION,  # e.g. "follow_joint_trajectory"
            execute_callback=self.follow_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self._set_joint_gains_service = self.create_service(
            SetControllerGains,
            endpoints.SET_GAINS_SERVICE,
            self.set_gains_callback,
        )
        self_set_controller_mode = self.create_service(
            SetControllerMode,
            endpoints.SET_CONTROL_MODE_SERVICE,
            self.set_mode_callback,
        )

        self.get_logger().info(
            f"Gen3 controller ready with {self.n} joints. "
            f"controller_rate={self.controller_rate} Hz"
        )

<<<<<<< HEAD
        self.control_mode = ControlMode.PID
=======
        self.control_mode = ControlMode.CT
>>>>>>> 1105632288f60f141b1f98b0e04ba2e3712ef373
        self.tracking_mode = TrackingMode.TRAJ


    def set_gains_callback(self, request, response):
        kp = request.kp
        kv = request.kv
        ki = request.ki
        mode = (request.mode or "").strip().lower() or "joint"

        match mode:
            case "joint":
                self.robot.ctrl.set_joint_gains(kp, kv, ki)
            case "impedance":
                self.robot.ctrl.set_task_gains(kp, kv, ki)
            case _:
                response.success = False
                response.message = (
                    f"Unknown gain mode '{request.mode}'. Expected 'joint' or 'impedance'."
                )
                self.get_logger().error(response.message)
                return response

        response.success = True
        response.message = f"Set {mode} gains: Kp={kp}, Kv={kv}, Ki={ki}"
        self.get_logger().info(response.message)
        return response


    def set_mode_callback(self, request, response):
        mode = (request.mode or "").strip().lower()

        match mode:
            case "ct":
                self.control_mode = ControlMode.CT
            case "pid":
                self.control_mode = ControlMode.PID
            case "im":
                self.control_mode = ControlMode.IM
            case _:
                response.success = False
                response.message = (
                    f"Unknown control mode '{request.mode}'. Expected 'ct', 'pid', or 'im'."
                )
                self.get_logger().error(response.message)
                return response

        response.success = True
        response.message = f"Set control mode to '{mode}'"
        self.get_logger().info(response.message)
        return response
    

    def joint_state_callback(self, msg: JointStateSim) -> None:
        self.q = np.asarray(msg.position, dtype=npu.dtype)
        self.qd = np.asarray(msg.velocity, dtype=npu.dtype)
        self.t = msg.sim_time


    def publish_effort_cmd(self) -> None:
        tau = self.compute_effort()
        self.tau = tau

        cmd = JointEffortCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = self.joint_names
        cmd.effort = tau.tolist()
        self.effort_pub.publish(cmd)
        
        self.t_pref = self.t


    def compute_effort(self) -> FloatArray:
        q = self.q
        qd = self.qd
        t = self.t
        t_pref = self.t_pref

        self.robot.kin.step(q=q, qd=qd)

        match self.tracking_mode:
            case TrackingMode.PT:
                q_des = self.robot.q_des
                qd_des = self.robot.qd_des
                qdd_des = self.robot.qdd_des
            case TrackingMode.TRAJ:
                q_des, qd_des, qdd_des = self.robot.get_desired_state(t)

        match self.control_mode:
            case ControlMode.CT:
                tau = self.robot.ctrl.computed_torque(q, qd, q_des, qd_des, qdd_des)

            case ControlMode.PID:
                dt = t - t_pref
                tau = self.robot.ctrl.pid(q, qd, q_des, qd_des, dt)
            
            case _:
                tau = np.zeros(self.n, dtype=npu.dtype)

        return tau


    def _load_trajectory(self, msg: JointTrajectory) -> bool:
        """Load JointTrajectory into the robot, minimal assumptions/checks."""
        points = msg.points
        n_pts = len(points)

        if n_pts == 0:
            self.get_logger().warning("Received empty JointTrajectory")
            self.traj_duration = 0.0
            return False

        # Assume positions/velocities/accelerations are all provided and consistent.
        q = np.asarray([p.positions for p in points], dtype=npu.dtype)
        qd = np.asarray([p.velocities for p in points], dtype=npu.dtype)
        qdd = np.asarray([p.accelerations for p in points], dtype=npu.dtype)

        # Duration from time_from_start of last point
        last = points[-1].time_from_start
        time_to_end = float(last.sec) + float(last.nanosec) * 1e-9
        steps = max(n_pts - 1, 1)
        freq = steps / time_to_end

        T = np.stack([q, qd, qdd], axis=0)

        self.robot.set_trajectory(T, freq=freq, ti=self.t)
        self.traj_duration = time_to_end

        self.get_logger().info(
            f"Trajectory loaded: {n_pts} points, {self.n} joints, freq={freq:.3f} Hz"
        )
        return True


    def follow_trajectory(self, goal_handle):
        """Action callback for control_msgs/FollowJointTrajectory."""
        goal: FollowJointTrajectory.Goal = goal_handle.request
        traj_msg: JointTrajectory = goal.trajectory

        ok = self._load_trajectory(traj_msg)
        result = FollowJointTrajectory.Result()

        if not ok:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Empty or invalid trajectory"
            goal_handle.abort()
            return result

        self.tracking_mode = TrackingMode.TRAJ
        # Start timing
        self.traj_start_t = self.t
        end_time = self.traj_start_t + self.traj_duration

        feedback = FollowJointTrajectory.Feedback()

        self.get_logger().info(
            f"Executing FollowJointTrajectory: "
            f"{len(traj_msg.points)} points over {self.traj_duration:.3f} s"
        )

        while rclpy.ok() and self.t < end_time:
            # Handle cancel
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Trajectory cancel requested")
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "Trajectory cancelled"
                return result

            # time since trajectory start
            t_rel = self.t - self.traj_start_t

            # Get desired from your robot API
            try:
                q_des, qd_des, _ = self.robot.get_desired_state(t_rel)
            except Exception:
                q_des = self.q
                qd_des = self.qd

            # Fill feedback fields (minimal: just positions/velocities)
            feedback.actual.positions = self.q.tolist()
            feedback.actual.velocities = self.qd.tolist()
            feedback.desired.positions = q_des.tolist()
            feedback.desired.velocities = qd_des.tolist()
            e = q_des - self.q
            feedback.error.positions = e.tolist()

            goal_handle.publish_feedback(feedback)

            time.sleep(self.publish_period)

        # Done
        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = ""
        self.get_logger().info("FollowJointTrajectory finished successfully")
        return result


    def goal_callback(
        self, goal_request: FollowJointTrajectory.Goal
    ) -> GoalResponse:
        """Decide whether to accept a new trajectory goal.

        Minimal version: always accept.
        """
        self.get_logger().info("Received FollowJointTrajectory goal")
        return GoalResponse.ACCEPT


    def cancel_callback(self, goal_handle) -> CancelResponse:
        """Decide whether to accept a cancel request.

        Minimal version: always accept.
        """
        self.get_logger().info("Received request to cancel FollowJointTrajectory goal")
        return CancelResponse.ACCEPT


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Gen3ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
