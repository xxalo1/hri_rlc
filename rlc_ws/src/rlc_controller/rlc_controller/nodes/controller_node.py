from __future__ import annotations

import numpy as np
from enum import Enum, auto
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle

from rlc_common.endpoints import TOPICS, ACTIONS, SERVICES
from rlc_common.endpoints import (
    JointStateMsg, JointEffortCmdMsg, 
    SetControllerGainsSrv, SetControllerModeSrv,
    FollowTrajAction,
)
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
        self.joint_names = list(self.robot.spec.joint_names)
        self.robot.ctrl.set_joint_gains(Kp=1.0, Kv=1.0, Ki=0.0)

        q_des = np.array(
            [np.pi/4, -np.pi/2, np.pi/3, -np.pi/3, 0.0, np.pi/6, 0.0], 
            dtype=npu.dtype
        )
        self.robot.setup_quintic_traj(q_des, tf=5.0, freq=1000.0)

        self.traj_duration = 0.0
        self.traj_start_t = 0.0
        self._active_goal_handle = None

        self.declare_parameter("controller_rate_hz", 200.0)
        self.controller_rate = (
            self.get_parameter("controller_rate_hz").get_parameter_value().double_value
        )

        self.joint_state_sub = self.create_subscription(
            TOPICS.joint_state.type,
            TOPICS.joint_state.name,
            self.joint_state_callback,
            10,
        )

        self.effort_pub = self.create_publisher(
            TOPICS.effort_cmd.type,
            TOPICS.effort_cmd.name,
            10,
        )

        self._traj_action_server = ActionServer(
            self,
            ACTIONS.follow_traj.type,
            ACTIONS.follow_traj.name,  # e.g. "follow_joint_trajectory"
            execute_callback=self.follow_trajectory,
        )

        self._set_controller_gains = self.create_service(
            SERVICES.set_controller_gains.type,
            SERVICES.set_controller_gains.name,
            self.set_gains_callback,
        )
        self._set_controller_mode = self.create_service(
            SERVICES.set_controller_mode.type,
            SERVICES.set_controller_mode.name,
            self.set_mode_callback,
        )

        self.publish_period = 1.0 / self.controller_rate
        self.publish_timer = self.create_timer(
            self.publish_period, self.publish_effort_cmd
        )

        self.get_logger().info(
            f"Gen3 controller ready with {self.robot.n} joints. "
            f"controller_rate={self.controller_rate} Hz"
        )

    def set_gains_callback(self, 
        request: SetControllerGainsSrv.Request, 
        response: SetControllerGainsSrv.Response
    ) -> SetControllerGainsSrv.Response:
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


    def set_mode_callback(self, 
        request: SetControllerModeSrv.Request, 
        response: SetControllerModeSrv.Response
    ) -> SetControllerModeSrv.Response:
        mode = (request.mode or "").strip().lower()

        try:
            self.robot.set_ctrl_mode(mode)
        except ValueError as e:
            response.success = False 
            response.message = str(e)
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = f"Set control mode to '{mode}'"
        self.get_logger().info(response.message)
        return response
    

    def joint_state_callback(self, msg: JointStateMsg) -> None:
        q = np.asarray(msg.position, dtype=npu.dtype)
        qd = np.asarray(msg.velocity, dtype=npu.dtype)
        qdd = np.asarray(msg.acceleration, dtype=npu.dtype)
        t = msg.sim_time

        self.robot.set_joint_state(q=q, qd=qd, qdd=qdd, t=t)
        self.robot.update_joint_des()


    def publish_effort_cmd(self) -> None:
        tau = self.robot.compute_ctrl_effort()

        cmd = JointEffortCmdMsg()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = self.joint_names
        cmd.effort = tau.tolist()
        self.effort_pub.publish(cmd)


    def _load_trajectory(self, msg: JointTrajectory) -> bool:
        """Load JointTrajectory into the robot, minimal assumptions/checks."""
        points: list[JointTrajectoryPoint] = list(msg.points)
        n_pts = len(points)

        if n_pts == 0:
            self.get_logger().warning("Received empty JointTrajectory")
            return False

        q = np.asarray([p.positions for p in points], dtype=npu.dtype)
        qd = np.asarray([p.velocities for p in points], dtype=npu.dtype)
        qdd = np.asarray([p.accelerations for p in points], dtype=npu.dtype)
        traj = np.stack([q, qd, qdd], axis=0)

        last = points[-1].time_from_start
        time_to_end = float(last.sec) + float(last.nanosec) * 1e-9
        tf = self.robot.t + time_to_end

        self.robot.set_joint_traj(traj, tf)

        self.get_logger().info(
            f"Trajectory loaded: {n_pts} points, {self.robot.n} joints"
        )
        return True


    def follow_trajectory(self, 
        goal_handle: ServerGoalHandle,
    ) -> FollowTrajAction.Result:
        """Action callback."""
        goal: FollowTrajAction.Goal = goal_handle.request
        traj_msg = goal.trajectory

        ok = self._load_trajectory(traj_msg)
        result = FollowTrajAction.Result()

        if not ok:
            result.error_code = FollowTrajAction.Result.INVALID_GOAL
            result.error_string = "Empty or invalid trajectory"
            goal_handle.abort()
            return result

        feedback = FollowTrajAction.Feedback()
        self.get_logger().info(
            f"Executing FollowTrajAction: "
            f"{len(traj_msg.points)} points over {self.traj_duration:.3f} s"
        )

        robot = self.robot
        t = robot.t
        tf = robot.tf
        while rclpy.ok() and t < tf:

            # handle cancel
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Trajectory cancel requested")
                robot.clear_traj()
                goal_handle.canceled()
                result.error_code = FollowTrajAction.Result.SUCCESSFUL
                result.error_string = "Trajectory cancelled"
                return result

            t = robot.t
            q_des, qd_des = robot.q_des, robot.qd_des

            # publish feedback
            feedback.actual.positions = robot.q.tolist()
            feedback.actual.velocities = robot.qd.tolist()
            feedback.desired.positions = q_des.tolist()
            feedback.desired.velocities = qd_des.tolist()
            e = q_des - robot.q
            feedback.error.positions = e.tolist()
            goal_handle.publish_feedback(feedback)

            time.sleep(self.publish_period)

        # Done
        goal_handle.succeed()
        result.error_code = FollowTrajAction.Result.SUCCESSFUL
        result.error_string = ""
        self.get_logger().info("FollowTrajAction finished successfully")
        return result


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
