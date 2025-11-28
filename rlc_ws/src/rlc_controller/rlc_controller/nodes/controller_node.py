from __future__ import annotations

import numpy as np
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rlc_interfaces.msg import JointEffortCmd, JointStateSim
from rlc_interfaces.srv import SetTrajectory

from rlc_common import topics
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
        self.robot.set_target(np.zeros(self.n, dtype=npu.dtype))
        self.robot.ctrl.set_joint_gains(Kp=2.0, Kv=2.0, Ki=0.0)

        self.q = np.zeros(self.n, dtype=npu.dtype)
        self.qd = np.zeros_like(self.q)
        self.t: float = 0.0
        self.t_pref: float = 0.0
        self.tau = np.zeros(self.n, dtype=npu.dtype)

        self.has_traj = False

        self.declare_parameter("controller_rate_hz", 200.0)
        self.controller_rate = (
            self.get_parameter("controller_rate_hz").get_parameter_value().double_value
        )

        self.joint_state_sub = self.create_subscription(
            JointStateSim,
            topics.JOINT_STATE_TOPIC,
            self.joint_state_callback,
            10,
        )

        self.effort_pub = self.create_publisher(
            JointEffortCmd,
            topics.EFFORT_COMMAND_TOPIC,
            10,
        )

        self.publish_period = 1.0 / self.controller_rate
        self.publish_timer = self.create_timer(
            self.publish_period, self.publish_effort_cmd
        )

        self.set_traj_service = self.create_service(
            SetTrajectory,
            topics.SET_TRAJECTORY_SERVICE,
            self.set_trajectory_callback,
        )

        self.get_logger().info(
            f"Gen3 controller ready with {self.n} joints. "
            f"controller_rate={self.controller_rate} Hz"
        )

        self.control_mode = ControlMode.CT
        self.tracking_mode = TrackingMode.PT


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


    def set_trajectory_callback(
        self,
        request: SetTrajectory.Request,
        response: SetTrajectory.Response,
    ) -> SetTrajectory.Response:
        n = self.n
        n_pts = int(request.n_points)

        if n_pts <= 0:
            response.success = False
            response.message = "n_points must be positive"
            return response
        if request.freq <= 0.0:
            response.success = False
            response.message = "freq must be positive"
            return response

        expected_len = n_pts * n
        try:
            q = np.asarray(request.positions, dtype=npu.dtype).reshape(n_pts, n)
            qd = np.asarray(request.velocities, dtype=npu.dtype).reshape(n_pts, n)
            qdd = np.asarray(request.accelerations, dtype=npu.dtype).reshape(n_pts, n)
        except ValueError:
            response.success = False
            response.message = (
                f"positions/velocities/accelerations must have length {expected_len}"
            )
            return response

        T = np.stack([q, qd, qdd], axis=0)
        self.robot.set_trajectory(T, freq=request.freq, ti=0.0)
        self.has_traj = True

        response.success = True
        response.message = (
            f"Trajectory loaded: {n_pts} points, {n} joints, freq={request.freq} Hz"
        )
        return response


    def compute_effort(self) -> FloatArray:
        q = self.q
        qd = self.qd
        t = self.t

        match self.tracking_mode:
            case TrackingMode.PT:
                q_des = self.robot.q_des
                qd_des = self.robot.qd_des
                qdd_des = np.zeros_like(q_des)

            case TrackingMode.TRAJ:
                q_des, qd_des, qdd_des = self.robot.get_desired_state(t)


        match self.control_mode:
            case ControlMode.CT:
                tau = self.robot.ctrl.computed_torque(q, qd, q_des, qd_des, qdd_des)
                return tau

            case ControlMode.PID:
                dt = self.t - self.t_pref
                tau = self.robot.ctrl.pid(q, qd, q_des, qd_des, dt)
        
        return tau


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
