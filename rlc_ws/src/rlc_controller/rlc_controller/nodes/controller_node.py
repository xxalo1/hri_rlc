from __future__ import annotations

import numpy as np
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle

from robots.kinova_gen3 import init_kinova_robot
from common_utils import numpy_util as npu
from common_utils import FloatArray
from ros_utils import msg_conv as rmsg
from ros_utils import time_util as rtime

from rlc_common.endpoints import TOPICS, ACTIONS, SERVICES
from rlc_common.endpoints import (
    JointStateMsg, 
    SetControllerGainsSrv, SetControllerModeSrv,
    FollowTrajAction,
)

class Gen3ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("gen3_controller")

        self.robot = init_kinova_robot()
        self.robot.ctrl.set_joint_gains(Kp=1.0, Kv=1.0, Ki=0.0)

        self.declare_parameter("controller_rate_hz", 2000.0)
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
        self.ctrl_state_pub = self.create_publisher(
            TOPICS.controller_state.type,
            TOPICS.controller_state.name,
            10,
        )

        self._traj_action_server = ActionServer(
            self,
            ACTIONS.follow_traj.type,
            ACTIONS.follow_traj.name,  # e.g. "follow_joint_trajectory"
            execute_callback=self.follow_trajectory,
        )

        self._set_ctrl_gains = self.create_service(
            SERVICES.set_controller_gains.type,
            SERVICES.set_controller_gains.name,
            self.set_gains_callback,
        )
        self._set_ctrl_mode = self.create_service(
            SERVICES.set_controller_mode.type,
            SERVICES.set_controller_mode.name,
            self.set_mode_callback,
        )

        self.publish_period = 1.0 / self.controller_rate
        self.publish_timer = self.create_timer(
            self.publish_period, self.control_step
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
        """Callback for joint state messages."""
        state = rmsg.from_joint_state_msg(msg)

        t = state.stamp
        q = state.positions
        qd = state.velocities

        self.robot.set_joint_state(q=q, qd=qd, t=t)
        self.robot.update_joint_des()
        tau = self.robot.compute_ctrl_effort()

        cmd_msg = rmsg.to_joint_effort_cmd_msg(
            self.robot.t,
            self.robot.joint_names,
            tau,
        )
        self.effort_pub.publish(cmd_msg)


    def control_step(self) -> None:
        return
        robot = self.robot
        tau = robot.compute_ctrl_effort()

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

        self.effort_pub.publish(cmd_msg)
        self.ctrl_state_pub.publish(state_msg)


    def _load_trajectory(self, msg: JointTrajectory) -> bool:
        """Load JointTrajectory into the robot, minimal assumptions/checks."""
        n_pts = len(msg.points)

        if n_pts == 0:
            self.get_logger().warning("Received empty JointTrajectory")
            return False

        joint_traj = rmsg.from_joint_traj_msg(msg)
        traj = joint_traj.traj

        duration = float(joint_traj.time_from_start[-1])
        self.robot.set_joint_traj(traj, duration, ti=self.robot.t)

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
            f"{len(traj_msg.points)} points over {self.robot.tf - self.robot.ti:.3f} s"
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
            feedback.header.stamp = rtime.to_ros_time(t)
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
