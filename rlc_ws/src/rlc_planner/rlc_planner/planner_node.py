"""Trajectory planner node for building and sending joint trajectories."""

from __future__ import annotations

from typing import Iterable, Optional

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rlc_common import endpoints
from rlc_interfaces.msg import JointStateSim
from rlc_interfaces.srv import SetTargetPoint
from rbt_core import TrajPlanner
from common_utils import numpy_util as npu
from common_utils import FloatArray

class TrajectoryPlannerNode(Node):
    """ROS 2 node that builds and dispatches joint trajectories."""

    def __init__(self) -> None:
        super().__init__("trajectory_planner")

        self.planner = TrajPlanner()
        self.q: FloatArray  = np.zeros(0, dtype=npu.dtype)
        self.qd: FloatArray = np.zeros(0, dtype=npu.dtype)
        self.joint_names: list[str] = []

        self.declare_parameter("default_plan_frequency", 100.0)
        self.default_freq = (
            self.get_parameter("default_plan_frequency")
            .get_parameter_value()
            .double_value
        )

        self.joint_state_sub = self.create_subscription(
            JointStateSim,
            endpoints.JOINT_STATE_TOPIC,
            self.joint_state_callback,
            10,
        )

        self.quintic_service = self.create_service(
            SetTargetPoint,
            endpoints.QUINTIC_TRAJECTORY_SERVICE,
            self.plan_quintic_callback,
        )
        self.point_service = self.create_service(
            SetTargetPoint,
            endpoints.POINT_TRAJECTORY_SERVICE,
            self.track_point_callback,
        )
        self.track_current_service = self.create_service(
            Trigger,
            endpoints.TRACK_CURRENT_SERVICE,
            self.track_current_callback,
        )

        self._traj_action_client: ActionClient = ActionClient(
            self, FollowJointTrajectory, endpoints.FOLLOW_TRAJECTORY_ACTION
        )

        self.get_logger().info("Trajectory planner node ready")


    def joint_state_callback(self, msg: JointStateSim) -> None:
        """Store the latest joint state for planning."""

        self.q = np.asarray(msg.position, dtype=npu.dtype)
        self.qd = np.asarray(msg.velocity, dtype=npu.dtype)
        self.joint_names = list(msg.name)


    def plan_quintic_callback(self, request: SetTargetPoint.Request, response: SetTargetPoint.Response):
        """Plan a quintic trajectory from the current state to a target point."""

        if not self._check_ready(response):
            return response

        q0 = self.q
        qf = np.asarray(request.target_positions, dtype=npu.dtype)
        tf = request.time_from_start.sec + request.time_from_start.nanosec * 1e-9
        freq = (request.n_points - 1) / tf

        traj = self.planner.quintic_trajs(q0, qf, 0.0, tf, freq)
        traj_msg = self._trajectory_from_array(traj, freq)

        return self._send_follow_goal(traj_msg, response, "quintic")


    def track_point_callback(self, request: SetTargetPoint.Request, response: SetTargetPoint.Response):
        """Track a single target point as a trivial trajectory."""

        if not self._check_ready(response):
            return response

        qf = np.asarray(request.target_positions, dtype=npu.dtype)
        traj_msg = self._single_point_trajectory(qf)

        return self._send_follow_goal(traj_msg, response, "point")


    def track_current_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Resend the latest position as a hold-point trajectory."""

        if  not self.joint_names:
            response.success = False
            response.message = "No joint state received yet"
            self.get_logger().warning(response.message)
            return response

        traj_msg = self._single_point_trajectory(self.q)
        success = self._send_follow_goal(traj_msg, None, "hold current")
        response.success = success
        response.message = "Sent current position trajectory" if success else "Failed to send trajectory"
        return response


    def _check_ready(self, response: SetTargetPoint.Response) -> bool:
        if not self.joint_names:
            response.success = False
            response.message = "Waiting for initial joint state"
            self.get_logger().warning(response.message)
            return False
        return True


    def _trajectory_from_array(self, 
        positions: FloatArray, 
        velocities: FloatArray | None = None, 
        accelerations: FloatArray | None = None,
        freq: float = 100.0,
    ) -> JointTrajectory:
        """Convert planner output (3, N, n) into JointTrajectory."""

        if velocities is None: velocities = np.zeros_like(positions)
        if accelerations is None: accelerations = np.zeros_like(positions)    

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        dt = 1.0 / freq
        for idx in range(positions.shape[0]):
            point = JointTrajectoryPoint()
            point.positions = positions[idx].tolist()
            point.velocities = velocities[idx].tolist()
            point.accelerations = accelerations[idx].tolist()
            point.time_from_start = Duration(seconds=dt * idx).to_msg()
            traj_msg.points.append(point)

        return traj_msg


    def _single_point_trajectory(self, positions: np.ndarray) -> JointTrajectory:
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions.tolist()
        point.velocities = [0.0] * len(point.positions)
        point.accelerations = [0.0] * len(point.positions)
        point.time_from_start = Duration(seconds=0.0).to_msg()
        traj_msg.points.append(point)

        return traj_msg


    def _send_follow_goal(
        self,
        trajectory: JointTrajectory,
        response: Optional[SetTrajectory.Response],
        label: str,
    ) -> bool:
        if not self._traj_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("FollowJointTrajectory action server not available")
            if response is not None:
                response.success = False
                response.message = "FollowJointTrajectory action not available"
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        send_future = self._traj_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"{label} goal rejected by controller")
            if response is not None:
                response.success = False
                response.message = "Goal rejected"
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if not result or result.status != GoalStatus.STATUS_SUCCEEDED:
            status = result.status if result is not None else "unknown"
            self.get_logger().error(f"{label} goal failed with status {status}")
            if response is not None:
                response.success = False
                response.message = "Controller reported failure"
            return False

        self.get_logger().info(f"Successfully sent {label} trajectory with {len(trajectory.points)} point(s)")
        if response is not None:
            response.success = True
            response.message = "Trajectory sent"
        return True


def main() -> None:
    rclpy.init()
    node = TrajectoryPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
