"""Trajectory planner node for building joint trajectories and publishing them."""

from __future__ import annotations

import uuid

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rlc_common import endpoints
from rlc_interfaces.msg import JointStateSim, PlannedTrajectory
from rlc_interfaces.srv import PlanTrajectory
from rbt_core import TrajPlanner
from common_utils import numpy_util as npu
from common_utils import FloatArray

class TrajectoryPlannerNode(Node):
    """ROS 2 node that builds joint trajectories and publishes them for execution."""

    def __init__(self) -> None:
        super().__init__("trajectory_planner")

        self.planner = TrajPlanner()
        self.q  = np.zeros(0, dtype=npu.dtype)
        self.qd = np.zeros(0, dtype=npu.dtype)
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

        self.planned_traj_pub = self.create_publisher(
            PlannedTrajectory,
            endpoints.PLANNED_TRAJECTORY_TOPIC,
            10,
        )

        self.quintic_service = self.create_service(
            PlanTrajectory,
            endpoints.QUINTIC_TRAJECTORY_SERVICE,
            self.plan_quintic_callback,
        )
        self.point_service = self.create_service(
            PlanTrajectory,
            endpoints.POINT_TRAJECTORY_SERVICE,
            self.track_point_callback,
        )

        self.get_logger().info("Trajectory planner node ready")


    def joint_state_callback(self, msg: JointStateSim) -> None:
        """Store the latest joint state for planning."""

        self.q = np.asarray(msg.position, dtype=npu.dtype)
        self.qd = np.asarray(msg.velocity, dtype=npu.dtype)
        self.joint_names = list(msg.name)


    def plan_quintic_callback(self, request: PlanTrajectory.Request, response: PlanTrajectory.Response):
        """Plan a quintic trajectory from the current state to a target point and publish it."""

        if not self._check_ready(response):
            return response

        q0 = self.q
        qf = np.asarray(request.target_positions, dtype=npu.dtype)
        tf = request.time_from_start.sec + request.time_from_start.nanosec * 1e-9
        freq = (request.n_points - 1) / tf

        traj = self.planner.quintic_trajs(q0, qf, 0.0, tf, freq)
        q_traj = traj[0]
        v_traj = traj[1]
        a_traj = traj[2]
        traj_msg = self._trajectory_from_arrays(q_traj, v_traj, a_traj, freq)

        return self._publish_planned_trajectory(traj_msg, response, "quintic")


    def track_point_callback(self, request: PlanTrajectory.Request, response: PlanTrajectory.Response):
        """Plan a single target point as a trivial trajectory and publish it."""

        if not self._check_ready(response):
            return response

        qf = np.asarray(request.target_positions, dtype=npu.dtype)
        tf = request.time_from_start.sec + request.time_from_start.nanosec * 1e-9
        freq = (request.n_points - 1) / tf
        traj_msg = self._trajectory_from_arrays(qf, freq=freq)

        return self._publish_planned_trajectory(traj_msg, response, "point")


    def _check_ready(self, response: PlanTrajectory.Response) -> bool:
        if not self.joint_names:
            response.success = False
            response.message = "Waiting for initial joint state"
            self.get_logger().warning(response.message)
            return False
        return True


    def _trajectory_from_arrays(self, 
        positions: FloatArray, 
        velocities: FloatArray | None = None, 
        accelerations: FloatArray | None = None,
        freq: float = 100.0,
    ) -> JointTrajectory:
        """Convert planner output (3, N, n) into JointTrajectory."""

        positions = np.atleast_2d(positions)
        if velocities is None:
            velocities = np.zeros_like(positions)
        else:
            velocities = np.atleast_2d(velocities)
        if accelerations is None:
            accelerations = np.zeros_like(positions)
        else:
            accelerations = np.atleast_2d(accelerations)

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


    def _publish_planned_trajectory(
        self,
        trajectory: JointTrajectory,
        response: PlanTrajectory.Response,
        label: str,
    ) -> PlanTrajectory.Response:
        traj_id = str(uuid.uuid4())

        planned_msg = PlannedTrajectory()
        planned_msg.trajectory_id = traj_id
        planned_msg.trajectory = trajectory
        planned_msg.label = label
        self.planned_traj_pub.publish(planned_msg)

        response.success = True
        response.message = f"Published {label} trajectory"
        response.trajectory_id = traj_id

        self.get_logger().info(
            f"Planned {label} trajectory with id {traj_id} "
            f"({len(trajectory.points)} point(s))"
        )
        return response


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
