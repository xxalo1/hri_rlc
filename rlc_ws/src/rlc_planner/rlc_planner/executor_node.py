"""Trajectory executor node that consumes planned trajectories and sends them to the controller."""

from __future__ import annotations

from typing import Dict, Tuple

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
from rlc_interfaces.msg import PlannedTrajectory
from rlc_interfaces.srv import ExecuteTrajectory
from common_utils import numpy_util as npu
from common_utils import FloatArray


class TrajectoryExecutorNode(Node):
    """ROS 2 node that executes previously planned trajectories by ID."""

    def __init__(self) -> None:
        super().__init__("trajectory_executor")

        self.traj_cache: Dict[str, Tuple[float, JointTrajectory, str]] = {}
        self.cache_ttl_sec = 180.0  # seconds

        self.planned_traj_sub = self.create_subscription(
            PlannedTrajectory,
            endpoints.PLANNED_TRAJECTORY_TOPIC,
            self.planned_trajectory_callback,
            10,
        )

        self.execute_service = self.create_service(
            ExecuteTrajectory,
            endpoints.EXECUTE_TRAJECTORY_SERVICE,
            self.execute_trajectory_callback,
        )

        self._traj_action_client: ActionClient = ActionClient(
            self, FollowJointTrajectory, endpoints.FOLLOW_TRAJECTORY_ACTION
        )

        self.get_logger().info("Trajectory executor node ready")


    def planned_trajectory_callback(self, 
        msg: PlannedTrajectory
    ) -> None:
        """Cache the latest planned trajectory by its identifier."""

        now = self._now()
        self.traj_cache[msg.trajectory_id] = (
            now, msg.trajectory, msg.label, msg.execute_immediately
            )
        self._cleanup_cache(now)
        self.get_logger().info(
            f"Cached trajectory {msg.trajectory_id} ({msg.label}) "
            f"with {len(msg.trajectory.points)} point(s)"
        )
        


    def execute_trajectory_callback(self, 
        request: ExecuteTrajectory.Request, 
        response: ExecuteTrajectory.Response
    ) -> ExecuteTrajectory.Response:
        """Send a cached trajectory to the controller."""

        now = self._now()
        self._cleanup_cache(now)

        cached = self.traj_cache.get(request.trajectory_id)
        if cached is None:
            response.success = False
            response.message = "Trajectory id not found or expired"
            self.get_logger().warning(response.message)
            return response

        _, trajectory, label = cached
        success, message = self._send_follow_goal(
            trajectory,
            f"execute {label or 'trajectory'} ({request.trajectory_id})",
        )
        response.success = success
        response.message = message

        if not success:
            self.get_logger().error(message)
        else:
            self.get_logger().info(
                f"Executed trajectory {request.trajectory_id} ({label})"
            )
        return response


    def _cleanup_cache(self, 
        now: float
    ) -> None:
        """Drop cached trajectories that exceeded the TTL."""

        expired_ids = [
            traj_id
            for traj_id, (timestamp, _, _) in self.traj_cache.items()
            if now - timestamp > self.cache_ttl_sec
        ]
        for traj_id in expired_ids:
            self.traj_cache.pop(traj_id, None)


    def _send_follow_goal(self, 
        trajectory: JointTrajectory, 
        label: str
    ) -> tuple[bool, str]:
        """Send a trajectory to the controller and wait for completion."""

        if not self._traj_action_client.wait_for_server(timeout_sec=2.0):
            return False, "FollowJointTrajectory action not available"

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        send_future = self._traj_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            return False, f"{label} goal rejected by controller"

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if not result or result.status != GoalStatus.STATUS_SUCCEEDED:
            status = result.status if result is not None else "unknown"
            return False, f"{label} goal failed with status {status}"

        return True, f"Successfully sent {label}"


    def _now(self) -> float:
        """Current time in seconds."""

        return self.get_clock().now().nanoseconds * 1e-9


def main() -> None:
    rclpy.init()
    node = TrajectoryExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
