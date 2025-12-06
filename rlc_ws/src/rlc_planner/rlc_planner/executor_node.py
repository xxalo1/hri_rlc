"""Trajectory executor node that consumes planned trajectories and sends them to the controller."""

from __future__ import annotations

import stat
from typing import Dict, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from rlc_common.endpoints imporPlannedJointTrajectory
    TOPICS, SERVICES, ACTIONS, 
    PlannedTrajMsg, ExecuteTrajSrv
)


class TrajectoryExecutorNode(Node):
    """ROS 2 node that executes previously planned trajectories by ID."""

    def __init__(self) -> None:
        super().__init__("trajectory_executor")

        self.traj_cache: Dict[str, Tuple[float, JointTrajectory, str]] = {}
        self.cache_ttl_sec = 180.0  # seconds
PlannedJointTrajectory
        self.planned_traj_sub = self.create_subscription(
            TOPICS.planned_traj.type,
            TOPICS.planned_traj.name,
            self.planned_trajectory_callback,
            10
        )

        self.execute_service = self.create_service(
            SERVICES.execute_traj.type,
            SERVICES.execute_traj.name,
            self.execute_trajectory_callback,
        )

        self._traj_action_client: ActionClient = ActionClient(self, 
            ACTIONS.follow_traj.type,
            ACTIONS.follow_traj.name
        )

        self.get_logger().info("Trajectory executor node ready")
PlannedJointTrajectory

    def _publish_current_plan(
        self,
        trajectory_id: str,
        label: str,
        status: int,
    ) -> None:
        msg = CurrentPlan()
        msg.trajectory_id = trajectory_id
        msg.label = label or ""
        msg.status = status

        now = self.get_clock().now().to_msg()
        msg.timestamp = now  # or msg.stamp, depending on your field name

        self.current_plan_pub.publish(msg)


    def planned_trajectory_callback(self,
        msg: PlannedTrajMsg,
    ) -> None:
        """Cache the latest planned trajectory by its identifier; execute if flagged."""

        now = self._now()
        self.traj_cache[msg.trajectory_id] = (now, msg.trajectory, msg.label)
        self._cleanup_cache(now)
        self.get_logger().info(
            f"Cached trajectory {msg.trajectory_id} ({msg.label}) "
            f"with {len(msg.trajectory.points)} point(s)"
        )

        if msg.execute_immediately:
            success, message = self._execute_cached(msg.trajectory_id, now)
            if not success:
                self.get_logger().error(message)
            else:
                self.get_logger().info(message)


    def execute_trajectory_callback(self, 
        request: ExecuteTrajSrv.Request, 
        response: ExecuteTrajSrv.Response
    ) -> ExecuteTrajSrv.Response:
        """Send a cached trajectory to the controller."""

        success, message = self._execute_cached(request.trajectory_id)
        response.success = success
        response.message = message

        if not success:
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)
        return response


    def _execute_cached(self, trajectory_id: str, now: float | None = None) -> tuple[bool, str]:
        """Lookup a cached trajectory by id and send it to the controller."""

        now = self._now() if now is None else now
        self._cleanup_cache(now)

        cached = self.traj_cache.get(trajectory_id)
        if cached is None:
            return False, "Trajectory id not found or expired"

        _, trajectory, label = cached
        return self._send_follow_goal(
            trajectory_id,
            trajectory,
            f"execute {label or 'trajectory'} ({trajectory_id})",
        )


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


    def _send_follow_goal(
        self,
        trajectory_id: str,
        trajectory: JointTrajectory,
        label: str,
    ) -> tuple[bool, str]:
        """Send a trajectory to the controller and wait for completion."""

        if not self._traj_action_client.wait_for_server(timeout_sec=2.0):
            return False, "FollowJointTrajectory action not available"

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        send_future = self._traj_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_feedback,  # you can add this later if you want
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            return self._on_goal_rejected(trajectory_id, label)

        self._on_goal_accepted(trajectory_id, label)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None:
            return self._on_goal_unknown(trajectory_id, label, None)

        return self._handle_goal_result(trajectory_id, label, result)


    def _handle_goal_result(
        self,
        trajectory_id: str,
        label: str,
        result,
    ) -> tuple[bool, str]:
        status = result.status

        match status:
            case GoalStatus.STATUS_SUCCEEDED:
                msg = f"{label} ({trajectory_id}) succeeded"
                self._publish_current_plan(trajectory_id, label, CurrentPlan.STATUS_SUCCEEDED)
                ok = True

            case GoalStatus.STATUS_ABORTED:
                msg = f"{label} ({trajectory_id}) aborted"
                self._publish_current_plan(trajectory_id, label, CurrentPlan.STATUS_ABORTED)
                ok = False

            case GoalStatus.STATUS_CANCELED:
                msg = f"{label} ({trajectory_id}) canceled"
                self._publish_current_plan(trajectory_id, label, CurrentPlan.STATUS_CANCELED)
                ok = False
                
            case _:
                msg = f"{label} ({trajectory_id}) finished with unknown status"
                self._publish_current_plan(trajectory_id, label, CurrentPlan.STATUS_NONE)
                ok = False

        return ok, msg


    def _on_goal_accepted(self, trajectory_id: str, label: str) -> None:
        self.get_logger().info(f"{label} ({trajectory_id}) accepted by controller")
        self._publish_current_plan(trajectory_id, label, CurrentPlan.STATUS_ACTIVE)


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
