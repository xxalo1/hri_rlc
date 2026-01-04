"""Trajectory executor node that consumes planned trajectories and sends them to the controller."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from functools import partial
from typing import Any, Dict, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, MultiDOFJointTrajectory

from rlc_utils import msg_conv as rmsg
from rlc_utils import time_util as rtime

from rlc_common.endpoints import (
    TOPICS, SERVICES, ACTIONS, CurrentPlanMsg, 
    PlannedJointTrajMsg, ExecuteTrajSrv, PlannedCartTrajMsg,
    CurrentPlan
)

class TrajKind(Enum):
    JOINT = auto()
    CART = auto()

@dataclass
class CachedTraj:
    stamp: float
    kind: TrajKind
    label: str
    traj: JointTrajectory | MultiDOFJointTrajectory


class TrajectoryCache:
    def __init__(self, *, ttl_sec: float) -> None:
        self._ttl_sec = ttl_sec
        self._items: Dict[str, CachedTraj] = {}


    def put(self, traj_id: str, item: CachedTraj) -> None:
        """Insert or update a trajectory in the cache."""
        self._items[traj_id] = item


    def get(self, traj_id: str, now: float) -> CachedTraj | None:
        """Get a trajectory if it exists and is not expired."""
        item = self._items.get(traj_id)
        if item is None:
            return None
        if now - item.stamp > self._ttl_sec:
            # expired: drop and return None
            self._items.pop(traj_id, None)
            return None
        return item


    def cleanup(self, now: float) -> None:
        """Drop all expired trajectories."""
        expired = [
            traj_id
            for traj_id, item in self._items.items()
            if now - item.stamp > self._ttl_sec
        ]
        for traj_id in expired:
            self._items.pop(traj_id, None)


class TrajectoryExecutorNode(Node):
    """ROS 2 node that executes previously planned trajectories by ID."""

    def __init__(self) -> None:
        super().__init__("trajectory_executor")

        self.cache = TrajectoryCache(ttl_sec=180.0)

        self.current_plan_pub = self.create_publisher(
            TOPICS.current_plan.type,
            TOPICS.current_plan.name,
            10
        )

        self.planned_joint_traj_sub = self.create_subscription(
            TOPICS.planned_joint_traj.type,
            TOPICS.planned_joint_traj.name,
            self.planned_traj_callback,
            10,
        )
        self.planned_cart_traj_sub = self.create_subscription(
            TOPICS.planned_cart_traj.type,
            TOPICS.planned_cart_traj.name,
            self.planned_traj_callback,
            10,
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


    def _now(self) -> float:
        """Current time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9


    def _execute_cached(self, 
        trajectory_id: str, 
        now: float | None = None
    ) -> tuple[bool, str]:
        if now is None:
            now = self._now()

        cached = self.cache.get(trajectory_id, now)
        if cached is None:
            return False, "Trajectory id not found or expired"

        if cached.kind == TrajKind.JOINT:
            return self._send_joint_goal(
                trajectory_id,
                cached.traj, # type: ignore
                f"execute {cached.label or 'joint traj'} ({trajectory_id})",
            )
        else:
            return self._send_cart_goal(
                trajectory_id,
                cached.traj, # type: ignore
                f"execute {cached.label or 'cart traj'} ({trajectory_id})",
            )


    def _cache_traj(self,
        trajectory_id: str,
        traj_kind: TrajKind,
        traj: JointTrajectory | MultiDOFJointTrajectory,
        label: str,
    ) -> None:
        now = self._now()
        item = CachedTraj(
            stamp=now,
            kind=traj_kind,
            label=label,
            traj=traj,
        )
        self.cache.put(trajectory_id, item)
        self.cache.cleanup(now)

        self.get_logger().info(
            f"Cached {traj_kind.name.lower()} trajectory {trajectory_id} ({label}) "
            f"with {len(traj.points)} point(s)"
        )


    def planned_traj_callback(self,
        msg: PlannedCartTrajMsg | PlannedJointTrajMsg,
    ) -> None:
        """Currently unused; placeholder for future extensions."""

        if isinstance(msg, PlannedCartTrajMsg):
            traj_kind = TrajKind.CART
            if msg.derived_from_joint: 
                return
        else:
            traj_kind = TrajKind.JOINT

        
        self._cache_traj(
            trajectory_id=msg.trajectory_id,
            traj_kind=traj_kind,
            traj=msg.trajectory,
            label=msg.label,
        )

        if msg.execute_immediately:
            success, message = self._execute_cached(msg.trajectory_id, self._now())
            if not success:
                self.get_logger().error(message)
            else:
                self.get_logger().info(message)


    def _publish_current_plan(self,
        trajectory_id: str,
        label: str,
        status: int,
    ) -> None:
        msg = CurrentPlanMsg()
        msg.plan_id = trajectory_id
        msg.label = label
        msg.status = status

        now = self.get_clock().now().to_msg()
        msg.stamp = now

        self.current_plan_pub.publish(msg)


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


    def _send_joint_goal(self,
        trajectory_id: str,
        trajectory: JointTrajectory,
        label: str,
    ) -> tuple[bool, str]:
        """Send a trajectory to the controller without blocking callbacks."""

        if not self._traj_action_client.wait_for_server(timeout_sec=2.0):
            return False, "FollowJointTrajectory action not available"

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        send_future = self._traj_action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            partial(self._on_goal_response, trajectory_id=trajectory_id, label=label)
        )

        return True, f"{label} ({trajectory_id}) dispatched for execution"


    def _on_goal_response(self,
        future: Any,
        *,
        trajectory_id: str,
        label: str,
    ) -> None:
        """Handle goal acceptance/rejection and chain result handling."""
        try:
            goal_handle = future.result()
        except Exception as exc:
            msg = f"{label} ({trajectory_id}) failed to send goal: {exc}"
            self.get_logger().error(msg)
            self._publish_current_plan(
                trajectory_id, label, CurrentPlanMsg.STATUS_NONE
            )
            return

        if goal_handle is None or not goal_handle.accepted:
            msg = f"{label} ({trajectory_id}) rejected by controller"
            self.get_logger().error(msg)
            self._publish_current_plan(
                trajectory_id, label, CurrentPlanMsg.STATUS_NONE
            )
            return

        self._on_goal_accepted(trajectory_id, label)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._on_goal_result, trajectory_id=trajectory_id, label=label)
        )


    def _on_goal_result(self,
        future: Any,
        *,
        trajectory_id: str,
        label: str,
    ) -> None:
        """Handle final action result and publish CurrentPlan status."""
        try:
            get_result_resp = future.result()
        except Exception as exc:
            msg = f"{label} ({trajectory_id}) failed to get result: {exc}"
            self.get_logger().error(msg)
            self._publish_current_plan(
                trajectory_id, label, CurrentPlanMsg.STATUS_NONE
            )
            return

        ok, msg = self._handle_goal_result(
            trajectory_id=trajectory_id,
            label=label,
            result=get_result_resp,
        )
        if ok:
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)


    def _send_cart_goal(self,
        trajectory_id: str,
        trajectory: MultiDOFJointTrajectory,
        label: str,
    ) -> tuple[bool, str]: # type: ignore
        """Send a trajectory to the controller and wait for completion."""
        pass # TODO: implement cartesian trajectory execution


    def _handle_goal_result(self,
        trajectory_id: str,
        label: str,
        result: Any | None = None,
    ) -> tuple[bool, str]:
        """
        Interpret the action result and update the CurrentPlan status.

        Parameters
        ----------
        trajectory_id : str
            Identifier of the trajectory that was executed.
        label : str
            Human-readable label (e.g., planner/source).
        result : FollowJointTrajectory_GetResult.Response or None
            Action GetResult response from the controller. If None, it indicates
            that no result was returned (e.g., goal rejected or communication error).
        """
        if result is None:
            msg = f"{label} ({trajectory_id}) failed: no result returned"
            self._publish_current_plan(trajectory_id, label, CurrentPlanMsg.STATUS_NONE)
            return False, msg

        status = result.status

        match status:
            case GoalStatus.STATUS_SUCCEEDED:
                msg = f"{label} ({trajectory_id}) succeeded"
                self._publish_current_plan(
                    trajectory_id, label, CurrentPlanMsg.STATUS_SUCCEEDED
                )
                ok = True

            case GoalStatus.STATUS_ABORTED:
                msg = f"{label} ({trajectory_id}) aborted"
                self._publish_current_plan(
                    trajectory_id, label, CurrentPlanMsg.STATUS_ABORTED
                )
                ok = False

            case GoalStatus.STATUS_CANCELED:
                msg = f"{label} ({trajectory_id}) canceled"
                self._publish_current_plan(
                    trajectory_id, label, CurrentPlanMsg.STATUS_CANCELED
                )
                ok = False

            case _:
                msg = f"{label} ({trajectory_id}) finished with unknown status ({status})"
                self._publish_current_plan(
                    trajectory_id, label, CurrentPlanMsg.STATUS_NONE
                )
                ok = False

        return ok, msg


    def _on_goal_accepted(self, 
        trajectory_id: str, 
        label: str
    ) -> None:
        """Handle logging/state update when the controller accepts a goal."""
        self.get_logger().info(f"{label} ({trajectory_id}) accepted by controller")
        self._publish_current_plan(trajectory_id, label, CurrentPlanMsg.STATUS_ACTIVE)


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
