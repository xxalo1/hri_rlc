"""Trajectory planner node for building joint trajectories and publishing them."""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from robots.kinova_gen3 import init_kinova_robot
from common_utils import numpy_util as npu
from common_utils import ros_util as ru

from rlc_common.endpoints import (
    TOPICS, SERVICES, ACTIONS, 
    JointStateMsg, 
    PlanQuinticSrv
)

class TrajectoryPlannerNode(Node):
    """ROS 2 node that builds joint trajectories and publishes them for execution."""

    def __init__(self) -> None:
        super().__init__("trajectory_planner")

        self.robot = init_kinova_robot()

        self.declare_parameter("default_plan_frequency", 100.0)
        self.default_freq = (
            self.get_parameter("default_plan_frequency")
            .get_parameter_value()
            .double_value
        )

        self.joint_traj_pub = self.create_publisher(
            TOPICS.planned_joint_traj.type,
            TOPICS.planned_joint_traj.name,
            10,
        )
        self.cart_traj_pub = self.create_publisher(
            TOPICS.planned_cart_traj.type,
            TOPICS.planned_cart_traj.name,
            10,
        )

        self.joint_state_sub = self.create_subscription(
            TOPICS.joint_state.type,
            TOPICS.joint_state.name,
            self.joint_state_callback,
            10,
        )

        self.quintic_service = self.create_service(
            SERVICES.plan_quintic.type,
            SERVICES.plan_quintic.name,
            self.plan_quintic_callback,
        )

        self.get_logger().info("Trajectory planner node ready")

    
    def joint_state_callback(self, msg: JointStateMsg) -> None:
        """Store the latest joint state for planning."""
        joint_state = ru.from_joint_state_msg(msg)

        t = joint_state.stamp
        q = joint_state.positions
        qd = joint_state.velocities

        # update current states
        self.robot.set_joint_state(q=q, qd=qd, t=t)


    def plan_quintic_callback(self, 
        request: PlanQuinticSrv.Request, 
        response: PlanQuinticSrv.Response
    ) -> PlanQuinticSrv.Response:
        """Plan a quintic trajectory from the current state to a target point and publish it."""
        label = "quintic"
        execute_immediately = request.execute_immediately
        robot = self.robot
        t = robot.t
        qf = npu.to_array(request.target_positions)
        duration = ru.from_ros_time(request.time_from_start)

        if request.frequency: freq = request.frequency
        else: freq = self.default_freq

        joint_traj = robot.setup_quintic_traj(qf, duration=duration, freq=freq)
        cart_traj = robot.get_cartesian_traj()

        planned_joint_traj_msg, traj_id = ru.planned_joint_traj(
            t, 
            robot.joint_names, 
            joint_traj,
            label,
            execute_immediately = execute_immediately
        )
        planned_cart_traj_msg, _ = ru.planned_cartesian_traj(
            t,
            cart_traj,
            label,
            derived_from_joint = True,
            execute_immediately = execute_immediately,
            traj_id = traj_id
        )
        
        self.cart_traj_pub.publish(planned_cart_traj_msg)
        self.joint_traj_pub.publish(planned_joint_traj_msg)

        # populate response
        response.success = True
        response.message = (
            f"Published {label} trajectory"
            + (" (will execute)" if execute_immediately else "")
        )
        response.trajectory_id = traj_id

        self.get_logger().info(
            f"Planned {label} trajectory with id {traj_id} "
            f"({joint_traj.t.shape[0]} point(s)) "
            + ("and flagged for execution" if execute_immediately else "")
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
