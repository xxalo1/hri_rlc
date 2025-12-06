"""Trajectory planner node for building joint trajectories and publishing them."""

from __future__ import annotations

import uuid

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rbt_core import TrajPlanner
from robots.kinova_gen3 import init_kinova_robot
from common_utils import numpy_util as npu
from common_utils import ros_util as ru
from common_utils import FloatArray

from rlc_common.endpoints import (
    TOPICS, SERVICES, ACTIONS, 
    JointStateMsg, PlannedTrajMsg, 
    PlanQuinticSrv
)
from rlc_interfaces.msg import JointStateSim, PlannedJointTrajectory
from rlc_interfaces.srv import PlanTrajectory

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
        self.ee_traj_pub = self.create_publisher(
            TOPICS.planned_ee_traj.type,
            TOPICS.planned_ee_traj.name,
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

    @property
    def joint_names(self) -> list[str]:
        """Get the robot's joint names."""
        return self.robot.spec.joint_names
    
    def joint_state_callback(self, msg: JointStateMsg) -> None:
        """Store the latest joint state for planning."""
        self.t = ru.from_ros_time(msg.header.stamp)
        self.q = np.asarray(msg.position, dtype=npu.dtype)
        self.qd = np.asarray(msg.velocity, dtype=npu.dtype)
        self.qdd = np.asarray(msg.acceleration, dtype=npu.dtype)

        self.robot.set_joint_state(self.q, self.qd, self.qdd, self.t)        


    def plan_quintic_callback(self, 
        request: PlanQuinticSrv.Request, 
        response: PlanQuinticSrv.Response
    ) -> PlanQuinticSrv.Response:
        """Plan a quintic trajectory from the current state to a target point and publish it."""

        if not self._check_ready(response):
            return response

        robot = self.robot
        qf = np.asarray(request.target_positions, dtype=npu.dtype)
        dt = ru.from_ros_time(request.time_from_start)
        freq = (request.n_points - 1) / dt
        tf = dt + robot.t

        traj = robot.setup_quintic_traj(qf, tf=tf, freq=freq)
        q_traj = traj[0]
        v_traj = traj[1]
        a_traj = traj[2]

        joint_traj_msg = ru.joint_traj_from_arrays(
            robot.t, self.joint_names, 
            q_traj, v_traj, a_traj, freq
        )

        poses = robot.get_poses()

        poses_msg = ru.poses_from_arrays(
            robot.t, poses,
        )
        
        return self._publish_joint_traj(
            joint_traj_msg,
            response,
            "quintic",
            request.execute_immediately,
        )


    def _check_ready(self, response: PlanTrajectory.Response) -> bool:
        if not self.joint_names:
            response.success = False
            response.message = "Waiting for initial joint state"
            self.get_logger().warning(response.message)
            return False
        return True


    def _publish_joint_traj(self,
        trajectory: JointTrajectory,
        response: PlanQuinticSrv.Response,
        label: str,
        execute_immediately: bool,
    ) -> PlanQuinticSrv.Response:
        traj_id = str(uuid.uuid4())

        planned_msg = PlannedTrajMsg()
        planned_msg.trajectory_id = traj_id
        planned_msg.trajectory = trajectory
        planned_msg.label = label
        planned_msg.execute_immediately = execute_immediately
        self.joint_traj_pub.publish(planned_msg)

        response.success = True
        response.message = (
            f"Published {label} trajectory"
            + (" (will execute)" if execute_immediately else "")
        )
        response.trajectory_id = traj_id

        self.get_logger().info(
            f"Planned {label} trajectory with id {traj_id} "
            f"({len(trajectory.points)} point(s)) "
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
