from __future__ import annotations

from sympy import Q
import rclpy
from rclpy.node import Node
import numpy as np

from robots.kinova_gen3 import init_kinova_robot
from common_utils import FloatArray
from common_utils import numpy_util as npu
from common_utils.buffers import RingBuffer, BufferSet

from rlc_common.endpoints import TOPICS, ACTIONS, SERVICES
from rlc_common.endpoints import (
    JointStateMsg, JointEffortCmdMsg, PlannedTrajMsg,
    ResetSimSrv, PauseSimSrv, SetControllerGainsSrv, SetControllerModeSrv,
    PlanQuinticSrv, ExecuteTrajSrv,
    FollowTrajAction,
    )
class RLCMonitorNode(Node):
    """RLC Monitor Node for monitoring robot state and publishing diagnostics."""

    def __init__(self) -> None:
        super().__init__('rlc_monitor')

        self.declare_parameter('capacity', 10000)
        self.capacity = self.get_parameter('capacity').integer
        
        # Initialize Kinova Gen3 Robot
        self.robot = init_kinova_robot()

        # Initialize buffers for storing robot state
        self.state_buffer = BufferSet(capacity=self.capacity)

        self.joint_state_sub = self.create_subscription(
            TOPICS.joint_state.type,
            TOPICS.joint_state.name,
            self.joint_state_callback,
            10,
        )
        self.controller_state_sub = self.create_subscription(
            TOPICS.effort_cmd.type,
            TOPICS.effort_cmd.name,
            self.controller_state_callback,
            10,
        )
        self.planned_traj_sub = self.create_subscription(
            TOPICS.planned_traj.type,
            TOPICS.planned_traj.name,
            self.planned_traj_callback,
            10,
        )
        self.frames_pub = self.create_publisher(
            TOPICS.frames.type,
            TOPICS.frames.name,
            10,
        )
        self.end_effector_traj_pub = self.create_publisher(
            ,
            'rlc_monitor/end_effector_trajectory',
            10,
        )
        
    def joint_state_callback(self, msg: JointStateMsg) -> None:
        """Callback for joint state messages."""
        # Process joint state message and store in buffer
        q = np.asarray(msg.position)
        qd = np.asarray(msg.velocity)
        tau_sim = np.asarray(msg.effort)
        t_sim = msg.sim_time

        # update current states
        self.q = q
        self.qd = qd
        self.tau_sim = tau_sim
        self.t_sim = t_sim

        # Store data in buffers
        self.state_buffer.append('joint_positions', t_sim, q)
        self.state_buffer.append('joint_velocities', t_sim, qd)
        self.state_buffer.append('joint_efforts', t_sim, tau_sim)


    def controller_state_callback(self, msg: JointEffortCmdMsg) -> None:
        """Callback for controller effort command messages."""
        tau_cmd = np.asarray(msg.effort)
        t_ctrl = msg.time_sim

        # update current controller efforts
        self.tau_cmd = tau_cmd
        self.t_ctrl = t_ctrl

        # Store data in buffers
        self.state_buffer.append('controller_efforts', t_ctrl, tau_cmd)


    def planned_traj_callback(self, msg: PlannedTrajMsg) -> None:
        """Callback for planned trajectory messages."""
        # Process planned trajectory message and store in buffer
        times = np.asarray(msg.times)
        positions = npu.msgs_to_numpy_array(msg.positions)
        velocities = npu.msgs_to_numpy_array(msg.velocities)
        accelerations = npu.msgs_to_numpy_array(msg.accelerations)

        # Store data in buffers
        self.state_buffer.append('planned_traj_times', times)
        self.state_buffer.append('planned_traj_positions', positions)
        self.state_buffer.append('planned_traj_velocities', velocities)
        self.state_buffer.append('planned_traj_accelerations', accelerations)

        
def main():
    print('Hi from rlc_monitor.')


if __name__ == '__main__':
    main()
