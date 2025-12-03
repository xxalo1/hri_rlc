from __future__ import annotations
import rclpy
from rclpy.node import Node
import numpy as np

from robots.kinova_gen3 import init_kinova_robot
from common_utils import FloatArray
from common_utils import numpy_util as npu
from common_utils.buffers import RingBuffer, BufferSet

from rlc_common.endpoints import TOPICS, ACTIONS, SERVICES

joint_state_msg = TOPICS.joint_state.type
effort_cmd_msg = TOPICS.effort_cmd.type
planned_traj_msg = TOPICS.planned_traj.type
follow_traj_action = ACTIONS.follow_traj.type
set_gains_service = SERVICES.set_joint_gains.type

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
        
    def joint_state_callback(self, msg: TOPICS.joint_state.type) -> None:
        """Callback for joint state messages."""
        # Process joint state message and store in buffer
        joint_positions = np.asarray(msg.position)
        joint_velocities = np.asarray(msg.velocity)
        t = msg.
        timestamp = self.get_clock().now().to_msg().sec_nanosec[0]
        self.state_buffer.add('joint_positions', FloatArray(joint_positions, timestamp))
        self.state_buffer.add('joint_velocities', FloatArray(joint_velocities, timestamp))

def main():
    print('Hi from rlc_monitor.')


if __name__ == '__main__':
    main()
