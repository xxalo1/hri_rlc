from __future__ import annotations
import rclpy
from rclpy.node import Node

from robots.kinova_gen3 import init_kinova_robot
from common_utils import FloatArray
from common_utils import numpy_util as npu
from common_utils.buffers import RingBuffer, BufferSet

from rlc_common import endpoints

class RLCMonitorNode(Node):
    """RLC Monitor Node for monitoring robot state and publishing diagnostics."""

    def __init__(self) -> None:
        super().__init__('rlc_monitor')

        self.declare_parameter('capacity', 10000)
        self.capacity = self.get_parameter('capacity').int
        
        
        # Initialize Kinova Gen3 Robot
        self.robot = init_kinova_robot()

        # Initialize buffers for storing robot state
        self.state_buffer = BufferSet(capacity=self.capacity)

        self.joint_state_sub = self.create_subscription(
            FloatArray,
            endpoints.JOINT_STATE_TOPIC,
            self.joint_state_callback,
            10,
        )

        # Create publishers, subscribers, and timers as needed
        # ...

def main():
    print('Hi from rlc_monitor.')


if __name__ == '__main__':
    main()
