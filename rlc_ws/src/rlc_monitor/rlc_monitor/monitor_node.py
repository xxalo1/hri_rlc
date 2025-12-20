from __future__ import annotations

import rclpy
from rclpy.node import Node

from robots.kinova_gen3 import init_kinova_robot
from ros_utils import msg_conv as rmsg
from ros_utils.config import qos_latest
from common_utils import transforms as tfm
from common_utils.buffers import RingBuffer, BufferSet

from rlc_common.endpoints import TOPICS, ACTIONS, SERVICES
from rlc_common.endpoints import (
    JointStateMsg,
    ControllerStateMsg,
    )

class RLCMonitorNode(Node):
    """RLC Monitor Node for monitoring robot state and publishing diagnostics."""

    def __init__(self) -> None:
        super().__init__('rlc_monitor')

        self.declare_parameter('capacity', 10000)
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.freq = self.get_parameter(
            'publish_rate'
            ).get_parameter_value().double_value
        self.capacity = self.get_parameter(
            'capacity'
            ).get_parameter_value().integer_value
        
        self.robot = init_kinova_robot()

        self.state_buffer = BufferSet(capacity=self.capacity)

        self.joint_state_sub = self.create_subscription(
            TOPICS.joint_state.type,
            TOPICS.joint_state.name,
            self.joint_state_callback,
            qos_latest,
        )
        self.controller_state_sub = self.create_subscription(
            TOPICS.controller_state.type,
            TOPICS.controller_state.name,
            self.controller_state_callback,
            qos_latest,
        )
        self.frame_states_pub = self.create_publisher(
            TOPICS.frame_states.type,
            TOPICS.frame_states.name,
            10,
        )

        n = 1 / self.freq
        self.frame_states_timer = self.create_timer(
            n, self.publish_frame_states
            )


    def joint_state_callback(self, msg: JointStateMsg) -> None:
        """Callback for joint state messages."""
        # Process joint state message and store in buffer

        joint_state = rmsg.from_joint_state_msg(msg)

        t = joint_state.stamp
        q = joint_state.positions
        qd = joint_state.velocities
        tau_sim = joint_state.efforts

        # update current states
        self.robot.set_joint_state(q=q, qd=qd, t=t)

        # Store data in buffers
        buf = self.state_buffer
        buf.append('joint_positions', t, q)
        buf.append('joint_velocities', t, qd)
        buf.append('joint_efforts', t, tau_sim)


    def controller_state_callback(self, msg: ControllerStateMsg) -> None:
        """
        Callback for JointTrajectoryControllerState messages from the Gen3 controller.

        Reads desired (reference), feedback, error joint states and the controller
        output effort, converts them to numpy, updates the latest controller fields,
        and stores them in the state buffer for logging/plotting.
        """
        ctrl_state = rmsg.from_joint_ctrl_state_msg(msg)
        t = ctrl_state.stamp

        buf = self.state_buffer
        buf.append("ctrl_q",        t, ctrl_state.q)
        buf.append("ctrl_qd",       t, ctrl_state.qd)
        buf.append("ctrl_qdd",      t, ctrl_state.qdd)
        buf.append("ctrl_q_des",    t, ctrl_state.q_des)
        buf.append("ctrl_qd_des",   t, ctrl_state.qd_des)
        buf.append("ctrl_qdd_des",  t, ctrl_state.qdd_des)
        buf.append("ctrl_e",        t, ctrl_state.e)
        buf.append("ctrl_ed",       t, ctrl_state.ed)
        buf.append("ctrl_efforts",  t, ctrl_state.tau)


    def publish_frame_states(self) -> None:
        """Publish the current frame states of the robot."""
        T_wf = self.robot.kin.forward_kinematics()
        poses = tfm.transform_to_pos_quat(T_wf)
        msg = rmsg.to_pose_array_msg(
            self.robot.t,
            poses,
        )
        self.frame_states_pub.publish(msg)


def main():
    rclpy.init()
    node = RLCMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
