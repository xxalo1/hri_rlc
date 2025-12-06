from __future__ import annotations

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseArray, Pose

from robots.kinova_gen3 import init_kinova_robot
from common_utils import numpy_util as npu
from common_utils import ros_util as ru
from common_utils import transforms as tfu
from common_utils.buffers import RingBuffer, BufferSet

from rlc_common.endpoints import TOPICS, ACTIONS, SERVICES
from rlc_common.endpoints import (
    JointStateMsg, FrameStatesMsg, 
    ControllerStateMsg,
    )

class RLCMonitorNode(Node):
    """RLC Monitor Node for monitoring robot state and publishing diagnostics."""

    def __init__(self) -> None:
        super().__init__('rlc_monitor')

        self.declare_parameter('capacity', 10000)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.freq = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.capacity = self.get_parameter('capacity').get_parameter_value().integer_value
        
        self.robot = init_kinova_robot()

        self.state_buffer = BufferSet(capacity=self.capacity)

        self.joint_state_sub = self.create_subscription(
            TOPICS.joint_state.type,
            TOPICS.joint_state.name,
            self.joint_state_callback,
            10,
        )
        self.controller_state_sub = self.create_subscription(
            TOPICS.controller_state.type,
            TOPICS.controller_state.name,
            self.controller_state_callback,
            10,
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
        t = ru.from_ros_time(msg.header.stamp)
        q = np.asarray(msg.position)
        qd = np.asarray(msg.velocity)
        qdd = np.asarray(msg.acceleration)
        tau_sim = np.asarray(msg.effort)

        # update current states
        self.robot.set_joint_state(q, qd, qdd, t)

        # Store data in buffers
        buf = self.state_buffer
        buf.append('joint_positions', t, q)
        buf.append('joint_velocities', t, qd)
        buf.append('joint_accelerations', t, qdd)
        buf.append('joint_efforts', t, tau_sim)


    def controller_state_callback(self, msg: ControllerStateMsg) -> None:
        """
        Callback for JointTrajectoryControllerState messages from the Gen3 controller.

        Reads desired (reference), feedback, error joint states and the controller
        output effort, converts them to numpy, updates the latest controller fields,
        and stores them in the state buffer for logging/plotting.
        """
        # Controller time (sim time) from header
        t = ru.from_ros_time(msg.header.stamp)

        # Desired / reference state
        ref = msg.reference
        q_des   = np.asarray(ref.positions,      dtype=npu.dtype)
        qd_des  = np.asarray(ref.velocities,     dtype=npu.dtype)
        qdd_des = np.asarray(ref.accelerations,  dtype=npu.dtype)

        # Measured / feedback state
        fb = msg.feedback
        q   = np.asarray(fb.positions,      dtype=npu.dtype)
        qd  = np.asarray(fb.velocities,     dtype=npu.dtype)
        qdd = np.asarray(fb.accelerations,  dtype=npu.dtype)

        # Error (as sent by the controller)
        err = msg.error
        e  = np.asarray(err.positions,  dtype=npu.dtype)
        ed = np.asarray(err.velocities, dtype=npu.dtype)

        # Controller output (torque command)
        tau = np.asarray(msg.output.effort, dtype=npu.dtype)

        # Store data in buffers for logging / plotting
        buf = self.state_buffer
        buf.append("ctrl_q",        t, q)
        buf.append("ctrl_qd",       t, qd)
        buf.append("ctrl_qdd",      t, qdd)
        buf.append("ctrl_q_des",    t, q_des)
        buf.append("ctrl_qd_des",   t, qd_des)
        buf.append("ctrl_qdd_des",  t, qdd_des)
        buf.append("ctrl_e",        t, e)
        buf.append("ctrl_ed",       t, ed)
        buf.append("ctrl_efforts",  t, tau)


    def publish_frame_states(self) -> None:
        """Publish the current frame states of the robot."""
        T_wf = self.robot.kin.forward_kinematics()
        poses = tfu.transform_to_pos_quat(T_wf)
        msg = ru.to_pose_array_msg(
            self.robot.t,
            poses,
            frame_id="world",
        )
        self.frame_states_pub.publish(msg)



def main():
    print('Hi from rlc_monitor.')


if __name__ == '__main__':
    main()
