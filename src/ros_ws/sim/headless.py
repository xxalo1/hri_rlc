from __future__ import annotations

import time
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

import mujoco as mj
from .env import Gen3Env


class Gen3MujocoSimNode(Node):
    def __init__(self) -> None:
        super().__init__("gen3_mujoco_sim")

        self.declare_parameter("xml_path", None)
        self.declare_parameter("nsubsteps", 10)
        self.declare_parameter("publish_rate_hz", 200.0)
        self.declare_parameter("realtime_factor", 1.0)

        xml_path = self.get_parameter("xml_path").get_parameter_value().string_value
        if xml_path is None:
            xml_path = "/home/g201951870/projects/hri_rlc/src/sim_env/kinova_gen3_table.xml"

        self.publish_rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.realtime_factor = self.get_parameter("realtime_factor").get_parameter_value().double_value

        self.env = Gen3Env(xml_path=xml_path, seed=0)

        self.m: mj.MjModel = self.env.m
        self.d: mj.MjData   = self.env.d

        self.joint_names = list(self.env.active_joints.keys())
        self.n_joints = len(self.joint_names)

        self.tau_cmd = np.zeros(self.n_joints, dtype=np.float64)

        self.torque_sub = self.create_subscription(
            Float64MultiArray,
            "/sim/gen3/torque_cmd",
            self.torque_cmd_callback,
            10,
        )

        self.joint_state_pub = self.create_publisher(
            JointState, "/sim/gen3/joint_states", 10
        )

        self.publish_period = 1.0 / self.publish_rate
        self.publish_timer = self.create_timer(
            self.publish_period, self.publish_joint_state
        )

        self.env.reset()

        self.get_logger().info(
            f"Headless Gen3 MuJoCo sim ready with {self.n_joints} joints. "
            f"publish_rate={self.publish_rate} Hz, realtime_factor={self.realtime_factor}"
        )

        self.last_pub_sim_time = 0.0


    def torque_cmd_callback(self, msg: Float64MultiArray) -> None:
        tau = np.asarray(msg.data, dtype=np.float64)
        if tau.size != self.n_joints:
            self.get_logger().warn(
                f"Received tau size {tau.size}, expected {self.n_joints}"
            )
            return
        self.tau_cmd = tau


    def publish_joint_state(self) -> None:
        q = self.d.qpos[self.env.joint_idx].copy()
        qd = self.d.qvel[self.env.joint_idx].copy()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = q.tolist()
        msg.velocity = qd.tolist()
        msg.effort = self.tau_cmd.tolist()

        self.joint_state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Gen3MujocoSimNode()

    m, d = node.m, node.d
    dt = m.opt.timestep
    rt_factor = node.realtime_factor

    t0 = time.time()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

            node.env.d.ctrl[node.env.act_idx[: node.n_joints]] = node.tau_cmd

            mj.mj_step(m, d)

            sim_time = d.time
            target_wall = t0 + sim_time / rt_factor
            now = time.time()
            if target_wall > now:
                time.sleep(target_wall - now)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
