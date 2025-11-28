from __future__ import annotations

import time

import mujoco as mj
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from rlc_interfaces.msg import JointEffortCmd, JointStateSim
from ......src.common_utils import numpy_util as npu
from .. import topics
from ..envs.env import Gen3Env


class Gen3MujocoSimNode(Node):
    def __init__(self) -> None:
        super().__init__("gen3_mujoco_sim")

        self.declare_parameter("xml_path", "")
        self.declare_parameter("nsubsteps", 10)
        self.declare_parameter("publish_rate_hz", 200.0)
        self.declare_parameter("realtime_factor", 1.0)

        xml_path = self.get_parameter("xml_path").get_parameter_value().string_value
        if not xml_path:
            xml_path = "/home/g201951870/projects/hri_rlc/src/sim_env/kinova_gen3_table.xml"

        self.publish_rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.realtime_factor = self.get_parameter("realtime_factor").get_parameter_value().double_value

        self.env = Gen3Env(xml_path=xml_path, seed=0)

        self.m: mj.MjModel = self.env.m
        self.d: mj.MjData   = self.env.d

        self.joint_names = list(self.env.active_joints.keys())
        self.n_joints = len(self.joint_names)

        self.tau_cmd = np.zeros(self.n_joints, dtype=np.float64)
        self.paused = False

        self.torque_sub = self.create_subscription(
            JointEffortCmd,
            topics.EFFORT_COMMAND_TOPIC,
            self.torque_cmd_callback,
            10,
        )

        self.joint_state_pub = self.create_publisher(
            JointStateSim, topics.JOINT_STATE_TOPIC, 10
        )

        self.publish_period = 1.0 / self.publish_rate
        self.publish_timer = self.create_timer(
            self.publish_period, self.publish_joint_state
        )

        self.reset_srv = self.create_service(
            Trigger,
            topics.RESET_SERVICE,
            self.reset_service_callback,
        )

        self.pause_srv = self.create_service(
            SetBool,
            topics.PAUSE_SERVICE,
            self.pause_service_callback,
        )

        self.env.reset()

        self.get_logger().info(
            f"Headless Gen3 MuJoCo sim ready with {self.n_joints} joints. "
            f"publish_rate={self.publish_rate} Hz, realtime_factor={self.realtime_factor}"
        )

        self.last_pub_sim_time = 0.0


    def reset_service_callback(self, request: Trigger.Request, response: Trigger.Response):
        """
        Reset the simulation.
        """
        self.env.reset()
        response.success = True
        response.message = "Simulation reset."
        return response


    def pause_service_callback(self, request: SetBool.Request, response: SetBool.Response):
        """
        Pause/unpause physics stepping.
        """
        self.paused = request.data
        response.success = True
        state = "paused" if self.paused else "running"
        response.message = f"Simulation is now {state}."
        return response


    def torque_cmd_callback(self, msg: JointEffortCmd) -> None:
        tau = np.asarray(msg.effort, dtype=npu.dtype)
        if tau.size != self.n_joints:
            self.get_logger().warn(
                f"Received tau size {tau.size}, expected {self.n_joints}"
            )
            return
        self.tau_cmd = tau


    def publish_joint_state(self) -> None:
        obs, t = self.env.observe()
        q = obs["qpos"]
        qd = obs["qvel"]

        msg = JointStateSim()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sim_time = t
        msg.name = self.joint_names
        msg.position = q.tolist()
        msg.velocity = qd.tolist()
        msg.effort = self.tau_cmd.tolist()

        self.joint_state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Gen3MujocoSimNode()
    env = node.env
    rt_factor = node.realtime_factor

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

            if node.paused:
                time.sleep(0.001)
                continue

            env.step_realtime(node.tau_cmd, realtime_factor=rt_factor)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
