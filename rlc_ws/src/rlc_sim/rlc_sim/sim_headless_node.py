from __future__ import annotations

import time

import mujoco as mj
import numpy as np
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from sim_env.mujoco.env import Gen3Env
from common_utils import numpy_util as npu
from common_utils import ros_util as ru

from rlc_common.endpoints import (
    TOPICS, SERVICES, ACTIONS, 
    JointEffortCmdMsg,
    ResetSimSrv, PauseSimSrv,
)

class Gen3MujocoSimNode(Node):
    def __init__(self) -> None:
        super().__init__("gen3_mujoco_sim")

        self.declare_parameter("publish_rate_hz", 200.0)
        self.declare_parameter("realtime_factor", 1.0)

        self.publish_rate = self.get_parameter(
            "publish_rate_hz"
            ).get_parameter_value().double_value
        
        self.realtime_factor = self.get_parameter(
            "realtime_factor"
            ).get_parameter_value().double_value

        self.env = Gen3Env.from_default_scene()
        self.env.reset()

        self.joint_names = list(self.env.active_joints.keys())
        self.n_joints = len(self.joint_names)

        self.tau_cmd = np.zeros(self.n_joints, dtype=npu.dtype)
        self.paused = False

        self.torque_sub = self.create_subscription(
            TOPICS.effort_cmd.type,
            TOPICS.effort_cmd.name,
            self.torque_cmd_callback,
            10,
        )

        self.joint_state_pub = self.create_publisher(
            TOPICS.joint_state.type, 
            TOPICS.joint_state.name, 
            10,
        )
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)

        self.reset_srv = self.create_service(
            SERVICES.reset_sim.type,
            SERVICES.reset_sim.name,
            self.reset_service_callback,
        )
        self.pause_srv = self.create_service(
            SERVICES.pause_sim.type,
            SERVICES.pause_sim.name,
            self.pause_service_callback,
        )

        self.publish_period = 1.0 / self.publish_rate
        self.publish_timer = self.create_timer(
            self.publish_period, self.publish_joint_state
        )

        self.get_logger().info(
            f"Headless Gen3 MuJoCo sim ready with {self.n_joints} joints. "
            f"publish_rate={self.publish_rate} Hz, realtime_factor={self.realtime_factor}"
        )


    def reset_service_callback(self, 
        request: ResetSimSrv.Request, 
        response: ResetSimSrv.Response
    ) -> ResetSimSrv.Response:
        """
        Reset the simulation.
        """
        self.env.reset()
        response.success = True
        response.message = "Simulation reset."
        return response


    def pause_service_callback(self, 
        request: PauseSimSrv.Request, 
        response: PauseSimSrv.Response
    ) -> PauseSimSrv.Response:
        """
        Pause/unpause physics stepping.
        """
        self.paused = request.data
        response.success = True
        state = "paused" if self.paused else "running"
        response.message = f"Simulation is now {state}."
        return response


    def torque_cmd_callback(self, msg: JointEffortCmdMsg) -> None:
        state = ru.from_joint_effort_cmd_msg(msg)
        tau = state.efforts
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
        qdd = obs["qacc"]

        msg = ru.to_joint_state_msg(
            stamp=t,
            joint_names=self.joint_names,
            positions=q,
            velocities=qd,
            efforts=self.tau_cmd,
        )

        clock_msg = Clock()
        clock_msg.clock = ru.to_ros_time(t)
        self.joint_state_pub.publish(msg)
        self.clock_pub.publish(clock_msg)


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

            env.step(node.tau_cmd)

            time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
