from __future__ import annotations
import time

import numpy as np
import mujoco as mj
from mujoco import viewer
import rclpy
from rclpy.node import Node
from rlc_interfaces.msg import JointStateSim
from std_srvs.srv import Trigger, SetBool
from sim_env.mujoco.env import Gen3Env
from common_utils import ros_util as ru

from rlc_common.endpoints import (
    TOPICS, SERVICES, ACTIONS, 
    JointStateMsg, JointEffortCmdMsg,
    ResetSimSrv, PauseSimSrv,
)


class Gen3MujocoVizNode(Node):
    """
    Visualization node for Kinova Gen3 using MuJoCo viewer.

    - Owns its own MuJoCo model/data
    - Subscribes to the headless sim joint state topic
    - Mirrors joint positions into d.qpos and calls mj_forward in the render loop
    """

    def __init__(self) -> None:
        super().__init__("gen3_mujoco_viz")

        self.paused = True
        self.reset = False
        self.show_frames = False
        self.show_jacobian = False
        self.show_dynamics = False
        self.show_inertia = False
        self.show_com = False

        self.env = Gen3Env.from_default_scene()

        self.qpos_target = self.env.d.qpos.copy()
        self.qvel_target = self.env.d.qvel.copy()
        self.target_t = 0.0

        # Subscribe to joint states from the headless sim
        self.joint_state_sub = self.create_subscription(
            TOPICS.joint_state.type,
            TOPICS.joint_state.name,
            self.joint_state_callback,
            10,
        )

        self.reset_client = self.create_client(
            SERVICES.reset_sim.type, 
            SERVICES.reset_sim.name
        )
        self.pause_client = self.create_client(
            SERVICES.pause_sim.type, 
            SERVICES.pause_sim.name
        )

        self.get_logger().info(
            f"Gen3 MuJoCo viz node ready. Listening to {TOPICS.joint_state.name}"
        )

    
    def key_callback(self, keycode):
        c = chr(keycode)
        match c:
            case " ":
                if self.pause_client.wait_for_service(timeout_sec=1.0):
                    req = PauseSimSrv.Request()
                    req.data = False
                    future = self.pause_client.call_async(req)
                    self.get_logger().info("pause/unpause service called")

            case "r" | "R":
                if self.reset_client.wait_for_service(timeout_sec=1.0):
                    req = ResetSimSrv.Request()
                    future = self.reset_client.call_async(req)
                    self.get_logger().info("Reset service called")
                
                if self.pause_client.wait_for_service(timeout_sec=1.0):
                    req = PauseSimSrv.Request()
                    req.data = False
                    future = self.pause_client.call_async(req)
                    self.get_logger().info("pause/unpause service called")
                    
            case "1":
                self.show_frames = not self.show_frames

            case "j" | "J":
                self.show_jacobian = True

            case "d" | "D":
                self.show_dynamics = True

            case "i" | "I":
                self.show_inertia = True

            case "'":
                self.show_com = not self.show_com


    def joint_state_callback(self, msg: JointStateMsg) -> None:
        """
        Update qpos_target from the latest JointStateSim message.

        Assumes msg.name entries match MuJoCo joint names.
        """
        target_t = ru.from_ros_time(msg.header.stamp)
        qpos_target = np.asarray(msg.position, dtype=self.qpos_target.dtype)
        qvel_target = np.asarray(msg.velocity, dtype=self.qvel_target.dtype)
        self.qpos_target = qpos_target
        self.qvel_target = qvel_target
        self.target_t = target_t


def main(args=None) -> None:
    rclpy.init(args=args)

    node = Gen3MujocoVizNode()
    env = node.env

    try:
        with viewer.launch_passive(env.m, env.d, key_callback=node.key_callback) as v:
            while v.is_running() and rclpy.ok():

                rclpy.spin_once(node, timeout_sec=0.0)

                with v.lock():
                    env.set_state(
                        qpos=node.qpos_target,
                        qvel=node.qvel_target,
                        t=node.target_t,
                    )
                    v.sync()
                time.sleep(0.0001)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
