#!/usr/bin/env python3
from __future__ import annotations

import time
from typing import Dict

import numpy as np
import mujoco as mj

import rclpy
from rclpy.node import Node
from rlc_interfaces.msg import JointStateSim
from std_srvs.srv import Trigger, SetBool

from ....rlc_common import topics
from ..envs.env import Gen3Env


class Gen3MujocoVizNode(Node):
    """
    Visualization node for Kinova Gen3 using MuJoCo viewer.

    - Owns its own MuJoCo model/data
    - Subscribes to the headless sim joint state topic
    - Mirrors joint positions into d.qpos and calls mj_forward in the render loop
    """

    def __init__(self, xml_path: str) -> None:
        super().__init__("gen3_mujoco_viz")

        self.paused = True
        self.reset = False
        self.show_frames = False
        self.show_jacobian = False
        self.show_dynamics = False
        self.show_inertia = False
        self.show_com = False

        self.declare_parameter("xml_path", None)

        self.env = Gen3Env(xml_path=xml_path, seed=0)

        self.m: mj.MjModel = self.env.m
        self.d: mj.MjData   = self.env.d

        self.qpos_target = self.d.qpos.copy()
        self.qvel_target = self.d.qvel.copy()
        self.target_t = 0.0

        # Subscribe to joint states from the headless sim
        self.joint_state_sub = self.create_subscription(
            JointStateSim,
            topics.JOINT_STATE_TOPIC,
            self.joint_state_callback,
            10,
        )

        self.reset_client = self.create_client(Trigger, topics.RESET_SERVICE)
        self.pause_client = self.create_client(SetBool, topics.PAUSE_SERVICE)

        self.get_logger().info(
            f"Gen3 MuJoCo viz node ready. Listening to {topics.JOINT_STATE_TOPIC}"
        )


    def key_callback(self, keycode):
        c = chr(keycode)
        match c:
            case " ":
                if self.pause_client.wait_for_service(timeout_sec=1.0):
                    req = SetBool.Request()
                    req.data = False
                    future = self.pause_client.call_async(req)
                    self.get_logger().info("pause/unpause service called")

            case "r" | "R":
                if self.reset_client.wait_for_service(timeout_sec=1.0):
                    req = Trigger.Request()
                    future = self.reset_client.call_async(req)
                    self.get_logger().info("Reset service called")
                
                if self.pause_client.wait_for_service(timeout_sec=1.0):
                    req = SetBool.Request()
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


    def joint_state_callback(self, msg: JointStateSim) -> None:
        """
        Update qpos_target from the latest JointStateSim message.

        Assumes msg.name entries match MuJoCo joint names.
        """
        qpos_target = np.asarray(msg.position, dtype=self.qpos_target.dtype)
        qvel_target = np.asarray(msg.velocity, dtype=self.qvel_target.dtype)
        target_t = msg.sim_time
        self.qpos_target = qpos_target
        self.qvel_target = qvel_target
        self.target_t = target_t


def main(args=None) -> None:
    rclpy.init(args=args)
    xml_path = "/home/g201951870/projects/hri_rlc/src/sim_env/mujoco/kinova_gen3_table.xml"

    # Create node
    node = Gen3MujocoVizNode(xml_path=xml_path)
    env = node.env
    m, d = node.m, node.d

    # Define render callback for MuJoCo viewer
    def render_callback(viewer):
        rclpy.spin_once(node, timeout_sec=0.0)
        env.set_state(qpos=node.qpos_target, qvel=node.qvel_target, t=node.target_t)  # keep env in sync
    try:
        # Launch passive MuJoCo viewer; this call blocks until window is closed
        mj.viewer.launch_passive( # type: ignore
            m, d, 
            render_callback=render_callback, 
            key_callback=node.key_callback)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
