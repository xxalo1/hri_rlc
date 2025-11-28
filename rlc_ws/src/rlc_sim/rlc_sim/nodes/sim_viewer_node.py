#!/usr/bin/env python3
from __future__ import annotations

import time
from typing import Dict

import numpy as np
import mujoco as mj

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, SetBool

from ..envs.env import Gen3Env


class Gen3MujocoVizNode(Node):
    """
    Visualization node for Kinova Gen3 using MuJoCo viewer.

    - Owns its own MuJoCo model/data
    - Subscribes to /sim/gen3/joint_states
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

        # Subscribe to joint states from the headless sim
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/sim/gen3/joint_states",
            self.joint_state_callback,
            10,
        )

        self.reset_client = self.create_client(Trigger, "/sim/gen3/reset")
        self.pause_client = self.create_client(SetBool, "/sim/gen3/set_paused")

        self.get_logger().info(
            f"Gen3 MuJoCo viz node ready. Listening to /sim/gen3/joint_states"
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


    def joint_state_callback(self, msg: JointState) -> None:
        """
        Update qpos_target from the latest JointState message.

        Assumes msg.name entries match MuJoCo joint names.
        """
        # For safety, keep a local copy
        qpos_new = self.qpos_target.copy()

        # Build a map name -> position from the message
        name_to_pos = dict(zip(msg.name, msg.position))

        for name, pos in name_to_pos.items():
            j_id = self.env.joint_ids[name]
            addr = self.m.jnt_qposadr[j_id]
            qpos_new[addr] = pos

        self.qpos_target = qpos_new


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
        env.set_state(qpos=d.qpos, qvel=d.qvel)  # keep env in sync
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
