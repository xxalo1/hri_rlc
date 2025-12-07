from __future__ import annotations
import time

import numpy as np
import mujoco as mj
from mujoco import viewer # type: ignore

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
from sim_env.mujoco.env import Gen3Env
from common_utils import ros_util as ru

from rlc_common.endpoints import (
    TOPICS, SERVICES, ACTIONS, 
    JointStateMsg, JointEffortCmdMsg, PlannedCartTrajMsg, CurrentPlanMsg,
    ResetSimSrv, PauseSimSrv, FrameStatesMsg
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
        self.cart_traj_sub = self.create_subscription(
            TOPICS.planned_cart_traj.type,
            TOPICS.planned_cart_traj.name,
            self.cart_traj_callback,
            10,
        )
        self.current_plan_sub = self.create_subscription(
            TOPICS.current_plan.type,
            TOPICS.current_plan.name,
            self.current_plan_callback,
            10,
        )
        self.frame_states_sub = self.create_subscription(
            TOPICS.frame_states.type,
            TOPICS.frame_states.name,
            self.frame_state_callback,
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


    def _invalidate_frames(self) -> None:
        if not self.frames_dirty: self.frames_dirty = True 


    def _pause_sim(self, pause: bool) -> None:
        if self.pause_client.wait_for_service(timeout_sec=1.0):
            req = PauseSimSrv.Request()
            req.data = pause
            future = self.pause_client.call_async(req)
            self.get_logger().info("pause/unpause service called")
    

    def _reset_sim(self) -> None:
        if self.reset_client.wait_for_service(timeout_sec=1.0):
            req = ResetSimSrv.Request()
            future = self.reset_client.call_async(req)
            self.get_logger().info("Reset service called")


    def key_callback(self, keycode):
        c = chr(keycode)
        match c:
            case " ":
                self._pause_sim(False) if self.paused else self._pause_sim(True)
            case "r" | "R":
                self._reset_sim()
                self._pause_sim(True)
            case "1":
                self.show_frames = not self.show_frames
                self._invalidate_frames()

            case "j" | "J":
                self.show_jacobian = True

            case "d" | "D":
                self.show_dynamics = True

            case "i" | "I":
                self.show_inertia = True

            case "'":
                self.show_com = not self.show_com


    def cart_traj_callback(self, msg: PlannedCartTrajMsg) -> None:
        """
        Update qpos_target from the latest JointStateSim message.

        Assumes msg.name entries match MuJoCo joint names.
        """
        pass


    def current_plan_callback(self, msg: CurrentPlanMsg) -> None:
        """
        Update qpos_target from the latest JointStateSim message.

        Assumes msg.name entries match MuJoCo joint names.
        """
        pass


    def frame_state_callback(self, msg: FrameStatesMsg) -> None:
        """
        Update qpos_target from the latest JointStateSim message.

        Assumes msg.name entries match MuJoCo joint names.
        """
        state = ru.from_pose_array_msg(msg)
        self.poses = state.poses
        self._invalidate_frames()


    def joint_state_callback(self, msg: JointStateMsg) -> None:
        """
        Update qpos_target from the latest JointStateSim message.

        Assumes msg.name entries match MuJoCo joint names.
        """
        state = ru.from_joint_state_msg(msg)
        qpos_target = np.asarray(state.positions, dtype=self.qpos_target.dtype)
        qvel_target = np.asarray(state.velocities, dtype=self.qvel_target.dtype)
        self.qpos_target = qpos_target
        self.qvel_target = qvel_target
        self.target_t = state.stamp


def handle_frame_visuals(env: Gen3Env, viewer: viewer.MjViewer) -> None:
    """Render frame visuals in the MuJoCo viewer."""
    if node.frames_dirty and node.frames_exists:
        # destroy frames
        if node.show_frames:
            # create frames
            node.frames_exists = True
        else:
            node.frames_exists = False
        node.frames_dirty = False


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
                    
                    if self.show_frames and node.frames_dirty:
                        mj.
                        node.frames_dirty = False
                    
                    v.sync()
                time.sleep(0.0001)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
