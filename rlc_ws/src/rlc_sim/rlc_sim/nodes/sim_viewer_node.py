from __future__ import annotations
from dataclasses import dataclass
import time

from mujoco import viewer # type: ignore

import rclpy
from rclpy.node import Node
from sim_env.mujoco.kinova_gen3_env import Gen3Env
from sim_env.mujoco.viz_env import VizEnv
from rlc_utils import msg_conv as rmsg
from rlc_utils.config import qos_latest
from rlc_common.endpoints import (
    TOPICS, SERVICES,
    JointStateMsg, PlannedCartTrajMsg, CurrentPlanMsg,
    ResetSimSrv, PauseSimSrv, FrameStatesMsg,
)
from rlc_planner.executor_node import TrajectoryCache, CachedTraj, TrajKind

@dataclass
class CurrentPlan:
    traj_id: str = ""
    active_traj_id: str | None = None
    status: int = CurrentPlanMsg.STATUS_NONE
    stamp: float = 0.0

class Gen3MujocoVizNode(Node):
    """
    Visualization node for Kinova Gen3 using MuJoCo viewer.

    - Owns its own MuJoCo model/data
    - Subscribes to the headless sim joint state topic
    - Mirrors joint positions into d.qpos and calls mj_forward in the render loop
    """
    def __init__(self) -> None:
        super().__init__("gen3_mujoco_viz")

        self.cache = TrajectoryCache(ttl_sec=120.0)
        self.current_plan = CurrentPlan()
        self.env = Gen3Env.from_default_scene()
        self.viz = VizEnv(
            env=self.env,
            on_pause=self._pause_sim,
            on_reset=self._reset_sim,
        )

        # Subscribe to joint states from the headless sim
        self.joint_state_sub = self.create_subscription(
            TOPICS.joint_state.type,
            TOPICS.joint_state.name,
            self.joint_state_callback,
            qos_latest,
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


    def cart_traj_callback(self, msg: PlannedCartTrajMsg) -> None:
        """
        """
        now = self._now()
        item = CachedTraj(
            stamp=now,
            kind=TrajKind.CART,
            label=msg.label,
            traj=msg.trajectory,
        )
        self.cache.put(msg.trajectory_id, item)
        self.cache.cleanup(now)


    def current_plan_callback(self, msg: CurrentPlanMsg) -> None:
        """
        Update qpos_target from the latest JointStateSim message.

        Assumes msg.name entries match MuJoCo joint names.
        """
        self.get_logger().info(f"Received current plan update: traj_id={msg.plan_id}, status={msg.status}")
        self.current_plan.stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.current_plan.traj_id = msg.plan_id
        self.current_plan.status = msg.status

        self.refresh_planned_trajectory()


    def refresh_planned_trajectory(self) -> None:
        traj_id = self.current_plan.traj_id
        status = self.current_plan.status
        active_traj_id = self.current_plan.active_traj_id

        if status == CurrentPlanMsg.STATUS_ACTIVE:
            if active_traj_id == traj_id:
                return
            traj = self.cache.get(traj_id=traj_id, now=self._now())
            cart_traj = rmsg.from_multi_dof_traj_msg(traj.traj) # type: ignore
            self.viz.set_planned_traj(cart_traj.poses)
            self.current_plan.active_traj_id = traj_id
            return
        else:
            if active_traj_id == traj_id:
                self.viz.set_planned_traj(None)
                self.current_plan.active_traj_id = None


    def frame_state_callback(self, msg: FrameStatesMsg) -> None:
        """
        Update qpos_target from the latest JointStateSim message.

        Assumes msg.name entries match MuJoCo joint names.
        """
        state = rmsg.from_pose_array_msg(msg)
        self.viz.set_frame_states(state.poses)


    def joint_state_callback(self, msg: JointStateMsg) -> None:
        """
        Update qpos_target from the latest JointStateSim message.

        Assumes msg.name entries match MuJoCo joint names.
        """
        state = rmsg.from_joint_state_msg(msg)
        self.viz.set_joint_states(
            state.positions, 
            state.velocities, 
            state.stamp,
            state.efforts, 
        )


    def _now(self) -> float:
        """Current time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)

    node = Gen3MujocoVizNode()
    env = node.env
    viz = node.viz

    try:
        with viewer.launch_passive(env.m, env.d, key_callback=viz.key_callback) as v:
            while v.is_running() and rclpy.ok():

                rclpy.spin_once(node, timeout_sec=0.0)

                with v.lock():
                    viz.sync(v)
                    v.sync()
                time.sleep(0.0001)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
