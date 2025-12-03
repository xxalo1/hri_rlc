"""Shared, typed topic / service / action names for the MuJoCo Gen3 stack."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, TypeVar

from control_msgs.action import FollowJointTrajectory
from rlc_interfaces.msg import JointEffortCmd, JointStateSim, PlannedTrajectory
from rlc_interfaces.srv import (
    ExecuteTrajectory,
    PlanTrajectory,
    SetControllerGains,
    SetControllerMode,
)
from std_srvs.srv import SetBool, Trigger

T = TypeVar("T")


@dataclass(frozen=True)
class Endpoint(Generic[T]):
    """A named ROS endpoint (topic, service, or action) with an associated type."""
    name: str
    type: type[T]


SIM_NS = "sim/gen3"
CTRL_NS = "controller/gen3"
PLANNER_NS = "planner/gen3"
EXEC_NS = "executor/gen3"


@dataclass(frozen=True)
class TopicEndpoints:
    """All Gen3 topics with their message types."""
    joint_state: Endpoint[JointStateSim]
    effort_cmd: Endpoint[JointEffortCmd]
    planned_traj: Endpoint[PlannedTrajectory]


@dataclass(frozen=True)
class ServiceEndpoints:
    """All Gen3 services with their service types."""
    reset_sim: Endpoint[Trigger]
    pause_sim: Endpoint[SetBool]
    set_controller_gains: Endpoint[SetControllerGains]
    set_controller_mode: Endpoint[SetControllerMode]
    plan_quintic: Endpoint[PlanTrajectory]
    plan_point: Endpoint[PlanTrajectory]
    execute_traj: Endpoint[ExecuteTrajectory]


@dataclass(frozen=True)
class ActionEndpoints:
    """All Gen3 actions with their action types."""
    follow_traj: Endpoint[FollowJointTrajectory]


TOPICS = TopicEndpoints(
    joint_state=Endpoint(
        name=f"{SIM_NS}/state/joints",
        type=JointStateSim,
    ),
    effort_cmd=Endpoint(
        name=f"{CTRL_NS}/command/effort",
        type=JointEffortCmd,
    ),
    planned_traj=Endpoint(
        name=f"{PLANNER_NS}/planned_trajectory",
        type=PlannedTrajectory,
    ),
)

SERVICES = ServiceEndpoints(
    reset_sim=Endpoint(
        name=f"{SIM_NS}/reset",
        type=Trigger,
    ),
    pause_sim=Endpoint(
        name=f"{SIM_NS}/pause",
        type=SetBool,
    ),
    set_controller_gains=Endpoint(
        name=f"{CTRL_NS}/set_controller_gains",
        type=SetControllerGains,
    ),
    set_controller_mode=Endpoint(
        name=f"{CTRL_NS}/set_control_mode",
        type=SetControllerMode,
    ),
    plan_quintic=Endpoint(
        name=f"{PLANNER_NS}/plan_quintic",
        type=PlanTrajectory,
    ),
    plan_point=Endpoint(
        name=f"{PLANNER_NS}/plan_point",
        type=PlanTrajectory,
    ),
    execute_traj=Endpoint(
        name=f"{EXEC_NS}/execute",
        type=ExecuteTrajectory,
    ),
)

ACTIONS = ActionEndpoints(
    follow_traj=Endpoint(
        name=f"{CTRL_NS}/follow_trajectory",
        type=FollowJointTrajectory,
    ),
)

# Type aliases for imports
# messages
JointStateMsg = TOPICS.joint_state.type
JointEffortCmdMsg = TOPICS.effort_cmd.type
PlannedTrajMsg = TOPICS.planned_traj.type

# services
ResetSimSrv = SERVICES.reset_sim.type
PauseSimSrv = SERVICES.pause_sim.type
SetControllerGainsSrv = SERVICES.set_controller_gains.type
SetControllerModeSrv = SERVICES.set_controller_mode.type
PlanQuinticSrv = SERVICES.plan_quintic.type
ExecuteTrajSrv = SERVICES.execute_traj.type

# actions
FollowTrajAction = ACTIONS.follow_traj.type

__all__ = [
    "TOPICS",
    "SERVICES",
    "ACTIONS",
    "JointStateMsg",
    "JointEffortCmdMsg",
    "PlannedTrajMsg",
    "ResetSimSrv",
    "PauseSimSrv",
    "SetControllerGainsSrv",
    "SetControllerModeSrv",
    "PlanQuinticSrv",
    "ExecuteTrajSrv",
    "FollowTrajAction",
]