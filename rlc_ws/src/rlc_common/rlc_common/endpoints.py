"""Shared, typed topic / service / action names for the MuJoCo Gen3 stack."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Generic, TypeVar

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
from rlc_interfaces.msg import (
    JointEffortCmd,
    JointStateAction, 
    PlannedJointTrajectory, 
    PlannedCartesianTrajectory,
    CurrentPlan,
)

from rlc_interfaces.srv import (
    ExecuteTrajectory,
    PlanJointTrajectory,
    SetControllerGains,
    SetControllerMode,
)
from std_srvs.srv import SetBool, Trigger
from ament_index_python.packages import get_package_share_directory
import yaml

T = TypeVar("T")


@dataclass(frozen=True)
class Endpoint(Generic[T]):
    """A named ROS endpoint (topic, service, or action) with an associated type."""
    name: str
    type: type[T]

_CFG_PATH = Path(get_package_share_directory("rlc_common")) / "config" / "endpoints.yaml"
_CFG = yaml.safe_load(_CFG_PATH.read_text(encoding="utf-8"))
_NAMESPACES = {str(k): str(v) for k, v in _CFG["namespaces"].items()}


def _resolve_name(raw: str) -> str:
    for key, value in _NAMESPACES.items():
        raw = raw.replace(f"{{{key}}}", value)
    return raw

SIM_NS = _NAMESPACES["sim"]
CTRL_NS = _NAMESPACES["ctrl"]
PLAN_NS = _NAMESPACES["plan"]
EXEC_NS = _NAMESPACES["exec"]
MON_NS = _NAMESPACES["mon"]

@dataclass(frozen=True)
class TopicEndpoints:
    joint_state: Endpoint[JointState]
    joint_state_action: Endpoint[JointStateAction]
    effort_cmd: Endpoint[JointEffortCmd]
    planned_joint_traj: Endpoint[PlannedJointTrajectory]
    frame_states: Endpoint[PoseArray]
    planned_cart_traj: Endpoint[PlannedCartesianTrajectory]
    controller_state: Endpoint[JointTrajectoryControllerState]
    current_plan: Endpoint[CurrentPlan]


@dataclass(frozen=True)
class ServiceEndpoints:
    """All Gen3 services with their service types."""
    reset_sim: Endpoint[Trigger]
    pause_sim: Endpoint[SetBool]
    set_controller_gains: Endpoint[SetControllerGains]
    set_controller_mode: Endpoint[SetControllerMode]
    plan_quintic: Endpoint[PlanJointTrajectory]
    execute_traj: Endpoint[ExecuteTrajectory]


@dataclass(frozen=True)
class ActionEndpoints:
    """All Gen3 actions with their action types."""
    follow_traj: Endpoint[FollowJointTrajectory]


TOPICS = TopicEndpoints(
    joint_state=Endpoint(
        _resolve_name(_CFG["topics"]["joint_state"]),
        JointState,
    ),
    joint_state_action=Endpoint(
        _resolve_name(_CFG["topics"]["joint_state_action"]),
        JointStateAction,
    ),
    effort_cmd=Endpoint(
        _resolve_name(_CFG["topics"]["effort_cmd"]),
        JointEffortCmd,
    ),
    planned_joint_traj=Endpoint(
        _resolve_name(_CFG["topics"]["planned_joint_traj"]),
        PlannedJointTrajectory,
    ),
    frame_states=Endpoint(
        _resolve_name(_CFG["topics"]["frame_states"]),
        PoseArray,
    ),
    planned_cart_traj=Endpoint(
        _resolve_name(_CFG["topics"]["planned_cart_traj"]),
        PlannedCartesianTrajectory,
    ),
    controller_state=Endpoint(
        _resolve_name(_CFG["topics"]["controller_state"]),
        JointTrajectoryControllerState,
    ),
    current_plan=Endpoint(
        _resolve_name(_CFG["topics"]["current_plan"]),
        CurrentPlan,
    ),
)

SERVICES = ServiceEndpoints(
    reset_sim=Endpoint(
        name=_resolve_name(_CFG["services"]["reset_sim"]),
        type=Trigger,
    ),
    pause_sim=Endpoint(
        name=_resolve_name(_CFG["services"]["pause_sim"]),
        type=SetBool,
    ),
    set_controller_gains=Endpoint(
        name=_resolve_name(_CFG["services"]["set_controller_gains"]),
        type=SetControllerGains,
    ),
    set_controller_mode=Endpoint(
        name=_resolve_name(_CFG["services"]["set_controller_mode"]),
        type=SetControllerMode,
    ),
    plan_quintic=Endpoint(
        name=_resolve_name(_CFG["services"]["plan_quintic"]),
        type=PlanJointTrajectory,
    ),
    execute_traj=Endpoint(
        name=_resolve_name(_CFG["services"]["execute_traj"]),
        type=ExecuteTrajectory,
    ),
)

ACTIONS = ActionEndpoints(
    follow_traj=Endpoint(
        name=_resolve_name(_CFG["actions"]["follow_traj"]),
        type=FollowJointTrajectory,
    ),
)

# Type aliases for imports
# messages
JointStateMsg = TOPICS.joint_state.type
JointEffortCmdMsg = TOPICS.effort_cmd.type
PlannedJointTrajMsg = TOPICS.planned_joint_traj.type
FrameStatesMsg = TOPICS.frame_states.type
PlannedCartTrajMsg = TOPICS.planned_cart_traj.type
ControllerStateMsg = TOPICS.controller_state.type
CurrentPlanMsg = TOPICS.current_plan.type
JointStateActionMsg = TOPICS.joint_state_action.type

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
    "PlannedJointTrajMsg",
    "FrameStatesMsg",
    "ControllerStateMsg",
    "PlannedCartTrajMsg",
    "CurrentPlanMsg",
    "JointStateActionMsg",

    "ResetSimSrv",
    "PauseSimSrv",
    "SetControllerGainsSrv",
    "SetControllerModeSrv",
    "PlanQuinticSrv",
    "ExecuteTrajSrv",

    "FollowTrajAction",
]
