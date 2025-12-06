"""Shared, typed topic / service / action names for the MuJoCo Gen3 stack."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, TypeVar

from trajectory_msgs.msg import pos
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from rlc_interfaces.msg import (
    JointEffortCmd, 
    PlannedJointTrajectory, 
    FrameStates, 
    PlannedEeTrajectory,
    CurrentPlan,
)

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


SIM_NS   = "sim/gen3"
CTRL_NS  = "ctrl/gen3"
PLAN_NS  = "plan/gen3"
EXEC_NS  = "exec/gen3"
MON_NS   = "mon/gen3"

@dataclass(frozen=True)
class TopicEndpoints:
    joint_state: Endpoint[JointState]
    effort_cmd: Endpoint[JointEffortCmd]
    planned_joint_traj: Endpoint[PlannedJointTrajectory]
    frame_states: Endpoint[FrameStates]
    planned_ee_traj: Endpoint[PlannedEeTrajectory]
    controller_state: Endpoint[JointTrajectoryControllerState]
    current_plan: Endpoint[CurrentPlan]


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
    joint_state=Endpoint(f"{SIM_NS}/joint_state", JointState),
    effort_cmd=Endpoint(f"{CTRL_NS}/effort_cmd", JointEffortCmd),
    planned_joint_traj=Endpoint(f"{PLAN_NS}/planned_joint_traj", PlannedJointTrajectory),
    frame_states=Endpoint(f"{MON_NS}/frame_states", FrameStates),
    planned_ee_traj=Endpoint(f"{PLAN_NS}/planned_ee_traj", PlannedEeTrajectory),
    controller_state=Endpoint(f"{CTRL_NS}/controller_state", JointTrajectoryControllerState),
    current_plan=Endpoint(f"{EXEC_NS}/current_plan", CurrentPlan),
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
        name=f"{PLAN_NS}/plan_quintic",
        type=PlanTrajectory,
    ),
    plan_point=Endpoint(
        name=f"{PLAN_NS}/plan_point",
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
PlannedJointTrajMsg = TOPICS.planned_joint_traj.type
FrameStatesMsg = TOPICS.frame_states.type
PlannedEeTrajMsg = TOPICS.planned_ee_traj.type
ControllerStateMsg = TOPICS.controller_state.type
CurrentPlanMsg = TOPICS.current_plan.type

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
    "PlannedEeTrajMsg",
    "CurrentPlanMsg",

    "ResetSimSrv",
    "PauseSimSrv",
    "SetControllerGainsSrv",
    "SetControllerModeSrv",
    "PlanQuinticSrv",
    "ExecuteTrajSrv",

    "FollowTrajAction",
]