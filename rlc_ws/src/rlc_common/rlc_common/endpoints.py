"""Shared, typed topic/service/action names for the MuJoCo Gen3 stack."""
from __future__ import annotations
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Generic, Type, TypeVar

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
    name: str
    type: Type[T]


SIM_NS = "sim/gen3"
CTRL_NS = "controller/gen3"
PLANNER_NS = "planner/gen3"
EXEC_NS = "executor/gen3"


TOPICS = SimpleNamespace(
    joint_state=Endpoint(f"{SIM_NS}/state/joints", JointStateSim),
    effort_cmd=Endpoint(f"{CTRL_NS}/command/effort", JointEffortCmd),
    planned_traj=Endpoint(f"{PLANNER_NS}/planned_trajectory", PlannedTrajectory),
)


SERVICES = SimpleNamespace(
    reset_sim=Endpoint(f"{SIM_NS}/reset", Trigger),
    pause_sim=Endpoint(f"{SIM_NS}/pause", SetBool),
    set_joint_gains=Endpoint(f"{CTRL_NS}/set_joint_gains", SetControllerGains),
    set_control_mode=Endpoint(f"{CTRL_NS}/set_control_mode", SetControllerMode),
    plan_quintic=Endpoint(f"{PLANNER_NS}/plan_quintic", PlanTrajectory),
    plan_point=Endpoint(f"{PLANNER_NS}/plan_point", PlanTrajectory),
    execute_traj=Endpoint(f"{EXEC_NS}/execute", ExecuteTrajectory),
)


ACTIONS = SimpleNamespace(
    follow_traj=Endpoint(f"{CTRL_NS}/follow_trajectory", FollowJointTrajectory),
)

