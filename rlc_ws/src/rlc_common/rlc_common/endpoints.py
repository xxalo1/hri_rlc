"""Shared topic and service names for MuJoCo Gen3 simulation nodes."""
from __future__ import annotations
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Generic, Type, TypeVar

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
    joint_state=Endpoint(f"/{SIM_NS}/state/joints", JointState),
    effort_cmd=Endpoint(f"/{CTRL_NS}/command/effort", Float64MultiArray),
    # planned_traj=Endpoint(f"/{PLANNER_NS}/planned_trajectory", PlannedTrajectory),
)

SERVICES = SimpleNamespace(
    reset_sim=Endpoint(f"/{SIM_NS}/reset", Trigger),
    pause_sim=Endpoint(f"/{SIM_NS}/pause", Trigger),
    # set_joint_gains=Endpoint(f"/{CTRL_NS}/set_joint_gains", SetJointGains),
    # set_control_mode=Endpoint(f"/{CTRL_NS}/set_control_mode", SetControlMode),
)

ACTIONS = SimpleNamespace(
    follow_traj=Endpoint(f"/{CTRL_NS}/follow_trajectory", FollowJointTrajectory),
)

SIM_NAMESPACE = "sim/gen3"
JOINT_STATE_TOPIC = f"{SIM_NAMESPACE}/state/joints"
RESET_SIM_SERVICE = f"{SIM_NAMESPACE}/reset"
PAUSE_SIM_SERVICE = f"{SIM_NAMESPACE}/pause"

CONTROLLER_NAMESPACE = "controller/gen3"
EFFORT_COMMAND_TOPIC = f"{CONTROLLER_NAMESPACE}/command/effort"
SET_GAINS_SERVICE = f"{CONTROLLER_NAMESPACE}/set_joint_gains"
FOLLOW_TRAJECTORY_ACTION = f"{CONTROLLER_NAMESPACE}/follow_trajectory"
SET_CONTROL_MODE_SERVICE = f"{CONTROLLER_NAMESPACE}/set_control_mode"

PLANNER_NAMESPACE = "planner/gen3"
QUINTIC_TRAJECTORY_SERVICE = f"{PLANNER_NAMESPACE}/plan_quintic"
POINT_TRAJECTORY_SERVICE = f"{PLANNER_NAMESPACE}/plan_point"
PLANNED_TRAJECTORY_TOPIC = f"{PLANNER_NAMESPACE}/planned_trajectory"

EXECUTOR_NAMESPACE = "executor/gen3"
EXECUTE_TRAJECTORY_SERVICE = f"{EXECUTOR_NAMESPACE}/execute"

__all__ = [
    "EXECUTE_TRAJECTORY_SERVICE",
    "EFFORT_COMMAND_TOPIC",
    "FOLLOW_TRAJECTORY_ACTION",
    "JOINT_STATE_TOPIC",
    "PAUSE_SIM_SERVICE",
    "PLANNED_TRAJECTORY_TOPIC",
    "PLANNER_NAMESPACE",
    "POINT_TRAJECTORY_SERVICE",
    "RESET_SIM_SERVICE",
    "SET_GAINS_SERVICE",
    "QUINTIC_TRAJECTORY_SERVICE",
    "SET_CONTROL_MODE_SERVICE"
]
