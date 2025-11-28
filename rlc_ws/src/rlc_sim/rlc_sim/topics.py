"""Shared topic and service names for MuJoCo Gen3 simulation nodes."""

SIM_NAMESPACE = "/sim/gen3"

EFFORT_COMMAND_TOPIC = f"{SIM_NAMESPACE}/command/effort"
JOINT_STATE_TOPIC = f"{SIM_NAMESPACE}/state/joints"
RESET_SERVICE = f"{SIM_NAMESPACE}/control/reset"
PAUSE_SERVICE = f"{SIM_NAMESPACE}/control/pause"

__all__ = [
    "SIM_NAMESPACE",
    "EFFORT_COMMAND_TOPIC",
    "JOINT_STATE_TOPIC",
    "RESET_SERVICE",
    "PAUSE_SERVICE",
]
