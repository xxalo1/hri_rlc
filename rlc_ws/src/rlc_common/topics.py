"""Shared topic and service names for MuJoCo Gen3 simulation nodes."""

SIM_NAMESPACE = "sim/gen3"
JOINT_STATE_TOPIC = f"{SIM_NAMESPACE}/state/joints"
RESET_SERVICE = f"{SIM_NAMESPACE}/reset"
PAUSE_SERVICE = f"{SIM_NAMESPACE}/pause"

CONTROLLER_NAMESPACE = "controller/gen3"
EFFORT_COMMAND_TOPIC = f"{CONTROLLER_NAMESPACE}/command/effort"

__all__ = [
    "EFFORT_COMMAND_TOPIC",
    "JOINT_STATE_TOPIC",
    "RESET_SERVICE",
    "PAUSE_SERVICE",
]
