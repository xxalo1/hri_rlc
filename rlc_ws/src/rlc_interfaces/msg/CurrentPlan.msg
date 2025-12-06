# CurrentPlan.msg

# Identifier of the plan / trajectory in the system.
string plan_id

# Optional human-readable label for this plan (e.g., "quintic", "cartesian_pick").
string label

# Execution status of the plan.
uint8 status

# Time at which this status was generated.
builtin_interfaces/Time stamp

uint8 STATUS_NONE=0
uint8 STATUS_PENDING=1
uint8 STATUS_ACTIVE=2
uint8 STATUS_SUCCEEDED=3
uint8 STATUS_ABORTED=4
uint8 STATUS_CANCELED=5
