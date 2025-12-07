from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist, Accel
from trajectory_msgs.msg import JointTrajectory, MultiDOFJointTrajectory
from builtin_interfaces.msg import Time

__all__ = [
    "FrameStates",
    "JointEffortCmd",
    "PlannedJointTrajectory",
    "CurrentPlan",
    "PlannedCartesianTrajectory"
]


class CurrentPlan:
    plan_id: str
    label: str
    status: int
    stamp: Time 
    trajectory_id: str
    STATUS_NONE: int = ...
    STATUS_PENDING: int = ...
    STATUS_ACTIVE: int = ...
    STATUS_SUCCEEDED: int = ...
    STATUS_ABORTED: int = ...
    STATUS_CANCELED: int = ...

    def __init__(
        self,
        plan_id: str = ...,
        label: str = ...,
        status: int = ...,
        stamp: Time = ...,  # Time
        trajectory_id: str = ...,
        *,
        check_fields: bool | None = ...,
    ) -> None: ...


class FrameStates:
    header: Header
    frame_ids: list[str]
    poses: list[Pose]

    def __init__(
        self,
        header: Header = ...,
        frame_ids: list[str] = ...,
        poses: list[Pose] = ...,
        *,
        check_fields: bool | None = ...,
    ) -> None: ...


class JointEffortCmd:
    header: Header
    name: list[str]
    effort: list[float]
    time_sim: float

    def __init__(
        self,
        header: Header = ...,
        name: list[str] = ...,
        effort: list[float] = ...,
        time_sim: float = ...,
        *,
        check_fields: bool | None = ...,
    ) -> None: ...


class PlannedJointTrajectory:
    trajectory_id: str
    trajectory: JointTrajectory
    execute_immediately: bool
    label: str

    def __init__(
        self,
        trajectory_id: str = ...,
        trajectory: JointTrajectory = ...,
        execute_immediately: bool = ...,
        label: str = ...,
        *,
        check_fields: bool | None = ...,
    ) -> None: ...


class PlannedCartesianTrajectory:
    trajectory_id: str
    trajectory: MultiDOFJointTrajectory
    execute_immediately: bool
    label: str
    derived_from_joint: bool

    def __init__(
        self,
        trajectory_id: str = ...,
        trajectory: MultiDOFJointTrajectory = ...,
        execute_immediately: bool = ...,
        label: str = ...,
        derived_from_joint: bool = ...,
        *,
        check_fields: bool | None = ...,
    ) -> None: ...
