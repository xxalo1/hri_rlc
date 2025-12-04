from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist
from trajectory_msgs.msg import JointTrajectory

__all__ = [
    "CartesianState",
    "JointEffortCmd",
    "JointStateSim",
    "PlannedTrajectory",
]


class CartesianState:
    header: Header
    pose: Pose
    twist: Twist

    def __init__(
        self,
        header: Header = ...,
        pose: Pose = ...,
        twist: Twist = ...,
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


class JointStateSim:
    header: Header
    sim_time: float
    name: list[str]
    position: list[float]
    velocity: list[float]
    effort: list[float]

    def __init__(
        self,
        header: Header = ...,
        sim_time: float = ...,
        name: list[str] = ...,
        position: list[float] = ...,
        velocity: list[float] = ...,
        effort: list[float] = ...,
        *,
        check_fields: bool | None = ...,
    ) -> None: ...


class PlannedTrajectory:
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
