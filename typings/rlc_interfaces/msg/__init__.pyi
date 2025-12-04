from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist, Accel
from trajectory_msgs.msg import JointTrajectory
from builtin_interfaces.msg import Duration

__all__ = [
    "CartesianState",
    "FrameStates",
    "JointEffortCmd",
    "JointStateSim",
    "PlannedTrajectory",
    "EeTrajPoint",
    "EeTrajectory",
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


class JointStateSim:
    header: Header
    sim_time: float
    name: list[str]
    position: list[float]
    velocity: list[float]
    acceleration: list[float]
    effort: list[float]

    def __init__(
        self,
        header: Header = ...,
        sim_time: float = ...,
        name: list[str] = ...,
        position: list[float] = ...,
        velocity: list[float] = ...,
        acceleration: list[float] = ...,
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


class EeTrajPoint:
    time_from_start: Duration
    pose: Pose
    twist: Twist
    accel: Accel

    def __init__(
        self,
        time_from_start: Duration = ...,
        pose: Pose = ...,
        twist: Twist = ...,
        accel: Accel = ...,
        *,
        check_fields: bool | None = ...,
    ) -> None: ...


class EeTrajectory:
    header: Header
    points: list[EeTrajPoint]

    def __init__(
        self,
        header: Header = ...,
        points: list[EeTrajPoint] = ...,
        *,
        check_fields: bool | None = ...,
    ) -> None: ...
