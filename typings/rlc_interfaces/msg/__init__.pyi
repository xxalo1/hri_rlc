from typing import List
from std_msgs.msg import Header

class JointEffortCmd:
    header: Header
    name: List[str]
    effort: List[float]

    def __init__(self, *args, **kwargs) -> None: ...


class JointStateSim:
    header: Header
    sim_time: float
    name: List[str]
    position: List[float]
    velocity: List[float]
    effort: List[float]

    def __init__(self, *args, **kwargs) -> None: ...

__all__ = ["JointEffortCmd", "JointStateSim"]
