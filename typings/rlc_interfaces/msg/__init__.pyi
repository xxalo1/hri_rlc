from typing import List

class JointEffortCmd:
    header: object
    name: List[str]
    effort: List[float]

    def __init__(self, *args, **kwargs) -> None: ...


class JointStateSim:
    header: object
    sim_time: float
    name: List[str]
    position: List[float]
    velocity: List[float]
    effort: List[float]

    def __init__(self, *args, **kwargs) -> None: ...

__all__ = ["JointEffortCmd", "JointStateSim"]
