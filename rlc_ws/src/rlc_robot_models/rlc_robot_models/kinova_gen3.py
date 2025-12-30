from __future__ import annotations
from pathlib import Path

from enum import Enum, auto 
from .paths import kinova_gen3_urdf

from rbt_core import Robot, RobotSpec


class Gen3Variant(Enum):
    DOF7_BASE = auto()
    DOF7_VISION = auto()
    DOF6_BASE = auto()
    DOF6_VISION = auto()
    DOF6_WITH_GRIPPER = auto()
    DOF7_WITH_GRIPPER = auto()


def gen3_urdf_path(
    *, 
    variant: Gen3Variant = Gen3Variant.DOF7_VISION
) -> Path:
    match variant:
        case Gen3Variant.DOF7_BASE:
            return kinova_gen3_urdf("gen3")
        case Gen3Variant.DOF7_VISION:
            return kinova_gen3_urdf("gen3_vision")
        case Gen3Variant.DOF6_BASE:
            return kinova_gen3_urdf("gen3_dof6")
        case Gen3Variant.DOF6_VISION:
            return kinova_gen3_urdf("gen3_dof6_vision")
        case Gen3Variant.DOF6_WITH_GRIPPER:
            return kinova_gen3_urdf("gen3_dof6_robotiq_2f_85")
        case Gen3Variant.DOF7_WITH_GRIPPER:
            return kinova_gen3_urdf("gen3_robotiq_2f_85")

def make_gen3_robot(
    *, 
    variant: Gen3Variant = Gen3Variant.DOF7_VISION, 
    with_geometry: bool = True
) -> Robot:
    urdf = gen3_urdf_path(variant=variant)

    robot_spec = RobotSpec(
                name=variant.name, 
                urdf=urdf, 
                tcp_frame="tool_frame")

    return Robot.from_spec(robot_spec)
