from __future__ import annotations
from pathlib import Path

from enum import Enum, auto 
from .paths import kinova_gen3_urdf

from rbt_core import Robot, RobotSpec

class Gen3Config(Enum):
    BASE = auto()
    WITH_GRIPPER = auto()

def gen3_urdf_path(*, with_gripper: bool = False) -> Path:
    return kinova_gen3_urdf("gen3_robotiq_2f_85" if with_gripper else "gen3")

def make_gen3_robot(*, with_gripper: bool = False, with_geometry: bool = True):
    urdf = gen3_urdf_path(with_gripper=with_gripper)

    robot_spec = RobotSpec(name="gen3", urdf=urdf, T_base=None, tcp_frame="")
    robot = Robot.from_spec(robot_spec)

    raise NotImplementedError("Connect to your rbt_core Pinocchio builder.")
