from __future__ import annotations
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

DESCRIPTIONS_PKG = "rlc_robot_descriptions"  # change if you renamed

def descriptions_share() -> Path:
    return Path(get_package_share_directory(DESCRIPTIONS_PKG))

def kinova_gen3_dir() -> Path:
    return descriptions_share() / "robots" / "kinova_gen3"

def kinova_gen3_urdf(variant: str = "gen3") -> Path:
    if variant == "gen3":
        return kinova_gen3_dir() / "urdf" / "gen3.urdf"
    if variant == "gen3_robotiq_2f_85":
        return kinova_gen3_dir() / "urdf" / "gen3_robotiq_2f_85.urdf"
    raise ValueError(f"Unknown Gen3 variant: {variant}")
