from __future__ import annotations
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

DESCRIPTIONS_PKG = "rlc_robot_descriptions"  # change if you renamed

def descriptions_share() -> Path:
    return Path(get_package_share_directory(DESCRIPTIONS_PKG))

def kinova_gen3_dir() -> Path:
    return descriptions_share() / "robots" / "kinova_gen3"

def kinova_gen3_urdf(variant: str = "gen3") -> Path:
    return kinova_gen3_dir() / "urdf" / f"{variant}.urdf"
