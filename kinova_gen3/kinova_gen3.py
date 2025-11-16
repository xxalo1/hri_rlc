from typing import Sequence
import numpy as np
import torch
import yaml, copy, pandas as pd
from numpy import pi
from pathlib import Path

from..utils import numpy_util as npu
from..utils import dtype, FloatArray, ArrayT
from ..utils import pytorch_util as ptu
dtype = npu.dtype

HERE = Path(__file__).parent
INERT_FILE = HERE / "inertial.yaml"
DH_FILE = HERE / "dh.yaml"
ANGLES = [0, pi, pi, pi, pi, pi, pi, pi]
SKIP_LINKS = ("ee_with_vision",)

def Rx(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0],
                     [0, c,-s],
                     [0, s, c]], dtype=np.float32)


def load_inertial(yaml_path, skip_links=SKIP_LINKS) -> dict[str, FloatArray]:
    """
    Load inertial data and return stacked NumPy arrays.

    Returns a dict with:
      id  : (n,)   int64
      m   : (n,)   float_dtype
      com : (n,3)  float_dtype
      Ic  : (n,3,3) float_dtype
    """
    # accept a path or a preloaded dict
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    rows = [r for r in data["links"] if r.get("name") not in skip_links]
    n = len(rows)

    m   = np.fromiter((float(r["mass_kg"]) for r in rows), dtype=dtype, count=n)
    com = np.array([r["com_m"] for r in rows], dtype=dtype)        # (n,3)
    Ic  = np.array([r["Ic_kgm2"] for r in rows], dtype=dtype)      # (n,3,3)

    return {"m": m, "com": com, "Ic": Ic}


def rotated_inertials(model: dict[str, FloatArray], R_wf: FloatArray) -> dict[str, FloatArray]:
    """
    Rotate COM and inertia for each link.
      com' = R com
      Ic'  = R Ic R^T
    Shapes:
      R_wf : (n,3,3)
      model['com'] : (n,3)
      model['Ic']  : (n,3,3)
    Returns a new dict with same keys, arrays updated.
    """
    com = model["com"]            # (n,3)
    Ic  = model["Ic"]             # (n,3,3)

    com_rot = np.einsum("nij,nj->ni", R_wf, com)      # R @ com
    Ic_rot  = np.einsum("nij,njk,nlk->nil", R_wf, Ic, R_wf)       # R Ic R^T

    out = dict(model)
    out["com"] = com_rot
    out["Ic"]  = Ic_rot
    return out


def load_inertia() -> dict[str, FloatArray]:
    """Load Kinova Gen3 inertial model and apply per-link flips (R @ com, R Ic R^T)."""
    inertia = load_inertial(INERT_FILE)
    n = inertia["m"].shape[0]

    angles = np.asarray(ANGLES, dtype=dtype)
    Rflip  = np.stack([Rx(theta) for theta in angles], axis=0).astype(inertia["com"].dtype)

    return rotated_inertials(inertia, Rflip)


def load_dh() -> dict[str, FloatArray]:
    """
    Read a DH YAML (with key 'dh': list of rows) and return a dict of NumPy arrays:
      {'id': (n,), 'd': (n,), 'a': (n,), 'alpha': (n,), 'theta0': (n,), 'b': (n,)}
    """
    with open(DH_FILE, "r") as f:
        data = yaml.safe_load(f)

    rows = list(data["dh"])
    rows.sort(key=lambda r: int(r.get("id", 0)))
    n = len(rows)

    d      = np.fromiter((float(r["d"])      for r in rows), dtype=dtype,    count=n)
    a      = np.fromiter((float(r["a"])      for r in rows), dtype=dtype,    count=n)
    alpha  = np.fromiter((float(r["alpha"])  for r in rows), dtype=dtype,    count=n)
    q0 = np.fromiter((float(r["q0"]) for r in rows), dtype=dtype,    count=n)
    b      = np.fromiter((float(r.get("b", 0.0)) for r in rows), dtype=dtype, count=n)

    return {"d": d, "a": a, "alpha": alpha, "q0": q0, "b": b}

def load_kinova_gen3() -> tuple[dict[str, FloatArray], dict[str, FloatArray], dict[str, torch.Tensor],   dict[str, torch.Tensor]]:
    """Load both DH and inertial parameters for Kinova Gen3 robot."""
    dh = load_dh()
    inertia = load_inertia()
    dh_torch = ptu.from_numpy_dict(dh)
    inertia_torch = ptu.from_numpy_dict(inertia)
    return dh, inertia, dh_torch, inertia_torch