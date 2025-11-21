from typing import Sequence
import numpy as np
import torch
import yaml, copy, pandas as pd
from numpy import pi
from pathlib import Path

from..utils import numpy_util as npu
from..utils import dtype, FloatArray, ArrayT
from ..utils import pytorch_util as ptu
from ..robot.kin import core_ops as cops
dtype = npu.dtype

HERE = Path(__file__).parent
INERT_FILE = HERE / "inertial.yaml"
DH_FILE = HERE / "dh_v2.yaml"
ZD =      [0,  0,     0,    0,     0,    0,     0,    -0.0615]
YD =      [0, -0.1284, 0, -0.2104, 0,  -0.1059, 0,    0]
THETA_Z = [0,  0,     pi,   0,     pi,   0,     pi,   0]
THETA_X = [pi, pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, -pi]
SKIP_LINKS = ("ee_no_vision",)

def compute_Rx(angle):
    c, s = np.cos(angle), np.sin(angle)
    T = np.eye(4, dtype=npu.dtype)
    R = np.array([[1, 0, 0],
                     [0, c,-s],
                     [0, s, c]], dtype=npu.dtype)
    T[:3, :3] = R
    return T

def compute_Rz(angle):
    c, s = np.cos(angle), np.sin(angle)
    T = np.eye(4, dtype=npu.dtype)
    R =  np.array([[ c,-s, 0],
                     [ s, c, 0],
                     [ 0, 0, 1]], dtype=npu.dtype)
    T[:3, :3] = R
    return T

def compute_Ry(angle):
    c, s = np.cos(angle), np.sin(angle)
    T = np.eye(4, dtype=npu.dtype)
    R = np.array([[ c, 0, s],
                  [ 0, 1, 0],
                  [-s, 0, c]], dtype=npu.dtype)
    T[:3, :3] = R
    return T

def compute_Ty(distance):
    T = np.eye(4, dtype=npu.dtype)
    T[1, 3] = distance
    return T

def compute_Tz(distance):
    T = np.eye(4, dtype=npu.dtype)
    T[2, 3] = distance
    return T


def load_inertia(yaml_path, skip_links=SKIP_LINKS) -> dict[str, FloatArray]:
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


def load_dh(file) -> dict[str, FloatArray]:
    """
    Read a DH YAML (with key 'dh': list of rows) and return a dict of NumPy arrays:
      {'id': (n,), 'd': (n,), 'a': (n,), 'alpha': (n,), 'theta0': (n,), 'b': (n,)}
    """
    with open(file, "r") as f:
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


def change_of_basis(d: dict[str, FloatArray], dh: dict[str, FloatArray]) -> tuple[dict[str, FloatArray], FloatArray]:

    angles_x = np.asarray(THETA_X, dtype=dtype)
    angles_z = np.asarray(THETA_Z, dtype=dtype)
    y_d      = np.asarray(YD,      dtype=dtype)
    z_d      = np.asarray(ZD,      dtype=dtype)

    Ty  = np.stack([compute_Ty(distance) for distance in y_d], axis=0).astype(d["com"].dtype)
    TRx = np.stack([compute_Rx(theta)    for theta in angles_x], axis=0).astype(d["com"].dtype)
    TRz = np.stack([compute_Rz(theta)    for theta in angles_z], axis=0).astype(d["com"].dtype)
    Tz  = np.stack([compute_Tz(distance) for distance in z_d], axis=0).astype(d["com"].dtype)

    T = Tz @ TRx @ TRz @ Ty              # (n, 4, 4)
    com = d["com"]                       # (n, 3)
    Ic  = d["Ic"]                        # (n, 3, 3)

    ones   = np.ones((com.shape[0], 1), dtype=com.dtype)
    com_h  = np.concatenate([com, ones], axis=1)                 # (n, 4)

    T_inv     = np.linalg.inv(T)                                 # (n, 4, 4)
    com_new_h = np.einsum("nij,nj->ni", T_inv, com_h)            # (n, 4)
    com_new   = com_new_h[:, :3]                                 # (n, 3)

    R = T_inv[:, :3, :3]

    Ic_new = np.einsum("nik,nkl,njl->nij", R, Ic, R)

    out = dict(d)
    out["com"] = com_new
    out["Ic"]  = Ic_new
    return out, T[0]


def load_kinova_gen3_v2() -> tuple[dict[str, FloatArray], dict[str, FloatArray], FloatArray, dict[str, torch.Tensor], dict[str, torch.Tensor], torch.Tensor]:
    """Load both DH and inertial parameters for Kinova Gen3 robot."""
    dh = load_dh(DH_FILE)
    inertia = load_inertia(INERT_FILE)
    inertia, T_base = change_of_basis(inertia, dh)
    # take out the first dimension (base link) of both dh and inertia
    dh = {k: v[1:] for k, v in dh.items()}
    inertia = {k: v[1:] for k, v in inertia.items()}
    dh_torch = ptu.from_numpy_dict(dh)
    inertia_torch = ptu.from_numpy_dict(inertia)
    T_base_torch = ptu.from_numpy(T_base)
    return dh, inertia, T_base, dh_torch, inertia_torch, T_base_torch