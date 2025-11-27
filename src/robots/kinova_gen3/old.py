import numpy as np
import torch
import yaml
from numpy import pi
from pathlib import Path
from ...common_utils import numpy_util as npu
from ...common_utils import FloatArray
from ...common_utils import pytorch_util as ptu
from ...rbt_core.kin import ops as ops
dtype = npu.dtype

HERE = Path(__file__).parent
INERT_FILE = HERE / "inertial_mj.yaml"
DH_FILE = HERE / "dh.yaml"
THETA_X = [0, pi, pi, pi, pi, pi, pi, pi]
THETA_Y = [0, 0, 0, 0, 0, 0, 0, 0]
THETA_Z = [pi, pi, pi, pi, pi, pi, pi, pi]
SKIP_LINKS = ("ee_no_vision",)

def compute_Rx(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0],
                     [0, c,-s],
                     [0, s, c]], dtype=npu.dtype)

def compute_Rz(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[ c,-s, 0],
                     [ s, c, 0],
                     [ 0, 0, 1]], dtype=npu.dtype)

def compute_Ry(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]], dtype=npu.dtype)

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


def change_of_basis(d: dict[str, FloatArray], dh: dict[str, FloatArray]) -> dict[str, FloatArray]:
    
    ## Apply change of basis to link's i -1 frame.
    angles_x = np.asarray(THETA_X, dtype=dtype)
    angles_y = np.asarray(THETA_Y, dtype=dtype)
    angles_z = np.asarray(THETA_Z, dtype=dtype)
    Rx  = np.stack([compute_Rx(theta) for theta in angles_x], axis=0).astype(d["com"].dtype)
    Ry = np.stack([compute_Ry(theta) for theta in angles_y], axis=0).astype(d["com"].dtype)
    Rz = np.stack([compute_Rz(theta) for theta in angles_z], axis=0).astype(d["com"].dtype)
    R = Rz @ Ry @ Rx

    com = d["com"]            # (n,3)
    Ic  = d["Ic"]             # (n,3,3)

    com_new = np.einsum("nij,ni->nj", R, com)      # R^T @ com
    Ic_new = np.einsum("nji,njk,nlk->nil", R, Ic, R)      # R Ic R^T

    ## apply change of basis express those to link's i frame.
    A = ops.transform_matrices(dh["q0"], dh["d"], dh["a"], dh["alpha"], dh["b"])   # (n,4,4)

    R = A[:, :3, :3]
    p = A[:, :3, 3]

    com_new = np.einsum("nij,ni->nj", R, com_new - p) # R^T (com - p)
    Ic_new = np.einsum("nia,nib,njb->naj", R, Ic_new, R) # R^T Ic R

    out = dict(d)
    out["com"] = com_new
    out["Ic"]  = Ic_new
    return out


def load_kinova_gen3() -> tuple[dict[str, FloatArray], dict[str, FloatArray], dict[str, torch.Tensor],   dict[str, torch.Tensor]]:
    """Load both DH and inertial parameters for Kinova Gen3 robot."""
    dh = load_dh()
    inertia = load_inertia(INERT_FILE)
    inertia = change_of_basis(inertia, dh)
    dh_torch = ptu.from_numpy_dict(dh)
    inertia_torch = ptu.from_numpy_dict(inertia)
    return dh, inertia, dh_torch, inertia_torch