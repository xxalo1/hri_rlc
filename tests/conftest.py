from __future__ import annotations

import numpy as np
import torch
import pytest

from ..kinova_gen3 import kinova_gen3 as kg3
from ..utils import numpy_util as npu
from ..utils import pytorch_util as ptu
from ..robot import Kinematics, Dynamics

FloatArray = npu.FloatArray
ptu.init_gpu()


@pytest.fixture(scope="session")
def kinova_inertia_numpy() -> dict[str, FloatArray]:
    """
    Inertia parameters for Kinova Gen3 in NumPy form,
    as loaded directly from kg3.
    """
    inertia = kg3.load_inertia()
    assert isinstance(inertia, dict)

    for key in ("Ic", "m", "com"):
        assert key in inertia, f"DH dict missing key {key!r}"
        assert isinstance(inertia[key], np.ndarray), f"DH param {key!r} is not a numpy array"
    assert inertia["com"].shape == (8, 3), f"Expected com shape (8,3), got {inertia['com'].shape}"
    assert inertia["m"].shape == (8,), f"Expected mass shape (8,), got {inertia['m'].shape}"
    assert inertia["Ic"].shape == (8, 3, 3), f"Expected Ic shape (8,3,3), got {inertia['Ic'].shape}"
    return inertia


@pytest.fixture(scope="session")
def kinova_dh_numpy() -> dict[str, FloatArray]:
    """
    DH parameters for Kinova Gen3 in NumPy form,
    as loaded directly from kg3.
    """
    dh = kg3.load_dh()
    assert isinstance(dh, dict), "DH parameters is not a dict"

    for key in ("d", "a", "alpha", "q0"):
        assert key in dh, f"DH dict missing key {key!r}"
        assert isinstance(dh[key], np.ndarray), f"DH param {key!r} is not a numpy array"

    return dh


# ---------------------------------------------------------------------
# Converted parameter fixtures (Torch)
# ---------------------------------------------------------------------

@pytest.fixture(scope="session")
def kinova_inertia_torch(
    kinova_inertia_numpy: dict[str, FloatArray],
) -> dict[str, torch.Tensor]:
    """
    Inertia parameters converted to Torch tensors (device/dtype from ptu).
    """
    inertia_torch = ptu.from_numpy_dict(kinova_inertia_numpy)
    assert isinstance(inertia_torch, dict), "Inertia parameters is not a dict"
    assert all(isinstance(v, torch.Tensor) for v in inertia_torch.values()),\
        "Not all inertia parameters are torch Tensors"
    assert inertia_torch["com"].shape == (8, 3), f"Expected com shape (8,3), got {inertia_torch['com'].shape}"
    assert inertia_torch["m"].shape == (8,), f"Expected mass shape (8,), got {inertia_torch['m'].shape}"
    assert inertia_torch["Ic"].shape == (8, 3, 3), f"Expected Ic shape (8,3,3), got {inertia_torch['Ic'].shape}"
    return inertia_torch


@pytest.fixture(scope="session")
def kinova_dh_torch(
    kinova_dh_numpy: dict[str, FloatArray],
) -> dict[str, torch.Tensor]:
    """
    DH parameters converted to Torch tensors (device/dtype from ptu).
    """
    dh_torch = ptu.from_numpy_dict(kinova_dh_numpy)
    assert isinstance(dh_torch, dict), "DH parameters is not a dict"
    assert all(isinstance(v, torch.Tensor) for v in dh_torch.values()),\
        "Not all DH parameters are torch Tensors"
    return dh_torch

# ---------------------------------------------------------------------
# Kinematics fixtures (NumPy and Torch)
# ---------------------------------------------------------------------

@pytest.fixture(scope="session")
def kin_numpy(
    kinova_dh_numpy: dict[str, FloatArray],
    kinova_inertia_numpy: dict[str, FloatArray],
) -> Kinematics[FloatArray]:
    """
    Kinematics instance using NumPy arrays.
    Base transform is identity in world frame.
    """
    T_0 = np.eye(4, dtype=npu.dtype)

    # origin and axes of base in world coordinates
    o_wb = T_0[0:3, 3]       # (3,)
    axes_wb = T_0[0:3, 0:3]  # (3, 3)

    kin = Kinematics(
        dh=kinova_dh_numpy,
        o_wb=o_wb,
        axes_wb=axes_wb,
        inertia=kinova_inertia_numpy,
    )
    return kin


@pytest.fixture(scope="session")
def kin_torch(
    kinova_dh_torch: dict[str, torch.Tensor],
    kinova_inertia_torch: dict[str, torch.Tensor],
) -> Kinematics[torch.Tensor]:
    """
    Kinematics instance using Torch tensors.
    Base transform is identity in world frame.
    """
    T_0 = torch.eye(4, dtype=ptu.dtype, device=ptu.device)

    o_wb = T_0[0:3, 3]       # (3,)
    axes_wb = T_0[0:3, 0:3]  # (3, 3)

    kin = Kinematics(
        dh=kinova_dh_torch,
        o_wb=o_wb,
        axes_wb=axes_wb,
        inertia=kinova_inertia_torch,
    )
    return kin


# ---------------------------------------------------------------------
# Dynamics fixture (Torch)
# ---------------------------------------------------------------------

@pytest.fixture(scope="session")
def dyn_torch(kin_torch: Kinematics[torch.Tensor]) -> Dynamics:
    """
    Dynamics instance for Kinova Gen3 using Torch kinematics.
    """
    g_vec = torch.tensor([0.0, 0.0, -9.81], dtype=ptu.dtype, device=ptu.device)
    dyn = Dynamics(kin=kin_torch, g_vec=g_vec)
    return dyn
