from __future__ import annotations

import numpy as np
import torch
import pytest

from ..kinova_gen3 import kinova_gen3 as kg3
from ..utils import numpy_util as npu
from ..utils import pytorch_util as ptu
from ..robot import Kinematics, Dynamics

FloatArray = npu.FloatArray
# ---------------------------------------------------------------------
# Basic construction and parameter consistency
# ---------------------------------------------------------------------

@pytest.mark.parametrize("backend", ["numpy", "torch"])
def test_kinematics_init_basic(
    backend: str,
    kin_numpy: Kinematics[FloatArray],
    kin_torch: Kinematics[torch.Tensor],
) -> None:
    """Basic sanity checks on Kinematics initialisation for both backends."""
    kin = kin_numpy if backend == "numpy" else kin_torch

    n = kin.n
    assert n > 0

    # DH parameters must all be length n
    assert kin.d.shape == (n,)
    assert kin.a.shape == (n,)
    assert kin.alpha.shape == (n,)
    assert kin.q0.shape == (n,)

    # state vectors
    assert kin.q.shape == (n,)
    assert kin.qd.shape == (n,)
    assert kin.qdd.shape == (n,)

    # base transform T_wb should be 4x4 homogeneous
    assert kin.T_wb.shape == (4, 4)


# ---------------------------------------------------------------------
# Forward kinematics: shapes and basic invariants
# ---------------------------------------------------------------------

@pytest.mark.parametrize("backend", ["numpy", "torch"])
def test_forward_kinematics_shape(
    backend: str,
    kin_numpy: Kinematics[FloatArray],
    kin_torch: Kinematics[torch.Tensor],
) -> None:
    """Forward kinematics returns (n+1, 4, 4) transforms including base frame."""
    kin = kin_numpy if backend == "numpy" else kin_torch

    n = kin.n
    if backend == "numpy":
        q = np.zeros(n, dtype=npu.dtype)
    else:
        q = torch.zeros(n, dtype=ptu.dtype, device=ptu.device)

    T_wf = kin.forward_kinematics(q=q)

    assert T_wf.shape == (n + 1, 4, 4),\
        f"FK output shape incorrect for backend {backend}"

    # first transform should be the base transform
    if backend == "numpy":
        assert np.allclose(T_wf[0], kin.T_wb), \
            f"Base transform incorrect in FK output for backend {backend}"
    else:
        assert torch.allclose(T_wf[0], kin.T_wb), \
            f"Base transform incorrect in FK output for backend {backend}"

    # last row of each transform should be [0, 0, 0, 1]
    if backend == "numpy":
        expected = np.array([0.0, 0.0, 0.0, 1.0], dtype=npu.dtype)
        assert np.allclose(T_wf[:, 3, :], expected), \
            f"Last row of FK transforms incorrect for {backend}"
    else:
        expected = torch.tensor([0.0, 0.0, 0.0, 1.0], dtype=ptu.dtype, device=ptu.device)
        assert torch.allclose(T_wf[:, 3, :], expected), \
            f"Last row of FK transforms incorrect for {backend}"


@pytest.mark.parametrize(
        "backend", ["numpy", "torch"]
        )
def test_forward_kinematics_consistent_with_internal_state(
    backend: str,
    kin_numpy: Kinematics[FloatArray],
    kin_torch: Kinematics[torch.Tensor],
) -> None:
    """
    Calling forward_kinematics with explicit q should match using internal state
    after step(q).
    """
    kin = kin_numpy if backend == "numpy" else kin_torch

    n = kin.n
    if backend == "numpy":
        q = np.linspace(-0.5, 0.5, n)
    else:
        q = torch.linspace(-0.5, 0.5, n, dtype=ptu.dtype, device=ptu.device)

    # explicit q
    T_explicit = kin.forward_kinematics(q=q)

    # via internal state
    kin.step(q=q)
    T_internal = kin.forward_kinematics()

    if backend == "numpy":
        assert np.allclose(T_explicit, T_internal), \
            f"FK with explicit q does not match internal state after step(q) for {backend}"
    else:
        assert torch.allclose(T_explicit, T_internal), \
            f"FK with explicit q does not match internal state after step(q) for {backend}"


@pytest.mark.parametrize("backend", ["numpy", "torch"])
def test_jacobian_shapes_and_last_link_consistency(
    backend: str,
    kin_numpy: Kinematics[FloatArray],
    kin_torch: Kinematics[torch.Tensor],
) -> None:
    """jac(q) shape and consistency with full_jac(q) for both backends."""
    kin = kin_numpy if backend == "numpy" else kin_torch
    
    n = kin.n
    if backend == "numpy":
        q = np.random.randn(n).astype(npu.dtype)
    else:
        q = torch.randn(n, dtype=ptu.dtype, device=ptu.device)

    J_ee = kin.jac(q)        # (6, n)
    J_all = kin.full_jac(q)  # (n, 6, n)

    assert J_ee.shape == (6, n), f"jac(q) shape incorrect for {backend}"
    assert J_all.shape == (n, 6, n), f"full_jac(q) shape incorrect for {backend}"

    # By design, jac(q) should be the Jacobian of the last link
    if backend == "numpy":
        assert np.allclose(J_ee, J_all[-1], atol=1e-6), \
            f"jac(q) does not match full_jac(q)[-1] for {backend}"
    else:
        assert torch.allclose(J_ee, J_all[-1], atol=1e-6), \
            f"jac(q) does not match full_jac(q)[-1] for {backend}"


@pytest.mark.parametrize("backend", ["numpy", "torch"])
def test_com_and_inertia_shapes(
    backend: str,
    kin_numpy: Kinematics[FloatArray],
    kin_torch: Kinematics[torch.Tensor],
) -> None:
    """Shapes and basic symmetry checks for COM and inertia tensors."""
    kin = kin_numpy if backend == "numpy" else kin_torch
    
    n = kin.n
    if backend == "numpy":
        q = np.random.randn(n).astype(npu.dtype)
    else:
        q = torch.randn(n, dtype=ptu.dtype, device=ptu.device)

    kin.step(q=q)

    com_wl = kin.com_wl   # (n, 3)
    Ic_wl = kin.Ic_wl     # (n, 3, 3)

    assert com_wl.shape == (n, 3), f"COM world shape incorrect for {backend}"
    assert Ic_wl.shape == (n, 3, 3), f"Inertia world shape incorrect for {backend}"

    # inertia tensors should be symmetric
    if backend == "numpy":
        assert np.allclose(Ic_wl, Ic_wl.transpose(0, 2, 1), atol=1e-6), \
            f"Inertia tensors in world frame are not symmetric for {backend}"
    else:
        assert torch.allclose(Ic_wl, Ic_wl.transpose(-1, -2), atol=1e-6), \
            f"Inertia tensors in world frame are not symmetric for {backend}"

# ---------------------------------------------------------------------
# Cross backend sanity: NumPy vs Torch FK at zero
# ---------------------------------------------------------------------

def test_forward_kinematics_numpy_vs_torch_zero(
    kin_numpy: Kinematics[FloatArray],
    kin_torch: Kinematics[torch.Tensor],
) -> None:
    """
    For q = 0, NumPy and Torch backends should produce the same forward kinematics
    (up to numeric precision).
    """
    n = kin_numpy.n
    assert kin_torch.n == n

    q_np = np.zeros(n, dtype=float)
    q_t = torch.zeros(n, dtype=ptu.dtype, device=ptu.device)

    T_np = kin_numpy.forward_kinematics(q=q_np)                # (n+1, 4, 4)
    T_t = kin_torch.forward_kinematics(q=q_t)    # (n+1, 4, 4)
    T_t = ptu.to_numpy(T_t)

    assert T_np.shape == T_t.shape, \
        "Forward kinematics output shapes do not match between NumPy and Torch backends"
    assert np.allclose(T_np, T_t, atol=1e-6), \
        "Forward kinematics outputs do not match between NumPy and Torch backends"
    
# ---------------------------------------------------------------------
# Forward kinematics with partial index i (NumPy + Torch)
# ---------------------------------------------------------------------

@pytest.mark.parametrize("backend", ["numpy", "torch"])
def test_forward_kinematics_partial_prefix(
    backend: str,
    kin_numpy: Kinematics[np.ndarray],
    kin_torch: Kinematics[torch.Tensor],
) -> None:
    """forward_kinematics(q, i=k) should return frames [0..k+1] consistent with full FK."""
    kin = kin_numpy if backend == "numpy" else kin_torch
    n = kin.n

    if backend == "numpy":
        q = np.random.randn(n).astype(npu.dtype)
    else:
        q = torch.randn(n, dtype=ptu.dtype, device=ptu.device)

    T_full = kin.forward_kinematics(q=q)   # (n+1, 4, 4)

    k = n // 2
    T_part = kin.forward_kinematics(q=q, i=k)  # (k+2, 4, 4)

    assert T_part.shape[0] == k + 2, \
        f"Partial FK output shape incorrect for backend {backend}"

    if backend == "numpy":
        assert np.allclose(T_part[0], kin.T_wb), \
            f"Base transform incorrect in partial FK output for backend {backend}"
        assert np.allclose(T_part[-1], T_full[k + 1]), \
            f"End transform incorrect in partial FK output for backend {backend}"
    else:
        assert torch.allclose(T_part[0], kin.T_wb), \
            f"Base transform incorrect in partial FK output for backend {backend}"
        assert torch.allclose(T_part[-1], T_full[k + 1]), \
            f"End transform incorrect in partial FK output for backend {backend}"

# ---------------------------------------------------------------------
# Cache invalidation behaviour (NumPy + Torch)
# ---------------------------------------------------------------------

@pytest.mark.parametrize("backend", ["numpy", "torch"])
def test_kinematics_cache_invalidation_on_q_change(
    backend: str,
    kin_numpy: Kinematics[np.ndarray],
    kin_torch: Kinematics[torch.Tensor],
) -> None:
    """_fk_valid should be reset to False whenever q is updated."""
    kin = kin_numpy if backend == "numpy" else kin_torch
    n = kin.n

    if backend == "numpy":
        q0 = np.zeros(n, dtype=npu.dtype)
        q1 = np.ones(n, dtype=npu.dtype) * 0.1
    else:
        q0 = torch.zeros(n, dtype=ptu.dtype, device=ptu.device)
        q1 = torch.ones(n, dtype=ptu.dtype, device=ptu.device) * 0.1

    kin._invalidate_kinematics()
    kin.forward_kinematics(q=q0)
    assert kin._fk_valid is False, f"FK cache valid after not internal forward_kinematics call for {backend}"

    kin.q = q1
    assert kin._fk_valid is False, f"FK cache not invalidated after q update for {backend}"

    kin.forward_kinematics()
    assert kin._fk_valid is True, f"FK cache not valid after internal forward_kinematics call for {backend}"


# ---------------------------------------------------------------------
# Index range validation (NumPy + Torch)
# ---------------------------------------------------------------------

@pytest.mark.parametrize("backend", ["numpy", "torch"])
def test_validate_index_range(
    backend: str,
    kin_numpy: Kinematics[np.ndarray],
    kin_torch: Kinematics[torch.Tensor],
) -> None:
    """_validate_index_range should accept [0, n-1] and reject out-of-range values."""
    kin = kin_numpy if backend == "numpy" else kin_torch
    n = kin.n

    kin._validate_index_range(0)
    kin._validate_index_range(n - 1)

    with pytest.raises(ValueError):
        kin._validate_index_range(-1)

    with pytest.raises(ValueError):
        kin._validate_index_range(n)

# ---------------------------------------------------------------------
# jac_com vs full_jac when COM is at link origins (NumPy + Torch)
# ---------------------------------------------------------------------

@pytest.mark.parametrize("backend", ["numpy", "torch"])
def test_jac_com_equals_jac_when_com_zero(
    backend: str,
    kin_numpy: Kinematics[np.ndarray],
    kin_torch: Kinematics[torch.Tensor],
) -> None:
    """
    If all COMs are at the link origins (com_fl = 0),
    then the linear part of jac_com should equal the linear part of full_jac.
    """
    kin = kin_numpy if backend == "numpy" else kin_torch
    n = kin.n

    if backend == "numpy":
        q = np.random.randn(n).astype(npu.dtype)
        kin.com_fl = np.zeros((n, 3), dtype=npu.dtype)
    else:
        q = torch.randn(n, dtype=ptu.dtype, device=ptu.device)
        kin.com_fl = torch.zeros(n, 3, dtype=ptu.dtype, device=ptu.device)

    kin._com_valid = False
    kin._J_valid = False

    kin.step(q=q)

    J_full = kin.full_jac()  # (n, 6, n)
    J_com  = kin.jac_com()   # (n, 6, n)

    Jv_orig = J_full[:, 0:3, :]
    Jv_com  = J_com[:, 0:3, :]

    if backend == "numpy":
        assert np.allclose(Jv_orig, Jv_com, atol=1e-6)
    else:
        assert torch.allclose(Jv_orig, Jv_com, atol=1e-6)


# ---------------------------------------------------------------------
# spatial_vel consistency with J @ qd (NumPy + Torch)
# ---------------------------------------------------------------------

@pytest.mark.parametrize("backend", ["numpy", "torch"])
def test_spatial_vel_matches_jacobian(
    backend: str,
    kin_numpy: Kinematics[np.ndarray],
    kin_torch: Kinematics[torch.Tensor],
) -> None:
    """
    spatial_vel(q, qd) should match full_jac(q) @ qd.
    """
    kin = kin_numpy if backend == "numpy" else kin_torch
    n = kin.n

    if backend == "numpy":
        q = np.random.randn(n).astype(npu.dtype)
        qd = np.random.randn(n).astype(npu.dtype)
    else:
        q = torch.randn(n, dtype=ptu.dtype, device=ptu.device)
        qd = torch.randn(n, dtype=ptu.dtype, device=ptu.device)

    kin.step(q=q, qd=qd)

    v = kin.spatial_vel(q=q, qd=qd)  # (n, 6)
    J = kin.full_jac(q=q)            # (n, 6, n)

    if backend == "numpy":
        v_ref = np.einsum("ijk,k->ij", J, qd)
        assert v.shape == (n, 6)
        assert np.allclose(v, v_ref, atol=1e-6)
    else:
        v_ref = torch.einsum("ijk,k->ij", J, qd)
        assert v.shape == (n, 6)
        assert torch.allclose(v, v_ref, atol=1e-6)
