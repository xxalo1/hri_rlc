from __future__ import annotations

import numpy as np
from common_utils import FloatArray
from common_utils import numpy_util as npu

def rotation_matrix_to_quat(R: FloatArray) -> FloatArray:
    """
    Convert rotation matrix/matrices to unit quaternion(s) [w, x, y, z].

    Parameters
    ----------
    R : ndarray, shape (3, 3) or (n, 3, 3)
        Rotation matrix/matrices.

    Returns
    ----------
    q : ndarray, shape (4,) or (n, 4)
        Unit quaternion(s) ordered as [w, x, y, z].
    """

    squeeze_output = False
    if R.ndim == 2:
        R = R[None, :, :]  # (1,3,3)
        squeeze_output = True

    n = R.shape[0]
    trace = np.trace(R, axis1=1, axis2=2)  # (n,)

    # Pre-allocate outputs
    w = np.empty(n, dtype=npu.dtype)
    x = np.empty(n, dtype=npu.dtype)
    y = np.empty(n, dtype=npu.dtype)
    z = np.empty(n, dtype=npu.dtype)

    # Masks for different cases
    mask_pos = trace > 0.0
    mask_else = ~mask_pos

    if np.any(mask_pos):
        tr = trace[mask_pos]
        S = np.sqrt(tr + 1.0) * 2.0  # S = 4w
        w[mask_pos] = 0.25 * S
        x[mask_pos] = (R[mask_pos, 2, 1] - R[mask_pos, 1, 2]) / S
        y[mask_pos] = (R[mask_pos, 0, 2] - R[mask_pos, 2, 0]) / S
        z[mask_pos] = (R[mask_pos, 1, 0] - R[mask_pos, 0, 1]) / S

    if np.any(mask_else):
        Rm = R[mask_else]
        # Choose branch per row dominance
        cond0 = (Rm[:, 0, 0] > Rm[:, 1, 1]) & (Rm[:, 0, 0] > Rm[:, 2, 2])
        cond1 = (~cond0) & (Rm[:, 1, 1] > Rm[:, 2, 2])
        cond2 = ~(cond0 | cond1)

        if np.any(cond0):
            idx = np.where(mask_else)[0][cond0]
            S = np.sqrt(1.0 + R[idx, 0, 0] - R[idx, 1, 1] - R[idx, 2, 2]) * 2.0
            w[idx] = (R[idx, 2, 1] - R[idx, 1, 2]) / S
            x[idx] = 0.25 * S
            y[idx] = (R[idx, 0, 1] + R[idx, 1, 0]) / S
            z[idx] = (R[idx, 0, 2] + R[idx, 2, 0]) / S

        if np.any(cond1):
            idx = np.where(mask_else)[0][cond1]
            S = np.sqrt(1.0 + R[idx, 1, 1] - R[idx, 0, 0] - R[idx, 2, 2]) * 2.0
            w[idx] = (R[idx, 0, 2] - R[idx, 2, 0]) / S
            x[idx] = (R[idx, 0, 1] + R[idx, 1, 0]) / S
            y[idx] = 0.25 * S
            z[idx] = (R[idx, 1, 2] + R[idx, 2, 1]) / S

        if np.any(cond2):
            idx = np.where(mask_else)[0][cond2]
            S = np.sqrt(1.0 + R[idx, 2, 2] - R[idx, 0, 0] - R[idx, 1, 1]) * 2.0
            w[idx] = (R[idx, 1, 0] - R[idx, 0, 1]) / S
            x[idx] = (R[idx, 0, 2] + R[idx, 2, 0]) / S
            y[idx] = (R[idx, 1, 2] + R[idx, 2, 1]) / S
            z[idx] = 0.25 * S

    q = np.stack([w, x, y, z], axis=-1)  # (n,4)

    # Normalize each quaternion to be safe
    norms = np.linalg.norm(q, axis=-1, keepdims=True)
    if np.any(norms == 0.0):
        raise ValueError("Zero norm quaternion produced from rotation matrix.")
    q = q / norms

    if squeeze_output:
        return q[0]
    return q


def quat_to_rotation_matrix(q: FloatArray) -> FloatArray:
    """
    Convert a quaternion(s) [w, x, y, z] to rotation matrix/matrices.

    Parameters
    ----------
    q : ndarray, shape (4,) or (n, 4)
        Quaternion(s) ordered as [w, x, y, z].

    Returns
    ----------
    R : ndarray, shape (3, 3) or (n, 3, 3)
        Rotation matrix/matrices corresponding to input quaternions.
    """

    # Handle single quaternion vs batch
    squeeze_output = False
    if q.ndim == 1:
        q = q[None, :]  # shape -> (1, 4)
        squeeze_output = True

    # Normalize each quaternion in the batch
    norms = np.linalg.norm(q, axis=-1, keepdims=True)
    if np.any(norms == 0.0):
        raise ValueError("Zero norm quaternion is not valid.")
    q = q / norms

    w = q[:, 0]
    x = q[:, 1]
    y = q[:, 2]
    z = q[:, 3]

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    # Build rotation matrices for each quaternion
    R = np.empty((q.shape[0], 3, 3), dtype=npu.dtype)
    R[:, 0, 0] = 1.0 - 2.0 * (yy + zz)
    R[:, 0, 1] = 2.0 * (xy - wz)
    R[:, 0, 2] = 2.0 * (xz + wy)

    R[:, 1, 0] = 2.0 * (xy + wz)
    R[:, 1, 1] = 1.0 - 2.0 * (xx + zz)
    R[:, 1, 2] = 2.0 * (yz - wx)

    R[:, 2, 0] = 2.0 * (xz - wy)
    R[:, 2, 1] = 2.0 * (yz + wx)
    R[:, 2, 2] = 1.0 - 2.0 * (xx + yy)

    if squeeze_output:
        return R[0]
    return R


def transform_to_pos_quat(T: FloatArray) -> tuple[FloatArray, FloatArray]:
    """
    Convert a 4x4 homogeneous transform to position and quaternion.

    Parameters
    ----------
    T : ndarray, shape (n, 4, 4) or (4, 4)
        homogeneous transform.

    Returns
    ----------
    pos : ndarray, shape (n, 3) or (3,)
        position vector.
    quat : ndarray, shape (n, 4) or (4,)
        quaternion [w, x, y, z].
    """
    R = T[..., :3, :3]
    pos = T[..., :3, 3]
    quat = rotation_matrix_to_quat(R)
    return pos, quat


def pos_quat_to_transform(p: FloatArray, q: FloatArray) -> FloatArray:
    """

    Build a 4x4 homogeneous transform from position and quaternion.

    Parameters
    ----------
    p : ndarray, shape (3,)
        position vector.
    q : ndarray, shape (4,) 
        quaternion [w, x, y, z].

    Returns
    ----------
    T : ndarray, shape (4, 4)
        homogeneous transform.
    """
    R = quat_to_rotation_matrix(q)

    T = np.eye(4, dtype=npu.dtype)
    T[:3, :3] = R
    T[:3, 3] = p
    return T
