from __future__ import annotations

import numpy as np
from common_utils import FloatArray
from common_utils import numpy_util as npu

def rotation_matrix_to_quat(R: FloatArray) -> FloatArray:
    """
    Convert a 3x3 rotation matrix to a unit quaternion [w, x, y, z].

    Args:
        R: (3, 3) rotation matrix.

    Returns:
        q: (4,) unit quaternion [w, x, y, z].
    """

    trace = np.trace(R)

    if trace > 0.0:
        S = np.sqrt(trace + 1.0) * 2.0  # S = 4 * w
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0  # S = 4 * x
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0  # S = 4 * y
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0  # S = 4 * z
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S

    q = np.array([w, x, y, z], dtype=npu.dtype)

    # Normalize to be safe
    norm = np.linalg.norm(q)
    if norm == 0.0:
        raise ValueError("Zero norm quaternion produced from rotation matrix.")
    q /= norm
    return q


def quat_to_rotation_matrix(q: FloatArray) -> FloatArray:
    """
    Convert a quaternion [w, x, y, z] to a 3x3 rotation matrix.

    Parameters
    ----------
    q: ndarray, shape (4,)
        quaternion [w, x, y, z].

    Returns
    ----------
    R: ndarray, shape (3, 3)
        rotation matrix.
    """

    norm = np.linalg.norm(q)
    if norm == 0.0:
        raise ValueError("Zero norm quaternion is not valid.")
    q = q / norm

    w, x, y, z = q

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    R = np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )

    return R


def transform_to_pos_quat(T: FloatArray) -> tuple[FloatArray, FloatArray]:
    """
    Convert a 4x4 homogeneous transform to position and quaternion.

    Parameters
    ----------
    T: ndarray, shape (4, 4)
        homogeneous transform.

    Returns
    ----------
    pos: ndarray, shape(3,) 
        position vector.
    quat: ndarray, shape (4,) 
        quaternion [w, x, y, z].
    """
    R = T[:3, :3]
    pos = T[:3, 3]
    quat = rotation_matrix_to_quat(R)
    return pos, quat


def pos_quat_to_transform(p: FloatArray, q: FloatArray) -> FloatArray:
    """

    Build a 4x4 homogeneous transform from position and quaternion.

    Parameters
    ----------
    p: ndarray, shape (3,)
        position vector.
    q: ndarray, shape (4,) 
        quaternion [w, x, y, z].

    Returns
    ----------
    T: ndarray, shape (4, 4)
        homogeneous transform.
    """
    R = quat_to_rotation_matrix(q)

    T = np.eye(4, dtype=npu.dtype)
    T[:3, :3] = R
    T[:3, 3] = p
    return T
