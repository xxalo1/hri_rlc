
import numpy as np
from numpy.typing import NDArray
from typing import Sequence
FloatArray = NDArray[np.float32]
dtype = np.float32

def eu_to_quaternion(euler: Sequence[float]) -> np.ndarray:
    """Convert Euler angles (roll, pitch, yaw) to a quaternion (x, y, z, w)."""
    roll, pitch, yaw = euler
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([x, y, z, w], dtype=np.float32)


def project_to_so3(R, tol=1e-6):
    U, _, Vt = np.linalg.svd(R)
    R_hat = U @ Vt
    # Ensure det = +1 (fix possible reflection)
    if np.linalg.det(R_hat) < 0:
        U[:, -1] *= -1
        R_hat = U @ Vt
    # Optional: reject if far from SO(3)
    err = np.linalg.norm(R_hat.T @ R_hat - np.eye(3), ord='fro')
    if err > 1e-2:   # tune for your pipeline
        raise ValueError(f"R is too far from SO(3): ||RᵀR-I||_F = {err:.2e}")
    return R_hat


def rotmat_to_quat(R):
    R = np.asarray(R, float)

    t = np.trace(R)
    if t > 0.0:
        S = 2.0 * np.sqrt(max(t + 1.0, 0.0))
        w = 0.25 * S
        x = (R[2,1] - R[1,2]) / S
        y = (R[0,2] - R[2,0]) / S
        z = (R[1,0] - R[0,1]) / S
    else:
        # Pick the dominant diagonal for numerical stability near 180°
        if (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = 2.0 * np.sqrt(max(1.0 + R[0,0] - R[1,1] - R[2,2], 0.0))
            w = (R[2,1] - R[1,2]) / S
            x = 0.25 * S
            y = (R[0,1] + R[1,0]) / S
            z = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = 2.0 * np.sqrt(max(1.0 - R[0,0] + R[1,1] - R[2,2], 0.0))
            w = (R[0,2] - R[2,0]) / S
            x = (R[0,1] + R[1,0]) / S
            y = 0.25 * S
            z = (R[1,2] + R[2,1]) / S
        else:
            S = 2.0 * np.sqrt(max(1.0 - R[0,0] - R[1,1] + R[2,2], 0.0))
            w = (R[1,0] - R[0,1]) / S
            x = (R[0,2] + R[2,0]) / S
            y = (R[1,2] + R[2,1]) / S
            z = 0.25 * S

    q = np.array([w, x, y, z], float)
    q /= np.linalg.norm(q) + 1e-12
    return q


def enforce_hemisphere(q, q_prev):
    return q if np.dot(q, q_prev) >= 0.0 else -q
