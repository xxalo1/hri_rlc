from __future__ import annotations

import numpy as np
from collections.abc import Sequence

from builtin_interfaces.msg import Time as TimeMsg
from builtin_interfaces.msg import Duration as DurationMsg
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
    MultiDOFJointTrajectory,
    MultiDOFJointTrajectoryPoint,
)
from geometry_msgs.msg import Transform, Twist

from common_utils import FloatArray


# ---------------------------------------------------------------------------
# Time / Duration helpers
# ---------------------------------------------------------------------------

def to_ros_time(t: float) -> TimeMsg:
    """Convert time in seconds (float) to a ROS Time message."""
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return TimeMsg(sec=sec, nanosec=nanosec)


def to_ros_duration(dt: float) -> DurationMsg:
    """Convert a duration in seconds (float) to a ROS Duration message."""
    sec = int(dt)
    nanosec = int((dt - sec) * 1e9)
    return DurationMsg(sec=sec, nanosec=nanosec)


def from_ros_time(t: TimeMsg | DurationMsg) -> float:
    """Convert a ROS Time/Duration message to seconds (float)."""
    return float(t.sec) + float(t.nanosec) * 1e-9


# ---------------------------------------------------------------------------
# Trajectory builders
# ---------------------------------------------------------------------------

def joint_traj_from_arrays(
    t: float,
    joint_names: Sequence[str],
    positions: FloatArray,
    velocities: FloatArray | None = None,
    accelerations: FloatArray | None = None,
    freq: float = 100.0,
) -> JointTrajectory:
    """
    Build a JointTrajectory from arrays.

    Parameters
    ----------
    t : float
        Start time of the trajectory in seconds.
    joint_names : sequence of str
        Names of the joints, length n.
    positions : array, shape (N, n)
        Joint positions for each time step.
    velocities : array, shape (N, n), optional
        Joint velocities. If None, zeros are used.
    accelerations : array, shape (N, n), optional
        Joint accelerations. If None, zeros are used.
    freq : float
        Sampling frequency in Hz.

    Returns
    -------
    JointTrajectory
        Populated JointTrajectory message.
    """
    if velocities is None:
        velocities = np.zeros_like(positions)
    if accelerations is None:
        accelerations = np.zeros_like(positions)

    positions = np.atleast_2d(positions)
    velocities = np.atleast_2d(velocities)
    accelerations = np.atleast_2d(accelerations)

    traj_msg = JointTrajectory()
    traj_msg.joint_names = list(joint_names)
    traj_msg.header.stamp = to_ros_time(t)

    dt = 1.0 / freq
    points: list[JointTrajectoryPoint] = []

    for idx in range(positions.shape[0]):
        pt = JointTrajectoryPoint()
        pt.positions = positions[idx].tolist()
        pt.velocities = velocities[idx].tolist()
        pt.accelerations = accelerations[idx].tolist()
        pt.time_from_start = to_ros_duration(dt * idx)
        points.append(pt)

    traj_msg.points = points
    return traj_msg


def poses_from_arrays(
    t: float,
    poses: FloatArray,
    twists: FloatArray | None = None,
    accels: FloatArray | None = None,
    freq: float = 100.0,
    frame_id: str = "world",
    ee_name: str = "ee",
) -> MultiDOFJointTrajectory:
    """
    Build a MultiDOFJointTrajectory for the end-effector from arrays.

    Parameters
    ----------
    t : float
        Start time of the trajectory in seconds.
    poses : array, shape (N, 7)
        EE poses as [x, y, z, qx, qy, qz, qw] for each time step.
    twists : array, shape (N, 6), optional
        EE twists as [vx, vy, vz, wx, wy, wz] for each time step.
        If None, the velocities field in the message is left empty.
    accels : array, shape (N, 6), optional
        EE accelerations as [ax, ay, az, alphax, alphay, alphaz].
        If None, the accelerations field in the message is left empty.
    freq : float
        Sampling frequency of the trajectory in Hz.
    frame_id : str
        Fixed frame of the EE trajectory (default: "world").
    ee_name : str
        Name used in joint_names for the EE (default: "ee").

    Returns
    -------
    MultiDOFJointTrajectory
        Populated MultiDOFJointTrajectory message.
    """
    poses = np.atleast_2d(poses)
    n_steps = poses.shape[0]

    if twists is not None: twists = np.atleast_2d(twists)
    if accels is not None: accels = np.atleast_2d(accels)

    traj_msg = MultiDOFJointTrajectory()
    traj_msg.header.stamp = to_ros_time(t)
    traj_msg.header.frame_id = frame_id
    traj_msg.joint_names = [ee_name]

    dt = 1.0 / freq
    points: list[MultiDOFJointTrajectoryPoint] = []

    for idx in range(n_steps):
        pt = MultiDOFJointTrajectoryPoint()

        # Pose -> Transform
        x, y, z, qx, qy, qz, qw = poses[idx]
        transform = Transform()
        transform.translation.x = float(x)
        transform.translation.y = float(y)
        transform.translation.z = float(z)
        transform.rotation.x = float(qx)
        transform.rotation.y = float(qy)
        transform.rotation.z = float(qz)
        transform.rotation.w = float(qw)
        pt.transforms.append(transform)

        # Optional twist
        if twists is not None:
            vx, vy, vz, wx, wy, wz = twists[idx]
            twist = Twist()
            twist.linear.x = float(vx)
            twist.linear.y = float(vy)
            twist.linear.z = float(vz)
            twist.angular.x = float(wx)
            twist.angular.y = float(wy)
            twist.angular.z = float(wz)
            pt.velocities.append(twist)

        # Optional acceleration
        if accels is not None:
            ax, ay, az, awx, awy, awz = accels[idx]
            acc = Twist()
            acc.linear.x = float(ax)
            acc.linear.y = float(ay)
            acc.linear.z = float(az)
            acc.angular.x = float(awx)
            acc.angular.y = float(awy)
            acc.angular.z = float(awz)
            pt.accelerations.append(acc)

        pt.time_from_start = to_ros_duration(dt * idx)
        points.append(pt)

    traj_msg.points = points
    return traj_msg
