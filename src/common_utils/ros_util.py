from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from collections.abc import Sequence, Mapping
from typing import NamedTuple

from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time as TimeMsg
from builtin_interfaces.msg import Duration as DurationMsg
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
    MultiDOFJointTrajectory,
    MultiDOFJointTrajectoryPoint,
)
from rlc_interfaces.msg import JointEffortCmd
from geometry_msgs.msg import Transform, Twist
from geometry_msgs.msg import PoseArray, Pose

from common_utils import FloatArray
from common_utils import numpy_util as npu  


class JointStateData(NamedTuple):
    positions: FloatArray
    velocities: FloatArray | None
    efforts: FloatArray | None
    stamp: float
    joint_names: list[str]


class JointEffortCmdData(NamedTuple):
    efforts: FloatArray
    stamp: float
    joint_names: list[str]

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
# ROS Message builders
# ---------------------------------------------------------------------------

def to_joint_traj_msg(
    positions: FloatArray,
    time_from_start: FloatArray,
    stamp: float,
    joint_names: Sequence[str],
    velocities: FloatArray | None = None,
    accelerations: FloatArray | None = None,
) -> JointTrajectory:
    """
    Build a JointTrajectory of joint positions, velocities, and accelerations.

    Parameters
    ----------
    positions : ndarray, shape (N, n) or (n,)
        Joint positions for each time step.
    time_from_start: ndarray, shape (N,) or (1,)
        Time from the start of the trajectory for each time step.
    stamp : float
        Start time of the trajectory in seconds.
    joint_names : sequence of str
        Names of the joints, length n.
    velocities : ndarray, shape (N, n) or (n,), optional
        Joint velocities. If None, they are omitted in the message.
    accelerations : ndarray, shape (N, n) or (n,), optional
        Joint accelerations. If None, they are omitted in the message.

    Returns
    -------
    JointTrajectory
        Populated JointTrajectory message.
    """

    positions = np.atleast_2d(positions)
    N = positions.shape[0]
    n = len(joint_names)

    npu.validate_array_shape(positions, (N, n), "positions")

    time_from_start = np.atleast_1d(time_from_start)
    npu.validate_array_shape(time_from_start, (N,), "time_from_start")

    if velocities is not None: 
        velocities = np.atleast_2d(velocities)
        npu.validate_array_shape(velocities, (N, n), "velocities")

    if accelerations is not None: 
        accelerations = np.atleast_2d(accelerations)
        npu.validate_array_shape(accelerations, (N, n), "accelerations")

    msg = JointTrajectory()
    msg.joint_names = list(joint_names)
    msg.header.stamp = to_ros_time(stamp)

    points: list[JointTrajectoryPoint] = []

    for idx in range(positions.shape[0]):
        pt = JointTrajectoryPoint()
        pt.positions = positions[idx].tolist()

        if velocities is not None:
            pt.velocities = velocities[idx].tolist()

        if accelerations is not None:
            pt.accelerations = accelerations[idx].tolist()

        pt.time_from_start = to_ros_duration(time_from_start[idx])
        points.append(pt)

    msg.points = points
    return msg


def to_multi_dof_traj_msg(
    poses: FloatArray,
    time_from_start: FloatArray,
    stamp: float,
    twists: FloatArray | None = None,
    accels: FloatArray | None = None,
    frame_id: str = "world",
    joint_name: str = "ee",
) -> MultiDOFJointTrajectory:
    """
    Build a MultiDOFJointTrajectory message from poses, twists, and accelerations.

    Parameters
    ----------
    poses : array-like, shape (N, 7) or (7,)
        Poses as [x, y, z, qx, qy, qz, qw] for each time step.
    time_from_start : array-like, shape (N,) or scalar
        Time from the start of the trajectory for each time step [s].
    stamp : float
        Start time of the trajectory in seconds (for the header stamp).
    frame_id : str, default "world"
        Fixed frame of the poses.
    joint_name : str, default "ee"
        Name used in joint_names.
    twists : array-like, shape (N, 6) or (6,), optional
        Twists as [vx, vy, vz, wx, wy, wz] for each time step.
        If None, the velocities field in the message is left empty.
    accels : array-like, shape (N, 6) or (6,), optional
        Accelerations as [ax, ay, az, alphax, alphay, alphaz].
        If None, the accelerations field in the message is left empty.

    Returns
    -------
    MultiDOFJointTrajectory
        Populated MultiDOFJointTrajectory message.
    """
    poses = np.atleast_2d(poses)
    N = poses.shape[0]
    
    npu.validate_array_shape(poses, (N, 7), "poses")

    time_from_start = np.atleast_1d(time_from_start)
    npu.validate_array_shape(time_from_start, (N,), "time_from_start")

    if twists is not None: 
        twists = np.atleast_2d(twists)
        npu.validate_array_shape(twists, (N, 6), "twists")

    if accels is not None: 
        accels = np.atleast_2d(accels)
        npu.validate_array_shape(accels, (N, 6), "accels")

    traj_msg = MultiDOFJointTrajectory()
    traj_msg.header.stamp = to_ros_time(stamp)
    traj_msg.header.frame_id = frame_id
    traj_msg.joint_names = [joint_name]

    points: list[MultiDOFJointTrajectoryPoint] = []

    for idx in range(N):
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
        pt.transforms.append(transform) # type: ignore

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
            pt.velocities.append(twist) # type: ignore

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
            pt.accelerations.append(acc) # type: ignore

        pt.time_from_start = to_ros_duration(time_from_start[idx])
        points.append(pt)

    traj_msg.points = points
    return traj_msg


def to_pose_array_msg(
    poses: FloatArray,
    stamp: float,
    frame_id: str = "world",
) -> PoseArray:
    """
    Build a PoseArray message from an array of poses.

    Parameters
    ----------
    poses : array, shape (N, 7)
        poses as [x, y, z, qx, qy, qz, qw].
    stamp : float
        Timestamp of the message in seconds.
    frame_id : str
        Fixed frame of the poses (default: "world").

    Returns
    -------
    PoseArray
        Populated PoseArray message.
    """

    poses = np.atleast_2d(poses)
    N = poses.shape[0]
    npu.validate_array_shape(poses, (N, 7), "poses")

    msg = PoseArray()
    msg.header.stamp = to_ros_time(stamp)
    msg.header.frame_id = frame_id

    poses_msg: list[Pose] = []
    for i in range(poses.shape[0]):
        p = poses[i]
        pose = Pose()
        pose.position.x = p[0]
        pose.position.y = p[1]
        pose.position.z = p[2]
        pose.orientation.x = p[3]
        pose.orientation.y = p[4]
        pose.orientation.z = p[5]
        pose.orientation.w = p[6]
        poses_msg.append(pose)

    msg.poses = poses_msg
    return msg


def to_joint_state_msg(
    positions: FloatArray,
    velocities: FloatArray | None,
    efforts: FloatArray | None,
    stamp: float,
    joint_names: Sequence[str],
) -> JointState:
    """
    Build a JointState message from joint positions, velocities, and efforts.

    Parameters
    ----------
    positions : array, shape (n,)
        Joint positions.
    velocities : array, shape (n,), optional
        Joint velocities. If None, the field is omitted.
    efforts : array, shape (n,), optional
        Joint efforts. If None, the field is omitted.
    stamp : float
        Timestamp of the message in seconds.
    joint_names : sequence of str
        Names of the joints.

    Returns
    -------
    JointState
        Populated JointState message.
    """

    n = len(joint_names)

    positions = np.atleast_1d(positions)
    npu.validate_array_shape(positions, (n,), "positions")

    if velocities is not None:
        velocities = np.atleast_1d(velocities)
        npu.validate_array_shape(velocities, (n,), "velocities")

    if efforts is not None:
        efforts = np.atleast_1d(efforts)
        npu.validate_array_shape(efforts, (n,), "efforts")

    msg = JointState()
    msg.header.stamp = to_ros_time(stamp)
    msg.name = list(joint_names)

    if positions is not None:
        msg.position = positions.tolist()

    if velocities is not None:
        msg.velocity = velocities.tolist()

    if efforts is not None:
        msg.effort = efforts.tolist()

    return msg


def from_joint_state_msg(msg: JointState) -> JointStateData:
    """
    Convert a JointState message to a JointStateData structure.

    Parameters
    ----------
    msg : JointState
        Input JointState message.

    Returns
    -------
    JointStateData
        Container with:
        - positions : ndarray, shape (n,)
        - velocities : ndarray, shape (n,) or None
        - efforts : ndarray, shape (n,) or None
        - stamp : float
        - joint_names : list of str
    """
    positions = npu.to_array(msg.position)

    if msg.velocity: velocities = npu.to_array(msg.velocity)
    else: velocities = None

    if msg.effort: efforts = npu.to_array(msg.effort)
    else: efforts = None

    stamp = from_ros_time(msg.header.stamp)
    
    joint_names = list(msg.name)

    data = JointStateData(
        positions = positions,
        velocities = velocities,
        efforts = efforts,
        stamp = stamp,
        joint_names = joint_names,
    )
    return data


def to_joint_effort_cmd_msg(
    efforts: FloatArray,
    stamp: float,
    joint_names: Sequence[str],
) -> JointEffortCmd:
    """
    Build a JointEffortCmd message from joint efforts.

    Parameters
    ----------
    efforts : array, shape (n,)
        Joint efforts.
    stamp : float
        Timestamp of the message in seconds.
    joint_names : sequence of str
        Names of the joints.

    Returns
    -------
    JointEffortCmd
        Populated JointEffortCmd message.
    """

    n = len(joint_names)

    efforts = np.atleast_1d(efforts)
    npu.validate_array_shape(efforts, (n,), "efforts")

    msg = JointEffortCmd()
    msg.header.stamp = to_ros_time(stamp)
    msg.name = list(joint_names)
    msg.effort = efforts.tolist()

    return msg


def from_joint_effort_cmd_msg(msg: JointEffortCmd) -> JointEffortCmdData:
    """
    Convert a JointEffortCmdMsg message (used as JointEffortCmd) to a dict of NumPy arrays.

    Parameters
    ----------
    msg : JointState
        Input JointState message.

    Returns
    -------
    dict[str, ndarray]
        Dictionary with keys 'efforts' and corresponding array.
    """
    
    efforts = npu.to_array(msg.effort)

    stamp = from_ros_time(msg.header.stamp)
    
    joint_names = list(msg.name)

    data = JointEffortCmdData(
        efforts = efforts,
        stamp = stamp,
        joint_names = joint_names,
    )
    return data