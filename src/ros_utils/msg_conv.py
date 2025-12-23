from __future__ import annotations
import uuid

import numpy as np
from collections.abc import Sequence

from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
    MultiDOFJointTrajectory,
    MultiDOFJointTrajectoryPoint,
    
)
from rlc_interfaces.msg import (
    JointEffortCmd, PlannedCartesianTrajectory, PlannedJointTrajectory,
    JointStateAction
)
from geometry_msgs.msg import Transform, Twist
from geometry_msgs.msg import PoseArray, Pose

from rbt_core.planning import CartesianTraj, JointTraj
from common_utils import FloatArray
from common_utils import numpy_util as npu

from.msg_types import (
    JointStateData,
    JointEffortCommandData,
    JointControllerStateData,
    JointTrajectoryData,
    PoseArrayData,
    JointStateActionData,
    CartesianTrajectoryData,
)
from . import time_util as tu

def to_joint_traj_msg(
    stamp: float,
    joint_names: Sequence[str],
    time_from_start: FloatArray,
    positions: FloatArray,
    velocities: FloatArray | None = None,
    accelerations: FloatArray | None = None,
) -> JointTrajectory:
    """
    Build a JointTrajectory of joint positions, velocities, and accelerations.

    Parameters
    ----------
    stamp : float
        Start time of the trajectory in seconds.
    joint_names : sequence of str
        Names of the joints, length n.
    time_from_start : ndarray, shape (N,) or (1,)
        Time from the start of the trajectory for each time step.
    positions : ndarray, shape (N, n) or (n,)
        Joint positions for each time step.
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
    msg.header.stamp = tu.to_ros_time(stamp)

    points: list[JointTrajectoryPoint] = []

    for idx in range(positions.shape[0]):
        pt = JointTrajectoryPoint()
        pt.positions = positions[idx].tolist()

        if velocities is not None:
            pt.velocities = velocities[idx].tolist()

        if accelerations is not None:
            pt.accelerations = accelerations[idx].tolist()

        pt.time_from_start = tu.to_ros_duration(time_from_start[idx])
        points.append(pt)

    msg.points = points
    return msg


def to_multi_dof_traj_msg(
    stamp: float,
    time_from_start: FloatArray,
    poses: FloatArray,
    twists: FloatArray | None = None,
    accels: FloatArray | None = None,
    frame_id: str = "world",
    joint_name: str = "ee",
) -> MultiDOFJointTrajectory:
    """
    Build a MultiDOFJointTrajectory message from poses, twists, and accelerations.

    Parameters
    ----------
    stamp : float
        Start time of the trajectory in seconds (for the header stamp)
    time_from_start : ndarray, shape (N,) or scalar
        Time from the start of the trajectory for each time step [s].
    poses : ndarray, shape (N, 7) or (7,)
        Poses as [x, y, z, qw, qx, qy, qz] for each time step.
    frame_id : str, default "world"
        Fixed frame of the poses.
    joint_name : str, default "ee"
        Name used in joint_names.
    twists : ndarray, shape (N, 6) or (6,), optional
        Twists as [vx, vy, vz, wx, wy, wz] for each time step.
        If None, the velocities field in the message is left empty.
    accels : ndarray, shape (N, 6) or (6,), optional
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
    traj_msg.header.stamp = tu.to_ros_time(stamp)
    traj_msg.header.frame_id = frame_id
    traj_msg.joint_names = [joint_name]

    points: list[MultiDOFJointTrajectoryPoint] = []

    for idx in range(N):
        pt = MultiDOFJointTrajectoryPoint()

        # Pose -> Transform

        x, y, z, qw, qx, qy, qz = poses[idx]
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

        pt.time_from_start = tu.to_ros_duration(time_from_start[idx])
        points.append(pt)

    traj_msg.points = points
    return traj_msg


def to_pose_array_msg(
    stamp: float,
    poses: FloatArray,
    frame_id: str = "world",
) -> PoseArray:
    """
    Build a PoseArray message from an array of poses.

    Parameters
    ----------
    stamp : float
        Timestamp of the message in seconds.
    poses : array, shape (N, 7)
        poses as [x, y, z, qw, qx, qy, qz].
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
    msg.header.stamp = tu.to_ros_time(stamp)
    msg.header.frame_id = frame_id

    poses_msg: list[Pose] = []
    for i in range(poses.shape[0]):
        p = poses[i]
        pose = Pose()
        pose.position.x = p[0]
        pose.position.y = p[1]
        pose.position.z = p[2]
        pose.orientation.w = p[3]
        pose.orientation.x = p[4]
        pose.orientation.y = p[5]
        pose.orientation.z = p[6]
        poses_msg.append(pose)

    msg.poses = poses_msg
    return msg


def to_joint_state_action_msg(
    stamp: float,
    joint_names: Sequence[str],
    position: FloatArray,
    velocity: FloatArray,
    action: FloatArray,
    action_baseline: FloatArray,
) -> JointStateAction:
    """
    Build a JointStateAction message from joint positions, velocities, actions, and baseline actions.

    Parameters
    ----------
    stamp : float
        Timestamp of the message in seconds.
    joint_names : sequence of str
        Names of the joints.
    position : array, shape (n,)
        Joint positions.
    velocity : array, shape (n,)
        Joint velocities.
    action : array, shape (n,)
        Joint actions.
    action_baseline : array, shape (n,)
        Baseline joint actions.

    Returns
    -------
    JointStateAction
        Populated JointStateAction message.
    """

    n = len(joint_names)

    position = np.atleast_1d(position)
    npu.validate_array_shape(position, (n,), "position")

    velocity = np.atleast_1d(velocity)
    npu.validate_array_shape(velocity, (n,), "velocity")

    action = np.atleast_1d(action)
    npu.validate_array_shape(action, (n,), "action")

    action_baseline = np.atleast_1d(action_baseline)
    npu.validate_array_shape(action_baseline, (n,), "action_baseline")

    msg = JointStateAction()
    msg.header.stamp = tu.to_ros_time(stamp)
    msg.name = list(joint_names)
    msg.position = position.tolist()
    msg.velocity = velocity.tolist()
    msg.action = action.tolist()
    msg.action_baseline = action_baseline.tolist()

    return msg


def from_joint_state_action_msg(msg: JointStateAction) -> JointStateActionData:
    """
    Convert a JointStateAction message to JointStateActionData.

    Parameters
    ----------
    msg : JointStateAction
        Input JointStateAction message.

    Returns
    -------
    JointStateActionData
        Container with:
        - stamp : float
        - joint_names : list of str
        - position : ndarray, shape (n,)
        - velocity : ndarray, shape (n,)
        - action : ndarray, shape (n,)
        - action_baseline : ndarray, shape (n,)
    """

    stamp = tu.from_ros_time_or_duration(msg.header.stamp)
    joint_names = list(msg.name)

    position = npu.to_array(msg.position)
    velocity = npu.to_array(msg.velocity)
    action = npu.to_array(msg.action)
    action_baseline = npu.to_array(msg.action_baseline)

    return JointStateActionData(
        stamp=stamp,
        joint_names=joint_names,
        positions=position,
        velocities=velocity,
        actions=action,
        actions_baseline=action_baseline,
    )


def from_pose_array_msg(msg: PoseArray) -> PoseArrayData:
    """
    Convert a PoseArray message to NumPy arrays.

    Parameters
    ----------
    msg : PoseArray
        Input PoseArray message.

    Returns
    -------
    PoseArrayData
        Container with:
        - stamp : float
        - frame_id : str
        - poses : ndarray, shape (N, 7) as [x, y, z, qw, qx, qy, qz]
    """

    stamp = tu.from_ros_time_or_duration(msg.header.stamp)
    frame_id = msg.header.frame_id

    if not msg.poses:
        poses = np.zeros((0, 7), dtype=npu.dtype)
    else:
        poses_list = [
            [p.position.x, p.position.y, p.position.z,
             p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]
            for p in msg.poses
        ]
        poses = npu.to_array(poses_list)

    return PoseArrayData(
        stamp=stamp,
        frame_id=frame_id,
        poses=poses,
    )


def to_joint_state_msg(
    stamp: float,
    joint_names: Sequence[str],
    positions: FloatArray,
    velocities: FloatArray | None = None,
    efforts: FloatArray | None = None,
) -> JointState:
    """
    Build a JointState message from joint positions, velocities, and efforts.

    Parameters
    ----------
    stamp : float
        Timestamp of the message in seconds.
    joint_names : sequence of str
        Names of the joints.
    positions : array, shape (n,)
        Joint positions.
    velocities : array, shape (n,), optional
        Joint velocities. If None, the field is omitted.
    efforts : array, shape (n,), optional
        Joint efforts. If None, the field is omitted.

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
    msg.header.stamp = tu.to_ros_time(stamp)
    msg.name = list(joint_names)

    if positions is not None:
        msg.position = positions.tolist()

    if velocities is not None:
        msg.velocity = velocities.tolist()

    if efforts is not None:
        msg.effort = efforts.tolist()

    return msg


def to_joint_effort_cmd_msg(
    stamp: float,
    joint_names: Sequence[str],
    efforts: FloatArray,
) -> JointEffortCmd:
    """
    Build a JointEffortCmd message from joint efforts.

    Parameters
    ----------
    stamp : float
        Timestamp of the message in seconds.
    joint_names : sequence of str
        Names of the joints.
    efforts : array, shape (n,)
        Joint efforts.

    Returns
    -------
    JointEffortCmd
        Populated JointEffortCmd message.
    """

    n = len(joint_names)

    efforts = np.atleast_1d(efforts)
    npu.validate_array_shape(efforts, (n,), "efforts")

    msg = JointEffortCmd()
    msg.header.stamp = tu.to_ros_time(stamp)
    msg.name = list(joint_names)
    msg.effort = efforts.tolist()

    return msg


def to_joint_ctrl_state_msg(
    stamp: float,
    joint_names: Sequence[str],
    tau: FloatArray,
    q: FloatArray,
    q_des: FloatArray,
    qd: FloatArray | None = None,
    qd_des: FloatArray | None = None,
    qdd: FloatArray | None = None,
    qdd_des: FloatArray | None = None,
) -> JointTrajectoryControllerState:
    """
    Build a JointTrajectoryControllerState message from joint state arrays.

    Parameters
    ----------
    stamp : float
        time stamp
    joint_names : sequence of str
        Names of the joints, length n.
    tau : array, shape (n,)
        Controller output torque command.
    q : array, shape (n,)
        Measured joint positions.
    q_des : array, shape (n,)
        Desired joint positions.
    qd : array, shape (n,), optional
        Measured joint velocities.
        if None it is omitted from the message
    qd_des : array, shape (n,), optional
        Desired joint velocities.
        if None it is omitted from the message
    qdd : array, shape (n,) or optional.
        Measured joint accelerations. 
        if None it is omitted from the message.
    qdd_des : array, shape (n,) or optional
        Desired joint accelerations. 
        if None it is omitted from the message.

    Returns
    -------
    JointTrajectoryControllerState
        Populated controller state message.
    """

    n = len(joint_names)
    npu.validate_array_shape(tau, (n,), "tau")
    npu.validate_array_shape(q, (n,), "q")
    npu.validate_array_shape(q_des, (n,), "q_des")
    if qd is not None: npu.validate_array_shape(qd, (n,), "qd")
    if qd_des is not None: npu.validate_array_shape(qd_des, (n,), "qd_des")
    if qdd is not None: npu.validate_array_shape(qdd, (n,), "qdd")
    if qdd_des is not None: npu.validate_array_shape(qdd_des, (n,), "qdd_des")

    msg = JointTrajectoryControllerState()
    msg.header.stamp = tu.to_ros_time(stamp)
    msg.joint_names = list(joint_names)

    # Desired / reference state
    ref = JointTrajectoryPoint()
    ref.positions = q_des.tolist()
    if qd_des is not None: ref.velocities = qd_des.tolist()
    if qdd_des is not None: ref.accelerations  = qdd_des.tolist()
    msg.reference = ref

    # Measured / feedback state
    fb = JointTrajectoryPoint()
    fb.positions = q.tolist()
    if qd is not None: fb.velocities = qd.tolist()
    if qdd is not None: fb.accelerations  = qdd.tolist()
    msg.feedback = fb

    # Error = reference - feedback
    err_pt = JointTrajectoryPoint()
    e  = q_des - q
    err_pt.positions = e.tolist()

    if (qd_des is not None) and (qd is not None):
        ed = qd_des - qd
        err_pt.velocities = ed.tolist()

    msg.error = err_pt

    # Controller output (tau)
    out = JointTrajectoryPoint()
    out.effort = tau.tolist()
    msg.output = out

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
    velocities = npu.to_array(msg.velocity)
    efforts = npu.to_array(msg.effort) if msg.effort else None

    stamp = tu.from_ros_time_or_duration(msg.header.stamp)

    joint_names = list(msg.name)

    data = JointStateData(
        positions = positions,
        velocities = velocities,
        efforts = efforts,
        stamp = stamp,
        joint_names = joint_names,
    )
    return data


def from_joint_effort_cmd_msg(msg: JointEffortCmd) -> JointEffortCommandData:
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
    stamp = tu.from_ros_time_or_duration(msg.header.stamp)
    joint_names = list(msg.name)

    data = JointEffortCommandData(
        efforts = efforts,
        stamp = stamp,
        joint_names = joint_names,
    )
    return data


def from_joint_ctrl_state_msg(
    msg: JointTrajectoryControllerState,
) -> JointControllerStateData:
    """
    Convert a JointTrajectoryControllerState message to NumPy arrays.

    Parameters
    ----------
    msg : JointTrajectoryControllerState
        Controller state message.

    Returns
    -------
    JointCtrlStateData
        Dataclass with time stamp, joint names, measured/desired joint
        states, and controller output torque.
    """
    stamp = tu.from_ros_time_or_duration(msg.header.stamp)
    joint_names = list(msg.joint_names)

    # Desired / reference
    ref = msg.reference
    q_des = npu.to_array(ref.positions)
    qd_des = npu.to_array(ref.velocities) if ref.velocities else None
    qdd_des = npu.to_array(ref.accelerations) if ref.accelerations else None

    # Measured / feedback
    fb = msg.feedback
    q = npu.to_array(fb.positions)
    qd = npu.to_array(fb.velocities) if fb.velocities else None
    qdd = npu.to_array(fb.accelerations) if fb.accelerations else None

    # Error
    err = msg.error
    e = npu.to_array(err.positions) if err.positions else None
    ed = npu.to_array(err.velocities) if err.velocities else None
    # Controller output
    tau = npu.to_array(msg.output.effort)

    return JointControllerStateData(
        stamp=stamp,
        joint_names=joint_names,
        tau=tau,
        q=q,
        q_des=q_des,
        qd=qd,
        qd_des=qd_des,
        qdd=qdd,
        qdd_des=qdd_des,
        e=e,
        ed=ed
    )


def from_joint_traj_msg(msg: JointTrajectory) -> JointTrajectoryData:
    """
    Convert a JointTrajectory message to NumPy arrays.

    Parameters
    ----------
    msg : JointTrajectory
        Trajectory message.

    Returns
    -------
    JointTrajData
        Dataclass with time stamp, joint names, time_from_start,
        positions, velocities, and accelerations.
    """
    stamp = tu.from_ros_time_or_duration(msg.header.stamp)
    joint_names = list(msg.joint_names)
    points: list[JointTrajectoryPoint] = list(msg.points)

    if not points:
        raise ValueError("JointTrajectory has no points.")

    N = len(points)
    n = len(joint_names)

    # time_from_start (N,)
    time_from_start = npu.to_array(
        [tu.from_ros_time_or_duration(p.time_from_start) for p in points],
    )

    # positions (N, n)
    pos_list = [p.positions for p in points]
    positions = npu.to_array(pos_list)

    # velocities (N, n), (optional)
    if any(p.velocities for p in points):
        vel_list = [p.velocities for p in points]
        velocities = npu.to_array(vel_list)
    else:
        velocities = None

    # accelerations (N, n), (optional)
    if any(p.accelerations for p in points):
        acl_list = [p.accelerations for p in points]
        accelerations = npu.to_array(acl_list)
    else:
        accelerations = None


    return JointTrajectoryData(
        stamp=stamp,
        joint_names=joint_names,
        time_from_start=time_from_start,
        positions=positions,
        velocities=velocities,
        accelerations=accelerations,
    )


def from_multi_dof_traj_msg(
    msg: MultiDOFJointTrajectory,
) -> CartesianTrajectoryData:
    """
    Convert a MultiDOFJointTrajectory message to NumPy arrays.

    Parameters
    ----------
    msg : MultiDOFJointTrajectory
        Cartesian trajectory message.

    Returns
    -------
    CartesianTrajData
        Dataclass with time stamp, poses, twists, accelerations,
        and time_from_start.
    """
    stamp = tu.from_ros_time_or_duration(msg.header.stamp)
    frame_id = msg.header.frame_id
    points: list[MultiDOFJointTrajectoryPoint] = list(msg.points)

    if not points:
        raise ValueError("MultiDOFJointTrajectory has no points.")

    N = len(points)

    # time_from_start (N,)
    time_from_start = npu.to_array(
        [tu.from_ros_time_or_duration(p.time_from_start) for p in points],
    )

    # poses (N, 7)
    pose_list = []
    for p in points:
        t: Transform = p.transforms[0] # type: ignore
        pose_list.append([
            t.translation.x,
            t.translation.y,
            t.translation.z,
            t.rotation.w,
            t.rotation.x,
            t.rotation.y,
            t.rotation.z,
        ])
    poses = npu.to_array(pose_list)

    # twists (N, 6), optional
    if any(p.velocities for p in points):
        twist_list = []
        for p in points:
            tw: Twist = p.velocities[0] # type: ignore
            twist_list.append([
                tw.linear.x,
                tw.linear.y,
                tw.linear.z,
                tw.angular.x,
                tw.angular.y,
                tw.angular.z,
            ])
        twists = npu.to_array(twist_list)
    else:
        twists = None

    # accelerations (N, 6), optional
    if any(p.accelerations for p in points):
        acc_list = []
        for p in points:
            ac: Twist = p.accelerations[0] # type: ignore
            acc_list.append([
                ac.linear.x,
                ac.linear.y,
                ac.linear.z,
                ac.angular.x,
                ac.angular.y,
                ac.angular.z,
            ])
        accelerations = npu.to_array(acc_list)
    else:
        accelerations = None

    return CartesianTrajectoryData(
        stamp=stamp,
        time_from_start=time_from_start,
        poses=poses,
        twists=twists,
        accelerations=accelerations
    )


def to_planned_joint_traj_msg(
    joint_traj_msg: JointTrajectory,
    label: str,
    execute_immediately: bool = False,
    traj_id: str | None = None,
) -> tuple[PlannedJointTrajectory, str]:
    """
    Wrap a `JointTrajectory` in a `PlannedJointTrajectory` message.

    Parameters
    ----------
    joint_traj_msg : JointTrajectory
        Trajectory message containing joint waypoints to execute.
    label : str
        Human-readable tag for the trajectory (e.g., planner/source).
    execute_immediately : bool
        If True, downstream components should begin execution on receipt.
    traj_id : str, optional
        Unique trajectory identifier. If None, a UUID4 is generated.

    Returns
    -------
    tuple[str, PlannedJointTrajectory]
        A tuple of `(trajectory_id, planned_msg)` where:
        - `trajectory_id` is the unique identifier used to track the plan.
        - `planned_msg` is the populated `PlannedJointTrajectory` message.

    Notes
    -----
    - `trajectory_id` is set on the message and returned for convenience.
    - `execute_immediately` is a hint for the consumer node; no side effects
      are performed here.
    """
    if traj_id is None: traj_id = str(uuid.uuid4())

    # build joint trajectory message
    joint_msg = PlannedJointTrajectory()
    joint_msg.trajectory_id = traj_id
    joint_msg.trajectory = joint_traj_msg
    joint_msg.label = label
    joint_msg.execute_immediately = execute_immediately

    return joint_msg, traj_id


def to_planned_cartesian_traj_msg(
    cart_traj_msg: MultiDOFJointTrajectory,
    label: str,
    derived_from_joint: bool,
    execute_immediately: bool = False,
    traj_id: str | None = None,
) -> tuple[PlannedCartesianTrajectory, str]:
    """
    Wrap a `MultiDOFJointTrajectory` in a `PlannedCartesianTrajectory` message.

    Parameters
    ----------
    cart_traj_msg : MultiDOFJointTrajectory
        Cartesian trajectory message (poses, optional twists/accels).
    label : str
        Human-readable tag for the trajectory (e.g., planner/source).
    derived_from_joint : bool
        True if this Cartesian plan was generated from a joint trajectory.
    execute_immediately : bool
        If True, downstream components should begin execution on receipt.
    traj_id : str, optional
        Unique trajectory identifier. If None, a UUID4 is generated.

    Returns
    -------
    tuple[str, PlannedCartesianTrajectory]
        A tuple of `(trajectory_id, planned_msg)` where:
        - `trajectory_id` is the unique identifier used to track the plan.
        - `planned_msg` is the populated `PlannedCartesianTrajectory` message.

    Notes
    -----
    - `trajectory_id` is set on the message and returned for convenience.
    - `derived_from_joint` helps the consumer node decide visualization
      linkage to the originating joint plan.
    - `execute_immediately` is a hint for the consumer; no side effects
      are performed here.
    """
    if traj_id is None: traj_id = str(uuid.uuid4())

    # build cartesian trajectory message
    cart_msg = PlannedCartesianTrajectory()
    cart_msg.trajectory_id = traj_id
    cart_msg.trajectory = cart_traj_msg
    cart_msg.label = label
    cart_msg.derived_from_joint = derived_from_joint
    cart_msg.execute_immediately = execute_immediately

    return cart_msg, traj_id


def planned_joint_traj(
    stamp: float,
    joint_names: Sequence[str],
    joint_traj: JointTraj,
    label: str,
    execute_immediately: bool = False,
    traj_id: str | None = None,
) -> tuple[PlannedJointTrajectory, str]:
    """
    Build a `PlannedJointTrajectory` from raw joint-space arrays.

    Parameters
    ----------
    stamp : float
        Start time of the trajectory in seconds (for the header stamp).
    joint_names : sequence of str
        Names of the joints, length n.
    joint_traj: JointTraj
        Joint trajectory data containing times positions, velocities, and accelerations.
    label : str
        Human-readable tag for the trajectory (e.g., planner/source).
    execute_immediately : bool, default False
        If True, downstream components should begin execution on receipt.
    traj_id : str, optional
        Unique trajectory identifier. If None, a UUID4 is generated.

    Returns
    -------
    tuple[PlannedJointTrajectory, str]
        `(planned_msg, trajectory_id)` where `planned_msg` is the populated
        `PlannedJointTrajectory` and `trajectory_id` is the unique identifier.

    Notes
    -----
    - Internally constructs a `JointTrajectory` via `to_joint_traj_msg`.
    - `trajectory_id` is set on the message and returned for convenience.
    - `execute_immediately` is a hint for consumers; no side effects here.
    """
    time_from_start = joint_traj.t
    positions = joint_traj.q
    velocities = joint_traj.qd
    accelerations = joint_traj.qdd

    joint_traj_msg = to_joint_traj_msg(
        stamp=stamp,
        joint_names=joint_names,
        time_from_start=time_from_start,
        positions=positions,
        velocities=velocities,
        accelerations=accelerations,
    )

    msg, traj_id = to_planned_joint_traj_msg(
                    joint_traj_msg=joint_traj_msg,
                    label=label,
                    execute_immediately=execute_immediately,
                    traj_id=traj_id
                    )

    return msg, traj_id


def planned_cartesian_traj(
    stamp: float,
    cart_traj: CartesianTraj,
    label: str,
    frame_id: str = "world",
    joint_name: str = "ee",
    derived_from_joint: bool = False,
    execute_immediately: bool = False,
    traj_id: str | None = None,
) -> tuple[PlannedCartesianTrajectory, str]:
    """
    Build a `PlannedCartesianTrajectory` from raw Cartesian arrays.

    Parameters
    ----------
    stamp : float
        Start time of the trajectory in seconds (for the header stamp).
    cart_traj: CartesianTraj
        Cartesian trajectory data containing times, poses, twists, and accelerations.
    label : str
        Human-readable tag for the trajectory (e.g., planner/source).
    frame_id : str, default "world"
        Fixed frame for the poses.
    joint_name : str, default "ee"
        Name used in `joint_names` of the trajectory message.
    derived_from_joint : bool, default False
        True if this Cartesian plan was generated from a joint trajectory.
    execute_immediately : bool, default False
        If True, downstream components should begin execution on receipt.
    traj_id : str, optional
        Unique trajectory identifier. If None, a UUID4 is generated.

    Returns
    -------
    tuple[PlannedCartesianTrajectory, str]
        `(planned_msg, trajectory_id)` where `planned_msg` is the populated
        `PlannedCartesianTrajectory` and `trajectory_id` is the unique id.

    Notes
    -----
    - Internally constructs a `MultiDOFJointTrajectory` via `to_multi_dof_traj_msg`.
    - `trajectory_id` is set on the message and returned for convenience.
    - `derived_from_joint` helps consumers link to an originating joint plan.
    - `execute_immediately` is a hint for consumers; no side effects here.
    """
    
    time_from_start = cart_traj.t
    poses = cart_traj.pose
    twists = cart_traj.twist
    accels = cart_traj.acc
    cart_traj_msg = to_multi_dof_traj_msg(
        stamp=stamp,
        time_from_start=time_from_start,
        poses=poses,
        twists=twists,
        accels=accels,
        frame_id=frame_id,
        joint_name=joint_name,
    )

    msg, traj_id = to_planned_cartesian_traj_msg(
                    cart_traj_msg=cart_traj_msg,
                    label=label,
                    derived_from_joint=derived_from_joint,
                    execute_immediately=execute_immediately,
                    traj_id=traj_id
                    )

    return msg, traj_id