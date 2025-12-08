from __future__ import annotations
import uuid

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from builtin_interfaces.msg import Duration as DurationMsg

from common_utils import FloatArray
from common_utils import numpy_util as npu  



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


def time_from_start(
    num_samples: int,
    *,
    freq: float | None = None,
    duration: float | None = None,
) -> FloatArray:
    """
    Build a time-from-start vector for an evenly sampled trajectory.

    You must specify exactly one of `freq` (Hz) or `duration` (seconds).

    Parameters
    ----------
    num_samples : int
        Number of trajectory samples N. Must be >= 1.
    freq : float, optional
        Sampling frequency in Hz. If given, times are t[k] = k / freq.
    duration : float, optional
        Total duration in seconds. If given, times are linearly spaced
        from 0.0 to duration inclusive.

    Returns
    -------
    t : ndarray, shape (N,)
        Time-from-start values in seconds.
    """

    if freq is not None:
        idx = np.arange(num_samples, dtype=npu.dtype)
        t = idx / freq
    elif duration is not None:
        t = np.linspace(0.0, duration, num_samples, dtype=npu.dtype)
    else:  # neither freq nor duration given
        raise ValueError("Must specify exactly one of freq or duration")

    return t