from __future__ import annotations
import math
import uuid

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from builtin_interfaces.msg import Duration as DurationMsg

from common_utils import FloatArray
from common_utils import numpy_util as npu  



NSEC = 1_000_000_000

def to_ros_time(t: float) -> TimeMsg:
    """Convert a time in seconds to a ROS Time message."""
    total_ns = int(round(t * NSEC))
    sec = int(math.floor(total_ns / NSEC))
    nanosec = int(total_ns - sec * NSEC)  # always 0..1e9-1
    return TimeMsg(sec=sec, nanosec=nanosec)

def to_ros_duration(dt: float) -> DurationMsg:
    """Convert a duration in seconds to a ROS Duration message."""
    total_ns = int(round(dt * NSEC))
    sec = int(math.floor(total_ns / NSEC))
    nanosec = int(total_ns - sec * NSEC)
    return DurationMsg(sec=sec, nanosec=nanosec)

def from_ros_time_or_duration(t: TimeMsg | DurationMsg) -> float:
    """Convert a ROS Time or Duration message to a time in seconds."""
    return float(t.sec) + 1e-9 * float(t.nanosec)


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