import numpy as np
import mujoco as mj

from ..utils import numpy_util as npu
FloatArray = npu.FloatArray
EE_TRAJ_SEGID = 1001
FRAME_SEGID    = 1002
FRAME_AT_COM_SEGID    = 1003


def draw_ee_traj(
    scn: mj.MjvScene,
    Q_EF: FloatArray,
    radius: float = 0.001,    # meters
    stride: int = 10,
    segid: int = EE_TRAJ_SEGID,
) -> None:
    """
    Draw end-effector trajectory as a 3D capsule tube.

    Q_EF : (N, 3) world positions.
    radius : capsule radius in meters.
    stride : keep every `stride`-th point.
    segid  : integer tag used to identify / replace these geoms later.
    """
    Q_EF = np.asarray(Q_EF, dtype=np.float64)
    assert Q_EF.ndim == 2 and Q_EF.shape[1] == 3

    # 1) Remove any old EE-trajectory geoms (same segid) by compacting the array
    write = 0
    for read in range(scn.ngeom):
        g = scn.geoms[read]
        if g.segid == segid:
            # skip old trajectory geom
            continue
        if write != read:
            scn.geoms[write] = g
        write += 1
    scn.ngeom = write

    # 2) Append new capsule segments
    start = scn.ngeom
    rgba  = np.array([0.8, 0.1, 0.8, 1.0], dtype=np.float32)

    # subsample path to avoid too many geoms
    idx = np.arange(0, Q_EF.shape[0], stride)
    if idx[-1] != Q_EF.shape[0] - 1:
        idx = np.r_[idx, Q_EF.shape[0] - 1]

    max_seg = scn.maxgeom - start
    n_pts   = min(len(idx), max_seg + 1)

    for k in range(n_pts - 1):
        i0, i1 = idx[k], idx[k + 1]
        p1, p2 = Q_EF[i0], Q_EF[i1]

        g = scn.geoms[start + k]
        mj.mjv_connector(
            g,
            mj.mjtGeom.mjGEOM_CAPSULE,  # tube segment
            float(radius),              # radius in meters
            p1,
            p2,
        )
        g.rgba[:] = rgba
        g.segid   = segid              # tag as EE trajectory

    scn.ngeom = start + (n_pts - 1)


def draw_all_frames(
    scn: mj.MjvScene,
    T_wf: FloatArray,
    axis_len: float = 0.2,
    width: float = 0.004,
    segid: int = FRAME_SEGID,
) -> None:
    """
    Draw coordinate frames for each transform in T_wf.

    - First remove any old geoms with this segid.
    - Then append new geoms tagged with this segid.
    """
    D = npu.dtype

    # 1) Remove old "frame" geoms (segid == FRAME_SEGID) by compacting the array
    write = 0
    for read in range(scn.ngeom):
        g = scn.geoms[read]
        if g.segid == segid:
            # skip old frame geom: don't copy to new position
            continue
        if write != read:
            scn.geoms[write] = g
        write += 1
    scn.ngeom = write

    # 2) Append new frame geoms, tagged with segid
    for i in range(T_wf.shape[0]):
        T = T_wf[i]
        o = T[:3, 3]
        R = T[:3, :3]

        cols = (
            np.array([1, 0, 0, 1], dtype=D),  # X
            np.array([0, 1, 0, 1], dtype=D),  # Y
            np.array([0, 0, 1, 1], dtype=D),  # Z
        )

        for a, rgba in zip(R.T, cols):
            p1 = o
            p2 = o + axis_len * a

            g = scn.geoms[scn.ngeom]

            mj.mjv_connector(
                g,
                mj.mjtGeom.mjGEOM_ARROW,
                float(width),
                np.asarray(p1, dtype=np.float64),
                np.asarray(p2, dtype=np.float64),
            )
            g.rgba[:] = rgba.astype(np.float32, copy=False)
            g.segid = segid       # <-- tag as "frame" geom

            scn.ngeom += 1


def draw_all_frames_at_com(
    scn: mj.MjvScene,
    T_wf: FloatArray,
    com_wl: FloatArray,
    axis_len: float = 0.2,
    width: float = 0.004,
    segid: int = FRAME_AT_COM_SEGID,
) -> None:
    """
    Draw coordinate frames for each transform in T_wf.z

    - First remove any old geoms with this segid.
    - Then append new geoms tagged with this segid.
    """
    D = npu.dtype

    # 1) Remove old "frame" geoms (segid == FRAME_SEGID) by compacting the array
    write = 0
    for read in range(scn.ngeom):
        g = scn.geoms[read]
        if g.segid == segid:
            # skip old frame geom: don't copy to new position
            continue
        if write != read:
            scn.geoms[write] = g
        write += 1
    scn.ngeom = write

    for i in range(com_wl.shape[0]):
        T = T_wf[i]
        o = com_wl[i]
        R = T[:3, :3]

        cols = (
            np.array([1, 0, 0, 1], dtype=D),  # X
            np.array([0, 1, 0, 1], dtype=D),  # Y
            np.array([0, 0, 1, 1], dtype=D),  # Z
        )

        for a, rgba in zip(R.T, cols):
            p1 = o
            p2 = o + axis_len * a

            g = scn.geoms[scn.ngeom]

            mj.mjv_connector(
                g,
                mj.mjtGeom.mjGEOM_ARROW,
                float(width),
                np.asarray(p1, dtype=np.float64),
                np.asarray(p2, dtype=np.float64),
            )
            g.rgba[:] = rgba.astype(np.float32, copy=False)
            g.segid = segid       # <-- tag as "frame" geom

            scn.ngeom += 1

