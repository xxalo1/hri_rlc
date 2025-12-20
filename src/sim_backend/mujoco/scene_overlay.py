from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np
import mujoco as mj
from numpy.typing import NDArray

Float32Array = NDArray[np.float32]
Float64Array = NDArray[np.float64]
FloatArray = NDArray[np.floating]


def _geom_none() -> int:
    # mjGEOM_NONE exists in MuJoCo; keep a fallback just in case.
    try:
        return int(mj.mjtGeom.mjGEOM_NONE)
    except Exception:
        return 0


@dataclass
class _Pool:
    segid: int
    start: int
    cap: int
    used: int = 0
    prev_used: int = 0


@dataclass(frozen=True)
class LayerRef:
    name: str
    segid: int


class SceneOverlay:
    """
    High performance overlay manager for mjvScene.
    Uses reserved pools per layer, so clearing one layer does not touch others.
    """

    def __init__(self, scn: mj.MjvScene, *, segid_base: int = 10_000_000) -> None:
        self._scn = scn
        self._next_segid = int(segid_base)
        self._layers: Dict[str, int] = {}
        self._pools: Dict[str, _Pool] = {}

    @property
    def scn(self) -> mj.MjvScene:
        return self._scn

    def layer_id(self, name: str) -> LayerRef:
        segid = self._layers.get(name)
        if segid is None:
            segid = self._next_segid
            self._next_segid += 1
            self._layers[name] = segid
        return LayerRef(name=name, segid=segid)

    def reserve(self, name: str, cap: int) -> None:
        """
        Reserve a fixed block of geoms for this layer.
        This is a one time cost. After that, updates are in place.
        """
        if name in self._pools:
            return

        scn = self._scn
        segid = self.layer_id(name).segid

        start = int(scn.ngeom)
        maxgeom = int(scn.maxgeom)
        if start + cap > maxgeom:
            raise RuntimeError(
                f"Not enough user_scn capacity for layer '{name}'. "
                f"Need {cap} slots, have {maxgeom - start}."
            )

        scn.ngeom = start + cap
        none_type = _geom_none()

        # Initialize slots to invisible
        for i in range(start, start + cap):
            g = scn.geoms[i]
            g.segid = int(segid)
            g.rgba[:] = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
            try:
                g.type = none_type
            except Exception:
                # If type is not writable in your binding, alpha 0 is sufficient.
                pass

        self._pools[name] = _Pool(segid=segid, start=start, cap=cap, used=0, prev_used=cap)

    def clear(self, name: str) -> None:
        """
        Fast clear: hide previously used slots for this layer.
        Does not touch other layers.
        """
        pool = self._pools.get(name)
        if pool is None:
            return

        scn = self._scn
        none_type = _geom_none()

        for idx in range(pool.start, pool.start + pool.used):
            g = scn.geoms[idx]
            g.rgba[3] = 0.0
            try:
                g.type = none_type
            except Exception:
                pass

        pool.prev_used = pool.used
        pool.used = 0

    def layer(self, name: str) -> "LayerWriter":
        if name not in self._pools:
            raise RuntimeError(f"Layer '{name}' not reserved. Call reserve('{name}', cap) first.")
        return LayerWriter(self, self.layer_id(name))

    def _begin(self, name: str) -> None:
        pool = self._pools[name]
        pool.prev_used = pool.used
        pool.used = 0

    def _end(self, name: str) -> None:
        pool = self._pools[name]
        scn = self._scn
        none_type = _geom_none()

        # Hide only slots that were used before but are unused now
        for idx in range(pool.start + pool.used, pool.start + pool.prev_used):
            g = scn.geoms[idx]
            g.rgba[3] = 0.0
            try:
                g.type = none_type
            except Exception:
                pass

        pool.prev_used = pool.used

    def _slot(self, name: str):
        pool = self._pools[name]
        if pool.used >= pool.cap:
            raise RuntimeError(f"Layer '{name}' exceeded reserved capacity {pool.cap}.")
        idx = pool.start + pool.used
        pool.used += 1
        return self._scn.geoms[idx], pool.segid

    # Primitives

    def add_sphere(self, name: str, center: Float64Array, radius: float, rgba: Float32Array) -> None:
        geom, segid = self._slot(name)

        # Initialize if needed
        if int(getattr(geom, "type", -1)) != int(mj.mjtGeom.mjGEOM_SPHERE):
            mj.mjv_initGeom(
                geom,
                mj.mjtGeom.mjGEOM_SPHERE,
                size=np.array([radius, 0.0, 0.0], dtype=np.float64),
                rgba=rgba,
                pos=center,
                mat=np.eye(3, dtype=np.float64).reshape(9),
            )
        else:
            geom.size[0] = float(radius)
            geom.pos[:] = center
            geom.rgba[:] = rgba

        geom.segid = int(segid)

    def add_line(self, name: str, p0: Float64Array, p1: Float64Array, rgba: Float32Array, width: float = 0.002) -> None:
        geom, segid = self._slot(name)

        # Initialize if needed
        if int(getattr(geom, "type", -1)) != int(mj.mjtGeom.mjGEOM_LINE):
            mj.mjv_initGeom(
                geom,
                mj.mjtGeom.mjGEOM_LINE,
                size=np.zeros(3, dtype=np.float64),
                rgba=rgba,
                pos=np.zeros(3, dtype=np.float64),
                mat=np.eye(3, dtype=np.float64).reshape(9),
            )

        # Update endpoints
        mj.mjv_connector(geom, mj.mjtGeom.mjGEOM_LINE, float(width), p0, p1)
        geom.rgba[:] = rgba
        geom.segid = int(segid)

    def add_arrow(self, name: str, p0: Float64Array, p1: Float64Array, rgba: Float32Array, width: float = 0.003) -> None:
        geom, segid = self._slot(name)

        # Initialize if needed
        if int(getattr(geom, "type", -1)) != int(mj.mjtGeom.mjGEOM_ARROW):
            mj.mjv_initGeom(
                geom,
                mj.mjtGeom.mjGEOM_ARROW,
                size=np.zeros(3, dtype=np.float64),
                rgba=rgba,
                pos=np.zeros(3, dtype=np.float64),
                mat=np.eye(3, dtype=np.float64).reshape(9),
            )

        mj.mjv_connector(geom, mj.mjtGeom.mjGEOM_ARROW, float(width), p0, p1)
        geom.rgba[:] = rgba
        geom.segid = int(segid)

    def add_capsule(self, name: str, p0: Float64Array, p1: Float64Array, rgba: Float32Array, radius: float = 0.003) -> None:
        geom, segid = self._slot(name)

        if int(getattr(geom, "type", -1)) != int(mj.mjtGeom.mjGEOM_CAPSULE):
            mj.mjv_initGeom(
                geom,
                mj.mjtGeom.mjGEOM_CAPSULE,
                size=np.zeros(3, dtype=np.float64),
                rgba=rgba,
                pos=np.zeros(3, dtype=np.float64),
                mat=np.eye(3, dtype=np.float64).reshape(9),
            )

        mj.mjv_connector(geom, mj.mjtGeom.mjGEOM_CAPSULE, float(radius), p0, p1)
        geom.rgba[:] = rgba
        geom.segid = int(segid)


class LayerWriter:
    def __init__(self, overlay: SceneOverlay, layer: LayerRef) -> None:
        self._overlay = overlay
        self._layer = layer

    def __enter__(self) -> "LayerWriter":
        self._overlay._begin(self._layer.name)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self._overlay._end(self._layer.name)
        return None

    def sphere(self, center: FloatArray, radius: float, rgba: FloatArray) -> None:
        c = center.astype(np.float64, copy=False)
        col = rgba.astype(np.float32, copy=False)
        self._overlay.add_sphere(self._layer.name, c, radius, col)

    def line(self, p0: FloatArray, p1: FloatArray, rgba: FloatArray, width: float = 0.002) -> None:
        a = p0.astype(np.float64, copy=False)
        b = p1.astype(np.float64, copy=False)
        col = rgba.astype(np.float32, copy=False)
        self._overlay.add_line(self._layer.name, a, b, col, width=width)

    def capsule(self, p0: FloatArray, p1: FloatArray, rgba: FloatArray, radius: float = 0.003) -> None:
        a = p0.astype(np.float64, copy=False)
        b = p1.astype(np.float64, copy=False)
        col = rgba.astype(np.float32, copy=False)
        self._overlay.add_capsule(self._layer.name, a, b, col, radius=radius)

    def arrow(self, p0: FloatArray, p1: FloatArray, rgba: FloatArray, width: float = 0.003) -> None:
        a = p0.astype(np.float64, copy=False)
        b = p1.astype(np.float64, copy=False)
        col = rgba.astype(np.float32, copy=False)
        self._overlay.add_arrow(self._layer.name, a, b, col, width=width)
