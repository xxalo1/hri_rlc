from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np
from numpy.typing import ArrayLike, NDArray


from common_utils import FloatArray
from common_utils import numpy_util as npu


@dataclass
class RingBuffer:
    """
    Fixed size ring buffer for time-stamped numeric data.

    Usage:
        buf = RingBuffer(capacity=20_000)          # shape inferred on first append
        buf.append(t, sample)                      # sample can be scalar or vector
        t_all, x_all = buf.all()
        t_win, x_win = buf.window(t0, t1)
        t_last, x_last = buf.last_n(500)
    """

    capacity: int
    sample_shape: tuple[int, ...] | None = None

    _t: FloatArray = field(init=False, repr=False)
    _x: FloatArray = field(init=False, repr=False)
    _i: int = field(default=0, init=False, repr=False)
    _full: bool = field(default=False, init=False, repr=False)
    _initialized: bool = field(default=False, init=False, repr=False)


    def __post_init__(self) -> None:
        if self.capacity <= 0:
            raise ValueError("capacity must be positive")

        self._t = np.zeros(self.capacity, dtype=npu.dtype)

        if self.sample_shape is not None:
            self._x = np.zeros((self.capacity, *self.sample_shape), dtype=npu.dtype)
            self._initialized = True
        else:
            # temporary placeholder; real array allocated on first append
            self._x = np.zeros((self.capacity,), dtype=npu.dtype)


    def _ensure_initialized(self, sample: ArrayLike) -> None:
        if self._initialized:
            return
        arr = np.asarray(sample, dtype=npu.dtype)
        self.sample_shape = arr.shape
        self._x = np.zeros((self.capacity, *self.sample_shape), dtype=npu.dtype)
        self._initialized = True


    def _indices(self) -> NDArray[np.int64]:
        """Return indices in chronological order (oldest to newest)."""
        if not self._full:
            return np.arange(self._i, dtype=np.int64)
        first = np.arange(self._i, self.capacity, dtype=np.int64)
        second = np.arange(0, self._i, dtype=np.int64)
        return np.concatenate([first, second])


    def append(self, t: float, sample: ArrayLike) -> None:
        """Add one time-stamped sample."""
        if not self._initialized:
            self._ensure_initialized(sample)

        arr = np.asarray(sample, dtype=npu.dtype)
        if arr.shape != self.sample_shape:
            raise ValueError(
                f"Sample has shape {arr.shape}, expected {self.sample_shape}"
            )

        self._t[self._i] = float(t)
        self._x[self._i] = arr

        self._i += 1
        if self._i >= self.capacity:
            self._i = 0
            self._full = True


    def __len__(self) -> int:
        return self.capacity if self._full else self._i


    def is_empty(self) -> bool:
        return len(self) == 0


    def all(self) -> tuple[FloatArray, FloatArray]:
        """Return all samples as (t, x) in time order."""
        if self.is_empty():
            return self._t[:0].copy(), self._x[:0].copy()

        idx = self._indices()
        return self._t[idx].copy(), self._x[idx].copy()


    def window(
        self,
        t_start: float | None = None,
        t_end: float | None = None,
    ) -> tuple[FloatArray, FloatArray]:
        """
        Return samples with t in [t_start, t_end].

        If t_start is None: no lower bound.
        If t_end is None: no upper bound.
        """
        if self.is_empty():
            return self._t[:0].copy(), self._x[:0].copy()

        idx = self._indices()
        t = self._t[idx]

        mask = np.ones_like(t, dtype=bool)
        if t_start is not None:
            mask &= t >= t_start
        if t_end is not None:
            mask &= t <= t_end

        sel = idx[mask]
        return self._t[sel].copy(), self._x[sel].copy()


    def last_n(self, n: int) -> tuple[FloatArray, FloatArray]:
        """Return the last n samples (or fewer if buffer is smaller)."""
        n = max(0, min(n, len(self)))
        if n == 0 or self.is_empty():
            return self._t[:0].copy(), self._x[:0].copy()

        idx = self._indices()
        sel = idx[-n:]
        return self._t[sel].copy(), self._x[sel].copy()


    def clear(self) -> None:
        """Forget all stored samples. Capacity stays the same."""
        self._i = 0
        self._full = False
        # do not bother zeroing arrays; they will be overwritten


    def to_dict(self, name: str) -> dict[str, FloatArray]:
        """
        Helper for snapshots.

        Returns something like:
            {f"{name}/t": t_array, name: x_array}
        so you can pass it to np.savez_compressed(**arrays).
        """
        t, x = self.all()
        return {f"{name}/t": t, name: x}


@dataclass
class BufferSet:
    """
    Collection of named RingBuffers.

    Meant for monitor nodes:
        buffers = BufferSet()
        q_buf = buffers.ensure("joint_state/q", capacity=20_000, sample_shape=(7,))
        # later in callback:
        buffers.append("joint_state/q", t, q)

        # snapshot:
        buffers.save_npz("logs/gen3_monitor/run_20251203_073012.npz")
    """

    buffers: dict[str, RingBuffer] = field(default_factory=dict)

    def ensure(
        self,
        name: str,
        capacity: int,
        sample_shape: tuple[int, ...] | None = None,
    ) -> RingBuffer:
        """Get existing buffer by name or create it."""
        buf = self.buffers.get(name)
        if buf is not None:
            return buf
        buf = RingBuffer(capacity=capacity, sample_shape=sample_shape)
        self.buffers[name] = buf
        return buf


    def append(self, name: str, t: float, sample: ArrayLike) -> None:
        """Append to buffer, creating it on first use with inferred shape."""
        buf = self.buffers.get(name)
        if buf is None:
            # default capacity; you can tune or override by calling ensure first
            buf = RingBuffer(capacity=20_000)
            self.buffers[name] = buf
        buf.append(t, sample)


    def get(self, name: str) -> RingBuffer:
        return self.buffers[name]


    def save_npz(self, path: str) -> str:
        """Save all buffers into a single .npz file."""
        arrays: dict[str, FloatArray] = {}
        for name, buf in self.buffers.items():
            arrays.update(buf.to_dict(name))
        np.savez_compressed(path, **arrays)
        return path
