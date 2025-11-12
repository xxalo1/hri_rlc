from __future__ import annotations

import numpy as np
import torch

from functools import singledispatch
from typing import Any, overload

from . import numpy_util as npu
from . import pytorch_util as ptu
FloatArray = npu.FloatArray

@overload
def cos(x: FloatArray) -> FloatArray: ...
@overload
def cos(x: torch.Tensor) -> torch.Tensor: ...

@singledispatch
def cos(x: Any):
    raise NotImplementedError(f'cos not implemented for type {type(x)}')

@cos.register(np.ndarray)
def _(x: FloatArray) -> FloatArray:
    return np.cos(x)

@cos.register(torch.Tensor)
def _(x: torch.Tensor) -> torch.Tensor:
    return torch.cos(x)
