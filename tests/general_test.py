import torch
import numpy as np

from ..kinova_gen3 import kinova_gen3 as kg3
from ..utils import numpy_util as npu
from ..utils import pytorch_util as ptu
from ..robot import Kinematics, Dynamics
ptu.init_gpu()

inertia_numpy = kg3.load_inertia()
dh_numpy = kg3.load_dh()

inertia_torch: dict[str, torch.Tensor] = {}
dh_torch: dict[str, torch.Tensor] = {}
for key in inertia_numpy:
    inertia_torch[key] = ptu.from_numpy(inertia_numpy[key])
for key in dh_numpy:
    dh_torch[key] = ptu.from_numpy(dh_numpy[key])

assert isinstance(dh_torch['d'], torch.Tensor), "dh_torch['d'] is not a torch Tensor"
assert isinstance(inertia_torch['m'], torch.Tensor), "inertia_torch['m'] is not a torch Tensor"

T_0 = torch.eye(4, dtype=ptu.dtype, device=ptu.device)
kin = Kinematics(dh=dh_torch, o_wb = T_0[3:, 0:3], axes_wb = T_0[0:3, 0:3], inertia=inertia_torch)
dyn = Dynamics(kin=kin, g_vec=torch.tensor([0, 0, -9.81]))

