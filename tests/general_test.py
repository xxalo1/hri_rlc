from ..kinova_gen3 import kinova_gen3 as kg3
from ..utils import numpy_util as npu
mydict = kg3.load_inertia()
dhdict = kg3.load_dh()
print("inertia: \n", mydict)

print("dh: \n", dhdict)