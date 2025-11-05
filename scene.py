import mujoco as mj, importlib, sys, pathlib
print("pip package version:", mj.__version__)
print("runtime engine     :", mj.mj_versionString())
print("python module path :", pathlib.Path(mj.__file__).as_posix())

import mujoco as mj
from mujoco import viewer
m = mj.MjModel.from_xml_path("world/world.xml")
d = mj.MjData(m)

with viewer.launch_passive(m, d) as v:
    while v.is_running():
        mj.mj_step(m,d); v.sync()