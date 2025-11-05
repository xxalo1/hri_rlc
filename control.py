from calendar import c
import numpy as np
from env import Gen3Env
import mujoco as mj
from mujoco import viewer
import time
delay = 500  # milliseconds
env = Gen3Env(xml_path="world/world.xml", nsubsteps=10, seed=0)
obs = env.reset()

# build a home-ish posture (zeros) and add a small bend on joint_3 and joint_5
target = np.zeros(len(env.act_ids), dtype=float)

target[2] = -0.1   # bend joint_3
target[4] =  0.1   # bend joint_5
target[7] = (env.ctrl_min[7] + env.ctrl_max[7])
print ("Target joint angles (radians):", target)


paused = False

def key_callback(keycode):
  if chr(keycode) == ' ':
    global paused
    paused = not paused

def wait_if_paused():
    while paused and v.is_running():
        time.sleep(0.1)

with viewer.launch_passive(env.m, env.d, key_callback=key_callback) as v:
    while v.is_running():
        wait_if_paused()
        for t in range(5):
            wait_if_paused()
            obs = env.step(target)
            mj.mj_step(env.m, env.d)

            v.sync()
            time.sleep(delay / 1000.0)
        env.reset()
