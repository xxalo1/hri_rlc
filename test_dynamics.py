from Dynamics.dynamics import DH
import numpy as np
from util import to_array

import matplotlib.pyplot as plt

                # d        a         alpha         theta0
dh = np.array([[156.4,     0,         0,             0],
               [128.4,     0,       -np.pi/2,      0],
               [6.4,       0,     np.pi/2,       0],
               [210.4,     0,       -np.pi/2,      0],
               [6.4,       0,     np.pi/2,       0],
               [105.9,     0,         -np.pi/2,      0],
               [0,         0,     np.pi/2,       0],
               [61.5,      0,         -np.pi/2,      0]])
mm2m = 1e-3

b = np.array([0, 5.4, 210.4, 6.4, 208.4, 0, 105.9, 0]) * mm2m # frame offsets in mm along the new z axis

d = dh[:,0] * mm2m
print(d)
a = dh[:,1] * mm2m
theta0 = dh[:,2]
alpha = dh[:,3]

o_0 = np.array([0, 0, 0], dtype=np.float32)
z_0 = np.array([0, 0, 1], dtype=np.float32)
x_0 = np.array([1, 0, 0], dtype=np.float32)
y_0 = np.array([0, 1, 0], dtype=np.float32)
theta = to_array([0, np.pi/2])
robot = DH(d, a, alpha, theta0, o_0=o_0, z_0=z_0, b=b)

origins = [o_0]
R = []
axesX = [x_0]
axesY = [y_0]
axesZ = [z_0]
T = robot.fk(len(d)-1)

for i in range(len(d)):
    origins.append(T[i][:3, 3])
    print(T[i])
    print("\n--*20\n")
    R.append(T[i][:3, :3])
    axesX.append(R[i][:,0].copy())
    axesY.append(R[i][:,1].copy())
    axesZ.append(R[i][:,2].copy())

axis_len=0.013    
fig = plt.figure(figsize=(7,6))
ax = fig.add_subplot(111, projection='3d')

# draw links
O = np.array(origins)
ax.plot(O[:,0], O[:,1], O[:,2], '-k', lw=2)

# draw frames
for k in range(len(origins)):
    o = origins[k]
    ax.quiver(*o, *(axis_len*axesX[k]), color='r')  # x
    ax.quiver(*o, *(axis_len*axesY[k]), color='g')  # y
    ax.quiver(*o, *(axis_len*axesZ[k]), color='b')  # z

ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
ax.set_title('Gen3 DH frames (home)')
plt.show()