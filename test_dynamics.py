from Dynamics.dynamics import DH
import numpy as np
from util import to_array

import matplotlib.pyplot as plt

dh = np.array([[156.4,     0,  0,  0],
               [128.4,     5.4,  np.pi/2,      0],
               [6.4,     210.4,  0,      -np.pi/2],
               [210.4,     6.4,  np.pi/2,      0],
               [6.4,     208.4,  0,      -np.pi/2],
               [105.9,     0,  np.pi/2,      0],
               [0,     105.9,  0,      -np.pi/2],
               [61.5,     0,  0,      0]])

mm2m = 1e-3

d = dh[:,0]
print(d)
a = dh[:,1]
theta0 = dh[:,2]
alpha = dh[:,3]

theta = to_array([0, np.pi/2])
robot = DH(d, a, alpha, theta0)

origins = []
R = []
axesX = []
axesY = []
axesZ = []
T = robot.fk(len(d)-1)

for i in range(len(d)):
    origins.append(T[i][3, :3])
    print(T[i])
    print("\n--*20\n")
    R.append(T[i][:3, :3])
    axesX.append(R[i][:,0].copy())
    axesY.append(R[i][:,1].copy())
    axesZ.append(R[i][:,2].copy())

axis_len=0.08    
fig = plt.figure(figsize=(7,6))
ax = fig.add_subplot(111, projection='3d')

# draw links
O = np.array(origins)
ax.plot(O[:,0], O[:,1], O[:,2], '-k', lw=2)

# draw frames
for k in range(1, len(origins)):
    o = origins[k]
    ax.quiver(*o, *(axis_len*axesX[k-1]), color='r')  # x
    ax.quiver(*o, *(axis_len*axesY[k-1]), color='g')  # y
    ax.quiver(*o, *(axis_len*axesZ[k-1]), color='b')  # z

ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
ax.set_title('Gen3 DH frames (home)')
ax.view_init(elev=25, azim=35)
# equal aspect
max_range = (O.max(axis=0)-O.min(axis=0)).max()
mid = O.min(axis=0) + 0.5*(O.max(axis=0)-O.min(axis=0))
for i, m in enumerate(['x','y','z']):
    getattr(ax, f'set_{m}lim')(mid[i]-0.5*max_range, mid[i]+0.5*max_range)
plt.tight_layout(); plt.show()