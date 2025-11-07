from dynamics import DH
import numpy as np
from util import to_array

from numpy import pi
import matplotlib.pyplot as plt

                # d        a         alpha         theta0
dh = np.array([[156.4,     0,         0,             0],
               [128.4,     0,       -pi/2,      0],
               [6.4,       0,     pi/2,       0],
               [210.4,     0,       -pi/2,      0],
               [6.4,       0,     pi/2,       0],
               [105.9,     0,         -pi/2,      0],
               [0,         0,     pi/2,       0],
               [61.5,      0,         -pi/2,      0]], dtype=np.float32)


dh_m = np.array([
    [ 0.0,      0.0,  pi,      0.0      ],  # i = 0 (from base)
    [-0.2848,   0.0,  pi/2,    0.0      ],  # i = 1,  q1
    [-0.0118,   0.0,  pi/2,    pi       ],  # i = 2,  q2 + pi
    [-0.4208,   0.0,  pi/2,    pi       ],  # i = 3,  q3 + pi
    [-0.0128,   0.0,  pi/2,    pi       ],  # i = 4,  q4 + pi
    [-0.3143,   0.0,  pi/2,    pi       ],  # i = 5,  q5 + pi
    [ 0.0,      0.0,  pi/2,    pi       ],  # i = 6,  q6 + pi
    [-0.1674,   0.0,  pi,      pi       ],  # i = 7 (to interface), q7 + pi
], dtype=float)

mm2m = 1e-3

b = np.array([0, 5.4, 210.4, 6.4, 208.4, 0, 105.9, 0]) * mm2m # frame offsets in mm along the new z axis

d = dh[:,0] * mm2m
print(d)
a = dh[:,1] * mm2m
theta0 = dh[:,3]
alpha = dh[:,2]

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
fig = plt.figure(figsize=(10,8))
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
ax.view_init(elev=25, azim=35)
# equal aspect
max_range = (O.max(axis=0)-O.min(axis=0)).max()
mid = O.min(axis=0) + 0.5*(O.max(axis=0)-O.min(axis=0))
for i, m in enumerate(['x','y','z']):
    getattr(ax, f'set_{m}lim')(mid[i]-0.5*max_range, mid[i]+0.5*max_range)
plt.tight_layout(); plt.show()