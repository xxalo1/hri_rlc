import numpy as np
k, m, n = 4, 2, 4

R = np.arange(k*k*n).reshape(n, k, k)
b = np.arange(n).reshape(n)

print("R:\n", R)
print("b:\n", b)
R[:, :3, 3] += b @ R[:, :3, 2]

print("Updated R:\n", R)