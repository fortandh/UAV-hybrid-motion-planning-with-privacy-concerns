import numpy as np
import os

a=np.loadtxt('1.txt')
b=np.reshape(a,(10,10,10))
print(b)
print(type(b))
occ_grid_known_name = "../data/"+"occ_grid-10" + ".npy"
np.save(file=occ_grid_known_name, arr=b)

obstacle = 0
privacy = 0
for i in range (b.shape[0]):
    for j in range (b.shape[1]):
        for k in range (b.shape[2]):
            if b[i][j][k] == 1:
                obstacle += 1
            elif b[i][j][k] == 2 or b[i][j][k] == 3 or b[i][j][k] == 4:
                privacy += 1
obstacle_ratio = obstacle/(b.shape[0]*b.shape[1]*b.shape[2])
privacy_ratio = privacy/(b.shape[0]*b.shape[1]*b.shape[2])
print(obstacle, obstacle_ratio, privacy, privacy_ratio)