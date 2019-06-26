import numpy as np

occ_grid_known_name = "occ_grid_known_initial" + str(1) + ".npy"
occ_grid_known = np.load(file=occ_grid_known_name)
print(occ_grid_known)

occ_grid_known_name = "occ_grid_known_initial" + str(2) + ".npy"
occ_grid_known = np.load(file=occ_grid_known_name)
print(occ_grid_known)