import numpy as np
import copy
import math
np.set_printoptions(threshold=np.inf)


p = np.zeros([331, 5], dtype=int)
point = np.loadtxt('exc.txt', delimiter=',', dtype=int)
# print(point)
maplabel = np.loadtxt('maplabel.txt', delimiter=' ', dtype=int)
maplabel_height = copy.deepcopy(maplabel)
maplabel_privacy = np.loadtxt('maplabel_privacy.txt', delimiter=' ', dtype=int)
maplabel_index = np.loadtxt('maplabel_index.txt', delimiter=' ', dtype=int)

# print(maplabel[31][39])
# print(maplabel[0][22])
# print(point.shape[0])
p[:, 0] = point[:, 1]
p[:, 1] = point[:, 0]
p[:, 2] = point[:, 2]
p[:, 3] = point[:, 3] #privacy_level
# print(p)
# print(point[0][1])
A = np.zeros([331, 2], dtype=int)
A[:, 0] = point[:, 1]
A[:, 1] = point[:, 0]
# print(A)
# print(A[5])
# print(maplabel[A[11][0]][A[11][1]])

temp = []
for i in range(331):
    p[i][4] = maplabel[A[i][0]][A[i][1]]
    temp.append(p[i][4])
# print(p[116])
# print(p)
temp.sort()
print(temp)

print(point.shape[0])
'''
for i in range(point.shape[0]):
    x[i]=point[i,0]
    y[i]=point[i,1]
    A=[point[i,0],point[i,1]]
print(x)

'''

count = 0
group_num = 36
building_num = 331

for i in range(1, group_num + 1):
    num = 0
    for j in range(building_num):
        if p[j][4] == i: ## group index
            # print(p[j])
            count += 1
            num = num + p[j][2]
            # print(count, num)
    if count == 0:
        # print(p[j])
        # print(count, num)
        print("nothing")
    else:
        h = num / count
        for x in range(50):
            for y in range(50):
                if maplabel[x][y] == i :
                    maplabel_height[x][y] = round(h)
                    # print("average height",h,i)
        # print(h)
        count = 0
    # maplabel_1 = maplabel

print(maplabel_height)
np.savetxt("maplabel_height.txt",maplabel_height,fmt='%d',delimiter=',')
delta_h = 5
occ_grid_map = np.zeros((20,50,50))
for i in range(50):
    for j in range(50):
        if maplabel_height[i][j] > 0:
            if maplabel_index[i][j] > 0:
                height = point[maplabel_index[i][j]][2] ## 取该点高度
            else:
                height = maplabel_height[i][j]
            # print("filling ",height, i,j)
            height = min(math.ceil(height/delta_h), 19)
            label = maplabel_privacy[i][j]
            for ll in range(height):
                occ_grid_map[ll][i][j] = label


print(occ_grid_map)
np.save("occ_grid_map.npy",occ_grid_map)
# for i in range(50):
#     for j in range(50):
#         if occ_grid_map[i][0][j] == 1 and maplabel_height[i][j] == 0:
#             print ("wrong")
#         if occ_grid_map[i][0][j] == 0 and maplabel_height[i][j] > 0:
#             print("wrong")

# print(h)
