# gric visualization

from mayavi import mlab

figure = None
path_plot = None


def grid_visualization(occ_grid, starting_point, objectives, path, first_path):
    x = []
    y = []
    z = []
    v = []
    obj_x = []
    obj_y = []
    obj_z = []
    path_x = []
    path_y = []
    path_z = []
    path_x_ = []
    path_y_ = []
    path_z_ = []

    for i in range(len(occ_grid)):
        for j in range(len(occ_grid[i])):
            for k in range(len(occ_grid[i][j])):
                if occ_grid[i][j][k] > 0:
                    x.append(i)
                    y.append(j)
                    z.append(k)
                    v.append(1 - occ_grid[i][j][k])

    #for i in range(len(objectives)):
    #    obj_x.append(objectives[i].x)
    #    obj_y.append(objectives[i].y)
    #    obj_z.append(objectives[i].z)
    obj_x = objectives.x
    obj_y = objectives.y
    obj_z = objectives.z


    for i in range(len(path)):
        path_x.append(path[i].x)
        path_y.append(path[i].y)
        path_z.append(path[i].z)

    for i in range(len(first_path)):
        path_x_.append(first_path[i].x)
        path_y_.append(first_path[i].y)
        path_z_.append(first_path[i].z)

    mlab.figure('Occupancy Grid', bgcolor=(1, 1, 1), fgcolor = (0.2, 0.2, 0.2))
    mlab.points3d(x, y, z, v, mode='cube', colormap='black-white', scale_mode='none', scale_factor='1')
    mlab.points3d(obj_x, obj_y, obj_z, mode='cube', color=(1, 0, 0), scale_mode='none', scale_factor='1')
    mlab.points3d(starting_point.x, starting_point.y, starting_point.z,
                  mode='cube', color=(1, 1, 0), scale_mode='none', scale_factor='1')
    mlab.axes(extent=[0, len(occ_grid), 0, len(occ_grid),0, len(occ_grid)])
    mlab.plot3d(path_x, path_y, path_z, color=(0, 0, 1))
    mlab.plot3d(path_x_, path_y_, path_z_, color=(0, 1, 0))
    mlab.show()

