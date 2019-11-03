import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np

# y_size = 20
x_size = 20
# world_array = np.zeros([y_size, x_size])
# drawn = False
#
# def main():
#     global world_array
#     for i in range(0, y_size):
#         for j in range(0, x_size):
#             world_array[i, j] = 1
#             display_map(0)
#
# def display_map(x):
#     global drawn
#     global world_array
#     cmap = colors.ListedColormap(['blue', 'red'])
#     bounds=[0,0.5,1]
#
#     norm = colors.BoundaryNorm(bounds, cmap.N)
#     plt.imshow(world_array, interpolation='nearest', origin='lower', cmap=cmap, norm=norm)
#
#     plt.draw()
#     plt.pause(0.0001)
#     plt.clf()
def main():
    for i in range(80):
        print(cell_index_to_ij(i))

def ij_to_cell_index(i,j):
    #DONE: Convert from i,j coordinates to a single integer that identifies a grid cell
    return j + i * x_size


def cell_index_to_ij(cell_index):
    #DONE: Convert from cell_index to (i,j) coordinates
    i = '%.0f'%(cell_index / x_size)
    j = cell_index % x_size
    return i, j


def cost(cell_index_from, cell_index_to):
    #TODO: Return cost of traversing from one cell to another
    # This should be an admissible heuristic. Since this doesn't account for obsticles
    # the manhattan distance should always be optimistic.
    start_i, start_j = cell_index_to_ij(cell_index_from)
    dest_i, dest_j = cell_index_to_ij(cell_index_to)
    return abs(start_i - dest_i) + abs(start_j - dest_j)


if __name__ == "__main__":
    main()
