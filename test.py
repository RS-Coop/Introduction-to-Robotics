import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np

y_size = 20
x_size = 20
world_array = np.zeros([y_size, x_size])
drawn = False

def main():
    global world_array
    for i in range(0, y_size):
        for j in range(0, x_size):
            world_array[i, j] = 1
            display_map(0)

def display_map(x):
    global drawn
    global world_array
    cmap = colors.ListedColormap(['blue', 'red'])
    bounds=[0,0.5,1]

    norm = colors.BoundaryNorm(bounds, cmap.N)
    plt.imshow(world_array, interpolation='nearest', origin='lower', cmap=cmap, norm=norm)

    plt.draw()
    plt.pause(0.0001)
    plt.clf()


if __name__ == "__main__":
    main()

