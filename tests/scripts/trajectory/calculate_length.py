import numpy as np


def calculate_length(solution, order, waypoint, n, t):
    length = 0

    # divided by n point, so it becomes n-1 segment
    n_point = 50

    for i in xrange(0, waypoint - 1):
        x_trajec = np.polyval(
            solution[i * n * (order + 1) + 0 * (order + 1): i * n * (order + 1) + (order + 1) + 0 * (order + 1)],
            np.linspace(t[i], t[i + 1], n_point))
        y_trajec = np.polyval(
            solution[i * n * (order + 1) + 1 * (order + 1): i * n * (order + 1) + (order + 1) + 1 * (order + 1)],
            np.linspace(t[i], t[i + 1], n_point))
        z_trajec = np.polyval(
            solution[i * n * (order + 1) + 2 * (order + 1): i * n * (order + 1) + (order + 1) + 2 * (order + 1)],
            np.linspace(t[i], t[i + 1], n_point))

        # calculate distance for each n-1 segment
        for j in xrange(0, n_point-1):
            x_length = x_trajec[j+1] - x_trajec[j]
            y_length = y_trajec[j+1] - y_trajec[j]
            z_length = z_trajec[j + 1] - z_trajec[j]
            length = length + np.sqrt(x_length**2 + y_length**2 + z_length**2)

    return length

