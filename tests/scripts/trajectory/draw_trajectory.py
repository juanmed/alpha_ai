import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def draw_trajectory(solution, order, m, n, t):
    x_trajec = []
    y_trajec = []
    z_trajec = []
    psi_trajec = []

    for i in range(0, m):
        # we can use np.arrange instead of np.linspace
        x_trajec = np.append(x_trajec, np.polyval(
            solution[i * n * (order + 1) + 0 * (order + 1): i * n * (order + 1) + (order + 1) + 0 * (order + 1)],
            np.linspace(t[i], t[i + 1], 50)))
        y_trajec = np.append(y_trajec, np.polyval(
            solution[i * n * (order + 1) + 1 * (order + 1): i * n * (order + 1) + (order + 1) + 1 * (order + 1)],
            np.linspace(t[i], t[i + 1], 50)))
        z_trajec = np.append(z_trajec, np.polyval(
            solution[i * n * (order + 1) + 2 * (order + 1): i * n * (order + 1) + (order + 1) + 2 * (order + 1)],
            np.linspace(t[i], t[i + 1], 50)))
        psi_trajec = np.append(psi_trajec, np.polyval(
            solution[i * n * (order + 1) + 3 * (order + 1): i * n * (order + 1) + (order + 1) + 3 * (order + 1)],
            np.linspace(t[i], t[i + 1], 50)))

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(x_trajec, y_trajec, z_trajec)
    ax.set_xlim(-60, 60)
    ax.set_ylim(-60, 60)
    ax.set_zlim(-60, 60)
    ax.set_xlabel('x label')
    ax.set_ylabel('y label')
    ax.set_zlabel('z label')
    ax.text(0, 0, 0, "0", color='red')
    ax.text(17.25, 3.63, 6.31, "1", color='red')
    ax.text(16.75, 39.60, 6.76, "2", color='red')
    ax.text(2.33, 27.87, 2.55, "3", color='red')
    ax.text(2.20, 9.00, 1.99, "4", color='red')
    ax.text(-7.31, -12.1, 3.23, "5", color='red')
    ax.text(-7.61, -28.5, 2.64, "6", color='red')
    ax.text(0.00, -33.9, 2.10, "7", color='red')
    ax.text(5.87, -28.44, 2.57, "8", color='red')
    ax.text(7.25, -11.79, 2.55, "9", color='red')
    ax.text(-10.23, 7.77, 2.10, "10", color='red')
    ax.text(-10.05, 30.62, 2.90, "11", color='red')
    plt.show()