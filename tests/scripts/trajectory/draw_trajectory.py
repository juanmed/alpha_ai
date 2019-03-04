import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def draw_trajectory(solution, order, gate, n, t, keyframe):
    x_trajec = []
    y_trajec = []
    z_trajec = []
    psi_trajec = []

    for i in range(0, gate):
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

    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_trajec, y_trajec, z_trajec, 'r')
    ax.set_xlim(-60, 60)
    ax.set_ylim(-60, 60)
    ax.set_zlim(-60, 60)
    ax.set_xlabel('x label')
    ax.set_ylabel('y label')
    ax.set_zlabel('z label')
    keyframe = np.transpose(keyframe)
    for i in range(0, len(keyframe)):
        ax.text(keyframe[i][0], keyframe[i][1], keyframe[i][2], i, color='red')
