import numpy as np


def compute_trajectory(solution, order, time):

    solution = np.hstack(solution)

    x_coeff = np.array(solution[0 * (order + 1): 1 * (order + 1)])
    y_coeff = np.array(solution[1 * (order + 1): 2 * (order + 1)])
    z_coeff = np.array(solution[2 * (order + 1): 3 * (order + 1)])
    psi_coeff = np.array(solution[3 * (order + 1): 4 * (order + 1)])

    x_dot_coeff = np.polyder(x_coeff)
    y_dot_coeff = np.polyder(y_coeff)
    z_dot_coeff = np.polyder(z_coeff)
    psi_dot_coeff = np.polyder(psi_coeff)

    x_ddot_coeff = np.polyder(x_dot_coeff)
    y_ddot_coeff = np.polyder(y_dot_coeff)
    z_ddot_coeff = np.polyder(z_dot_coeff)
    psi_ddot_coeff = np.polyder(psi_dot_coeff)

    x_dddot_coeff = np.polyder(x_ddot_coeff)
    y_dddot_coeff = np.polyder(y_ddot_coeff)
    z_dddot_coeff = np.polyder(z_ddot_coeff)

    x_ddddot_coeff = np.polyder(x_dddot_coeff)
    y_ddddot_coeff = np.polyder(y_dddot_coeff)
    z_ddddot_coeff = np.polyder(z_dddot_coeff)

    pos_x = np.polyval(x_coeff, time)
    pos_y = np.polyval(y_coeff, time)
    pos_z = np.polyval(z_coeff, time)
    pos_psi = np.polyval(psi_coeff, time)

    vel_x = np.polyval(x_dot_coeff, time)
    vel_y = np.polyval(y_dot_coeff, time)
    vel_z = np.polyval(z_dot_coeff, time)
    vel_psi = np.polyval(psi_dot_coeff, time)


    acc_x = np.polyval(x_ddot_coeff, time)
    acc_y = np.polyval(y_ddot_coeff, time)
    acc_z = np.polyval(z_ddot_coeff, time)
    acc_psi = np.polyval(psi_ddot_coeff, time)


    jerk_x = np.polyval(x_dddot_coeff, time)
    jerk_y = np.polyval(y_dddot_coeff, time)
    jerk_z = np.polyval(z_dddot_coeff, time)
    #print jerk_x
    #jerk_x, jerk_y, jerk_z = 0.0
    jerk_x = 0.0
    jerk_y = 0.0
    jerk_z = 0.0

    snap_x = np.polyval(x_ddddot_coeff, time)
    snap_y = np.polyval(y_ddddot_coeff, time)
    snap_z = np.polyval(z_ddddot_coeff, time)
    #print snap_x
    snap_x = 0.0
    snap_y = 0.0
    snap_z = 0.0

    pos = np.array([pos_x, pos_y, pos_z])
    vel = np.array([vel_x, vel_y, vel_z])
    acc = np.array([acc_x, acc_y, acc_z])
    jerk = np.array([jerk_x, jerk_y, jerk_z])
    snap = np.array([snap_x, snap_y, snap_z])
    yaw = pos_psi
    yaw_dot = vel_psi
    yaw_ddot = acc_psi

    trajectory = [pos, vel, acc, jerk, snap, yaw, yaw_dot, yaw_ddot]

    return trajectory
