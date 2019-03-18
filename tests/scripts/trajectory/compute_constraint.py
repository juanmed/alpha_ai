import numpy as np
from cvxopt import matrix


# TODO : velocity and acceleration constraint after update.
class ComputeConstraint:
    def __init__(self, order, m, k_r, k_psi, t, key_frame, current_state, corridor_position=0, n_intermediate=0, corridor_width=0):
        # k_r, k_psi = set continuity iteration
        self.order = order
        self.m = m
        self.k_r = k_r
        self.k_psi = k_psi
        self.t = t
        self.keyframe = key_frame
        self.current_state = current_state
        self.corridor_position = corridor_position
        self.n_intermediate = n_intermediate
        self.corridor_width = corridor_width
        self.n = 4

    def constraint_data(self):
        # constraint_data_r[m, k_r]
        # 0.0 -> zero ... In this case, Initial Final velocity and acceleration is zero.
        # 1   -> continuity             In Checkpoint, it must be continuous.
        constraint_data_r = np.zeros(shape=(self.m, self.k_r))
        if self.k_r >= 1:
            constraint_data_r[0, 0] = 0.0
            constraint_data_r[1:self.m, 0] = 1
        if self.k_r >= 2:
            constraint_data_r[0, 1] = 0.0
            constraint_data_r[1:self.m, 1] = 1
        if self.k_r >= 3:
            constraint_data_r[0, 2] = 0.0
            constraint_data_r[1:self.m, 2] = 1
        if self.k_r >= 4:
            constraint_data_r[0, 3] = 0.0
            constraint_data_r[1:self.m, 3] = 1

        constraint_data_psi = np.zeros(shape=(self.m, self.k_psi))
        if self.k_psi >= 1:
            constraint_data_psi[0, 0] = 0.0
            constraint_data_psi[1:self.m, 0] = 1
        if self.k_psi >= 2:
            constraint_data_psi[0, 1] = 0.0
            constraint_data_psi[1:self.m, 1] = 1

        return constraint_data_r, constraint_data_psi

    def compute_eq(self):
        A1 = np.zeros((2 * self.m * self.n, self.n * (self.order + 1) * self.m))
        b1 = np.ones(2 * self.m * self.n)
        compute_mat = np.eye(self.order + 1)

        # Way_point Constraint
        for i in range(0, self.m):
            way_point = self.keyframe[:, i]

            if i == 0:
                # Initial
                values = np.zeros(self.order+1)
                for j in range(0, self.order+1):
                    values[j] = np.polyval(compute_mat[j, :], self.t[i])
                for k in range(0, self.n):
                    a = np.zeros(self.n * (self.order + 1) * self.m)
                    a[i * (self.order+1) * self.n + k * (self.order + 1): i * (self.order+1) * self.n + k * (self.order + 1) + self.order + 1] = values
                    A1[k, :] = a
                b1[0: self.n] = way_point

                # Final
                for j in range(0, self.order + 1):
                    values[j] = np.polyval(compute_mat[j, :], self.t[self.m])
                for k in range(0, self.n):
                    a = np.zeros(self.n * (self.order + 1) * self.m)
                    a[(self.m-1) * (self.order+1) * self.n + k * (self.order + 1): (self.m - 1) * (self.order+1) * self.n + k * (self.order + 1) + self.order + 1] = values
                    A1[k + self.n, :] = a
                b1[self.n: 2 * self.n] = self.keyframe[:, self.m]

            else:
                # Elsewhere
                values = np.zeros(self.order+1)
                for j in range(0, self.order+1):
                    values[j] = np.polyval(compute_mat[j, :], self.t[i])
                for k in range(0, self.n):
                    a = np.zeros(self.n * (self.order + 1) * self.m)
                    a[(i - 1) * (self.order + 1) * self.n + k * (self.order+1): (i-1)*(self.order + 1)*self.n + k*(self.order+1)+self.order + 1] = values
                    A1[k + 2*self.n*i, :] = a
                b1[(2*self.n*i): (2*self.n*i)+self.n] = way_point

                for k in range(0, self.n):
                    a = np.zeros(self.n * (self.order + 1) * self.m)
                    a[i * (self.order + 1) * self.n + k * (self.order + 1): i * (self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = values
                    A1[k+2*self.n * i + self.n, :]= a
                b1[(2 * self.n * i) + self.n: (2 * self.n * i) + 2 * self.n] = way_point

        A1 = matrix(A1)
        b1 = matrix(b1)

        # Derivative Constraint
        constraint_data_r, constraint_data_psi = self.constraint_data()

        # position, yaw excluded here (n-1)
        A2 = np.zeros((2 * self.m * (self.n - 1) * self.k_r, self.n * (self.order + 1) * self.m))
        b2 = np.ones((2 * self.m * (self.n - 1) * self.k_r, 1)) * 0.001

        for i in range(0, self.m):
            for h in range(0, self.k_r):
                if i == 0:
                    # Initial
                    values = np.zeros(self.order+1)
                    for j in range(0, self.order+1):       # continuity for velocity acceleration jerk
                        tempCoeffs = compute_mat[j, :]
                        for k in range(0, h+1):
                            tempCoeffs = np.polyder(tempCoeffs)
                        values[j] = np.polyval(tempCoeffs, self.t[i])

                    continuity = 0.0
                    if constraint_data_r[i, h] == 1:
                        # Continuity
                        continuity = 1.0  # True

                    for k in range(0, self.n - 1):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        if continuity == 1.0:
                            a[i*(self.order+1)*self.n + k * (self.order + 1): i * (self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = values
                            a[(self.m-1) * (self.order + 1) * self.n + k * (self.order+1):  (self.m -1) * (self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = -values
                            A2[k + h*(self.n-1), :] = a
                            b2[k + h*(self.n-1)] = 0
                        else:
                            a[i*(self.order+1)*self.n + k * (self.order + 1): i * (self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = values
                            A2[k + h*(self.n-1), :] = a
                            b2[k + h*(self.n-1)] = self.current_state[h+1][k]        # constraint_data_r[i, h]


                    #Final
                    values = np.zeros(self.order + 1)
                    for j in range(0, self.order+1):
                        tempCoeffs = compute_mat[j, :]
                        for k in range(0, h+1):
                            tempCoeffs = np.polyder(tempCoeffs)
                        values[j] = np.polyval(tempCoeffs, self.t[self.m])

                    continuity = 0.0
                    if constraint_data_r[i, h] == 1:
                        continuity = 1.0
                        # Continuity

                    for k in range(0, self.n - 1):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        if continuity != 1.0:
                            a[(self.m-1)*(self.order+1)*self.n + k*(self.order+1): (self.m-1)*(self.order+1)*self.n+k*(self.order+1)+self.order + 1] = values
                            A2[k + h*(self.n-1) + (self.n-1)*self.k_r, :] = a
                            b2[k + h*(self.n-1) + (self.n-1)*self.k_r] = constraint_data_r[i, h]

                else:
                    # Elsewhere
                    values = np.zeros(self.order+1)
                    for j in range(0, self.order+1):       # continuity for velocity acceleration jerk
                        tempCoeffs = compute_mat[j, :]
                        for k in range(0 , h+1):
                            tempCoeffs = np.polyder(tempCoeffs)
                        values[j] = np.polyval(tempCoeffs, self.t[i])

                    continuity = 0.0
                    if constraint_data_r[i, h] == 1:
                        # Continuity
                        continuity = 1.0

                    for k in range(0, self.n - 1):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        if continuity == 1.0:
                            a[(i-1)*(self.order+1)*self.n + k * (self.order + 1): (i-1) * (self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = values
                            a[i * (self.order + 1) * self.n + k * (self.order+1): i * (self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = -values
                            A2[k + h*(self.n-1) + 2*i*(self.n-1)*self.k_r, :] = a
                            b2[k + h*(self.n-1) + 2*i*(self.n-1)*self.k_r] = 0
                        else:
                            a[(i-1)*(self.order+1)*self.n + k * (self.order + 1): (i-1) * (self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = values
                            A2[k + h*(self.n-1) + 2*i*(self.n-1)*self.k_r, :] = a
                            b2[k + h*(self.n-1) + 2*i*(self.n-1)*self.k_r] = constraint_data_r[i, h]

                    for k in range(0, self.n - 1):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        if continuity != 1.0:
                            a[i * (self.order+1) * self.n + k * (self.order+1): i * (self.order+1) * self.n + k * (self.order+1) + self.order + 1] = values
                            A2[k + h*(self.n - 1) + 2*i*(self.n -1)*self.k_r + (self.n-1)*self.k_r, :] = a
                            b2[k + h*(self.n - 1) + 2*i*(self.n -1)*self.k_r + (self.n-1)*self.k_r] = constraint_data_r[i, h]

        A2 = matrix(A2)
        b2 = matrix(b2)

        A = matrix([A1, A2])
        b = matrix([b1, b2])

        # Yaw
        A3 = np.zeros((2 * self.m * self.k_psi, self.n * (self.order + 1) * self.m))
        b3 = np.ones(2 * self.m * self.k_psi) * 0.001

        for i in range(0, self.m):
            for h in range(0, self.k_psi):
                if i == 0:
                    # Initial
                    values = np.zeros(self.order + 1)
                    for j in range(0, self.order + 1):
                        tempCoeffs = compute_mat[j, :]
                        for k in range(0, h+1):
                            tempCoeffs = np.polyder(tempCoeffs)
                        values[j] = np.polyval(tempCoeffs, self.t[i])

                    continuity = 0.0
                    if constraint_data_psi[i, h] == 1:
                        # Continuity
                        continuity = 1.0  # True

                    for k in range(0, 1):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        if continuity == 1.0:
                            a[i * (self.order + 1) * self.n + (k + 3) * (self.order + 1): i * (self.order + 1) * self.n + (k + 3) * (self.order + 1) + self.order + 1] = values
                            a[(self.m-1) * (self.order + 1) * self.n + (k+3) * (self.order+1): (self.m-1) * (self.order + 1) * self.n + (k + 3) * (self.order + 1) + self.order + 1] = -values
                            A3[k + h * 1, :] = a
                            b3[k + h * 1] = 0
                        else:
                            a[i * (self.order + 1) * self.n + (k + 3) * (self.order + 1): i * (self.order + 1) * self.n + (k + 3) * (self.order + 1) + self.order + 1] = values
                            A3[k + h * 1, :] = a
                            b3[k + h * 1] = self.current_state[h+1][k+3]                  #constraint_data_psi[i, h]

                    # Final
                    values = np.zeros(self.order + 1)
                    for j in range(0, self.order + 1):
                        tempCoeffs = compute_mat[j, :]
                        for k in range(0, h + 1):
                            tempCoeffs = np.polyder(tempCoeffs)
                        values[j] = np.polyval(tempCoeffs, self.t[self.m])

                    continuity = 0.0
                    if constraint_data_psi[i, h] == 1:
                        # Continuity
                        continuity = 1.0  # True

                    for k in range(0, 1):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        if continuity != 1.0:
                            a[(self.m - 1) * (self.order + 1) * self.n + (k + 3) * (self.order + 1):
                              (self.m - 1) * (self.order + 1) * self.n + (k + 3) * (self.order + 1) + self.order + 1] = values
                            A3[k + h * 1 + 1 * self.k_psi, :] = a
                            b3[k + h * 1 + 1 * self.k_psi] = constraint_data_psi[i, h]

                else:

                    # Elsewhere
                    values = np.zeros(self.order + 1)
                    for j in range(0, self.order + 1):
                        tempCoeffs = compute_mat[j, :]
                        for k in range(0, h + 1):
                            tempCoeffs = np.polyder(tempCoeffs)
                        values[j] = np.polyval(tempCoeffs, self.t[i])

                    continuity = 0.0
                    if constraint_data_psi[i, h] == 1:
                        # Continuity
                        continuity = 1.0  # True

                    for k in range(0, 1):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        if continuity == 1.0:
                            a[(i - 1) * (self.order + 1) * self.n + (k + 3) * (self.order + 1): (i - 1) * (self.order + 1) * self.n + (k + 3) * (self.order + 1) + self.order + 1] = values
                            a[(i) * (self.order + 1) * self.n + (k + 3) * (self.order + 1): (i) * (self.order + 1) * self.n + (k + 3) * (self.order + 1) + self.order + 1] = -values
                            A3[k + h * 1 + 2 * i * 1 * self.k_psi, :] = a
                            b3[k + h * 1 + 2 * i * 1 * self.k_psi] = 0
                        else:
                            a[(i-1) * (self.order + 1) * self.n + (k + 3) * (self.order + 1): (i-1) * (self.order + 1) * self.n + (k+3) * (self.order + 1) + self.order + 1] = values
                            A3[k + h * 1 + 2 * i * 1 * self.k_psi, :] = a
                            b3[k + h * 1 + 2 * i * 1 * self.k_psi] = constraint_data_psi[i, h]

                    for k in range(0, 1):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        if continuity != 1.0:
                            a[i * (self.order + 1) * self.n + (k + 3) * (self.order + 1):
                              i * (self.order + 1) * self.n + (k + 3) * (self.order + 1) + self.order + 1] = values
                            A3[k + h * 1 + 2 * i * 1 * self.k_psi + 1 * self.k_psi, :] = a
                            b3[k + h * 1 + 2 * i * 1 * self.k_psi + 1 * self.k_psi] = constraint_data_psi[i, h]

        A3 = matrix(A3)
        b3 = matrix(b3)

        A = matrix([A, A3])
        b = matrix([b, b3])

        i = 0
        while 1:
            if b[i] == 0.001:
                b = np.delete(b, i, axis=0)
                A = np.delete(A, i, axis=0)
                i = i - 1
            else:
                i = i + 1
            length = len(b)

            if i == length:
                break

        A = matrix(A)
        b = matrix(b)

        return A, b

    def compute_cr_constraint(self):
        return



