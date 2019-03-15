import numpy as np
from cvxopt import matrix, spdiag
import math


class ComputeP:
    def __init__(self, order, m, mu_r, mu_psi, k_r, k_psi, t):
        self.order = order
        self.m = m
        self.mu_r = mu_r
        self.mu_psi = mu_psi
        self.k_r = k_r
        self.k_psi = k_psi
        self.t = t

    def compute_p(self):
        polynomial_r = np.ones(self.order+1)
        for i in range(0, self.k_r):
            polynomial_r = np.polyder(polynomial_r)

        polynomial_psi = np.ones(self.order+1)
        for i in range(0, self.k_psi):
            polynomial_psi = np.polyder(polynomial_psi)

        for i in range(0, self.m):
            p_x = np.zeros((self.order + 1, self.order + 1))
            p_y = np.zeros((self.order + 1, self.order + 1))
            p_z = np.zeros((self.order + 1, self.order + 1))
            p_psi = np.zeros((self.order + 1, self.order + 1))
            for j in range(0, self.order+1):
                for k in range(j, self.order+1):
                    # position
                    if j <= len(polynomial_r)-1 and k <= len(polynomial_r)-1:
                        order_t_r = ((self.order - self.k_r - j) + (self.order - self.k_r - k))
                        if j == k:
                            p_x[j, k] = math.pow(polynomial_r[j], 2) / (order_t_r + 1) * (math.pow(self.t[i + 1],
                                                                                                   (order_t_r + 1)) - math.pow(self.t[i], (order_t_r + 1)))
                            p_y[j, k] = math.pow(polynomial_r[j], 2) / (order_t_r + 1) * (math.pow(self.t[i + 1],
                                                                                                   (order_t_r + 1)) - math.pow(self.t[i], (order_t_r + 1)))
                            p_z[j, k] = math.pow(polynomial_r[j], 2) / (order_t_r + 1) * (math.pow(self.t[i + 1],
                                                                                                   (order_t_r + 1)) - math.pow(self.t[i], (order_t_r + 1)))

                        else:
                            p_x[j, k] = 2 * polynomial_r[j] * polynomial_r[k] / (order_t_r + 1) * (math.pow(self.t[i + 1],
                                                                                                            (order_t_r + 1)) - math.pow(self.t[i], (order_t_r + 1)))
                            p_y[j, k] = 2 * polynomial_r[j] * polynomial_r[k] / (order_t_r + 1) * (math.pow(self.t[i + 1],
                                                                                                            (order_t_r + 1)) - math.pow(self.t[i], (order_t_r + 1)))
                            p_z[j, k] = 2 * polynomial_r[j] * polynomial_r[k] / (order_t_r + 1) * (math.pow(self.t[i + 1],
                                                                                                            (order_t_r + 1)) - math.pow(self.t[i], (order_t_r + 1)))

                    # yaw
                    if j <= len(polynomial_psi) - 1 and k <= len(polynomial_psi) - 1:
                        order_t_psi = ((self.order - self.k_psi - j) + (self.order - self.k_psi - k))
                        if j == k:
                            p_psi[j, k] = math.pow(polynomial_psi[j], 2) / (order_t_psi + 1) * (math.pow(self.t[i + 1],
                                                                                                         (order_t_psi + 1)) - math.pow(self.t[i], (order_t_psi + 1)))
                        else:
                            p_psi[j, k] = 2 * polynomial_psi[j] * polynomial_psi[k] / (order_t_psi + 1) * (math.pow(self.t[i + 1],
                                                                                                                    (order_t_psi + 1)) - math.pow(self.t[i], (order_t_psi + 1)))

            p_x = matrix(p_x) * self.mu_r
            p_y = matrix(p_y) * self.mu_r
            p_z = matrix(p_z) * self.mu_r
            p_psi = matrix(p_psi) * self.mu_psi
            if i == 0:
                p = spdiag([p_x, p_y, p_z, p_psi])
            else:
                p = spdiag([p, p_x, p_y, p_z, p_psi])

        # p = 0.5 * (p + np.transpose(p))
        p = matrix(p)
        p_1 = matrix(np.transpose(p))
        p_2 = (p + p_1) * 0.5
        p_2 = matrix(p_2)
        return p_2
