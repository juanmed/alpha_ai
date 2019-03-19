from cvxopt import matrix, solvers

import compute_p
import compute_constraint
import numpy as np


def qp_solution(order, n, waypoint, t, keyframe, current_state):
    # change format of keyframe
    # m : segment number
    keyframe = np.transpose(keyframe)
    m = waypoint - 1

    # compute P, q
    compute_p_cls = compute_p.ComputeP(order, m, 1, 1, 4, 2, t)
    P = compute_p_cls.compute_p()
    P = 2 * P
    q = matrix(0.0, (m * (order + 1) * n, 1))

    # compute equality constraint: A,b
    # k_r = 4, k_psi = 2 : continuity until jerk and angular acceleration
    # current state such as velocity, acceleration
    k_r = 4
    k_psi = 2
    compute_constraint_cls = compute_constraint.ComputeConstraint(order, m, k_r, k_psi, t, keyframe, current_state)
    A, b = compute_constraint_cls.compute_eq()

    # compute inequality constraint : G,h
    max_vel = 1
    max_acc = 0
    max_angular_vel = 0.1
    max_angular_acc = 0
    #G1, h1 = compute_constraint_cls.compute_in(max_vel, max_acc, max_angular_vel, max_angular_acc)

    # corridor_position : corridor constraint area setting
    #corridor_position = np.array([1, 2])        # None
    n_intermediate = 5
    corridor_width = 0.5
    #G2, h2 = compute_constraint_cls.compute_cr(corridor_position, n_intermediate, corridor_width)

    #G = matrix([G1, G2])
    #h = matrix([h1, h2])

    # check A,b and rank of A

    print np.size(A)
    print np.linalg.matrix_rank(A)

    # quadratic programming
    #sol = solvers.qp(P, q, G1, h1, A, b)
    sol = solvers.qp(P, q, None, None, A, b, kktsolver='ldl', options={'kktreg': 1e-9})
    sol_x = sol['x']  # fval = sol['primal objective']

    # save as csv file
    # np.savetxt("A.csv", A, delimiter=",")
    # np.savetxt("b.csv", b, delimiter=",")
    #np.savetxt("G1.csv", G1, delimiter=",")
    #np.savetxt("h1.csv", h1, delimiter=",")
    #np.savetxt("x.csv", sol_x, delimiter=",")

    return sol_x

