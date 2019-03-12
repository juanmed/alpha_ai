from cvxopt import matrix, solvers

import compute_p
import compute_constraint
import numpy as np


def qp_solution(order, n, gate, t, keyframe):
    # compute P, q, A, b
    compute_p_cls = compute_p.ComputeP(order, gate, 1, 1, 4, 2, t)
    P = compute_p_cls.compute_p()
    P = 2 * P
    q = matrix(0.0, (gate * (order + 1) * n, 1))
    compute_constraint_cls = compute_constraint.ComputeConstraint(order, gate, 3, 2, t, keyframe, None, None, None)
    A, b = compute_constraint_cls.compute_eq()

    # quadratic programming
    #sol = solvers.qp(P, q, None, None, A, b)
    sol = solvers.qp(P, q, None, None, A, b, kktsolver='ldl', options={'kktreg': 1e-9})
    sol_x = sol['x']  # fval = sol['primal objective']

    # check A,b and rank of A

    # print np.size(A)
    # print np.size(b)
    # print np.linalg.matrix_rank(A)

    # save A, b, x as csv file

    # np.savetxt("A.csv", A, delimiter=",")
    # np.savetxt("b.csv", b, delimiter=",")
    # np.savetxt("x.csv", sol_x, delimiter=",")

    return sol_x

