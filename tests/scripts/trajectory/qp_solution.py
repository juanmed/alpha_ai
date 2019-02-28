from cvxopt import matrix, solvers
import numpy as np

import compute_p
import compute_constraint
import keyframe_generation
import draw_trajectory

def qp_solution(order, n, gate, t):
    # generate keyframe
    keyframe_cls = keyframe_generation.KeyframeGeneration()
    keyframe = keyframe_cls.keyframe_generation(gate)

    # compute P, q, A, b
    compute_p_cls = compute_p.ComputeP(6, gate, 1, 1, 4, 2, t)
    P = compute_p_cls.compute_p()
    P = 2 * P
    q = matrix(0.0, (gate*(order+1)*n, 1))
    compute_constraint_cls = compute_constraint.ComputeConstraint(order, gate, 3, 2, t, keyframe, None, None, None)
    A, b = compute_constraint_cls.compute_eq()

    # quadratic programming
    sol = solvers.qp(P, q, None, None, A, b, kktsolver='ldl', options={'kktreg': 1e-9})
    sol_x = sol['x']            # fval = sol['primal objective']
    # np.savetxt("x.csv", sol_x, delimiter=",")

    #draw_trajectory.draw_trajectory(sol_x, order, gate, n, t)
    return sol_x

