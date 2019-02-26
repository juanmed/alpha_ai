#!/usr/bin/env python
import rospy
import compute_p
import compute_constraint
import numpy as np
from cvxopt import matrix, solvers
import keyframe_generation
import compute_trajectory
import draw_trajectory
import df_flat


rospy.init_node('trajectory_test1', anonymous=True)
last_time = rospy.get_time()
start_time = last_time
rate = rospy.Rate(10)

# parameter
order = 6
n = 4
gate = 11
m = gate                # without returning
time_interval = 2
t = np.linspace(0, time_interval*m, m+1)

keyframe_cls = keyframe_generation.KeyframeGeneration()
keyframe = keyframe_cls.keyframe_generation(gate)

# compute P, q, A, b
computep_cls = compute_p.ComputeP(6, m, 1, 1, 4, 2, t)
P = computep_cls.compute_p()
P = 2 * P
q = matrix(0.0, (m*(order+1)*n, 1))
computeConstraint_cls = compute_constraint.ComputeConstraint(order, m, 3, 2, t, keyframe, None, None, None)
A, b = computeConstraint_cls.compute_eq()

# quadratic programming
sol = solvers.qp(P, q, None, None, A, b, kktsolver='ldl', options={'kktreg': 1e-9})
sol_x = sol['x']            #  fval = sol['primal objective']
#np.savetxt("x.csv", sol_x, delimiter=",")

#draw_trajectory.draw_trajectory(sol_x, order, m, n, t)

i = 0
'''
time = 0
x = sol_x[n*(order+1)*i: n*(order+1)*(i+1)]
trajectory = compute_trajectory.compute_trajectory(x, order, time)
state_input = df_flat.compute_ref(trajectory)             # Juan's function
'''
while not rospy.is_shutdown():
    now_time = rospy.get_time()
    time = now_time - start_time
    x = sol_x[n*(order+1)*i: n*(order+1)*(i+1)+1]
    trajectory = compute_trajectory.compute_trajectory(x, order, time)
    state_input = df_flat.compute_ref(trajectory)             # Juan's function
    # print state_input
    if now_time - last_time > time_interval:
        i = i + 1
        last_time = now_time

    if i == m:
        print time
        exit()

    rate.sleep()
