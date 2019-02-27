#!/usr/bin/env python
import rospy
from tests.msg import UAV_traj

import numpy as np
import tf

import compute_trajectory
import df_flat
import parameter
import qp_solution


# parameter
order = parameter.order
n = parameter.n
gate = parameter.gate
t = parameter.t

sol_x = qp_solution.qp_solution(order, n, gate, t)
traj = UAV_traj()
traj_publisher = rospy.Publisher('uav_ref_trajectory', UAV_traj, queue_size = 10)
rospy.init_node('trajectory_test1', anonymous=True)

last_time = rospy.get_time()
start_time = last_time
rate = rospy.Rate(100)

i = 0
while not rospy.is_shutdown():
    now_time = rospy.get_time()
    time = now_time - start_time
    x = sol_x[n*(order+1)*i: n*(order+1)*(i+1)+1]
    trajectory = compute_trajectory.compute_trajectory(x, order, time)
    ref_trajectory = df_flat.compute_ref(trajectory)

    pos = ref_trajectory[0]
    pos = np.matrix(pos)

    vel = ref_trajectory[1]
    vel = np.matrix(vel)

    or_ = ref_trajectory[2]
    or_ = np.matrix(or_)
    phi = or_.item(0)
    theta = or_.item(1)
    psi = or_.item(2)

    w_ = ref_trajectory[3]
    w_ = np.matrix(w_)

    u_a = ref_trajectory[4]
    u_a = np.matrix(u_a)

    u_b = ref_trajectory[5]
    u_a = np.matrix(u_a)

    u_c = ref_trajectory[6]
    u_c = np.matrix(u_c)

    traj.header.stamp = rospy.Time.now()
    traj.header.frame_id = ""

    traj.pose.position.x = pos.item(0)
    traj.pose.position.y = pos.item(1)
    traj.pose.position.z = pos.item(2)

    quat = tf.transformations.quaternion_from_euler(phi, theta, psi, axes='rxyz')

    traj.pose.orientation.x = quat[0]
    traj.pose.orientation.y = quat[1]
    traj.pose.orientation.z = quat[2]
    traj.pose.orientation.w = quat[3]

    traj.twist.linear.x = vel.item(0)
    traj.twist.linear.y = vel.item(1)
    traj.twist.linear.z = vel.item(2)

    traj.twist.angular.x = w_.item(0)
    traj.twist.angular.y = w_.item(1)
    traj.twist.angular.z = w_.item(2)

    traj.ua.x = u_a.item(0)
    traj.ua.y = u_a.item(1)
    traj.ua.z = u_a.item(2)

    traj.ub.x = u_b.item(0)
    traj.ub.y = u_b.item(1)
    traj.ub.z = u_b.item(2)

    traj.uc.x = u_b.item(0)
    traj.uc.y = u_b.item(1)
    traj.uc.z = u_b.item(2)

    traj_publisher.publish(traj)
    rospy.loginfo(traj)

    # print state_input
    if now_time - last_time > t[i+1] - t[i]:

        print time
        i = i + 1
        last_time = now_time

    if i == gate:
        print time
        exit()

    rate.sleep()
