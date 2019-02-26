#!/usr/bin/env python

# This is a test node for publishing the reference trajectory
# at a constant rate. This reference trajectory is computed by
# using the differential flatness property of the UAV Dynamics.

import rospy
from tests.msg import UAV_traj
import tf
import numpy as np



def pub_traj():

	# create topic for publishing ref trajectory
	traj_publisher = rospy.Publisher('uav_ref_trajectory', UAV_traj, queue_size = 10)

	# init node
	rospy.init_node('uav_ref_trajectory_publisher', anonymous = True)

	# publish at 10Hz
	rate = rospy.Rate(10.0)

	while not rospy.is_shutdown():
		try:

			# Here compute reference trajectory
			x, y, z = [0]*3
			phi, theta, psi = [0,0,np.pi/2.0]
			vx, vy, vz = [3.0]*3
			p, q, r = [4.0]*3
			uax, uay, uaz = [5.0]*3
			ubx, uby, ubz = [6.0]*3
			ucx, ucy, ucz = [7.0]*3


			traj = UAV_traj()
			traj.header.stamp = rospy.Time.now()
			traj.header.frame_id = ""

			traj.pose.position.x = x
			traj.pose.position.y = y
			traj.pose.position.z = z

			quat = tf.transformations.quaternion_from_euler(phi,theta,psi, axes = 'rxyz')

			traj.pose.orientation.x = quat[0]
			traj.pose.orientation.y = quat[1]
			traj.pose.orientation.z = quat[2]
			traj.pose.orientation.w = quat[3]

			traj.twist.linear.x = vx
			traj.twist.linear.y = vy
			traj.twist.linear.z = vz

			traj.twist.angular.x = p
			traj.twist.angular.y = q
			traj.twist.angular.z = r

			traj.ua.x = uax
			traj.ua.y = uay
			traj.ua.z = uaz

			traj.ub.x = ubx
			traj.ub.y = uby
			traj.ub.z = ubz

			traj.uc.x = ucx
			traj.uc.y = ucy
			traj.uc.z = ucz

			traj_publisher.publish(traj)
			rospy.loginfo(traj)
			
			rate.sleep()


		except:
			rospy.loginfo('People...we have a problem')
			continue




if __name__ == '__main__':
	try:
		rospy.loginfo("UAV Trajectory Publisher Created")
		pub_traj()
	except rospy.ROSInterruptException:
		print("ROS Terminated.")
		pass