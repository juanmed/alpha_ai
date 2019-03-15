#!/usr/bin/env python

# ROS packages import
import rospy
import message_filters
import tf

# messages import
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from tests.msg import UAV_state

# 
import numpy as np

# constants
g = 9.81

class uavStatePublisher():

	# constructor
	def __init__(self):

		# create topic uav state publisher 
		self.state_pub = rospy.Publisher("uav_state", UAV_state, queue_size = 10)

		# create message filters
		self.pose_sub = message_filters.Subscriber('uav_PoseStamped', PoseStamped)
		self.imu_sub = message_filters.Subscriber('/uav/sensors/imu', Imu)

		# create a TimeSynchronizer and pass both subscribers,	
		# give a buffer size of 10, and register callback 
		# where we combine info and create and send a message containing
		# uav's state
		#ts = message_filters.TimeSynchronizer([self.pose_sub, self.imu_sub], 10)
		ts = message_filters.ApproximateTimeSynchronizer([self.pose_sub, self.imu_sub], 10, 0.005)
		ts.registerCallback(self.callback)

		# define variables to compute linear velocity from imu
		init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
		
		# init position
		self.init_pos = np.zeros(3)
		self.init_pos[0] = init_pose[0]
		self.init_pos[1] = init_pose[1]
		self.init_pos[2] = init_pose[2]

		# init orientation
		self.init_ori = np.zeros(4)
		self.init_ori[0] = init_pose[3]
		self.init_ori[1] = init_pose[4]
		self.init_ori[2] = init_pose[5]
		self.init_ori[3] = init_pose[6]

		# init timestamp
		self.t1 = rospy.Time.now()

		self.count = 0
		self.time_delta_acc = 0
		self.lags = 0

		# constants
		self.g = np.array([[0.0],[0.0],[-g]])




	def callback(self, pose_msg, imu_msg):

		# create state message and populate
		uav_state = UAV_state()

		uav_state.header.stamp = rospy.Time.now()#pose_msg.header.stamp # should correspond to the same time
		uav_state.header.frame_id = ""

		# Fill linear and angular position
		uav_state.pose.position.x = pose_msg.pose.position.x
		uav_state.pose.position.y = pose_msg.pose.position.y
		uav_state.pose.position.z = pose_msg.pose.position.z

		uav_state.pose.orientation.x = pose_msg.pose.orientation.x
		uav_state.pose.orientation.y = pose_msg.pose.orientation.y
		uav_state.pose.orientation.z = pose_msg.pose.orientation.z
		uav_state.pose.orientation.w = pose_msg.pose.orientation.w

		#Linear velocity is derivative of position

		# time delta 
		time_delta = pose_msg.header.stamp - self.t1
		time_delta = time_delta.to_sec() 		# convert to floating point
		self.t1 = pose_msg.header.stamp  		# prepare for next time step

		# init velocity
		v = np.zeros(3)
		v[0] = (pose_msg.pose.position.x - self.init_pos[0])/time_delta
		v[1] = (pose_msg.pose.position.y - self.init_pos[1])/time_delta
		v[2] = (pose_msg.pose.position.z - self.init_pos[2])/time_delta

		# prepare for next time step
		self.init_pos[0] = pose_msg.pose.position.x
		self.init_pos[1] = pose_msg.pose.position.y
		self.init_pos[2] = pose_msg.pose.position.z

		uav_state.twist.linear.x = v[0]
		uav_state.twist.linear.y = v[1]
		uav_state.twist.linear.z = v[2]

		# angular velocity by derivative of quaternion
		# look: https://math.stackexchange.com/questions/2282938/converting-from-quaternion-to-angular-velocity-then-back-to-quaternion
		# w_b = Im(2 * [q(t)'*q(t+h)]/delta_t )  q(t)': is the congujate
 		ori = np.zeros(4)
		ori[0] = pose_msg.pose.orientation.x
		ori[1] = pose_msg.pose.orientation.y
		ori[2] = pose_msg.pose.orientation.z
		ori[3] = pose_msg.pose.orientation.w

		
		prev_ori_cong = np.zeros(4)	
		prev_ori_cong[0] = -1.0*self.init_ori[0]
		prev_ori_cong[1] = -1.0*self.init_ori[1]
		prev_ori_cong[2] = -1.0*self.init_ori[2]
		prev_ori_cong[3] = self.init_ori[3]

		# Im(q1*q2) = [w1*v2 + w2*v1 + v1xv2]
		w_b = prev_ori_cong[3]*ori[0:3] + ori[3]*prev_ori_cong[0:3] + np.cross(prev_ori_cong[0:3],ori[0:3])
		w_b = 2.0*w_b/time_delta


		# prepare for next step
		self.init_ori[0] = ori[0]
		self.init_ori[1] = ori[1]
		self.init_ori[2] = ori[2]
		self.init_ori[3] = ori[3]

		uav_state.twist.angular.x = w_b[0]
		uav_state.twist.angular.y = w_b[1]
		uav_state.twist.angular.z = w_b[2]

		self.state_pub.publish(uav_state)
		rospy.loginfo(uav_state)

if __name__ == '__main__':
	try:
		# init a node
		rospy.init_node('uav_state', anonymous = True)

		# create an object that will get sensor data,
		# and estimate and publish UAV's state
		state_pub = uavStatePublisher()

		# wait for some time for ROS to
	
		# log and let ROS take charge of callback
		rospy.loginfo('UAV State publisher created')
		rospy.spin()
		rospy.loginfo('UAV State publisher terminated')

	except rospy.ROSInterruptException:
		print("ROS Terminated.")
		pass

