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
		ts = message_filters.TimeSynchronizer([self.pose_sub, self.imu_sub], 10)
		ts.registerCallback(self.callback)

		# define variables to compute linear velocity from
		# imu
		self.vo = np.zeros(3)
		self.t1 = rospy.Time.now()

		self.count = 0
		self.time_delta_acc = 0
		self.lags = 0

		# constants
		self.g = np.array([[0.0],[0.0],[-g]])




	def callback(self, pose_msg, imu_msg):

		# create state message and populate
		uav_state = UAV_state()
		uav_state.header.stamp = pose_msg.header.stamp # should correspond to the same time
		uav_state.header.frame_id = ""

		# Fill linear and angular position
		uav_state.pose.position.x = pose_msg.pose.position.x
		uav_state.pose.position.y = pose_msg.pose.position.y
		uav_state.pose.position.z = pose_msg.pose.position.z

		uav_state.pose.orientation.x = pose_msg.pose.orientation.x
		uav_state.pose.orientation.y = pose_msg.pose.orientation.y
		uav_state.pose.orientation.z = pose_msg.pose.orientation.z
		uav_state.pose.orientation.w = pose_msg.pose.orientation.w

		#Fill linear and angular velocities



		# Velocity, we have to integrate acceleration ... assuming this computation was perfect...
		# The process is:
		# a) transform gravity to body frame
		# b) subtract gravity from imu acceleration measuremtn
		# c) translate back to world frame
		# d) integrate

		Rwb = tf.transformations.quaternion_matrix([pose_msg.pose.orientation.x,
													pose_msg.pose.orientation.y,
													pose_msg.pose.orientation.z,
													pose_msg.pose.orientation.w]) # get World to Body Rotation
		#print("Rwb: {}, Type: {}".format(Rwb[0:3,0:3],type(Rwb)))

		g_b = np.dot(Rwb[0:3,0:3],self.g) # gravity in body frame
		#print("g_b: {}".format(g_b))

		a_b = np.array([[imu_msg.linear_acceleration.x + g_b[0][0]],  # acc in body frame
						[imu_msg.linear_acceleration.y + g_b[1][0]],
						[imu_msg.linear_acceleration.z + g_b[2][0]]])

		#print("a_b: {},{},{}".format(a_b[0][0], a_b[1][0], a_b[2][0]))
		a_w  = np.dot(Rwb[0:3,0:3].T,a_b)    # acc in world frame
		#print("a_w: {},{},{}".format(a_w[0][0], a_w[1][0], a_w[2][0]))

		# now 
		time_delta = pose_msg.header.stamp - self.t1
		#print (" Type: {}".format(type(time_delta)))
		if( time_delta > rospy.Duration(0.5)):
			time_delta = rospy.Duration(0.01)
			self.lags = self.lags + 1
			#print("****** DURACION > 0.5 s ********: {}".format(self.lags))
		time_delta = time_delta.to_sec() # convert to floating point
		#print("Time delta: {}".format(time_delta))
		self.t1 = pose_msg.header.stamp # prepare for next time step
		

		self.vo[0] = a_w[0]*time_delta + self.vo[0]
		self.vo[1] = a_w[1]*time_delta + self.vo[1]
		self.vo[2] = a_w[2]*time_delta + self.vo[2]

		uav_state.twist.linear.x = self.vo[0]
		uav_state.twist.linear.y = self.vo[1]
		uav_state.twist.linear.z = self.vo[2]

		uav_state.twist.angular.x = imu_msg.angular_velocity.x
		uav_state.twist.angular.y = imu_msg.angular_velocity.y
		uav_state.twist.angular.z = imu_msg.angular_velocity.z

		self.state_pub.publish(uav_state)
		

		self.count = self.count + 1
		self.time_delta_acc =  (self.time_delta_acc + time_delta)

		#rospy.loginfo(uav_state)
		#rospy.loginfo("Time Delta Average is: {}".format(self.time_delta_acc/self.count))

if __name__ == '__main__':
	try:
		# init a node
		rospy.init_node('uav_state', anonymous = True)

		# create an object that will get sensor data,
		# and estimate and publish UAV's state
		state_pub = uavStatePublisher()

		# wait for some time for ROS to
	
		# log and let ROS take charge of callback
		rospy.loginfo('UAV State publish created')
		rospy.spin()
		rospy.loginfo('UAV State publisher terminated')

	except rospy.ROSInterruptException:
		print("ROS Terminated.")
		pass

