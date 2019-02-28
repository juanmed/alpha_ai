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
		ts = message_filters.ApproximateTimeSynchronizer([self.pose_sub, self.imu_sub], 10, 0.1)
		ts.registerCallback(self.callback)

		# define variables to compute linear velocity from imu
		init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
		self.init_pos = np.zeros(3)

		# init position
		self.init_pos[0] = init_pose[0]
		self.init_pos[1] = init_pose[1]
		self.init_pos[2] = init_pose[2]

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

		# angular velocity directly from IMU
		uav_state.twist.angular.x = imu_msg.angular_velocity.x
		uav_state.twist.angular.y = imu_msg.angular_velocity.y
		uav_state.twist.angular.z = imu_msg.angular_velocity.z

		self.state_pub.publish(uav_state)
		

		#self.count = self.count + 1
		#self.time_delta_acc =  (self.time_delta_acc + time_delta)
		#rospy.loginfo(pose_msg.header.stamp - imu_msg.header.stamp)
		rospy.loginfo(uav_state)
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
		rospy.loginfo('UAV State publisher created')
		rospy.spin()
		rospy.loginfo('UAV State publisher terminated')

	except rospy.ROSInterruptException:
		print("ROS Terminated.")
		pass

