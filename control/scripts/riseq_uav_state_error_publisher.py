#!/usr/bin/env python

# support libraries
import rospy
import message_filters
import tf
import numpy as np


# messages
from tests.msg import UAV_traj
from tests.msg import UAV_state




g = 9.81

class state_Error_Publisher():

    # constructor
    def __init__(self):

        # publisher 
        self.error_publisher = rospy.Publisher('/riseq/control/uav_state_error_publisher', UAV_state, queue_size = 10)

        self.reftraj_sub = message_filters.Subscriber('/uav_ref_trajectory', UAV_traj)

        # create message message_filter
        self.state_sub = message_filters.Subscriber('/uav_state', UAV_state)

        # filter messages based on time
        ts = message_filters.ApproximateTimeSynchronizer([self.state_sub, self.reftraj_sub],10,0.006)
        ts.registerCallback(self.publish_error)


    def publish_error(self, state_msg, traj_msg):

        # extract reference values
        x_r, y_r, z_r = [traj_msg.pose.position.x, traj_msg.pose.position.y, traj_msg.pose.position.z]
        vx_r, vy_r, vz_r = [traj_msg.twist.linear.x, traj_msg.twist.linear.y, traj_msg.twist.linear.z] 
        ori_quat_r = [traj_msg.pose.orientation.x, traj_msg.pose.orientation.y, traj_msg.pose.orientation.z, traj_msg.pose.orientation.w]
        psi_r, theta_r, phi_r = tf.transformations.euler_from_quaternion(ori_quat_r, axes = 'rzyx')
        p_r, q_r, r_r = [traj_msg.twist.angular.x, traj_msg.twist.angular.y, traj_msg.twist.angular.z]

        # extract drone real state values
        x, y, z = [state_msg.pose.position.x, state_msg.pose.position.y, state_msg.pose.position.z]
        vx, vy, vz = [state_msg.twist.linear.x, state_msg.twist.linear.y, state_msg.twist.linear.z]
        ori_quat = [state_msg.pose.orientation.x, state_msg.pose.orientation.y, state_msg.pose.orientation.z, state_msg.pose.orientation.w]
        psi, theta, phi = tf.transformations.euler_from_quaternion(ori_quat, axes = 'rzyx')
        p, q, r = [state_msg.twist.angular.x, state_msg.twist.angular.y, state_msg.twist.angular.z]

        error_msg = UAV_state()
        error_msg.header.stamp = rospy.Time.now()
        error_msg.header.frame_id = 'uav/imu'

        error_msg.pose.position.x = x - x_r
        error_msg.pose.position.y = y - y_r
        error_msg.pose.position.z = z - z_r

        error_msg.pose.orientation.x = ori_quat[0] - ori_quat_r[0]
        error_msg.pose.orientation.y = ori_quat[1] - ori_quat_r[1]
        error_msg.pose.orientation.z = ori_quat[2] - ori_quat_r[2]
        error_msg.pose.orientation.w = ori_quat[3] - ori_quat_r[3]

        error_msg.twist.linear.x = vx - vx_r
        error_msg.twist.linear.y = vy - vy_r
        error_msg.twist.linear.z = vz - vz_r

        error_msg.twist.angular.x = p - p_r
        error_msg.twist.angular.y = q - q_r
        error_msg.twist.angular.z = r - r_r 

        self.error_publisher.publish(error_msg)
        rospy.loginfo(error_msg)


if __name__ == '__main__':
    try:

        rospy.init_node('riseq_uav_state_error_publisher', anonymous = True)
        
        state_error_pub = state_Error_Publisher()

        rospy.loginfo(' UAV State Error Publisher Created !')
        rospy.spin()
        rospy.loginfo(' UAV State Error Publisher terminated. ')

    except rospy.ROSInterruptException:
        rospy.loginfo('ROS Terminated.')
        pass