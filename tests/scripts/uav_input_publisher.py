#!/usr/bin/env python

# support libraries
import rospy
import message_filters
import tf
import numpy as np

# messages
from tests.msg import UAV_traj
from tests.msg import UAV_state
from mav_msgs.msg import RateThrust

import lqr_gains as lqrg

g = 9.81

class uav_Input_Publisher():

    # constructor
    def __init__(self):

        # publisher 
        self.input_publisher = rospy.Publisher('/uav/input/rateThrust', RateThrust, queue_size = 10)

        # create message message_filter
        self.state_sub = message_filters.Subscriber('/uav_state', UAV_state)
        self.reftraj_sub = message_filters.Subscriber('/uav_ref_trajectory', UAV_traj)

        # filter messages based on time
        ts = message_filters.ApproximateTimeSynchronizer([self.state_sub, self.reftraj_sub],10,0.1)
        ts.registerCallback(self.callback)

        self.m = rospy.get_param("/uav/flightgoggles_uav_dynamics/vehicle_mass")

        # output performance matrix for translation variables
        self.Qt = np.diag([100.0,1.0])
        # input effort matrix for translation variables
        self.Rt = np.array([1.0])   
        # output performance matrix for rotational variables
        self.Qr = np.diag([5.0])
        # input effort matrix for rotational variables
        self.Rr = np.array([1.0])

        # get controller gains
        self.Kt_lqr, self.N_ut_lqr, self.N_xt_lqr = lqrg.calculate_LQR_gains(lqrg.At, lqrg.Bt, lqrg.Ct, lqrg.D_, self.Qt, self.Rt)
        self.Kr_lqr, self.N_ur_lqr, self.N_xr_lqr = lqrg.calculate_LQR_gains(lqrg.Ar, lqrg.Br, lqrg.Cr, lqrg.D_, self.Qr, self.Rr)



    def callback(self, state_msg, traj_msg):
 

        # In general    u = -K*x + (N_u + K*N_x)*r
        # r = reference state
        # x = state
        # K = LQR gains
        # N_u, N_x = refrence input and reference state matrices       


        # create and fill message
        rt_msg = RateThrust()
        
        rt_msg.header.stamp = rospy.Time.now()
        rt_msg.header.frame_id = 'uav/imu'

        rt_msg.angular_rates.x = traj_msg.twist.angular.x
        rt_msg.angular_rates.y = traj_msg.twist.angular.y
        rt_msg.angular_rates.z = traj_msg.twist.angular.z

        rt_msg.thrust.x = 0.0
        rt_msg.thrust.y = 0.0
        rt_msg.thrust.z = self.m*np.linalg.norm(t)
        

        # publish
        self.input_publisher.publish(rt_msg)

        rospy.loginfo(rt_msg)
        

if __name__ == '__main__':
    try:

        rospy.init_node('uav_input_publisher', anonymous = True)
        
        uav_input_pub = uav_Input_Publisher()


        rospy.loginfo(' UAV Input Publisher Created !')
        rospy.spin()
        rospy.loginfo(' UAV Input Publisher terminated. ')

    except rospy.ROSInterruptException:
        rospy.loginfo('ROS Terminated.')
        pass
