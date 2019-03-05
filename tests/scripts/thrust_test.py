#!/usr/bin/env python

import rospy
from tests.msg import UAV_traj
from mav_msgs.msg import RateThrust
import numpy as np

class Thrust_Test:
    def __init__(self):
        # publisher
        self.thrust_publisher = rospy.Publisher('/uav/input/rateThrust', RateThrust, queue_size=10)

        # subcriber
        rospy.Subscriber('uav_ref_trajectory', UAV_traj, self.callback)

    def callback(self, msg):
        rt_msg = RateThrust()
        t = np.array([msg.ua.x, msg.ua.y, msg.ua.z+9.81])
        norm = np.linalg.norm(t)
        rt_msg.thrust.z = norm

        rt_msg.angular_rates.x = msg.twist.angular.x
        rt_msg.angular_rates.y = msg.twist.angular.y
        rt_msg.angular_rates.z = msg.twist.angular.z
        self.thrust_publisher.publish(rt_msg)

if __name__ == '__main__':
    try:
        # init node
        rospy.init_node('Thrust_Test_publisher', anonymous = True)

        # create an rateThrust_Publisher object
        rtp = Thrust_Test()

        rospy.spin()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass