#!/usr/bin/env python

import rospy
from tests.msg import UAV_traj
from mav_msgs.msg import RateThrust


class Thrust_Test:
    def __init__(self):
        # publisher
        self.thrust_publisher = rospy.Publisher('/uav/input/rateThrust', RateThrust, queue_size=10)

        # subcriber
        rospy.Subscriber('uav_ref_trajectory', UAV_traj, self.callback)

    def callback(self, msg):
        rt_msg = RateThrust()
        rt_msg.thrust.z = msg.ua.z + 9.8
        self.thrust_publisher.publish(rt_msg)

if __name__ == '__main__':
    try:
        # init node
        rospy.init_node('Thrust_Test_publisher', anonymous = True)

        # send take off command
        # this must be sent in order for the drone to receive commands
        #to_publisher = rospy.Publisher("/uav/input/takeoff", Empty, 5)
        #to_publisher.publish(Empty())

        # create an rateThrust_Publisher object
        rtp = Thrust_Test()

        rospy.spin()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass