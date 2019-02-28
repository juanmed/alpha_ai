#!/usr/bin/env python

# The purpose of this file is to understand how to publish messages to
# mav_msgs/RateThrust

import rospy

from std_msgs.msg import Empty
from mav_msgs.msg import RateThrust
from geometry_msgs.msg import Pose
from tf.msg import tfMessage

class rateThrust_Publisher():

    # constructor... if that expression is used in python
    def __init__(self):

        # create a publisher
        self.rt_publisher = rospy.Publisher('/uav/input/rateThrust', RateThrust, queue_size = 10)

        # subscribe to topic providing pose 
        # estimates... this should come from state estimator
        # ... now lets do it with ground truth state
        rospy.Subscriber('uav_pose', Pose, self.callback)

    def callback(self,data):

        rt_msg = RateThrust()
        rt_msg.header.stamp = rospy.Time.now()
        rt_msg.header.frame_id = 'uav/imu'

        rt_msg.angular_rates.x = 0.0
        rt_msg.angular_rates.y = 0.0
        rt_msg.angular_rates.z = 0.0

        rt_msg.thrust.x = 0.0
        rt_msg.thrust.y = 0.0
        rt_msg.thrust.z = 9.9
        self.rt_publisher.publish(rt_msg)

        #rospy.loginfo("Published rateThrust message :) !")
        rospy.loginfo(rt_msg)

def callback(data):
    print("Llego mensaje!")

def publish():

    # start node
    rospy.init_node('ratethrust_publisher', anonymous = True)

    # subscribe to tf
    rospy.Subscriber('tf', tfMessage, callback)

    rospy.loginfo("Testing ratethrust publishing.")
    rospy.spin()
    rospy.loginfo("Node closed down!")


if __name__ == "__main__":
    try:
        # init node
        rospy.init_node('rateThrust_publisher', anonymous = True)

        # send take off command
        # this must be sent in order for the drone to receive commands
        #to_publisher = rospy.Publisher("/uav/input/takeoff", Empty, 5)
        #to_publisher.publish(Empty())

        # create an rateThrust_Publisher object
        rtp = rateThrust_Publisher()

        rospy.loginfo('rateThrust Publisher created!')
        rospy.spin()
        rospy.loginfo("rateThrust Publisher shut down!")
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass

