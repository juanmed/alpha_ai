#!/usr/bin/env python

import rospy
from math import sqrt, sin, cos, asin, atan2

from geometry_msgs.msg import PoseStamped, Vector3


class GateDetector():
    def cb(self, q):
        self.setState(q.pose.orientation.w, q.pose.orientation.x, q.pose.orientation.y, q.pose.orientation.z)
        self.pub_attitude.publish(self.state)


    def toEuler(self, qw, qx, qy, qz):
        pi = atan2(2*(qw*qx+qy*qz), 1-2*(pow(qx, 2)+pow(qy, 2)))
        theta = asin(2*(qw*qy-qz*qx))
        psi = atan2(2*(qw*qz+qx*qy), 1-2*(pow(qy, 2)+pow(qz, 2)))
        return pi, theta, psi


    def setState(self, qw, qx, qy, qz):
        (pi, theta, psi) = self.toEuler(qw, qx, qy, qz)

        self.state.x = pi
        self.state.y = theta
        self.state.z = psi


    def __init__(self):
        rospy.init_node('chch')
        
        self.rate = 10
        self.r = rospy.Rate(self.rate)
        rospy.Subscriber('/uav_PoseStamped', PoseStamped, self.cb)
        self.pub_attitude = rospy.Publisher('/att', Vector3, queue_size=10)
        self.state = Vector3()


    def loop(self):
        self.r.sleep()


if __name__ == "__main__":
    gate_detector = GateDetector()
    while not rospy.is_shutdown():
        gate_detector.loop()