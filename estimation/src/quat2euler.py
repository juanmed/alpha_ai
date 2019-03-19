#!/usr/bin/env python

import rospy
from math import sqrt, sin, cos, asin, atan2

from geometry_msgs.msg import PoseStamped, Vector3
from tests.msg import UAV_traj


class GateDetector():
    def cb(self, q):
        self.setState(q.pose.orientation.w, q.pose.orientation.x, q.pose.orientation.y, q.pose.orientation.z)
        self.pub_attitude.publish(self.state)

    
    def cb2(self, q):
        self.setState2(q.pose.orientation.w, q.pose.orientation.x, q.pose.orientation.y, q.pose.orientation.z)
        self.pub_attitude2.publish(self.state2)
        print 'ok'


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

    def setState2(self, qw, qx, qy, qz):
        (pi, theta, psi) = self.toEuler(qw, qx, qy, qz)

        self.state2.x = pi
        self.state2.y = theta
        self.state2.z = psi


    def __init__(self):
        rospy.init_node('chch')
        
        self.rate = 10
        self.r = rospy.Rate(self.rate)
        rospy.Subscriber('/uav_PoseStamped', PoseStamped, self.cb)
        rospy.Subscriber('/uav_ref_trajectory', UAV_traj, self.cb2)
        self.pub_attitude = rospy.Publisher('/att', Vector3, queue_size=10)
        self.pub_attitude2 = rospy.Publisher('/att2', Vector3, queue_size=10)
        self.state = Vector3()
        self.state2 = Vector3()


    def loop(self):
        self.r.sleep()


if __name__ == "__main__":
    gate_detector = GateDetector()
    while not rospy.is_shutdown():
        gate_detector.loop()