#!/usr/bin/env python

import rospy
import cv2

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from flightgoggles.msg import IRMarker, IRMarkerArray


class GateDetector():
    def ir_cb(self, ir_array):
        num = len(ir_array.markers)
        for i in range(0, num):
            print ir_array.markers[i]



    def __init__(self):
        rospy.init_node('gate_detector')
        self.r = rospy.Rate(100)
        print rospy.get_param('/uav'+'/flightgoggles_imu/gyroscope_variance')
        rospy.Subscriber('/uav/camera/left/ir_beacons', IRMarkerArray, self.ir_cb)
        self.pub_pose = rospy.Publisher('/uav/pose_gate', Pose)
        self.pose = Pose()


    def loop(self):
        self.r.sleep()


if __name__ == "__main__":
    print("OpenCV: " + cv2.__version__)
    gate_detector = GateDetector()
    while not rospy.is_shutdown():
        gate_detector.loop()