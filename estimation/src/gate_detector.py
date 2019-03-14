#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import re
from math import sin, cos

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from flightgoggles.msg import IRMarker, IRMarkerArray


class GateDetector():
    def ir_cb(self, ir_array):
        num = len(ir_array.markers)
        object_points = np.zeros((3, num))
        image_points = np.zeros((2, num))
        for i in range(0, num):
            image_points[0][i] = ir_array.markers[i].x
            image_points[1][i] = ir_array.markers[i].y

            gate_id = self.getID(ir_array.markers[i].landmarkID)
            marker_id = self.getID(ir_array.markers[i].markerID)
            object_points[0][i] = self.gate_location[gate_id][marker_id][0]
            object_points[1][i] = self.gate_location[gate_id][marker_id][1]
            object_points[2][i] = self.gate_location[gate_id][marker_id][2]

        rvec, tvec = self.getPose(object_points, image_points)
        self.setState(rvec, tvec)
        self.pub_pose(self.state)


    def getID(self, landmarkID):
        i = int(re.findall("\d+", landmarkID)[0])
        return i


    def getPose(self, object_points, image_points):
        ret, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.CV_P3P)
        return np.array(rvec), np.array(tvec)


    def hat(self, v):
        return np.array([[0, -v[2], v[1]],
                         [v[2], 0, -v[0]],
                         [-v[1], v[0], 0]])


    def rodrigues2rotation(self, v):
        theta = sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        R = cos(theta)*np.eye(3) + sin(theta)*self.hat(v/theta) + (1-cos(theta))*np.dot(v/theta, v.T/theta)
        return R


    def setState(self, rvec, tvec):
        R = self.rodrigues2rotation(rvec)
        pi = asin(-R[0][2])
        theta = atan2(R[1][2], R[2][2])
        psi = atan2(R[0][1], R[0][0])

        self.state.header.stamp = rospy.Time.now()
        self.state.pose.position.x = tvec[0]
        self.state.pose.position.y = tvec[1]
        self.state.pose.position.z = tvec[2]
        self.state.pose.orientation.x = sin(pi/2)*cos(theta/2)*cos(psi/2) - cos(pi/2)*sin(theta/2)*sin(psi/2)
        self.state.pose.orientation.y = sin(pi/2)*cos(theta/2)*sin(psi/2) + cos(pi/2)*sin(theta/2)*cos(psi/2)
        self.state.pose.orientation.z = cos(pi/2)*cos(theta/2)*sin(psi/2) - sin(pi/2)*sin(theta/2)*cos(psi/2)
        self.state.pose.orientation.w = cos(pi/2)*cos(theta/2)*cos(psi/2) + sin(pi/2)*sin(theta/2)*sin(psi/2)


    def __init__(self):
        rospy.init_node('gate_detector')
        self.r = rospy.Rate(100)

        self.camera_matrix = np.array([[548.4088134765625, 0.0, 512.0],
                                       [0.0, 548.4088134765625, 384.0],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros(4)
        
        gate_num = len(rospy.get_param('/uav/gate_names'))
        self.gate_location = np.zeros((gate_num+1, 4, 3))
        print gate_num
        for i in range(1, gate_num+1):
            location = rospy.get_param('/uav/Gate' + i + '/nominal_location')
            for j in range(0, 4):
                for k in range(0, 3):
                    self.gate_location[i+1][j][k] = location[j][k]
        rospy.Subscriber('/uav/camera/left/ir_beacons', IRMarkerArray, self.ir_cb)
        self.pub_pose = rospy.Publisher('/uav/pose_gate', Pose)
        self.state = Pose()


    def loop(self):
        self.r.sleep()


if __name__ == "__main__":
    print("OpenCV: " + cv2.__version__)
    gate_detector = GateDetector()
    while not rospy.is_shutdown():
        gate_detector.loop()