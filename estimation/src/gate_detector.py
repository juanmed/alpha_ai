#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import re
from math import sqrt, sin, cos, asin, atan2

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from flightgoggles.msg import IRMarker, IRMarkerArray


class GateDetector():
    def ir_cb(self, ir_array):
        num = len(ir_array.markers)
        print num
        if num >= 8:
            object_points = np.zeros((num, 3))
            image_points = np.zeros((num, 2))
            for i in range(0, num):
                image_points[i][0] = ir_array.markers[i].x
                image_points[i][1] = ir_array.markers[i].y

                gate_id = self.getID(ir_array.markers[i].landmarkID)
                marker_id = self.getID(ir_array.markers[i].markerID)
                object_points[i][0] = self.gate_location[gate_id-1][marker_id-1][0]
                object_points[i][1] = self.gate_location[gate_id-1][marker_id-1][1]
                object_points[i][2] = self.gate_location[gate_id-1][marker_id-1][2]

            rvec, tvec = self.getPose(object_points, np.ascontiguousarray(image_points[:,:2]).reshape((num,1,2)))
            self.setState(rvec, tvec)
            self.pub_pose.publish(self.state)


    def getID(self, landmarkID):
        i = int(re.findall("\d+", str(landmarkID))[0])
        return i


    def getPose(self, object_points, image_points):
        ret, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, None, flags=cv2.SOLVEPNP_EPNP)
        return np.array(rvec), np.array(tvec)


    def hat(self, v):
        return np.array([[0, -v[2], v[1]],
                         [v[2], 0, -v[0]],
                         [-v[1], v[0], 0]])


    def rodrigues2rotation(self, v):
        theta = sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        r = np.array([v[0], v[1], v[2]]) / theta
        R = cos(theta)*np.eye(3) + sin(theta)*self.hat(r) + (1-cos(theta))*np.dot(r, r.T)
        print R
        return R


    def setState(self, rvec, tvec):
        R = self.rodrigues2rotation(rvec)
        
        pi = asin(-R[0][2])
        theta = atan2(R[1][2], R[2][2]) + np.pi/2
        psi = atan2(R[0][1], R[0][0]) + np.pi/2
        print pi, theta, psi

        self.state.position.x = tvec[0][0]
        self.state.position.y = tvec[2][0]
        self.state.position.z = tvec[1][0]
        self.state.orientation.x = sin(pi/2)*cos(theta/2)*cos(psi/2) - cos(pi/2)*sin(theta/2)*sin(psi/2)
        self.state.orientation.y = sin(pi/2)*cos(theta/2)*sin(psi/2) + cos(pi/2)*sin(theta/2)*cos(psi/2)
        self.state.orientation.z = cos(pi/2)*cos(theta/2)*sin(psi/2) - sin(pi/2)*sin(theta/2)*cos(psi/2)
        self.state.orientation.w = cos(pi/2)*cos(theta/2)*cos(psi/2) + sin(pi/2)*sin(theta/2)*sin(psi/2)

        print self.state


    def __init__(self):
        rospy.init_node('gate_detector')
        self.r = rospy.Rate(100)

        self.camera_matrix = np.array([[548.4088134765625, 0.0, 512.0],
                                       [0.0, 548.4088134765625, 384.0],
                                       [0.0, 0.0, 1.0]])
        gate_num = 23
        self.gate_location = np.zeros((gate_num+1, 4, 3))
        for i in range(0, gate_num):
            location = rospy.get_param('/uav/Gate' + str(i+1) + '/nominal_location')
            for j in range(0, 4):
                for k in range(0, 3):
                    self.gate_location[i][j][k] = location[j][k]
        rospy.Subscriber('/uav/camera/left/ir_beacons', IRMarkerArray, self.ir_cb)
        self.pub_pose = rospy.Publisher('/uav/pose_gate', Pose, queue_size=10)
        self.state = Pose()


    def loop(self):
        self.r.sleep()


if __name__ == "__main__":
    print("OpenCV: " + cv2.__version__)
    gate_detector = GateDetector()
    while not rospy.is_shutdown():
        gate_detector.loop()