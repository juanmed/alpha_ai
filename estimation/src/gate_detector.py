#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import re
import math
from math import sqrt, sin, cos, asin, atan2

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from flightgoggles.msg import IRMarker, IRMarkerArray


class GateDetector():
    def camera_info_cb(self, camera_info):
        self.camera_matrix = np.array(camera_info.K).reshape(3, 3)


    def next_cb(self, gate):
        self.next_gate = self.getID(gate.data)


    def ir_cb(self, ir_array):
        next_cnt = 0
        next_marker = [0, 0, 0, 0]
        next_array_num = [0, 0, 0, 0]
        num = len(ir_array.markers)
        print "Detected IR markers: ", num
        
        for i in range(0, num):
            if self.getID(ir_array.markers[i].landmarkID) == self.next_gate:
                next_marker[next_cnt] = self.getID(ir_array.markers[i].markerID)
                next_array_num[next_cnt] = i
                next_cnt += 1
        print "Next gate: ", self.next_gate, next_cnt
        
        if next_cnt == 4:
            dst_points = np.zeros((4, 2))
            object_points = np.zeros((4, 3))
            image_points = np.zeros((4, 2))
            for i in range(0, 4):
                object_points[i][0] = self.gate_location[self.next_gate-1][next_marker[i]-1][0]
                object_points[i][1] = self.gate_location[self.next_gate-1][next_marker[i]-1][1]
                object_points[i][2] = self.gate_location[self.next_gate-1][next_marker[i]-1][2]
                image_points[i][0] = ir_array.markers[next_array_num[i]].x
                image_points[i][1] = ir_array.markers[next_array_num[i]].y
            tx = object_points[0][0]
            ty = object_points[0][1]
            tz = object_points[0][2]
            object_points[:, 0] = object_points[:, 0] - tx
            object_points[:, 1] = object_points[:, 1] - ty
            object_points[:, 2] = object_points[:, 2] - tz
            rot = atan2((object_points[1][1]+object_points[2][1]+object_points[3][1])/2, (object_points[1][0]+object_points[2][0]+object_points[3][0])/2)
            rot_mat = np.array([[cos(rot), sin(rot), 0], [-sin(rot), cos(rot), 0], [0, 0, 1]])
            flat_points = np.dot(rot_mat, object_points.T)
            #src_points = np.delete(np.dot(np.array([[cos(rot), sin(rot), 0], [-sin(rot), cos(rot), 0], [0, 0, 1]]), object_points.T).T, 1, axis=1)
            Rwf = np.array([[-1, 0, 0], [0, 0, -1], [0, -1, 0]])
            src_points = np.dot(Rwf, flat_points).T #np.array([flat_points[0, :], flat_points[2, :]]).T

            H, status = cv2.findHomography(src_points[:, 0:2], image_points, method=0)
            rt = np.dot(np.linalg.inv(self.camera_matrix), H)
            R = np.hstack((rt[:, 0], rt[:, 1], np.cross(rt[:, 0], rt[:, 1]))).reshape(3, 3).T
            t = rt[:, 2]
            Rt = np.insert(np.hstack((rt[:, 0], rt[:, 1], np.cross(rt[:, 0], rt[:, 1]), t)).reshape(4, 3).T, 3, [0, 0, 0, 1], axis=0)

            RRR = np.linalg.multi_dot([R.T, Rwf.T, rot_mat.T])
            print self.rotation2euler(RRR)

            #print np.dot(np.linalg.inv(self.camera_matrix), np.vstack((image_points.T, [1, 1, 1, 1])))
            #print np.dot(np.dot(self.camera_matrix, rt), np.vstack((src_points.T, [1, 1, 1, 1])))
            #print image_points

            #self.setState(rvec, tvec)
            self.pub_pose.publish(self.state)
            self.pub_attitude.publish(self.euler)
        
        elif num >= 5:
            object_points = np.zeros((num, 3))
            image_points = np.zeros((num, 2))
            for i in range(0, num):
                gate_id = self.getID(ir_array.markers[i].landmarkID)
                marker_id = self.getID(ir_array.markers[i].markerID)
                object_points[i][0] = self.gate_location[gate_id-1][marker_id-1][0]
                object_points[i][1] = self.gate_location[gate_id-1][marker_id-1][1]
                object_points[i][2] = self.gate_location[gate_id-1][marker_id-1][2]
                image_points[i][0] = ir_array.markers[i].x
                image_points[i][1] = ir_array.markers[i].y

            rvec, tvec = self.getPosePnP(object_points, np.ascontiguousarray(image_points[:,:2]).reshape((num,1,2)), cv2.SOLVEPNP_EPNP)
            self.setState(rvec, tvec)
            self.pub_pose.publish(self.state)
            self.pub_attitude.publish(self.euler)


    def getID(self, landmarkID):
        i = int(re.findall("\d+", str(landmarkID))[0])
        return i


    def getPosePnP(self, object_points, image_points, method):
        ret, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, None, flags=method)
        return np.array(rvec), np.array(tvec)


    def hat(self, v):
        return np.array([[0, -v[2], v[1]],
                         [v[2], 0, -v[0]],
                         [-v[1], v[0], 0]])


    def rodrigues2rotation(self, v):
        theta = sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        r = np.array([v[0], v[1], v[2]]) / theta
        R = cos(theta)*np.eye(3) + sin(theta)*self.hat(r) + (1-cos(theta))*np.dot(r, r.T)
        return R


    def rotation2quaternion(self, R):
        qw = sqrt(1 + R[0][0] + R[1][1] + R[2][2]) / 2
        qx = (R[2][1] - R[1][2]) / (4*qw)
        qy = (R[0][2] - R[2][0]) / (4*qw)
        qz = (R[1][0] - R[0][1]) / (4*qw)
        return qw, qx, qy, qz


    def rotation2euler(self, R):
        pi = atan2(R[2][1], R[2][2])
        theta = asin(-R[2][0])
        psi = atan2(R[1][0], R[0][0])
        return pi, theta, psi


    def euler2quaternion(self, pi, theta, psi):
        qw = cos(pi/2)*cos(theta/2)*cos(psi/2) + sin(pi/2)*sin(theta/2)*sin(psi/2)
        qx = sin(pi/2)*cos(theta/2)*cos(psi/2) - cos(pi/2)*sin(theta/2)*sin(psi/2)
        qy = sin(pi/2)*cos(theta/2)*sin(psi/2) + cos(pi/2)*sin(theta/2)*cos(psi/2)
        qz = cos(pi/2)*cos(theta/2)*sin(psi/2) - sin(pi/2)*sin(theta/2)*cos(psi/2)
        return qw, qx, qy, qz


    def setState(self, rvec, tvec):
        R = self.rodrigues2rotation(rvec)
        t = np.dot(-R.T, tvec)
        
        (pi, theta, psi) = self.rotation2euler(R.T)
        while pi+np.pi/2 > np.pi:
            pi = pi - 2*np.pi
        while theta+np.pi*2 > np.pi:
            theta = theta - 2*np.pi
        while psi+np.pi/2 > np.pi:
            psi = psi - 2*np.pi
        pi_r = theta + np.pi*2
        theta_r = -pi - np.pi/2
        psi_r = psi + np.pi/2
        (qw, qx, qy, qz) = self.euler2quaternion(pi_r, theta_r, psi_r)

        self.state.position.x = t[0][0]
        self.state.position.y = t[1][0]
        self.state.position.z = t[2][0]
        self.state.orientation.x = qx
        self.state.orientation.y = qy
        self.state.orientation.z = qz
        self.state.orientation.w = qw

        self.euler.x = pi_r
        self.euler.y = theta_r
        self.euler.z = psi_r


    def __init__(self):
        rospy.init_node('ir_detector')
        self.init_pose = rospy.get_param('/uav/flightgoggles_uav_dynamics/init_pose')
        
        self.rate = 10
        self.r = rospy.Rate(self.rate)

        self.camera_matrix = np.array([[548.4088134765625, 0.0, 512.0],
                                       [0.0, 548.4088134765625, 384.0],
                                       [0.0, 0.0, 1.0]])
        max_gate = 23
        self.gate_location = np.zeros((max_gate+1, 4, 3))
        for i in range(0, max_gate):
            location = rospy.get_param('/uav/Gate' + str(i+1) + '/nominal_location')
            for j in range(0, 4):
                for k in range(0, 3):
                    self.gate_location[i][j][k] = location[j][k]
        self.next_gate = 2

        rospy.Subscriber('/gate_number', String, self.next_cb)
        rospy.Subscriber('/uav/camera/left/camera_info', CameraInfo, self.camera_info_cb)
        rospy.Subscriber('/uav/camera/left/ir_beacons', IRMarkerArray, self.ir_cb)
        self.pub_pose = rospy.Publisher('/estimator/ir_pose', Pose, queue_size=10)
        self.pub_velocity = rospy.Publisher('/estimator/ir_velocity', Vector3, queue_size=10)
        self.pub_attitude = rospy.Publisher('/estimator/ir_euler', Vector3, queue_size=10)

        self.state = Pose()
        self.state.position.x = self.init_pose[0]
        self.state.position.y = self.init_pose[1]
        self.state.position.z = self.init_pose[2]

        self.velocity = Vector3()
        self.velocity_tmp = Vector3()
        self.euler = Vector3()
        self.x_tmp = self.init_pose[0]
        self.y_tmp = self.init_pose[1]
        self.z_tmp = self.init_pose[2]

        self.limit_acc = 20


    def loop(self):
        self.x_tmp, self.state.position.x
        self.velocity.x = (self.state.position.x - self.x_tmp) * self.rate
        self.velocity.y = (self.state.position.y - self.y_tmp) * self.rate
        self.velocity.z = (self.state.position.z - self.z_tmp) * self.rate
        if sqrt((self.velocity.x-self.velocity_tmp.x)**2 + (self.velocity.y-self.velocity_tmp.y)**2 + (self.velocity.z-self.velocity_tmp.z)**2) < self.limit_acc:
            self.pub_velocity.publish(self.velocity)
            self.velocity_tmp = self.velocity

        self.x_tmp = self.state.position.x
        self.y_tmp = self.state.position.y
        self.z_tmp = self.state.position.z
        self.r.sleep()


if __name__ == "__main__":
    print("OpenCV: " + cv2.__version__)
    gate_detector = GateDetector()
    while not rospy.is_shutdown():
        gate_detector.loop()