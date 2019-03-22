#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import re
import math
from math import sqrt, sin, cos, asin, atan2

from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from flightgoggles.msg import IRMarker, IRMarkerArray
from tests.msg import UAV_state

class GateDetector():
    def camera_info_cb(self, camera_info):
        self.camera_matrix = np.array(camera_info.K).reshape(3, 3)

    def next_cb(self, gate):
        self.next_gate = self.getID(gate.data)

    def ir_cb(self, ir_array):
        next_marker = [0, 0, 0, 0]
        next_array_num = [0, 0, 0, 0]
        num = len(ir_array.markers)
        nnum = 0
        next_cnt = 0
        for i in range(0, num):
            if self.gate_perturbation[self.getID(ir_array.markers[i].landmarkID)-1] == 0:
                nnum += 1
            if self.getID(ir_array.markers[i].landmarkID) == self.next_gate:
                next_marker[next_cnt] = self.getID(ir_array.markers[i].markerID)
                next_array_num[next_cnt] = i
                next_cnt += 1
        '''
        if next_cnt == 4 and self.gate_perturbation[self.next_gate-1] != 0:
            object_points = np.zeros((4, 3))
            image_points = np.zeros((4, 2))
            for i in range(0, 4):
                object_points[i][0] = self.gate_location[self.next_gate-1][next_marker[i]-1][0]
                object_points[i][1] = self.gate_location[self.next_gate-1][next_marker[i]-1][1]
                object_points[i][2] = self.gate_location[self.next_gate-1][next_marker[i]-1][2]
                image_points[i][0] = ir_array.markers[next_array_num[i]].x
                image_points[i][1] = ir_array.markers[next_array_num[i]].y
            rvec, tvec = self.getPosePnP(object_points, np.ascontiguousarray(image_points[:, :2]).reshape((4, 1, 2)), cv2.SOLVEPNP_AP3P)
            #self.setGateState(rvec, tvec, self.next_gate)
        nnum = 0
        next_cnt = 0
        for i in range(0, num):
            if self.gate_perturbation[self.getID(ir_array.markers[i].landmarkID)-1] == 0:
                nnum += 1
            if self.getID(ir_array.markers[i].landmarkID) == self.next_gate:
                next_marker[next_cnt] = self.getID(ir_array.markers[i].markerID)
                next_array_num[next_cnt] = i
                next_cnt += 1
        '''
        print "Detected IR markers:", num, nnum
        if nnum >= 5:
            object_points = np.zeros((nnum, 3))
            image_points = np.zeros((nnum, 2))
            j = 0
            for i in range(0, num):
                gate_id = self.getID(ir_array.markers[i].landmarkID)
                marker_id = self.getID(ir_array.markers[i].markerID)
                if self.gate_perturbation[gate_id-1] == 0:
                    object_points[j][0] = self.gate_location[gate_id-1][marker_id-1][0]
                    object_points[j][1] = self.gate_location[gate_id-1][marker_id-1][1]
                    object_points[j][2] = self.gate_location[gate_id-1][marker_id-1][2]
                    image_points[j][0] = ir_array.markers[i].x
                    image_points[j][1] = ir_array.markers[i].y
                    j += 1
            rvec, tvec = self.getPosePnP(object_points, np.ascontiguousarray(image_points[:, :2]).reshape((nnum, 1, 2)), cv2.SOLVEPNP_EPNP)
            print 'EPNP'
            self.setState(rvec, tvec)
            self.pub_pose.publish(self.state)
            self.pub_attitude.publish(self.euler)
            if self.velocity_tf is True:
                self.pub_velocity.publish(self.velocity)
                self.velocity_tf = False

        #elif (next_cnt == 4) and (self.gate_perturbation[self.next_gate-1] == 0):
        elif next_cnt == 4:
            print "Next gate:", self.next_gate, next_cnt
            object_points = np.zeros((4, 3))
            image_points = np.zeros((4, 2))
            for i in range(0, 4):
                object_points[i][0] = self.gate_location[self.next_gate-1][next_marker[i]-1][0]
                object_points[i][1] = self.gate_location[self.next_gate-1][next_marker[i]-1][1]
                object_points[i][2] = self.gate_location[self.next_gate-1][next_marker[i]-1][2]
                image_points[i][0] = ir_array.markers[next_array_num[i]].x
                image_points[i][1] = ir_array.markers[next_array_num[i]].y
            rvec, tvec = self.getPosePnP(object_points, np.ascontiguousarray(
                image_points[:, :2]).reshape((4, 1, 2)), cv2.SOLVEPNP_AP3P)
            print 'AP3P'
            self.setState(rvec, tvec)
            self.pub_attitude.publish(self.euler)
            self.pub_velocity.publish(self.velocity)
            if self.velocity_tf is True:
                self.pub_pose.publish(self.state)
                self.velocity_tf = False

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
            rvec, tvec = self.getPosePnP(object_points, np.ascontiguousarray(image_points[:, :2]).reshape((num, 1, 2)), cv2.SOLVEPNP_EPNP)
            # print 'EPNP'
            self.setState(rvec, tvec)
            if self.velocity_tf is True:
                self.pub_pose.publish(self.state)
                self.pub_attitude.publish(self.euler)
                self.pub_velocity.publish(self.velocity)
                self.velocity_tf = False

    def state_cb(self, state):
        self.drone_position[0] = state.pose.position.x
        self.drone_position[1] = state.pose.position.y
        self.drone_position[2] = state.pose.position.z
        self.drone_orientation[0] = state.pose.orientation.x
        self.drone_orientation[1] = state.pose.orientation.y
        self.drone_orientation[2] = state.pose.orientation.z
        self.drone_orientation[3] = state.pose.orientation.w

    def gate_cb(self, gate_array):
        num = len(gate_array.markers)
        for i in range(0, num):
            self.gate_location[gate_array.markers[i].landmarkID-1][self.getID(ir_array.markers[i].markerID)-1][0] = gate_array.markers[i].x
            self.gate_location[gate_array.markers[i].landmarkID-1][self.getID(ir_array.markers[i].markerID)-1][1] = gate_array.markers[i].y
            self.gate_location[gate_array.markers[i].landmarkID-1][self.getID(ir_array.markers[i].markerID)-1][2] = gate_array.markers[i].z
            self.gate_perturbation[gate_array.markers[i].landmarkID-1] = 0

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

    def euler2rotation(self, pi, theta, psi):
        Rx = np.array([[1, 0, 0],
                       [0, cos(pi), -sin(pi)],
                       [0, sin(pi), cos(pi)]])
        Ry = np.array([[cos(theta), 0, sin(theta)],
                       [0, 1, 0],
                       [-sin(theta), 0, cos(theta)]])
        Rz = np.array([[cos(psi), -sin(psi), 0],
                       [sin(psi), cos(psi), 0],
                       [0, 0, 1]])
        return np.dot(Rx, np.dot(Ry, Rz))

    def setState(self, rvec, tvec):
        R = cv2.Rodrigues(rvec)[0]
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

        self.state.position.x = (1-self.alpha)*self.x_backup + self.alpha*t[0][0]
        self.state.position.y = (1-self.alpha)*self.y_backup + self.alpha*t[1][0]
        self.state.position.z = (1-self.alpha)*self.z_backup + self.alpha*t[2][0]
        self.state.orientation.x = qx
        self.state.orientation.y = qy
        self.state.orientation.z = qz
        self.state.orientation.w = qw

        self.euler.x = pi_r
        self.euler.y = theta_r
        self.euler.z = psi_r

        self.x_backup = self.state.position.x
        self.y_backup = self.state.position.y
        self.z_backup = self.state.position.z
    '''
    def setGateState(self, rvec, tvec, gate):
        print self.gate_location[gate-1]
        R = cv2.Rodrigues(rvec)[0]
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
        
        Gate = np.zeros((3, 1))
        for i in range(0, 4):
            Gate[0][0] += (self.gate_location[gate-1][i][0] - t[0][0]) / 4
            Gate[1][0] += (self.gate_location[gate-1][i][1] - t[1][0]) / 4
            Gate[2][0] += (self.gate_location[gate-1][i][2] - t[2][0]) / 4
        for i in range(0, 4):
            self.gate_location[gate-1][i][0] = self.drone_position[0] + g[1]
            self.gate_location[gate-1][i][1] = self.drone_position[1] + g[0]
            self.gate_location[gate-1][i][2] = self.drone_position[2] + g[2]
        self.gate_perturbation[gate-1] = 0
        
        print self.gate_location[gate-1]
    '''
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
        self.gate_perturbation = np.zeros(max_gate+1)
        for i in range(0, max_gate):
            location = rospy.get_param('/uav/Gate' + str(i+1) + '/nominal_location')
            perturbation = rospy.get_param('/uav/Gate' + str(i+1) + '/perturbation_bound')
            self.gate_perturbation[i] = perturbation[0] + perturbation[1] + perturbation[2]
            for j in range(0, 4):
                for k in range(0, 3):
                    self.gate_location[i][j][k] = location[j][k]
        self.next_gate = self.getID(rospy.get_param("/uav/gate_names")[0])

        rospy.Subscriber('/gate_number', String, self.next_cb)
        rospy.Subscriber('/uav/camera/left/camera_info', CameraInfo, self.camera_info_cb)
        rospy.Subscriber('/uav/camera/left/ir_beacons', IRMarkerArray, self.ir_cb)
        rospy.Subscriber('/estimator/state', UAV_state, self.state_cb)
        rospy.Subscriber('/modified_gate', IRMarkerArray, self.gate_cb)

        self.pub_pose = rospy.Publisher('/estimator/ir_pose', Pose, queue_size=10)
        self.pub_velocity = rospy.Publisher('/estimator/ir_velocity', Vector3, queue_size=10)
        self.pub_attitude = rospy.Publisher('/estimator/ir_euler', Vector3, queue_size=10)

        self.state = Pose()
        self.state.position.x = self.init_pose[0]
        self.state.position.y = self.init_pose[1]
        self.state.position.z = self.init_pose[2]
        self.drone_position = np.array([self.init_pose[0], self.init_pose[1], self.init_pose[2]])
        self.drone_orientation = np.array([self.init_pose[0], self.init_pose[1], self.init_pose[2], self.init_pose[3]])

        self.velocity = Vector3()
        self.euler = Vector3()

        self.alpha = 1.0
        self.x_backup = 0.0
        self.y_backup = 0.0
        self.z_backup = 0.0

        self.x_tmp = self.init_pose[0]
        self.y_tmp = self.init_pose[1]
        self.z_tmp = self.init_pose[2]
        self.vx_tmp = 0.0
        self.vy_tmp = 0.0
        self.vz_tmp = 0.0
        self.velocity_tf = False
        self.limit_acc = 5

    def loop(self):
        self.velocity.x = (self.state.position.x - self.x_tmp) * self.rate
        self.velocity.y = (self.state.position.y - self.y_tmp) * self.rate
        self.velocity.z = (self.state.position.z - self.z_tmp) * self.rate
        if sqrt((self.velocity.x-self.vx_tmp)**2 + (self.velocity.y-self.vy_tmp)**2 + (self.velocity.z-self.vz_tmp)**2)*self.rate < self.limit_acc:
            self.vx_tmp = self.velocity.x
            self.vy_tmp = self.velocity.y
            self.vz_tmp = self.velocity.z
            self.velocity_tf = True

        self.x_tmp = self.state.position.x
        self.y_tmp = self.state.position.y
        self.z_tmp = self.state.position.z

        self.r.sleep()

if __name__ == "__main__":
    #print("OpenCV: " + cv2.__version__)
    gate_detector = GateDetector()
    while not rospy.is_shutdown():
        gate_detector.loop()
