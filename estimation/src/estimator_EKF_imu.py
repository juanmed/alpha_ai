#!/usr/bin/env python

import rospy
import numpy as np
from numpy.linalg import inv, multi_dot
from scipy.signal import cont2discrete
from math import sin, cos, tan, asin, acos, atan2, sqrt

from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Range
from tests.msg import UAV_input


class KalmanFilter():
    def gyro_cb(self, gyro):
        self.u[0][0] = gyro.x
        self.u[1][0] = gyro.y
        self.u[2][0] = gyro.z


    def accel_cb(self, accel):
        self.u[3][0] = accel.x
        self.u[4][0] = accel.y
        self.u[5][0] = accel.z


    def vision_cb(self, pose):
        self.z[0][0] = pose.pose.pose.position.z / self.scale + self.init_pose[0]
        self.z[1][0] = -pose.pose.pose.position.x / self.scale + self.init_pose[1]
        self.z[2][0] = -pose.pose.pose.position.y / self.scale + self.init_pose[2]

        x = pose.pose.pose.orientation.x
        y = pose.pose.pose.orientation.y
        z = pose.pose.pose.orientation.z
        w = pose.pose.pose.orientation.w
        pi_c = atan2(2*(w*x+y*z), 1-2*(pow(x, 2)+pow(y, 2)))
        theta_c = asin(2*(w*y-z*x))
        psi_c = atan2(2*(w*z+x*y), 1-2*(pow(y, 2)+pow(z, 2)))
        self.z[3][0] = psi_c
        self.z[4][0] = -pi_c
        self.z[5][0] = -theta_c

        self.attitude_vo.x = psi_c
        self.attitude_vo.y = -pi_c
        self.attitude_vo.z = -theta_c
        '''
        for i in range(0, 6):
            for j in range(0, 6):
                self.R[i][j] = max(pose.pose.covariance[i*6 + j], 0.0001)
        '''
        self.vision_tf = True


    def range_cb(self, rangefinder):
        pi = self.x_est[3][0]
        theta = self.x_est[4][0]
        self.z[6][0] = (-rangefinder.range) * abs(cos(pi)*cos(theta))
        self.R[6][6] = self.range_var * abs(cos(pi)*cos(theta))


    def getFB(self):
        pi = self.x_est[3][0]
        theta = self.x_est[4][0]
        psi = self.x_est[5][0]
        p = self.u[0][0]
        q = self.u[1][0]
        r = self.u[2][0]
        pi_dot = p + q*sin(pi)*tan(theta) + r*cos(pi)*tan(theta)
        theta_dot = q*cos(pi) - r*sin(pi)
        psi_dot = q*sin(pi)/cos(theta) + r*cos(pi)/cos(theta)

        self.F = np.array([[0, 0, 0, 0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 1],
                           [0, 0, 0, q*cos(pi)*tan(theta)-r*sin(pi)*tan(theta), q*sin(pi)/pow(cos(theta),2)+r*cos(pi)/pow(cos(theta),2), 0, 0, 0, 0],
                           [0, 0, 0, -q*sin(pi)-r*cos(pi), 0, 0, 0, 0, 0],
                           [0, 0, 0, q*cos(pi)/cos(theta)-r*sin(pi)/cos(theta), q*sin(pi)*tan(theta)/cos(theta)+r*cos(pi)*tan(theta)/cos(theta), 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0]])
        self.B = np.array([[0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0]])


    def fx(self, X, U):
        x = X[0][0]
        y = X[1][0]
        z = X[2][0]
        pi = X[3][0]
        theta = X[4][0]
        psi = X[5][0]
        vx = X[6][0]
        vy = X[7][0]
        vz = X[8][0]
        p = U[0][0]
        q = U[1][0]
        r = U[2][0]
        ax_b = U[3][0]
        ay_b = U[4][0]
        az_b = U[5][0]

        ax_w = ax_b*cos(theta)*cos(psi) + ay_b*cos(theta)*sin(psi) + az_b*(-sin(theta))
        ay_w = ax_b*(sin(pi)*sin(theta)*cos(psi)-cos(pi)*sin(psi)) + ay_b*(sin(pi)*sin(theta)*sin(psi)+cos(pi)*cos(psi)) + az_b*sin(pi)*cos(theta)
        az_w = ax_b*(cos(pi)*sin(theta)*cos(psi)+sin(pi)*sin(psi)) + ay_b*(cos(pi)*sin(theta)*sin(psi)-sin(pi)*cos(psi)) + az_b*cos(pi)*cos(theta)
        p_dot = (self.arm_length/sqrt(2)*U[1][0] + (self.Iyy-self.Izz)*q*r)/self.Ixx
        q_dot = (self.arm_length/sqrt(2)*U[2][0] + (self.Izz-self.Ixx)*r*p)/self.Iyy
        r_dot = (self.k_torque/self.k_thrust*U[3][0] + (self.Ixx-self.Iyy)*p*q)/self.Izz
        pi_dot = p + q*sin(pi)*tan(theta) + r*cos(pi)*tan(theta)
        theta_dot = q*cos(pi) - r*sin(pi)
        psi_dot = q*sin(pi)/cos(theta) + r*cos(pi)/cos(theta)
        pi_dotdot = p_dot + q_dot*sin(pi)*tan(theta)+q*cos(pi)*tan(theta)+q*sin(pi)/pow(cos(theta),2) + r_dot*cos(pi)*tan(theta)-r*sin(pi)*tan(theta)+r*cos(pi)/pow(cos(theta),2)
        theta_dotdot = q_dot*cos(pi)-q*sin(pi) - r_dot*sin(pi)-r*cos(pi)
        psi_dotdot = q_dot*sin(pi)/cos(theta)+q*cos(pi)/cos(theta)+q*sin(pi)*tan(theta)/cos(theta) + r_dot*cos(pi)/cos(theta)-r*sin(pi)/cos(theta)-r*cos(pi)*tan(theta)/cos(theta)

        fx = np.array([[x+vx*self.dT+ax_w*(self.dT**2)/2], [y+vy*self.dT+ay_w*(self.dT**2)/2], [z+vz*self.dT+az_w*(self.dT**2)/2],
                       [pi+pi_dot*self.dT+pi_dotdot*(self.dT**2)/2], [theta+theta_dot*self.dT+theta_dotdot*(self.dT**2)/2], [psi+psi_dot*self.dT+psi_dotdot*(self.dT**2)/2],
                       [vx+ax_w*self.dT], [vy+ay_w*self.dT], [vz+az_w*self.dT]])
        return fx


    def gethx(self, X):
        x = X[0][0]
        y = X[1][0]
        z = X[2][0]
        pi = X[3][0]
        theta = X[4][0]
        psi = X[5][0]

        hx = np.array([[x], [y], [z], [pi], [theta], [psi],
                       [z]])
        return hx


    def limitAngle(self):
        while self.x_pre[3][0]>=np.pi or self.x_pre[3][0]<(-np.pi):
            if self.x_pre[3][0] >= np.pi:
                self.x_pre[3][0] -= 2*np.pi
            else:
                self.x_pre[3][0] += 2*np.pi
        while self.x_pre[4][0]>=np.pi or self.x_pre[4][0]<(-np.pi):
            if self.x_pre[4][0] >= np.pi:
                self.x_pre[4][0] -= 2*np.pi
            else:
                self.x_pre[4][0] += 2*np.pi
        while self.x_pre[5][0]>=np.pi or self.x_pre[5][0]<(-np.pi):
            if self.x_pre[5][0] >= np.pi:
                self.x_pre[5][0] -= 2*np.pi
            else:
                self.x_pre[5][0] += 2*np.pi

        while self.x_est[3][0]>=np.pi or self.x_est[3][0]<(-np.pi):
            if self.x_est[3][0] >= np.pi:
                self.x_est[3][0] -= 2*np.pi
            else:
                self.x_est[3][0] += 2*np.pi
        while self.x_est[4][0]>=np.pi or self.x_est[4][0]<(-np.pi):
            if self.x_est[4][0] >= np.pi:
                self.x_est[4][0] -= 2*np.pi
            else:
                self.x_est[4][0] += 2*np.pi
        while self.x_est[5][0]>=np.pi or self.x_est[5][0]<(-np.pi):
            if self.x_est[5][0] >= np.pi:
                self.x_est[5][0] -= 2*np.pi
            else:
                self.x_est[5][0] += 2*np.pi


    def setState(self):
        self.state.header.stamp = rospy.Time.now()
        self.state.pose.pose.position.x = self.x_est[0][0]
        self.state.pose.pose.position.y = self.x_est[1][0]
        self.state.pose.pose.position.z = self.x_est[2][0]
        self.state.pose.pose.orientation.x = sin(self.x_est[3][0]/2)*cos(self.x_est[4][0]/2)*cos(self.x_est[5][0]/2) - cos(self.x_est[3][0]/2)*sin(self.x_est[4][0]/2)*sin(self.x_est[5][0]/2)
        self.state.pose.pose.orientation.y = sin(self.x_est[3][0]/2)*cos(self.x_est[4][0]/2)*sin(self.x_est[5][0]/2) + cos(self.x_est[3][0]/2)*sin(self.x_est[4][0]/2)*cos(self.x_est[5][0]/2)
        self.state.pose.pose.orientation.z = cos(self.x_est[3][0]/2)*cos(self.x_est[4][0]/2)*sin(self.x_est[5][0]/2) - sin(self.x_est[3][0]/2)*sin(self.x_est[4][0]/2)*cos(self.x_est[5][0]/2)
        self.state.pose.pose.orientation.w = cos(self.x_est[3][0]/2)*cos(self.x_est[4][0]/2)*cos(self.x_est[5][0]/2) + sin(self.x_est[3][0]/2)*sin(self.x_est[4][0]/2)*sin(self.x_est[5][0]/2)
        self.state.twist.twist.linear.x = self.x_est[6][0]
        self.state.twist.twist.linear.y = self.x_est[7][0]
        self.state.twist.twist.linear.z = self.x_est[8][0]
        self.state.twist.twist.angular.x = self.u[0][0]
        self.state.twist.twist.angular.y = self.u[1][0]
        self.state.twist.twist.angular.z = self.u[2][0]

        self.position.x = self.x_est[0][0]
        self.position.y = self.x_est[1][0]
        self.position.z = self.x_est[2][0]
        self.attitude.x = self.x_est[3][0]
        self.attitude.y = self.x_est[4][0]
        self.attitude.z = self.x_est[5][0]
        self.linear_velocity.x = self.x_est[6][0]
        self.linear_velocity.y = self.x_est[7][0]
        self.linear_velocity.z = self.x_est[8][0]


    def __init__(self):
        rospy.init_node('estimator')

        self.mass = rospy.get_param('/uav/flightgoggles_uav_dynamics/vehicle_mass')
        self.Ixx = rospy.get_param('/uav/flightgoggles_uav_dynamics/vehicle_inertia_xx')
        self.Iyy = rospy.get_param('/uav/flightgoggles_uav_dynamics/vehicle_inertia_yy')
        self.Izz = rospy.get_param('/uav/flightgoggles_uav_dynamics/vehicle_inertia_zz')
        self.arm_length = rospy.get_param('/uav/flightgoggles_uav_dynamics/moment_arm')
        self.k_thrust = rospy.get_param('/uav/flightgoggles_uav_dynamics/thrust_coefficient')
        self.k_torque = rospy.get_param('/uav/flightgoggles_uav_dynamics/torque_coefficient')
        self.drag = rospy.get_param('/uav/flightgoggles_uav_dynamics/drag_coefficient')

        self.gyro_var = rospy.get_param('/uav/flightgoggles_imu/gyroscope_variance')
        self.accel_var = rospy.get_param('/uav/flightgoggles_imu/accelerometer_variance')
        self.range_var = rospy.get_param('/uav/flightgoggles_laser/rangefinder_variance')
        self.init_pose = rospy.get_param('/uav/flightgoggles_uav_dynamics/init_pose')

        self.rate = 100
        self.r = rospy.Rate(self.rate)
        self.dT = 1.0/self.rate

        self.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],     # visual odometry
                           [0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 1, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 1, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0, 0, 0, 0]])    # rangefinder
        self.D = np.zeros((7, 6))

        
        self.Q = np.array([[0.0005, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0.0005, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0.0005, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0.00025, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0.00025, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0.00025, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0]])
        
        self.R = np.array([[0.000001, 0, 0, 0, 0, 0, 0],        # visual odometry
                           [0, 0.000001, 0, 0, 0, 0, 0],
                           [0, 0, 0.000001, 0, 0, 0, 0],
                           [0, 0, 0, 0.000001, 0, 0, 0],
                           [0, 0, 0, 0, 0.000001, 0, 0],
                           [0, 0, 0, 0, 0, 0.000001, 0],
                           [0, 0, 0, 0, 0, 0, self.range_var]]) # rangefinder

        self.P_pre = np.eye(9)*0.001
        self.P_est = np.eye(9)*0.001

        self.x_pre = np.zeros((9, 1))
        self.x_est = np.zeros((9, 1))
        self.z = np.zeros((7, 1))
        self.hx = np.zeros((7, 1))
        self.u = np.zeros((6, 1))

        self.x_est[0][0] = self.init_pose[0]
        self.x_est[1][0] = self.init_pose[1]
        self.x_est[2][0] = self.init_pose[2]
        x = self.init_pose[3]
        y = self.init_pose[4]
        z = self.init_pose[5]
        w = self.init_pose[6]
        self.x_est[3][0] = atan2(2*(w*x+y*z), 1-2*(pow(x, 2)+pow(y, 2)))
        self.x_est[4][0] = asin(2*(w*y-z*x))
        self.x_est[5][0] = atan2(2*(w*z+x*y), 1-2*(pow(y, 2)+pow(z, 2)))

        self.vision_tf = False
        self.scale = 0.055 #*202/185

        rospy.Subscriber('/svo/pose_imu', PoseWithCovarianceStamped, self.vision_cb)
        rospy.Subscriber('/uav/angular_velocity', Vector3, self.gyro_cb)
        rospy.Subscriber('/uav/linear_acceleration', Vector3, self.accel_cb)
        rospy.Subscriber('/uav/sensors/downward_laser_rangefinder', Range, self.range_cb)

        self.pub_state = rospy.Publisher('/uav/state', Odometry, queue_size=10)
        self.pub_position = rospy.Publisher('/uav/position', Vector3, queue_size=10)
        self.pub_attitude = rospy.Publisher('/uav/attitude', Vector3, queue_size=10)
        self.pub_linear_velocity = rospy.Publisher('/uav/linear_velocity', Vector3, queue_size=10)
        self.pub_attitude_vo = rospy.Publisher('/uav/attitude_vo', Vector3, queue_size=10)

        self.state = Odometry()
        self.position = Vector3()
        self.attitude = Vector3()
        self.linear_velocity = Vector3()
        self.attitude_vo = Vector3()


    def loop(self):
        # predict
        self.getFB()
        (self.Fd, self.Bd, self.H, self.D, self.dT) = cont2discrete((self.F, self.B, self.H, self.D), self.dT)
        self.x_pre = self.fx(self.x_est, self.u)
        self.P_pre = multi_dot([self.Fd, self.P_est, self.Fd.T]) + self.Q

        # calculate Kalman Gain, estimate
        if self.vision_tf is True:
            self.K = multi_dot([self.P_pre, self.H.T, inv(multi_dot([self.H, self.P_pre, self.H.T]) + self.R)])
            self.hx = self.gethx(self.x_pre)
            self.x_est = self.x_pre + np.dot(self.K, self.z - self.hx)
            #self.P_est = np.dot(np.eye(9)-np.dot(self.K, self.H), self.P_pre)
            self.P_est = multi_dot([np.eye(9)-np.dot(self.K, self.H), self.P_pre, (np.eye(9)-np.dot(self.K, self.H)).T]) + multi_dot([self.K, self.R, self.K.T])
        else:
            self.K = multi_dot([self.P_pre, self.H[6:, :].T, inv(multi_dot([self.H[6:, :], self.P_pre, self.H[6:, :].T]) + self.R[6:, 6:])])
            self.hx = self.gethx(self.x_pre)
            self.x_est = self.x_pre + np.dot(self.K, self.z[6:, :] - self.hx[6:, :])
            #self.P_est = np.dot(np.eye(9)-np.dot(self.K, self.H[6:, :]), self.P_pre)
            self.P_est = multi_dot([np.eye(9)-np.dot(self.K, self.H[6:, :]), self.P_pre, (np.eye(9)-np.dot(self.K, self.H[6:, :])).T]) + multi_dot([self.K, self.R[6:, 6:], self.K.T])

        self.limitAngle()
        self.setState()
        self.pub_state.publish(self.state)
        self.pub_position.publish(self.position)
        self.pub_attitude.publish(self.attitude)
        self.pub_linear_velocity.publish(self.linear_velocity)
        self.pub_attitude_vo.publish(self.attitude_vo)

        self.vision_tf = False
        self.r.sleep()


if __name__ == "__main__":
    kalman_filter = KalmanFilter()
    while not rospy.is_shutdown():
        kalman_filter.loop()