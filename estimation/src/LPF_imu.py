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


class LowPassFilter():
    def imu_cb(self, imu):
        self.z[0][0] = imu.angular_velocity.x   # p
        self.z[1][0] = imu.angular_velocity.y   # q
        self.z[2][0] = imu.angular_velocity.z   # r

        ax_b = imu.linear_acceleration.x
        ay_b = imu.linear_acceleration.y
        az_b = imu.linear_acceleration.z

        self.z[3][0] = ax_b
        self.z[4][0] = ay_b
        self.z[5][0] = az_b

        self.x[0][0] = self.x[0][0]*(1-self.alpha_gyro) + self.z[0][0]*self.alpha_gyro
        self.x[1][0] = self.x[1][0]*(1-self.alpha_gyro) + self.z[1][0]*self.alpha_gyro
        self.x[2][0] = self.x[2][0]*(1-self.alpha_gyro) + self.z[2][0]*self.alpha_gyro
        self.x[3][0] = self.x[3][0]*(1-self.alpha_acc) + self.z[3][0]*self.alpha_acc
        self.x[4][0] = self.x[4][0]*(1-self.alpha_acc) + self.z[4][0]*self.alpha_acc
        self.x[5][0] = self.x[5][0]*(1-self.alpha_acc) + self.z[5][0]*self.alpha_acc

        self.setState()
        self.pub_angular_velocity.publish(self.angular_velocity)
        self.pub_linear_acceleration.publish(self.linear_acceleration)


    def setState(self):
        self.angular_velocity.x = self.x[0][0]
        self.angular_velocity.y = self.x[1][0]
        self.angular_velocity.z = self.x[2][0]
        self.linear_acceleration.x = self.x[3][0]
        self.linear_acceleration.y = self.x[4][0]
        self.linear_acceleration.z = self.x[5][0]


    def __init__(self):
        rospy.init_node('lpf')

        self.rate = 1000
        self.r = rospy.Rate(self.rate)
        self.dT = 1.0/self.rate

        self.g = 9.81
        self.gyro_var = rospy.get_param('/uav/flightgoggles_imu/gyroscope_variance')
        self.accel_var = rospy.get_param('/uav/flightgoggles_imu/accelerometer_variance')
        self.alpha_gyro = 0.05
        self.alpha_acc = 0.01
        
        self.x = np.zeros((6, 1))
        self.z = np.zeros((6, 1))

        rospy.Subscriber('/uav/sensors/imu', Imu, self.imu_cb)
        self.pub_angular_velocity = rospy.Publisher('/estimator/lpf/angular_velocity', Vector3, queue_size=10)
        self.pub_linear_acceleration = rospy.Publisher('/estimator/lpf/linear_acceleration', Vector3, queue_size=10)
        self.past = rospy.Time.now()

        self.angular_velocity = Vector3()
        self.linear_acceleration = Vector3()


    def loop(self):
        self.r.sleep()


if __name__ == "__main__":
    lpf = LowPassFilter()
    while not rospy.is_shutdown():
        lpf.loop()
