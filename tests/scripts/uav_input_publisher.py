#!/usr/bin/env python

# support libraries
import rospy
import message_filters
import tf
import numpy as np

# messages
from tests.msg import UAV_traj
from tests.msg import UAV_state
from mav_msgs.msg import RateThrust

import lqr_gains as lqrg

g = 9.81

class uav_Input_Publisher():

    # constructor
    def __init__(self):

        # publisher 
        self.input_publisher = rospy.Publisher('/uav/input/rateThrust', RateThrust, queue_size = 10)

        # create message message_filter
        self.state_sub = message_filters.Subscriber('/uav_state', UAV_state)
        self.reftraj_sub = message_filters.Subscriber('/uav_ref_trajectory', UAV_traj)

        # filter messages based on time
        ts = message_filters.ApproximateTimeSynchronizer([self.state_sub, self.reftraj_sub],10,0.01)
        ts.registerCallback(self.callback2)

        self.m = rospy.get_param("/uav/flightgoggles_uav_dynamics/vehicle_mass")

        # LQR PERFORMANCE MATRICES
        # output performance matrix for translation variables
        self.Qt = np.diag([100.0,1.0])
        # input effort matrix for translation variables
        self.Rt = np.array([1.0])   
        # output performance matrix for rotational variables
        self.Qr = np.diag([5.0])
        # input effort matrix for rotational variables
        self.Rr = np.array([1.0])

        # get controller gains with LQR method
        #self.Kt, self.N_ut, self.N_xt = lqrg.calculate_LQR_gains(lqrg.At, lqrg.Bt, lqrg.Ct, lqrg.D_, self.Qt, self.Rt)
        #self.Kr, self.N_ur, self.N_xr = lqrg.calculate_LQR_gains(lqrg.Ar, lqrg.Br, lqrg.Cr, lqrg.D_, self.Qr, self.Rr)


        # POLE PLACEMENT DESIRED POLES
        # Desired pole locations for pole placement method, for more aggresive tracking
        self.dpt = np.array([-3.0+10j,-3.0-10j])
        self.dpr = np.array([-8.0])

        # get controller gains with Pole Placement Method
        self.Kt, self.N_ut, self.N_xt = lqrg.calculate_pp_gains(lqrg.At, lqrg.Bt, lqrg.Ct, lqrg.D_, self.dpt)
        self.Kr, self.N_ur, self.N_xr = lqrg.calculate_pp_gains(lqrg.Ar, lqrg.Br, lqrg.Cr, lqrg.D_, self.dpr)


        # send one message with Thrust > 10 for uav/sensors/imu to start
        #rt_msg = RateThrust()
        #rt_msg.thrust.z = 10
        #self.input_publisher.publish(rt_msg)


    def callback(self, state_msg, traj_msg):
 

        # In general    u = -K*x + (N_u + K*N_x)*r
        # r = reference state
        # x = state
        # K = LQR gains
        # N_u, N_x = refrence input and reference state matrices       

        # extract reference values
        x_r, y_r, z_r = [traj_msg.pose.position.x, traj_msg.pose.position.y, traj_msg.pose.position.z]
        vx_r, vy_r, vz_r = [traj_msg.twist.linear.x, traj_msg.twist.linear.y, traj_msg.twist.linear.z] 
        ori_quat_r = [traj_msg.pose.orientation.x, traj_msg.pose.orientation.y, traj_msg.pose.orientation.z, traj_msg.pose.orientation.w]
        psi_r, theta_r, phi_r = tf.transformations.euler_from_quaternion(ori_quat_r, axes = 'rzyx')
        p_r, q_r, r_r = [traj_msg.twist.angular.x, traj_msg.twist.angular.y, traj_msg.twist.angular.z]

        # extract drone real state values
        x, y, z = [state_msg.pose.position.x, state_msg.pose.position.y, state_msg.pose.position.z]
        vx, vy, vz = [state_msg.twist.linear.x, state_msg.twist.linear.y, state_msg.twist.linear.z]
        ori_quat = [state_msg.pose.orientation.x, state_msg.pose.orientation.y, state_msg.pose.orientation.z, state_msg.pose.orientation.w]
        psi, theta, phi = tf.transformations.euler_from_quaternion(ori_quat, axes = 'rzyx')
        p, q, r = [state_msg.twist.angular.x, state_msg.twist.angular.y, state_msg.twist.angular.z]

        #compute state error:  error = reference - real
        x_e, y_e, z_e = [x - x_r, y - y_r, z - z_r]
        vx_e, vy_e, vz_e = [vx - vx_r, vy - vy_r, vz - vz_r]
        phi_e, theta_e, psi_e = [phi - phi_r, theta - theta_r, psi - psi_r]
        p_e, q_e, r_e = [p - p_r, q - q_r, r -r_r]

        # compute input error u = -Kx
        # first, for translation dynamics
        ua_e_x = -1.0*np.dot(self.Kt,np.array([[x_e],[vx_e]]))[0]
        ua_e_y = -1.0*np.dot(self.Kt,np.array([[y_e],[vy_e]]))[0]
        ua_e_z = -1.0*np.dot(self.Kt,np.array([[z_e],[vz_e]]))[0]

        # second, for orientation dynamics
        uc_e_x = -1.0*np.dot(self.Kr,np.array([phi_e]))
        uc_e_y = -1.0*np.dot(self.Kr,np.array([theta_e]))
        uc_e_z = -1.0*np.dot(self.Kr,np.array([psi_e]))
        #print("uc_e_x: {} type: {}".format(uc_e_x,type(uc_e_x)) )
        #print("ua_e_x: {} type: {}".format(ua_e_x,type(ua_e_x)) )

        # the final input to the drone is u = u_e + u_r
        # input =  error_input + reference_input
        #print("traj_msg.ua.x: {} type: {}".format(traj_msg.ua.x,type(traj_msg.ua.x)) )
        uax, uay, uaz = [traj_msg.ua.x + ua_e_x[0], traj_msg.ua.y + ua_e_y[0], traj_msg.ua.z + ua_e_z[0]] 
        ucx, ucy, ucz = [traj_msg.uc.x + uc_e_x[0], traj_msg.uc.y + uc_e_y[0], traj_msg.uc.z + uc_e_z[0]]
        #print("uax: {} type: {}".format(uax,type(uax)) )

        ua = np.array([[uax], [uay], [uaz]])
        uc = np.array([[ucx], [ucy], [ucz]])
        #print(ua,uc)

        # compute Thrust -T- as
        #   T = m*wzb.T*(ua + g*zw)
        #
        zb = np.array([[0.0],[0.0],[1.0]])  #  z axis of body expressed in body frame
        zw = np.array([[0.0],[0.0],[1.0]])  #  z axis of world frame expressed in world frame 
        Rwb = tf.transformations.euler_matrix(psi_r, theta_r, phi_r, axes='rzyx') # world to body frame rotation
        Rbw = Rwb[0:3,0:3].T  # body to world frame rotation only

        wzb = np.dot(Rbw,zb) # zb expressed in world frame
        T =  self.m*np.dot(wzb.T,(ua + g*zw))[0]
        #print(wzb.T,type(wzb))

        # compute w_b angular velocity commands as
        #  w_b = K.inv * uc
        #  where  (euler angle derivate) = K*w_b
        K = np.array([[1.0, np.sin(phi_r)*np.tan(theta_r), np.cos(phi_r)*np.tan(theta_r)],
                      [0.0, np.cos(phi_r), -1.0*np.sin(phi_r)],
                      [0.0, np.sin(phi_r)/np.cos(theta_r), np.cos(phi_r)/np.cos(theta_r)]])

        Kinv = np.linalg.inv(K)

        w_b = np.dot(Kinv, uc).flatten()

        # create and fill message
        rt_msg = RateThrust()
        
        rt_msg.header.stamp = rospy.Time.now()
        rt_msg.header.frame_id = 'uav/imu'

        rt_msg.angular_rates.x = w_b[0] #traj_msg.twist.angular.x
        rt_msg.angular_rates.y = w_b[1] #traj_msg.twist.angular.y
        rt_msg.angular_rates.z = w_b[2] #traj_msg.twist.angular.z

        rt_msg.thrust.x = 0.0
        rt_msg.thrust.y = 0.0
        rt_msg.thrust.z = T[0] # self.m*np.linalg.norm(t)
        
        # publish
        self.input_publisher.publish(rt_msg)
        rospy.loginfo(rt_msg)
        

    def callback2(self, state_msg, traj_msg):
        # In general    u = -K*x + (N_u + K*N_x)*r
        # r = reference state
        # x = state
        # K = LQR gains
        # N_u, N_x = refrence input and reference state matrices       

        # extract reference values
        x_r, y_r, z_r = [traj_msg.pose.position.x, traj_msg.pose.position.y, traj_msg.pose.position.z]
        vx_r, vy_r, vz_r = [traj_msg.twist.linear.x, traj_msg.twist.linear.y, traj_msg.twist.linear.z] 
        ori_quat_r = [traj_msg.pose.orientation.x, traj_msg.pose.orientation.y, traj_msg.pose.orientation.z, traj_msg.pose.orientation.w]
        psi_r, theta_r, phi_r = tf.transformations.euler_from_quaternion(ori_quat_r, axes = 'rzyx')
        p_r, q_r, r_r = [traj_msg.twist.angular.x, traj_msg.twist.angular.y, traj_msg.twist.angular.z]

        # extract drone real state values
        x, y, z = [state_msg.pose.position.x, state_msg.pose.position.y, state_msg.pose.position.z]
        vx, vy, vz = [state_msg.twist.linear.x, state_msg.twist.linear.y, state_msg.twist.linear.z]
        ori_quat = [state_msg.pose.orientation.x, state_msg.pose.orientation.y, state_msg.pose.orientation.z, state_msg.pose.orientation.w]
        psi, theta, phi = tf.transformations.euler_from_quaternion(ori_quat, axes = 'rzyx')
        p, q, r = [state_msg.twist.angular.x, state_msg.twist.angular.y, state_msg.twist.angular.z]

        # x position dynamics control law
        x_state = np.array([[x],[vx]])
        ua_e_x = -1.0*np.dot(self.Kt,x_state) + (self.N_ut + np.dot(self.Kt,self.N_xt))*x_r
        # y position dynamics control law
        y_state = np.array([[y],[vy]])
        ua_e_y = -1.0*np.dot(self.Kt,y_state) + (self.N_ut + np.dot(self.Kt,self.N_xt))*y_r
        # z position dynamics control law
        z_state = np.array([[z],[vz]])
        ua_e_z = -1.0*np.dot(self.Kt,z_state) + (self.N_ut + np.dot(self.Kt,self.N_xt))*z_r

        # compose position dynamics input vector
        #ua_e = np.array([[ua_e_x[0]],[ua_e_y[0]],[ua_e_z[0]]])


        # phi (x axis) angular position dynamics control law
        uc_e_x = -1.0*self.Kr*phi + (self.N_ur + np.dot(self.Kr,self.N_xr))*phi_r
        # theta (y axis) angular position dynamics control law
        uc_e_y = -1.0*self.Kr*theta + (self.N_ur + np.dot(self.Kr,self.N_xr))*theta_r
        # psi (z axis) angular position dynamics control law
        uc_e_z = -1.0*self.Kr*psi + (self.N_ur + np.dot(self.Kr,self.N_xr))*psi_r

        #compose angular position dynamics input vector
        #uc_e = np.array([[uc_e_x[0]],[uc_e_y[0]],[uc_e_z[0]]])


        # the final input to the drone is u = u_e + u_r
        # input =  error_input + reference_input
        uax, uay, uaz = [traj_msg.ua.x + ua_e_x.item(0), traj_msg.ua.y + ua_e_y.item(0), traj_msg.ua.z + ua_e_z.item(0)] 
        ucx, ucy, ucz = [traj_msg.uc.x + uc_e_x.item(0), traj_msg.uc.y + uc_e_y.item(0), traj_msg.uc.z + uc_e_z.item(0)]

        ua = np.array([[uax], [uay], [uaz]])
        uc = np.array([[ucx], [ucy], [ucz]])
        #print(ua,uc)

        # compute Thrust -T- as
        #   T = m*wzb.T*(ua + g*zw)
        #
        zb = np.array([[0.0],[0.0],[1.0]])  #  z axis of body expressed in body frame
        zw = np.array([[0.0],[0.0],[1.0]])  #  z axis of world frame expressed in world frame 
        Rwb = tf.transformations.euler_matrix(psi, theta, phi, axes='rzyx') # world to body frame rotation
        Rbw = Rwb[0:3,0:3].T  # body to world frame rotation only

        wzb = np.dot(Rbw,zb) # zb expressed in world frame
        T =  self.m*np.dot(wzb.T,(ua + g*zw))[0]
        #print(wzb.T,type(wzb))

        # compute w_b angular velocity commands as
        #  w_b = K.inv * uc
        #  where  (euler angle derivate) = K*w_b
        K = np.array([[1.0, np.sin(phi_r)*np.tan(theta_r), np.cos(phi_r)*np.tan(theta_r)],
                      [0.0, np.cos(phi_r), -1.0*np.sin(phi_r)],
                      [0.0, np.sin(phi_r)/np.cos(theta_r), np.cos(phi_r)/np.cos(theta_r)]])

        Kinv = np.linalg.inv(K)

        w_b = np.dot(Kinv, uc).flatten()

        # create and fill message
        rt_msg = RateThrust()
        
        rt_msg.header.stamp = rospy.Time.now()
        rt_msg.header.frame_id = 'uav/imu'

        rt_msg.angular_rates.x = w_b[0] #traj_msg.twist.angular.x
        rt_msg.angular_rates.y = w_b[1] #traj_msg.twist.angular.y
        rt_msg.angular_rates.z = w_b[2] #traj_msg.twist.angular.z

        rt_msg.thrust.x = 0.0
        rt_msg.thrust.y = 0.0
        rt_msg.thrust.z = T[0] # self.m*np.linalg.norm(t)
        
        # publish
        self.input_publisher.publish(rt_msg)
        rospy.loginfo(rt_msg)


    def publish_thrust(self, thrust):

        # create single message
        thrust_msg = RateThrust()
        thrust_msg.thrust.z = thrust
        self.input_publisher.publish(thrust_msg)
        rospy.loginfo(thrust_msg)
        rospy.loginfo("Published body vertical thrust: {}".format(thrust))


if __name__ == '__main__':
    try:

        rospy.init_node('uav_input_publisher', anonymous = True)
        
        uav_input_pub = uav_Input_Publisher()

        rate = rospy.Rate(100)

        # Send some thrust commands for simulator to start publishing IMU
        for i in range(10):
            uav_input_pub.publish_thrust(9.9)  # publish twice: once for initialization
            #uav_input_pub.publish_thrust(9.9)  # and another to minimize the effect of the 1st one
            rate.sleep()

        rospy.loginfo(' UAV Input Publisher Created !')
        rospy.spin()
        rospy.loginfo(' UAV Input Publisher terminated. ')

    except rospy.ROSInterruptException:
        rospy.loginfo('ROS Terminated.')
        pass
