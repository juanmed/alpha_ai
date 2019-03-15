#!/usr/bin/env python

# support libraries
import rospy
import message_filters
import tf
import numpy as np
from pyquaternion import Quaternion 

# messages
from tests.msg import UAV_traj
from tests.msg import UAV_state
from mav_msgs.msg import RateThrust

# import libraries
import trajectory.df_flat as df_flat
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
        ts = message_filters.ApproximateTimeSynchronizer([self.state_sub, self.reftraj_sub],10,0.005)
        ts.registerCallback(self.pid_controller)

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


        # compute maximum control outputs for saturation
        self.thrust_coeff = rospy.get_param("/uav/flightgoggles_uav_dynamics/thrust_coefficient")
        self.max_rotor_speed = rospy.get_param("/uav/flightgoggles_uav_dynamics/max_prop_speed")
        self.rotor_count = 4
        self.max_thrust = self.rotor_count*self.thrust_coeff*(self.max_rotor_speed**2)  # assuming cuadratic model for rotor thrust 


        # store position error for PID controller
        self.pos_err = np.zeros((3,1))
        
        # initialize variables to make position controller
        # slower than Orientation controller
        self.T = g
        self.Rbw_des = np.diag([1.0,1.0,1.0])
        self.loops = 0
        self.w_b_des = np.zeros((3,1))
        self.w_fb = np.zeros((3,1))
        self.pos_loop_freq = 30

        # Orientation control feedback gains

        self.Katt = np.zeros((3,4))
        self.Katt[0][1] = 1.0
        self.Katt[1][2] = 1.0
        self.Katt[2][3] = 1.0


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
        ua_e_x = -1.0*np.dot(self.Kt,np.array([[x_e],[vx_e]]))
        ua_e_y = -1.0*np.dot(self.Kt,np.array([[y_e],[vy_e]]))
        ua_e_z = -1.0*np.dot(self.Kt,np.array([[z_e],[vz_e]]))

        # second, for orientation dynamics
        uc_e_x = -1.0*np.dot(self.Kr,np.array([phi_e]))
        uc_e_y = -1.0*np.dot(self.Kr,np.array([theta_e]))
        uc_e_z = -1.0*np.dot(self.Kr,np.array([psi_e]))
        #print("uc_e_x: {} type: {}".format(uc_e_x,type(uc_e_x)) )
        #print("ua_e_x: {} type: {}".format(ua_e_x,type(ua_e_x)) )

        # the final input to the drone is u = u_e + u_r
        # input =  error_input + reference_input
        #print("traj_msg.ua.x: {} type: {}".format(traj_msg.ua.x,type(traj_msg.ua.x)) )
        uax, uay, uaz = [traj_msg.ua.x + ua_e_x.item(0), traj_msg.ua.y + ua_e_y.item(0), traj_msg.ua.z + ua_e_z.item(0)] 
        ucx, ucy, ucz = [traj_msg.uc.x + uc_e_x.item(0), traj_msg.uc.y + uc_e_y.item(0), traj_msg.uc.z + uc_e_z.item(0)]
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
        #                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               = np.array([[ua_e_x[0]],[ua_e_y[0]],[ua_e_z[0]]])


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
        K = np.array([[1.0, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                      [0.0, np.cos(phi), -1.0*np.sin(phi)],
                      [0.0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])

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

    def linear_velocity_control(self, state_msg, traj_msg):
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

        # vx dynamics control law
        ua_e_x = -1.0*self.Kr*vx + (self.N_ur + np.dot(self.Kr,self.N_xr))*vx_r
        # vy dynamics control law
        ua_e_y = -1.0*self.Kr*vy + (self.N_ur + np.dot(self.Kr,self.N_xr))*vy_r
        # vz dynamics control law
        ua_e_z = -1.0*self.Kr*vz + (self.N_ur + np.dot(self.Kr,self.N_xr))*vz_r

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
        K = np.array([[1.0, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                      [0.0, np.cos(phi), -1.0*np.sin(phi)],
                      [0.0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])

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

    def pid_controller(self, state_msg, traj_msg):
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
        #print("Traj rot: \n{}".format(traj_msg.rot))
        Rbw_ref = np.array([[traj_msg.rot[0],traj_msg.rot[1],traj_msg.rot[2]],
                         [traj_msg.rot[3],traj_msg.rot[4],traj_msg.rot[5]],
                         [traj_msg.rot[6],traj_msg.rot[7],traj_msg.rot[8]]])
        #print("Rbw ref: \n{}".format(Rbw_ref))

        # extract drone real state values
        x, y, z = [state_msg.pose.position.x, state_msg.pose.position.y, state_msg.pose.position.z]
        vx, vy, vz = [state_msg.twist.linear.x, state_msg.twist.linear.y, state_msg.twist.linear.z]
        ori_quat = [state_msg.pose.orientation.x, state_msg.pose.orientation.y, state_msg.pose.orientation.z, state_msg.pose.orientation.w]
        psi, theta, phi = tf.transformations.euler_from_quaternion(ori_quat, axes = 'rzyx')
        Rwb = tf.transformations.quaternion_matrix(ori_quat)
        Rbw = Rwb[0:3,0:3].T  # get body to world frame rotation 
        p, q, r = [state_msg.twist.angular.x, state_msg.twist.angular.y, state_msg.twist.angular.z]

        # ******************************************#
        #            POSITION CONTROL LOOP
        # ******************************************#
        if(self.loops == 0): 

            # compute desired input ua = ua_ref + ua_e
            # ue_ref : from dif flatness
            # ua_e = -Kp*(p - p_ref) - Kd * (v - v_ref)    using PID controller
            pos = np.array([[x],[y],[z]])
            pos_ref = np.array([[x_r],[y_r],[z_r]])
            v = np.array([[vx],[vy],[vz]])
            v_ref = np.array([[vx_r],[vy_r],[vz_r]])

            Kp = np.diag([lqrg.Kp2, lqrg.Kp2, lqrg.Kp2])
            Kd = np.diag([lqrg.Kd2, lqrg.Kd2, lqrg.Kd2])
            Ki = np.diag([lqrg.Ki2, lqrg.Ki2, lqrg.Ki2])

            self.pos_err = self.pos_err + (pos - pos_ref)
            ua_e = -1.0*np.dot(Kp,pos-pos_ref) -1.0*np.dot(Kd,v-v_ref) -1.0*np.dot(Ki,self.pos_err)  # PID control law
            #print("pos error mag: {}".format(np.linalg.norm(pos - pos_ref)))
            #print("vel error mag: {}".format(np.linalg.norm(v-v_ref)))
            ua_ref = np.array([[traj_msg.ua.x], [traj_msg.ua.y], [traj_msg.ua.z]])

            ua = ua_e + ua_ref  # desired acceleration
            #print("ua mag {}".format(np.linalg.norm(ua)))
            ua = self.clip_vector(ua, self.max_thrust)   # saturate input

            # compute Thrust -T- as
            #   T = m*wzb.T*(ua + g*zw)
            #
            e3 = np.array([[0.0],[0.0],[1.0]])  #  z axis of body expressed in body frame
            zw = np.array([[0.0],[0.0],[1.0]])  #  z axis of world frame expressed in world frame 
            #Rwb = tf.transformations.euler_matrix(psi, theta, phi, axes='rzyx') # world to body frame rotation
            #print("Rbw real: \n{}".format(Rbw))

            wzb = np.dot(Rbw,e3) # zb expressed in world frame
            self.T =  self.m*np.dot(wzb.T,(ua + g*zw))[0][0]
            #print("T : {}".format(self.T))

            # Following Fassler, Fontana, Theory and Math Behing RPG Quadrotor Control
            #  Rbw_des = [xb_des yb_des zb_des]  with zb_des = ua + g*zw / ||ua + g*zw||
            #  represents the desired orientation (and not the one coming from dif flatness)
            zb_des = (ua + g*zw)/np.linalg.norm(ua + g*zw)
            #T = self.m*np.dot(zb_des.T,(-ua + g*zw))[0][0]

            #print("PSI value: {}".format(psi_r))
            yc_des = np.array(df_flat.get_yc(psi_r))   #transform to np.array 'cause comes as np.matrix
            xb_des = np.cross(yc_des, zb_des, axis=0)
            xb_des = xb_des/np.linalg.norm(xb_des)
            yb_des = np.cross(zb_des, xb_des, axis = 0)

            self.Rbw_des = np.concatenate((xb_des, yb_des, zb_des), axis=1)

            # Orientation error matrix is
            # Rbw.T*Rbw_des  because iff Rbw = Rbw_des, then Rbw.T*Rbw_des = I_3



            # Define angular velocity error
            w_b = np.array([[p],[q],[r]])
            w_b_r = np.array([[p_r],[q_r],[r_r]])
            #print("w_b ref: \n{}".format(w_b_r))
            self.w_b_des = np.dot(Rbw.T, np.dot(Rbw_ref,w_b_r))
            #print("w_b des: \n{}".format(w_b_des))
            w_b_e = w_b - np.dot(Rbw.T, np.dot(self.Rbw_des, self.w_b_des))
            # compute input with PID control law


            # compute feedback angular velocity
            self.w_fb = self.omega_fb(Rbw, self.Rbw_des, self.Katt)


            Kp = np.diag([lqrg.Kp1, lqrg.Kp1, lqrg.Kp1])
            Kd = np.diag([lqrg.Kd1, lqrg.Kd1, lqrg.Kd1])
            #w_b_in = -np.dot(Kp,or_e) - np.dot(Kd,w_b_e)
            
            self.loops = self.loops + 1   

        else:
            self.loops = self.loops + 1
            if(self.loops == self.pos_loop_freq):
                self.loops = 0
                #print("**Loops: {}".format(self.loops))


        print("Position Error: {}".format(np.linalg.norm(self.pos_err)))
        print("T computed: {}".format(self.T))

        # ******************************************#
        #        ORIENTATION CONTROL LOOP
        # ******************************************#

                        # ********************************** #
                        #        USING EULER ANGLES          #
                        # ********************************** #

        or_des = np.array(df_flat.RotToRPY_ZYX(self.Rbw_des))  # ... need to convert to np.array to use this function..
        #print("Orientation des: {}".format(or_des))
        phi_des = or_des[0][0]
        theta_des = or_des[1][0]
        psi_des = or_des[2][0]

        # phi (x axis) angular position dynamics control law 
        uc_e_x = -1.0*self.Kr*phi + (self.N_ur + np.dot(self.Kr,self.N_xr))*phi_des
        # theta (y axis) angular position dynamics control law
        uc_e_y = -1.0*self.Kr*theta + (self.N_ur + np.dot(self.Kr,self.N_xr))*theta_des
        # psi (z axis) angular position dynamics control law
        uc_e_z = -1.0*self.Kr*psi + (self.N_ur + np.dot(self.Kr,self.N_xr))*psi_des

        #compose angular position dynamics input vector
        #uc_e = np.array([[uc_e_x[0]],[uc_e_y[0]],[uc_e_z[0]]])


        # the final input to the drone is u = u_e + u_r
        # input =  error_input + reference_input
        ucx, ucy, ucz = [traj_msg.uc.x + uc_e_x.item(0), traj_msg.uc.y + uc_e_y.item(0), traj_msg.uc.z + uc_e_z.item(0)]
        uc = np.array([[ucx], [ucy], [ucz]])
        #print(ua,uc)


        # compute w_b angular velocity commands as
        #  w_b = K.inv * uc
        #  where  (euler angle derivate) = K*w_b
        K = np.array([[1.0, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                      [0.0, np.cos(phi), -1.0*np.sin(phi)], 
                      [0.0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])

        Kinv = np.linalg.inv(K)

        w_b_in = np.dot(Kinv, uc)

        # create and fill message
        rt_msg = RateThrust()
        
        rt_msg.header.stamp = rospy.Time.now()
        rt_msg.header.frame_id = 'uav/imu'

        rt_msg.angular_rates.x = self.w_b_des[0][0] + w_b_in[0][0] #- p_r #traj_msg.twist.angular.x
        rt_msg.angular_rates.y = self.w_b_des[1][0] + w_b_in[1][0] #- q_r  #traj_msg.twist.angular.y
        rt_msg.angular_rates.z = self.w_b_des[2][0] + w_b_in[2][0] #- r_r #traj_msg.twist.angular.z



                        # ********************************** #
                        #      USING ROTATION MATRIX         #
                        # ********************************** #

        # Define orientation tracking error
        or_e = 0.5*self.vex( (np.dot(self.Rbw_des.T,Rbw) - np.dot(Rbw.T,self.Rbw_des)) )

        #rt_msg.angular_rates.x = 1.0*self.w_b_des[0][0] + 1.0*self.w_fb[0][0] #- p
        #rt_msg.angular_rates.y = 1.0*self.w_b_des[1][0] + 1.0*self.w_fb[1][0] #- q
        #rt_msg.angular_rates.z = 1.0*self.w_b_des[2][0] + 1.0*self.w_fb[2][0] #- r



        rt_msg.thrust.x = 0.0
        rt_msg.thrust.y = 0.0
        rt_msg.thrust.z = self.T # self.m*np.linalg.norm(t)
        
        # publish
        self.input_publisher.publish(rt_msg)
        rospy.loginfo(rt_msg) 

    # saturation function for scalars
    def clip_scalar(self, value, max_value):
        if( value < max_value):
            return value
        else:
            return max_value

    # saturation function for vectors
    def clip_vector(self, v, max_value):
        mag = np.linalg.norm(v)
        if( mag < max_value):
            return v
        else:
            return np.dot(v/mag,max_value)  # return vector in same direction but maximum possible magnitude

    # take skew-symmetric matrix and turn it into vector
    def vex(self,mx):
        v = np.array([[0],[0],[0]])
        v[2][0] = 0.5*(mx[1][0] - mx[0][1])
        v[1][0] = 0.5*(mx[0][2] - mx[2][0])
        v[0][0] = 0.5*(mx[2][1] - mx[1][2])

        return v

    # take vector and transform to 3x3 skew-symmetric matrix
    def hat(self,v):
        mx = np.zeros((3,3))
        mx[0][1] = -v[2][0]
        mx[0][2] =  v[1][0]
        mx[1][0] =  v[2][0]
        mx[1][2] = -v[0][0] 
        mx[2][0] = -v[1][0]
        mx[2][1] =  v[0][0]
        return mx

    # compute angular velocity to reach desired orientation (expressed as rot matrix)
    # from a current orientation (expressed as rot matrix)
    # Kp is 4x3 matrix with first colum of zeros and rest of columns a 3x3 diagonal matrix
    def omega_fb(self, Rbw, Rbw_des, Kp):
        """
        Take current and desired rotation matrices (body to world) and compute
        error angular velocity. Uses quaternion representations
        and computes as follows:
        
            omega = 2*Kp*q_e    where q_e = (q_inv)*q_des

        See https://github.com/uzh-rpg/rpg_quadrotor_control/blob/master/documents/theory_and_math/theory_and_math.pdf
        Section 4.2.1 Attitude Control for reference

        """

        # first transform to world->body frame rotations
        Rwb = Rbw.T 
        Rwb_des = Rbw_des.T

        #convert into homogenous transformation
        dummy1 = np.zeros((3,1))
        dummy2 = np.zeros((1,4))
        dummy2[0][3] = 1.0

        Rwb = np.concatenate((Rwb,dummy1), axis = 1)
        Rwb = np.concatenate((Rwb, dummy2), axis = 0)

        Rwb_des = np.concatenate((Rwb_des,dummy1), axis = 1)
        Rwb_des = np.concatenate((Rwb_des, dummy2), axis = 0)

        # transform current and desired orientation to quaternions
        q = tf.transformations.quaternion_from_matrix(Rwb)
        q_des = tf.transformations.quaternion_from_matrix(Rwb_des)

        # CAREFUL! tf uses q = [x y z w], pyquaternion uses q = [w x y z]
        q = [q[3], q[0], q[1], q[2]]  
        q_des = [q_des[3], q_des[0], q_des[1], q_des[2] ]

        # Use an object that lets us apply quaternion operations easily
        q = Quaternion(q)
        q_des = Quaternion(q_des)

        # calculate orientation error
        q_e = q.inverse * q_des
        q_e = np.array( [ [q_e[0]], [q_e[1]], [q_e[2]], [q_e[3]]] ) # back to np.array ....

        # calculate angular velocity depending on value of quaternion's real part
        if(q_e[0] >= 0.0):
            w_fb = 2.0*np.dot(Kp,q_e)
        else:
            w_fb = -2.0*np.dot(Kp,q_e)

        return w_fb

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
