#!/usr/bin/env python

# This is a test node for publishing the reference trajectory
# at a constant rate. This reference trajectory is computed by
# using the differential flatness property of the UAV Dynamics.

import rospy
from tests.msg import UAV_traj
import tf
import numpy as np

import trajectory.compute_trajectory as compute_trajectory
import trajectory.df_flat as df_flat
import trajectory.parameter as parameter
import trajectory.qp_solution as qp_solution


class Trajectory_Generator():

    def __init__(self):
        
        # init parameters
        self.order = parameter.order
        self.n = parameter.n
        self.gate = parameter.gate
        self.t = parameter.t
        
        # compute flat output trajectory
        self.sol_x = qp_solution.qp_solution(self.order, self.n, self.gate, self.t)
        
        # initialize time
        self.init_time = rospy.get_time()
        self.last_time = rospy.get_time()
        self.start_time = self.last_time

        # start in trajectory from 1st gate to 2nd gate
        self.i = 0

    def compute_reference_traj(self, time):

        ref_time = time - self.start_time
        x = self.sol_x[self.n*(self.order+1)*self.i: self.n*(self.order+1)*(self.i+1)] 
        flatout_trajectory = compute_trajectory.compute_trajectory(x, self.order, ref_time)
        ref_trajectory = df_flat.compute_ref(flatout_trajectory)  
        
        if (time - self.last_time) > (self.t[self.i+1] - self.t[self.i]):
            #print time
            self.i = self.i + 1   # trajectory to next gate
            self.last_time = time

        if (self.i == self.gate):
            print ("Total time: {}".format(time-self.init_time))
            exit()

        return ref_trajectory 

def pub_traj():

    # create topic for publishing ref trajectory
    traj_publisher = rospy.Publisher('uav_ref_trajectory', UAV_traj, queue_size = 10)

    # init node
    rospy.init_node('uav_ref_trajectory_publisher', anonymous = True)

    # publish at 10Hz
    rate = rospy.Rate(10.0)

    # create a trajectory generator
    traj_gen = Trajectory_Generator()

    while not rospy.is_shutdown():
        
        try:
            
            # Compute trajectory at time = now
            time = rospy.get_time()
            ref_traj = traj_gen.compute_reference_traj(time)

            # get all values... we need to do this becuase ref_traj contains old, ugly np.matrix
            # objects >:(
            x, y, z = np.array(ref_traj[0]).flatten()
            phi, theta, psi = np.array(ref_traj[2]).flatten()
            vx, vy, vz = np.array(ref_traj[1]).flatten()
            p, q, r = np.array(ref_traj[3]).flatten()
            uax, uay, uaz = np.array(ref_traj[4]).flatten()
            ubx, uby, ubz = np.array(ref_traj[5]).flatten()
            ucx, ucy, ucz = np.array(ref_traj[6]).flatten()

            # create and fill message
            traj = UAV_traj()
            traj.header.stamp = rospy.Time.now()
            traj.header.frame_id = ""

            traj.pose.position.x = x
            traj.pose.position.y = y
            traj.pose.position.z = z

            quat = tf.transformations.quaternion_from_euler(phi,theta,psi, axes = 'rxyz')

            traj.pose.orientation.x = quat[0]
            traj.pose.orientation.y = quat[1]
            traj.pose.orientation.z = quat[2]
            traj.pose.orientation.w = quat[3]

            traj.twist.linear.x = vx
            traj.twist.linear.y = vy
            traj.twist.linear.z = vz

            traj.twist.angular.x = p
            traj.twist.angular.y = q
            traj.twist.angular.z = r

            traj.ua.x = uax
            traj.ua.y = uay
            traj.ua.z = uaz

            traj.ub.x = ubx
            traj.ub.y = uby
            traj.ub.z = ubz

            traj.uc.x = ucx
            traj.uc.y = ucy
            traj.uc.z = ucz

            # publish message
            traj_publisher.publish(traj)
            rospy.loginfo(traj)

            rate.sleep()


        except:
            rospy.loginfo('People...we have a problem')
            continue


if __name__ == '__main__':
    try:
        rospy.loginfo("UAV Trajectory Publisher Created")
        pub_traj()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass