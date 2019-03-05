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
import trajectory.keyframe_generation as keyframe_generation
import trajectory.draw_trajectory as draw_trajectory
import trajectory.trajgen2_helper as trajGen3D
import matplotlib.pyplot as plt


class Trajectory_Generator():

    def __init__(self):
        
        # init parameters
        self.order = parameter.order
        self.n = parameter.n
        self.gate = parameter.gate
        self.t = parameter.t

        # generate keyframe
        self.keyframe_cls = keyframe_generation.KeyframeGeneration()
        self.keyframe = self.keyframe_cls.keyframe_generation(self.gate)

        # self.gate = 5
        # self.t = np.array([0, 1, 2, 3, 4, 5, 6])
        # self.keyframe = self.keyframe_cls.keyframe_generation(5)

        # compute flat output trajectory
        self.sol_x = qp_solution.qp_solution(self.order, self.n, self.gate, self.t, self.keyframe)
        draw_trajectory.draw_trajectory(self.sol_x, self.order, self.gate, self.n, self.t, self.keyframe)

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
            plt.show()
            exit()

        return ref_trajectory 

class Trajectory_Generator_Test():

    def __init__(self):

        # init parameters
        self.order = 6
        self.n = 4
        self.gate = 1
        self.t = [0, 5]

        # generate keyframe
        self.keyframe_cls = keyframe_generation.KeyframeGeneration()
        self.keyframe = self.keyframe_cls.keyframe_generation(self.gate)
        self.keyframe = np.array([[0.3, 52.0, 2.5, -1*np.pi/2.0],
                                  [0.3, 52.0, 5, -1*np.pi/2.0]])
        self.keyframe = np.transpose(self.keyframe)

        # compute flat output trajectory
        self.sol_x = qp_solution.qp_solution(self.order, self.n, self.gate, self.t, self.keyframe)
        draw_trajectory.draw_trajectory(self.sol_x, self.order, self.gate, self.n, self.t, self.keyframe)

        # initialize time
        self.init_time = rospy.get_time()
        self.last_time = rospy.get_time()
        self.start_time = self.last_time

        # start in trajectory from 1st gate to 2nd gate
        self.i = 0

    def compute_reference_traj(self, time):

        ref_time = time - self.start_time
        x = self.sol_x[self.n * (self.order + 1) * self.i: self.n * (self.order + 1) * (self.i + 1)]
        flatout_trajectory = compute_trajectory.compute_trajectory(x, self.order, ref_time)
        ref_trajectory = df_flat.compute_ref(flatout_trajectory)

        if (time - self.last_time) > (self.t[self.i + 1] - self.t[self.i]):
            # print time - self.last_time
            # print ref_time
            self.i = self.i + 1  # trajectory to next gate
            self.last_time = time

        if (self.i == self.gate):
            print ("Total time: {}".format(time - self.init_time))
            plt.show()
            exit()

        return ref_trajectory

class Trajectory_Generator2():
    def __init__(self):

        # first compute waypoints: the first one is the initial position
        # and orientation, and the rest are the position of the gates

        self.waypoints = self.get_vertical_waypoints()
        print("Waypoints: ")
        print(self.waypoints)
        (self.coeff_x, self.coeff_y, self.coeff_z) = trajGen3D.get_MST_coefficients(self.waypoints)

        print("coeff X:")
        print(self.coeff_x)
        print("coeff Y:")
        print(self.coeff_y)
        print("coeff Z:")
        print(self.coeff_z)

        # initialize time for trajectory generator
        self.start_time = rospy.get_time()

        # initialize heading
        init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
        init_quat = [init_pose[3],init_pose[4],init_pose[5],init_pose[6]]
        yaw, pitch, roll = tf.transformations.euler_from_quaternion(init_quat, axes = "rzyx")
        #print("Roll: {}, Pitch: {}, Yaw: {}".format(roll, pitch, yaw))
        trajGen3D.yaw = yaw
        trajGen3D.current_heading = np.array([np.cos(yaw),np.sin(yaw)])
        #print("Yaw: {}, Heading: {}".format(trajGen3D.yaw,trajGen3D.current_heading))



    def compute_reference_traj(self, time):
        vel = 5
        trajectory_time = time - self.start_time
        #print("Time traj: {}".format(trajectory_time))
        flatout_trajectory = trajGen3D.generate_trajectory(trajectory_time, vel, self.waypoints, self.coeff_x, self.coeff_y, self.coeff_z)
        ref_trajectory = df_flat.compute_ref(flatout_trajectory)
        return ref_trajectory

    # read initial position and gate positions
    # and return an np.array of these positions
    def get_gate_waypoints(self):

        # Then continue with the gates
        gate_names = rospy.get_param("/uav/gate_names")

        # First waypoint is initial position
        init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
        waypoints = np.zeros((len(gate_names)+1,3))
        waypoints[0][0] = init_pose[0]
        waypoints[0][1] = init_pose[1]
        waypoints[0][2] = init_pose[2]

        for i,name in enumerate(gate_names):
            gate_data = rospy.get_param("/uav/"+name)

            gate_corners = np.array(gate_data['location'])
            #print("Gate {}".format(i))
            #print(gate_corners,type(gate_corners),gate_corners.shape)
            #print("Center:")
            gate_center = np.mean(gate_corners, axis = 0)
            #print(gate_center)

            waypoints[i+1][0] = gate_center[0] 
            waypoints[i+1][1] = gate_center[1]
            waypoints[i+1][2] = gate_center[2]

        return waypoints

    def get_vertical_waypoints(self):

        # First waypoint is initial position
        init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
        waypoints = np.zeros((2,3))
        waypoints[0][0] = init_pose[0]
        waypoints[0][1] = init_pose[1]
        waypoints[0][2] = init_pose[2]   
        
        # now add a waypoint exactly 1m above the drone 
        waypoints[1][0] = waypoints[0][0]
        waypoints[1][1] = waypoints[0][1]
        waypoints[1][2] = waypoints[0][2] + 1

        return waypoints  



def pub_traj():

    # create topic for publishing ref trajectory
    traj_publisher = rospy.Publisher('uav_ref_trajectory', UAV_traj, queue_size = 10)

    # init node
    rospy.init_node('uav_ref_trajectory_publisher', anonymous = True)

    # IMPORTANT WAIT TIME! 
    # If this is not here, the "start_time" in the trajectory generator is 
    # initialized to zero (because the node has not started fully) and the
    # time for the trajectory will be degenerated
    rospy.sleep(0.1)
 
    # create a trajectory generator
    #traj_gen = Trajectory_Generator()
    #traj_gen = Trajectory_Generator2()
    traj_gen = Trajectory_Generator_Test()

    # publish at 10Hz
    rate = rospy.Rate(10.0)

    x_line = []
    y_line = []
    z_line = []

    while not rospy.is_shutdown():
        
        try:

            # Compute trajectory at time = now
            time = rospy.get_time()   
            ref_traj = traj_gen.compute_reference_traj(time)
            

            # create and fill message
            traj = UAV_traj()
            traj.header.stamp = rospy.Time.now()
            traj.header.frame_id = ""

            
            # get all values... we need to do this becuase ref_traj contains old, ugly np.matrix
            # objects >:(
            x, y, z = np.array(ref_traj[0]).flatten()
            phi, theta, psi = np.array(ref_traj[2]).flatten()
            vx, vy, vz = np.array(ref_traj[1]).flatten()
            p, q, r = np.array(ref_traj[3]).flatten()
            uax, uay, uaz = np.array(ref_traj[4]).flatten()
            ubx, uby, ubz = np.array(ref_traj[5]).flatten()
            ucx, ucy, ucz = np.array(ref_traj[6]).flatten()

            traj.pose.position.x = x
            traj.pose.position.y = y
            traj.pose.position.z = z

            # Following http://docs.ros.org/jade/api/tf/html/python/transformations.html
            # the axes parameter is such that 
            # r = apply rotation on the "new" frame
            # zyx = this means first a rotation of 'psi' radians around the z axis is performed,
            #       then of 'theta' radians about the new y axis ( because 'r' was specified)
            #       then of 'phi' radians about the new x axis ( becuase 'r' was specified)
            quat = tf.transformations.quaternion_from_euler(psi, theta, phi, axes = 'rzyx')

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

            x_line = np.append(x_line, x)
            y_line = np.append(y_line, y)
            z_line = np.append(z_line, z)

            fig = plt.figure(2)
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(x_line, y_line, z_line, '.', color='b')
            ax.set_xlim(-60, 60)
            ax.set_ylim(-60, 60)
            ax.set_zlim(-60, 60)
            ax.set_xlabel('x label')
            ax.set_ylabel('y label')
            ax.set_zlabel('z label')

            # publish message
            traj_publisher.publish(traj)
            rospy.loginfo(traj)
            rate.sleep()


        except Exception:
            rospy.loginfo('People...we have a problem: {}'.format(Exception))
            continue


if __name__ == '__main__':
    try:
        rospy.loginfo("UAV Trajectory Publisher Created")
        pub_traj()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass