#!/usr/bin/env python

# This is a test node for publishing the reference trajectory
# at a constant rate. This reference trajectory is computed by
# using the differential flatness property of the UAV Dynamics.

import rospy
from tests.msg import UAV_traj
from tests.msg import UAV_state
from std_msgs.msg import String
import tf
import numpy as np

import trajectory.compute_trajectory as compute_trajectory
import trajectory.df_flat as df_flat
import trajectory.parameter as parameter
import trajectory.qp_solution as qp_solution
import trajectory.keyframe_generation as keyframe_generation
import trajectory.draw_trajectory as draw_trajectory
import trajectory.trajgen2_helper as trajGen3D
import trajectory.get_gate_location as get_gate_location
import trajectory.optimal_time as optimal_time
import trajectory.gate_event as gate_event
import trajectory.calculate_length as calculate_length


class Trajectory_Generator():

    def __init__(self):
        self.level = True

        # init parameters
        self.order = 10
        self.n = 4
        self.inflation = 2
        self.tolerance = 1
        self.gate_name = rospy.get_param("/uav/gate_names")
        self.gate_count = len(self.gate_name)
        self.init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")


        # init Class
        self.gate_location_cls = get_gate_location.GateLocation()
        self.keyframe_cls = keyframe_generation.KeyframeGeneration()

        # gate location , gate event(checking gate pass)
        self.gate_location = self.gate_location_cls.get_gate_location(self.level, self.gate_count, self.gate_name)
        self.gates = []
        for i in range(self.gate_count):
            self.gates.append(gate_event.GateEvent(self.gate_location[i], self.inflation))

        # generate keyframe : waypoint = gate_count + 1
        self.is_quaternion = True
        self.keyframe, self.waypoint = self.keyframe_cls.keyframe_generation(self.init_pose, self.is_quaternion,
                                                                             self.gate_location, self.gate_count)

        # set time segment
        self.init_t = [0, 0.9, 1.7, 2.5, 3.5] #, 4.0, 4.5, 5.5, 6.0, 6.5, 7.5, 8.5, 9]
        self.init_t = np.array(self.init_t) * 10
        self.new_t = self.init_t
        #self.new_t = [0, 15]

        # current state(pos, vel, acc, jerk, snap)
        self.current_pos = self.keyframe[0]  # x y z psi
        self.current_vel = np.array([0, 0, 0, 0])  # for now in our system, we can only get velocity from estimation
        self.current_acc = np.array([0, 0, 0, 0])
        self.current_jerk = np.array([0, 0, 0, 0])
        self.current_snap = np.array([0, 0, 0, 0])
        self.current_state = np.vstack(
            (self.current_pos, self.current_vel, self.current_acc, self.current_jerk, self.current_snap))

        # This is solution for piecewise polynomial
        self.sol_x = qp_solution.qp_solution(self.order, self.n, self.waypoint, self.new_t, self.keyframe,
                                             self.current_state)
        # draw trajectory in plot
        #draw_trajectory.draw_trajectory(self.sol_x, self.order, 2, self.n, self.new_t, self.keyframe)

        # counting gate pass
        self.passed_gate = 0

        # publish gate number
        self.gate_pub = rospy.Publisher('gate_number', String, queue_size=10)
        # subscribe state from estimator
        rospy.Subscriber('/estimator/state', UAV_state, self.current_state_update)

        # initialize time
        self.start_time = rospy.get_time()
        self.last_time = self.start_time
        self.update_time = self.start_time

        # start in trajectory from 1st gate to 2nd gate
        self.i = 0

    def compute_reference_traj(self, time):
        ref_time = time - self.start_time
        x = self.sol_x[self.n*(self.order+1)*self.i: self.n*(self.order+1)*(self.i+1)]
        #ref_time = time - self.last_time
        #x = self.sol_x[self.n * (self.order + 1) * 0: self.n * (self.order + 1) * (0 + 1)]
        flatout_trajectory = compute_trajectory.compute_trajectory(x, self.order, ref_time)
        ref_trajectory = df_flat.compute_ref(flatout_trajectory)

        # Drone can not get sensor data of acc, jerk, snap
        # instead, it uses reference acc, jerk, snap for current state.
        # self.current_acc = np.append(flatout_trajectory[2], flatout_trajectory[7])
        # self.current_jerk = np.append(flatout_trajectory[3], 0)
        # self.current_snap = np.append(flatout_trajectory[4], 0)

        if (time - self.last_time) > (self.init_t[self.i+1] - self.init_t[self.i]):
        #if (time - self.last_time) > self.new_t[1] - self.new_t[0]:
            #print time
            self.i = self.i + 1   # trajectory to next gate
            self.last_time = time
            #self.trajectory_update()

        if self.i == self.waypoint - 1:
            print ("Total time: {}".format(time-self.start_time))
            #exit()

        return ref_trajectory

    def current_state_update(self, msg):
        #current_pose_orientation = tf.transformations.euler_from_quaternion(
        #    [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], axes='sxyz')
        #self.current_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, current_pose_orientation[2]])
        #self.current_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.z])
        #self.current_state = np.vstack((self.current_pos, self.current_vel, self.current_acc, self.current_jerk, self.current_snap))

        if self.passed_gate < len(self.gate_name):
            self.check_gate()
        else:
            pass

    # checking whether drone pass gate or not
    # It needs current position of drone.
    def check_gate(self):
        if self.gates[self.passed_gate].isEvent(self.current_state[0], self.tolerance):
                self.passed_gate = self.passed_gate + 1
        self.gate_pub.publish(self.gate_name[self.passed_gate])

    # keyframe and trajectory update
    # It needs current position of drone.
    # Also need current vel, acc, jerk, snap from flatout trajectory

    def trajectory_update(self, time=None):
        '''
        if time - self.update_time > 5:
            pass_time = time - self.start_time
            # delete previous pose and gate location which was passed before.
            for i in range(0, self.passed_gate + 1):
                self.keyframe = np.delete(self.keyframe, 0, axis=0)

            # add current pose at keyframe
            self.keyframe = np.vstack((self.current_state[0], self.keyframe))
            #print self.keyframe
            self.waypoint = self.gate_count + 1 - self.passed_gate

            # update time
            self.new_t = self.compute_optimal_time(pass_time)
            new_t = self.new_t

            # compute flat output trajectory
            self.sol_x = qp_solution.qp_solution(self.order, self.n, self.waypoint, new_t, self.keyframe, self.current_state)
            draw_trajectory.draw_trajectory(self.sol_x, self.order, self.waypoint, self.n, self.new_t, self.keyframe)
            self.update_time = time
        '''
        self.keyframe = np.delete(self.keyframe, 0, axis=0)
        self.sol_x = qp_solution.qp_solution(self.order, self.n, 2, self.new_t, self.keyframe,
                                             self.current_state)
        #draw_trajectory.draw_trajectory(self.sol_x, self.order, 2, self.n, self.new_t, self.keyframe)
    '''
    def compute_optimal_time(self, pass_time):
        n = 10
        standard_deviation = 0.1
        len_array = np.zeros(n)
        t_normal_array = np.zeros(n)

        for i in xrange(n):
            t_normal = np.random.normal(pass_time, standard_deviation)
            new_t = self.init_t - t_normal
            new_t = new_t[self.passed_gate:]
            new_t[0] = 0
            sol_x = qp_solution.qp_solution(self.order, self.n, self.waypoint - self.passed_gate, new_t, self.keyframe,
                                            self.current_state)
            len_array[i] = calculate_length.calculate_length(sol_x, self.order, self.waypoint - self.passed_gate,
                                                             self.n, new_t)
            t_normal_array[i] = t_normal

        count = np.argmin(len_array)
        t_normal = t_normal_array[count]

        self.new_t = self.init_t -t_normal
        self.new_t = self.new_t[self.passed_gate:]
        self.new_t[0] = 0

        return self.new_t
    '''
class Trajectory_Generator_Test():

    def __init__(self):
        # init parameters
        self.order = 6
        self.n = 4
        self.gate_count = 1
        self.init_t = [0, 5]

        # generate keyframe
        self.keyframe = np.array([[0.3, 52.0, 2.5, -1*np.pi/2.0],
                                  [0.3, 52.0, 10, -1*np.pi/2.0]])
        self.keyframe = np.transpose(self.keyframe)

        # compute flat output trajectory
        self.sol_x = qp_solution.qp_solution(self.order, self.n, self.gate_count, self.init_t, self.keyframe)

        # draw trajectory in plot
        draw_trajectory.draw_trajectory(self.sol_x, self.order, self.gate_count, self.n, self.init_t, self.keyframe)

        # initialize time
        self.start_time = rospy.get_time()
        self.last_time = rospy.get_time()

        # start in trajectory from 1st gate to 2nd gate
        self.i = 0

    def compute_reference_traj(self, time):

        ref_time = time - self.start_time
        x = self.sol_x[self.n * (self.order + 1) * self.i: self.n * (self.order + 1) * (self.i + 1)]
        flatout_trajectory = compute_trajectory.compute_trajectory(x, self.order, ref_time)
        ref_trajectory = df_flat.compute_ref(flatout_trajectory)

        if (time - self.last_time) > self.init_t[self.i + 1] - self.init_t[self.i]:
            # print time - self.last_time
            # print ref_time
            self.i = self.i + 1  # trajectory to next gate
            self.last_time = time

        if self.i == self.gate_count:
            print ("Total time: {}".format(time - self.start_time))
            exit()

        return ref_trajectory

class Trajectory_Generator2():
    def __init__(self):

        # first compute waypoints: the first one is the initial position
        # and orientation, and the rest are the position of the gates

        self.waypoints = self.get_gate_waypoints()
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
        vel = 2
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

        waypoints[waypoints.shape[0]-1][1] =  waypoints[waypoints.shape[0]-1][1] - 1

        return waypoints

    def get_vertical_waypoints(self, height):

        # First waypoint is initial position
        init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
        waypoints = np.zeros((2,3))
        waypoints[0][0] = init_pose[0]
        waypoints[0][1] = init_pose[1]
        waypoints[0][2] = init_pose[2]   
        
        # now add a waypoint exactly 1m above the drone 
        waypoints[1][0] = waypoints[0][0]
        waypoints[1][1] = waypoints[0][1]
        waypoints[1][2] = waypoints[0][2] + height

        return waypoints  

def pub_traj():
    # create topic for publishing ref trajectory
    traj_publisher = rospy.Publisher('uav_ref_trajectory', UAV_traj, queue_size=10)

    # init node
    # rospy.init_node('uav_ref_trajectory_publisher', anonymous = True)
    rospy.init_node('uav_ref_trajectory_input_publisher', anonymous=True)

    # wait time for simulator to get ready...
    wait_time = int(rospy.get_param("riseq/trajectory_wait"))
    while( rospy.Time.now().to_sec() < wait_time ):
        if( ( int(rospy.Time.now().to_sec()) % 1) == 0 ):
            rospy.loginfo("Starting Trajectory Generator in {:.2f} seconds".format(wait_time - rospy.Time.now().to_sec()))
    

    # create a trajectory generator
    traj_gen = Trajectory_Generator()
    #traj_gen = Trajectory_Generator2()
    # traj_gen = Trajectory_Generator_Test()

    rospy.sleep(0.1)

    # IMPORTANT WAIT TIME!
    # If this is not here, the "start_time" in the trajectory generator is 
    # initialized to zero (because the node has not started fully) and the
    # time for the trajectory will be degenerated

    # publish at 10Hz
    rate = rospy.Rate(200.0)

    while not rospy.is_shutdown():
        
        try:
            # Compute trajectory at time = now
            time = rospy.get_time()   
            ref_traj = traj_gen.compute_reference_traj(time)

            # create and fill message
            traj = UAV_traj()
            traj.header.stamp = rospy.Time.now()
            traj.header.frame_id = ""

            # get all values... we need to convert to np.array()
            # becuase ref_traj contains old, ugly np.matrix
            # objects >:(
            x, y, z = np.array(ref_traj[0]).flatten()
            vx, vy, vz = np.array(ref_traj[1]).flatten()
            ax, ay, az = np.array(ref_traj[10]).flatten()
            jx, jy, jz = np.array(ref_traj[11]).flatten()
            sx, sy, sz = np.array(ref_traj[12]).flatten()

            phi, theta, psi = np.array(ref_traj[2]).flatten()
            p, q, r = np.array(ref_traj[3]).flatten()
            yaw = ref_traj[13].item(0)
            yawdot = ref_traj[14].item(0)
            yawddot = ref_traj[15].item(0)

            uax, uay, uaz = np.array(ref_traj[4]).flatten()
            ubx, uby, ubz = np.array(ref_traj[5]).flatten()
            ucx, ucy, ucz = np.array(ref_traj[6]).flatten()
            Rbw = np.array(ref_traj[9]).flatten().tolist()
            #print("ref_traj[9]: {}".format(ref_traj[9]))

            # Input (T, M) publisher to be used in estimation
            u_1 = np.array(ref_traj[7]).flatten()
            u_xx, u_xy, u_xz = np.array(ref_traj[8]).flatten()


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

            traj.ub.x = np.linalg.norm(ref_traj[5])  # ubx
            traj.ub.y = uby
            traj.ub.z = ubz

            traj.uc.x = ucx
            traj.uc.y = ucy
            traj.uc.z = ucz

            traj.rot = Rbw

            traj.acc.x = ax 
            traj.acc.y = ay
            traj.acc.z = az

            traj.jerk.x = jx
            traj.jerk.y = jy
            traj.jerk.z = jz

            traj.snap.x = sx
            traj.snap.y = sy
            traj.snap.z = sz

            traj.yaw = yaw
            traj.yawdot = yawdot
            traj.yawddot = yawddot

            # publish message
            traj_publisher.publish(traj)
            rospy.loginfo(traj)
            rate.sleep()

            #traj_gen.trajectory_update(time)

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