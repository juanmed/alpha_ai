#!/usr/bin/env python

# This is a test node for publishing the reference trajectory
# at a constant rate. This reference trajectory is computed by
# using the differential flatness property of the UAV Dynamics.

import rospy
from tests.msg import UAV_traj
from tests.msg import UAV_input
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


class Trajectory_Generator():

    def __init__(self):

        # choose one : level course or full course?
        self.level = False

        # init parameters
        self.order = 10
        self.n = 4

        self.inflation = parameter.inflation
        self.tolerance = parameter.tolerance

        if self.level is True:
            self.gate_name = parameter.gate_name
            self.gate_count = len(self.gate_name)
            self.init_pose = parameter.init_pose
            time_interval = 5
            self.t = np.linspace(0, time_interval * self.gate_count, self.gate_count + 1)
        else:
            self.gate_count = 11
            self.init_pose = np.array([0.0, 0.0, 1.0, 0.0])
            # this is for trajectory with full gate ( maybe final test3 )
            # set time interval depending on distance between gate
            self.t = 2 * np.array([0, 1, 2, 3, 3.5, 4.5, 5, 5.5, 6, 6.5, 7.5, 8.5])

        self.gate_location_cls = get_gate_location.GateLocation()
        self.keyframe_cls = keyframe_generation.KeyframeGeneration()

        if self.level is True:
            # get gate location
            self.gate_location = self.gate_location_cls.get_gate_location(self.level, self.gate_count, self.gate_name)
            # generate keyframe
            is_quaternion = True
            self.keyframe, self.waypoint = self.keyframe_cls.keyframe_generation(self.init_pose, is_quaternion, self.gate_location, self.gate_count)

        else:
            # get gate location
            self.gate_location = self.gate_location_cls.get_gate_location(self.level, self.gate_count)
            # generate keyframe
            is_quaternion = False
            self.keyframe, self.waypoint = self.keyframe_cls.keyframe_generation(self.init_pose, is_quaternion, self.gate_location, self.gate_count)

        # for counting gates
        self.gates = []
        for i in range(self.gate_count):
            self.gates.append(gate_event.GateEvent(self.gate_location[i], self.inflation))

        self.total_time = 20
        self.t = optimal_time.compute_optimal_time(self.keyframe, self.waypoint, self.total_time)
        #self.t = [0, 2, 2.5, 3, 4, 4.5, 5, 6, 6.5, 7, 8, 9, 10]
        self.t = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
        #self.t = [0, 1, 2, 3, 4]
        #self.t = np.array(self.t) * 40

        # current state(pos, vel, acc, jerk, snap)
        self.current_pos = np.array([0, 0, 0, 0])  # x y z psi
        self.current_vel = np.array([0, 0, 0, 0])  # for now in our system, we can only get velocity from estimation
        self.current_acc = np.array([0, 0, 0, 0])
        self.current_jerk = np.array([0, 0, 0, 0])
        self.current_snap = np.array([0, 0, 0, 0])
        self.current_state = np.vstack((self.current_pos, self.current_vel, self.current_acc, self.current_jerk, self.current_snap))

        # compute flat output trajectory
        self.sol_x = qp_solution.qp_solution(self.order, self.n, self.waypoint, self.t, self.keyframe, self.current_state)
        # draw trajectory in plot
        draw_trajectory.draw_trajectory(self.sol_x, self.order, self.waypoint, self.n, self.t, self.keyframe)

        # for counting gate
        self.passed_gate = 0

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

        if self.i == self.waypoint - 1:
            print ("Total time: {}".format(time-self.init_time))
            exit()

        return ref_trajectory

    def current_state_update(self, current_pos, current_vel, current_acc, current_jerk, current_snap):
        self.current_pos = np.array(current_pos)
        self.current_vel = np.array(current_vel)
        self.current_acc = np.array(current_acc)
        self.current_jerk = np.array(current_jerk)
        self.current_snap = np.array(current_snap)
        self.current_state = np.vstack((self.current_pos, self.current_vel, self.current_acc, self.current_jerk, self.current_snap))

    # checking whether drone pass gate or not
    # It needs current position of drone.
    def check_gate(self, current_position):
        if self.gates[self.passed_gate].isEvent(current_position, self.tolerance):
            self.passed_gate = self.passed_gate + 1
            print "pass %d gate" % self.passed_gate


    # keyframe and trajectory update
    # It needs current position of drone.
    # Also need current vel, acc, jerk, snap from flatout trajectory
    def trajectory_update(self, pass_time):
        # delete previous pose and gate location which was passed before.
        # delete 3 * gate because we add before and after gate.
        for i in range(0, 3 * self.passed_gate + 1):
            self.keyframe = np.delete(self.keyframe, 0, axis=0)

        # add current pose at keyframe
        self.keyframe = np.vstack((self.current_state[0], self.keyframe))

        # update time
        self.t = optimal_time.compute_optimal_time(self.keyframe, self.waypoint - self.passed_gate, self.total_time - pass_time)

        # compute flat output trajectory
        self.sol_x = qp_solution.qp_solution(self.order, self.n, self.waypoint - self.passed_gate, self.t, self.keyframe, self.current_state)


class Trajectory_Generator_Test():

    def __init__(self):
        # init parameters
        self.order = 6
        self.n = 4
        self.gate_count = 1
        self.t = [0, 5]

        # generate keyframe
        self.keyframe = np.array([[0.3, 52.0, 2.5, -1*np.pi/2.0],
                                  [0.3, 52.0, 10, -1*np.pi/2.0]])
        self.keyframe = np.transpose(self.keyframe)

        # compute flat output trajectory
        self.sol_x = qp_solution.qp_solution(self.order, self.n, self.gate_count, self.t, self.keyframe)

        # draw trajectory in plot
        draw_trajectory.draw_trajectory(self.sol_x, self.order, self.gate_count, self.n, self.t, self.keyframe)

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

        if self.i == self.gate_count:
            print ("Total time: {}".format(time - self.init_time))
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
        vel = 3
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

        waypoints[waypoints.shape[0]-1][1] =  waypoints[waypoints.shape[0]-1][1] - 30

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
    traj_publisher = rospy.Publisher('uav_ref_trajectory', UAV_traj, queue_size = 10)

    # init node
    # rospy.init_node('uav_ref_trajectory_publisher', anonymous = True)
    rospy.init_node('uav_ref_trajectory_input_publisher', anonymous=True)

    # IMPORTANT WAIT TIME! 
    # If this is not here, the "start_time" in the trajectory generator is 
    # initialized to zero (because the node has not started fully) and the
    # time for the trajectory will be degenerated
    rospy.sleep(0.1)
 
    # create a trajectory generator
    traj_gen = Trajectory_Generator()
    #traj_gen = Trajectory_Generator2()
    #traj_gen = Trajectory_Generator_Test()

    # publish at 10Hz
    rate = rospy.Rate(200.0)

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

            input = UAV_input()
            input.header.stamp = rospy.Time.now()
            input.header.frame_id = ""

            
            # get all values... we need to do this becuase ref_traj contains old, ugly np.matrix
            # objects >:(
            x, y, z = np.array(ref_traj[0]).flatten()
            phi, theta, psi = np.array(ref_traj[2]).flatten()
            vx, vy, vz = np.array(ref_traj[1]).flatten()
            p, q, r = np.array(ref_traj[3]).flatten()
            uax, uay, uaz = np.array(ref_traj[4]).flatten()
            ubx, uby, ubz = np.array(ref_traj[5]).flatten()
            ucx, ucy, ucz = np.array(ref_traj[6]).flatten()
            Rbw = np.array(ref_traj[9]).flatten().tolist()
            print("ref_traj[9]: {}".format(ref_traj[9]))

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