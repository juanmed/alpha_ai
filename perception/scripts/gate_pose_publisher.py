#!/usr/bin/env python

# This script should publish (to /tf) the a best estimation of the gates pose
# based on drones pose and IR markers

import rospy
import tf
import numpy as np 
from utils import *

from flightgoggles.msg import IRMarker, IRMarkerArray
from std_msgs.msg import String
from perception.msg import GatePoseArray, GatePose


class Gate_Pose_Estimator():

    def __init__(self):

        # get initial position of drone
        self.init_pos, self.init_ori = self.get_init_pose()

        # publisher
        self.gate_publisher = rospy.Publisher("/riseq/perception/gate_poses", GatePoseArray, queue_size = 10)

        # subscribe to IR Marker topic
        rospy.Subscriber("/uav/camera/left/ir_beacons", IRMarkerArray, self.read_estimate_publish)

        self.gates = {}

        # This camera intrinsic is obtained from topic uav/camera/left/camera_info
        self.camera_intrinsic = np.zeros((3,3), dtype = 'float64')
        self.camera_intrinsic[0] = np.array([548.4088134765625, 0.0, 512.0])
        self.camera_intrinsic[1] = np.array([0.0, 548.4088134765625, 384.0])
        self.camera_intrinsic[2] = np.array([0.0, 0.0, 1.0])



    def read_estimate_publish(self, ir_markers):

        # This dictionary will contain all gates and their corresponding
        # markers stored as a dictionary where the key is the gate name
        # and the value is a list of its corresponding ir_markers 

        self.gates = {}
        gate_poses = []

        # extract and organize all markers in a dictionary
        for marker in ir_markers.markers:
            gate_name = marker.landmarkID.data
            # if this gate is not in dictionary
            if gate_name in self.gates.keys():
                # simply add markerID,marker coords tuple
                self.gates[gate_name].append( ( int(marker.markerID.data), np.array([marker.x, marker.y])) )
            else:
                # create key for this gate and add its markerID and marker coords inside a tuple
                self.gates[gate_name] = []
                self.gates[gate_name].append( (int(marker.markerID.data), np.array([marker.x, marker.y])))
        

        # compute Rotation, translation for gates having >=3 markers visible
        for gate_name in self.gates:
            # if 3 markers are present, is enough for P3P 
            markers = self.gates[gate_name]
            if( len(markers) > 3):
                #print("**{} has {} visible points".format(gate_name, len(markers)) )
                # get quaternion and translation: drone frame to gate frame
                q_dg, t_dg = self.estimate_gate_pose(gate_name, markers)

                gate_pose_msg = GatePose()
                gate_pose_msg.gate_name = gate_name

                gate_pose_msg.pose.position.x = t_dg[0][0]
                gate_pose_msg.pose.position.y = t_dg[1][0]
                gate_pose_msg.pose.position.z = t_dg[2][0]

                gate_pose_msg.pose.orientation.x = q_dg[0]
                gate_pose_msg.pose.orientation.y = q_dg[1]
                gate_pose_msg.pose.orientation.z = q_dg[2]
                gate_pose_msg.pose.orientation.w = q_dg[3]

                gate_poses.append(gate_pose_msg)

            else:
                #print("{} has {} visible points".format(gate_name, len(markers)))
                continue

        # Prepare and send list of gate poses 
        gate_poses_msg = GatePoseArray()
        gate_poses_msg.header.stamp = rospy.Time.now()
        gate_poses_msg.header.frame_id = "uav/gates"
        gate_poses_msg.gate_poses = gate_poses

        self.gate_publisher.publish(gate_poses_msg)
        rospy.loginfo(gate_poses_msg)


    def estimate_gate_pose(self, gate_name, markers):

        # get global frame position of markers
        gate_markers_3d_global = np.array(rospy.get_param("/uav/"+gate_name+"/nominal_location"))
        gate_markers_3d_global_real = np.array(rospy.get_param("/uav/"+gate_name+"/location"))

        # compute width, height, depth of gate
        x_nom = max(gate_markers_3d_global[:,0]) - min(gate_markers_3d_global[:,0])
        y_nom = max(gate_markers_3d_global[:,1]) - min(gate_markers_3d_global[:,1])        
        z_nom = max(gate_markers_3d_global[:,2]) - min(gate_markers_3d_global[:,2])

        x_real = max(gate_markers_3d_global_real[:,0]) - min(gate_markers_3d_global_real[:,0])
        y_real = max(gate_markers_3d_global_real[:,1]) - min(gate_markers_3d_global_real[:,1])        
        z_real = max(gate_markers_3d_global_real[:,2]) - min(gate_markers_3d_global_real[:,2])

        #print("x_nom {} x_real {} dif {}".format(x_nom, x_real, x_nom-x_real))
        #print("y_nom {} y_real {} dif {}".format(y_nom, y_real, y_nom-y_real))
        #print("z_nom {} z_real {} dif {}".format(z_nom, z_real, z_nom-z_real))

        # compute gate's origin coordinates in global frame
        origin_x_global = np.mean(gate_markers_3d_global[:,0])
        origin_y_global = np.mean(gate_markers_3d_global[:,1])        
        origin_z_global = np.mean(gate_markers_3d_global[:,2])
        gate_origin_global = np.array([origin_x_global, origin_y_global, origin_z_global])

        # compute poisition of markers in gate's local coordinate
        # where gate center is at 0,0,0
        gate_markers_3d_local = np.zeros_like(gate_markers_3d_global)
        for marker in range(gate_markers_3d_local.shape[0]):
            gate_markers_3d_local[marker] = gate_markers_3d_global[marker] - gate_origin_global
            gate_markers_3d_local[marker] = gate_markers_3d_global[marker] - gate_origin_global      


        #print(gate_name)
        #print(gate_origin_global)
        #print(gate_markers_3d_local)

        markers.sort()
        #extrack markers
        points_2D = np.zeros((len(markers),2))
        for i, element in enumerate(markers):
            points_2D[i] = element[1] 


        #print("Points 2D: {}".format(points_2D))

        # Calculate gate pose using pnp algorithm
        # The returned rotation matrix R, and translation t are from the Gate frame
        # to the Drone, and Z,Y axis are interchanged
        R_gd, t_gd = pnp(gate_markers_3d_local, points_2D, self.camera_intrinsic)

        # translation from drone to gate
        t_dg = np.zeros_like(t_gd)
        t_dg[0][0] = t_gd[0][0]
        t_dg[1][0] = t_gd[2][0]
        t_dg[2][0] = t_gd[1][0]
        t_dg = -t_dg

        # Rotation from drone to gate
        R = R_gd.T
        R_dg = np.zeros_like(R)
        R_dg[:,0] = R[:,0]
        R_dg[:,1] = R[:,2]
        R_dg[:,2] = R[:,1]

        dummy1 = np.zeros((3,1))
        dummy2 = np.zeros((1,4))
        dummy2[0][3] = 1.0
        R_dg = np.concatenate((R_dg,dummy1), axis = 1)
        R_dg = np.concatenate((R_dg, dummy2), axis = 0)
        
        q_dg = tf.transformations.quaternion_from_matrix(R_dg)

        #print("Rotation Drone -> Gate : {}".format(q_dg))
        #print("Translation Drone -> Gate : {}".format(t_dg))

        return q_dg, t_dg





    def get_init_pose(self):
        init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
        
        # init position
        init_pos = np.zeros(3)
        init_pos[0] = init_pose[0]
        init_pos[1] = init_pose[1]
        init_pos[2] = init_pose[2]

        # init orientation
        init_ori = np.zeros(4)
        init_ori[0] = init_pose[3]
        init_ori[1] = init_pose[4]
        init_ori[2] = init_pose[5]
        init_ori[3] = init_pose[6]

        return init_pose, init_ori

if __name__ == '__main__':
    try:

        rospy.init_node('gate_pose_publisher', anonymous = True)

        try:
            wait_time = int(rospy.get_param("riseq/perception_wait"))
        except:
            wait_time = 2

        while( rospy.Time.now().to_sec() < wait_time ):
            if( ( int(rospy.Time.now().to_sec()) % 1) == 0 ):
                rospy.loginfo("Starting Perception/Gate Pose Estimator node in {:.2f} seconds".format(wait_time - rospy.Time.now().to_sec()))
        

        uav_input_pub = Gate_Pose_Estimator()

        rospy.loginfo(' Gate Pose Estimator Publisher Created !')
        rospy.spin()
        rospy.loginfo(' Gate Pose Estimator terminated. ')

    except rospy.ROSInterruptException:
        rospy.loginfo('ROS Terminated.')
        pass

