#!/usr/bin/env python
import rospy

import numpy as np
import tf


class KeyframeGeneration:

    def __init__(self):
        self.keyframe = np.array([])

    def keyframe_generation(self, current_pose, is_quaternion, gate_location, gate_count):
        if is_quaternion is True:
            # init_pose = [ x y z x y z w ]   3 position 4 quaternion  -> 3 position and 1 yaw
            init_pose_orientation = tf.transformations.euler_from_quaternion([current_pose[3], current_pose[4], current_pose[5], current_pose[6]], axes='sxyz')
            init_pose = np.array([current_pose[0], current_pose[1], current_pose[2], init_pose_orientation[2]])
            current_pose = init_pose

        # generate keyframe
        self.keyframe = np.array([current_pose])
        for i in range(0, gate_count):
            #print self.compute_pose(gate_location[i])
            self.keyframe = np.vstack((self.keyframe, self.compute_pose(gate_location[i])))

        waypoint = np.size(self.keyframe)/4
        keyframe = np.reshape(self.keyframe, (waypoint, 4))
        keyframe = np.transpose(keyframe)

        # include initial position
        return keyframe, waypoint

    def compute_pose(self, gate_location):
        gate = np.array(gate_location)
        gate_x = np.sum(gate[:, 0]) / 4.0
        gate_y = np.sum(gate[:, 1]) / 4.0
        gate_z = np.sum(gate[:, 2]) / 4.0

        # cross product
        p1 = gate[0, :]
        p2 = gate[1, :]
        p3 = gate[2, :]

        v1 = p3-p1
        v2 = p2-p1
        cp = np.cross(v1, v2)
        gate_psi = np.arctan2(cp[1], cp[0])

        print "hi"
        gate_pose = np.array([gate_x, gate_y, gate_z, gate_psi])
        # compensate gate direction
        if self.compensate_direction(gate_pose):
            cp = -cp
            gate_psi = np.arctan2(cp[1], cp[0])
            gate_pose[3] = gate_psi
            print cp
        gate_pose = self.add_waypoint(gate_pose, cp)
        return gate_pose

    # add before and after waypoint at certain gate
    def add_waypoint(self, gate_pose, cp):
        relaxation = 5
        cp = cp / np.linalg.norm(cp)
        before_waypoint = np.array([gate_pose[0]-cp[0]*relaxation, gate_pose[1]-cp[1]*relaxation, gate_pose[2]-cp[2]*relaxation, gate_pose[3]])
        after_waypoint = np.array([gate_pose[0]+cp[0]*relaxation, gate_pose[1]+cp[1]*relaxation, gate_pose[2]+cp[2]*relaxation, gate_pose[3]])
        gate_pose = np.vstack((before_waypoint, gate_pose, after_waypoint))
        return gate_pose

    def compensate_direction(self, gate_pose):
        keyframe = self.keyframe[len(self.keyframe)-1]
        x_vector = gate_pose[0] - keyframe[0]
        y_vector = gate_pose[1] - keyframe[1]
        z_vector = gate_pose[2] - keyframe[2]
        vector = np.array([x_vector, y_vector, z_vector])
        gate_direction = ([np.cos(gate_pose[3]), np.sin(gate_pose[3]), 0])
        dotproduct = np.dot(vector, gate_direction)
        cosangle = dotproduct/(np.linalg.norm(vector)*np.linalg.norm(gate_direction))
        if cosangle < 0 :
            return True
        return False