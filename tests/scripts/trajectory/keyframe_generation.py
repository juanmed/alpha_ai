#!/usr/bin/env python
import rospy

import numpy as np
import get_gate_param
import tf
import parameter


# without returning : final keyframe != initial keyframe
class KeyframeGeneration:

    def __init__(self):
        self.gate_location_cls = get_gate_param.GateLocation()
        self.level = parameter.level

    def keyframe_generation(self, gate=11):
        # get gate location according to level
        if self.level is True:
            init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
            # init_pose = [ x y z x y z w ]   3 position 4 quaternion
            init_pose_orientation = tf.transformations.euler_from_quaternion([init_pose[3], init_pose[4], init_pose[5], init_pose[6]], axes='sxyz')
            init_pose = np.array([init_pose[0], init_pose[1], init_pose[2], init_pose_orientation[2]])
            gate_racing = self.gate_location_cls.level_gate()
        # when full gate
        else:
            init_pose = np.array([0.0, 0.0, 1.0, 0.0])
            gate_racing = self.gate_location_cls.full_gate(11)

        # keyframe with initial position : keyframe = gate + 1
        m = gate + 1

        # generate keyframe
        keyframe = []
        keyframe = np.append(keyframe, init_pose)
        for i in range(0, gate):
            keyframe = np.append(keyframe, self.compute_pose(gate_racing[i]))

        keyframe = np.reshape(keyframe, (m, 4))
        keyframe = np.transpose(keyframe)

        return keyframe

    def compute_pose(self, gate_location):
        gate = np.array(gate_location)
        gate_x = np.sum(gate[:, 0]) / 4.0
        gate_y = np.sum(gate[:, 1]) / 4.0
        gate_z = np.sum(gate[:, 2]) / 4.0

        x_vector = gate[1, 0] - gate[0, 0]
        y_vector = gate[1, 1] - gate[0, 1]

        # normal_vector = -1 * x_vector / y_vector    we should check this...
        gate_psi = np.arctan2(-x_vector, y_vector)

        gate_pose = np.array([gate_x, gate_y, gate_z, gate_psi])

        return gate_pose
