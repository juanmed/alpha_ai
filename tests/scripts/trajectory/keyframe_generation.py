import numpy as np
import tf


class KeyframeGeneration:

    def __init__(self):
        pass

    def keyframe_generation(self, current_pose, is_quaternion, gate_location, gate_count):
        if is_quaternion is True:
            # init_pose = [ x y z x y z w ]   3 position 4 quaternion  -> 3 position and 1 yaw
            init_pose_orientation = tf.transformations.euler_from_quaternion([current_pose[3], current_pose[4], current_pose[5], current_pose[6]], axes='sxyz')
            init_pose = np.array([current_pose[0], current_pose[1], current_pose[2], init_pose_orientation[2]])
            current_pose = init_pose

        # keyframe with initial position : keyframe = gate + 1
        m = gate_count + 1

        # generate keyframe
        keyframe = []
        keyframe = np.append(keyframe, current_pose)
        for i in range(0, gate_count):
            keyframe = np.append(keyframe, self.compute_pose(gate_location[i]))
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
        print gate_psi
        gate_pose = np.array([gate_x, gate_y, gate_z, gate_psi])

        return gate_pose
