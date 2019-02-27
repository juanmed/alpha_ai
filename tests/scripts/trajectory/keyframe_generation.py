import numpy as np
import gate_location
import get_gate_param


# without returning : final keyframe != initial keyframe
class KeyframeGeneration:

    def __init__(self):
        self.gate_location_cls = get_gate_param.GateLocation()

    def keyframe_generation(self, gate=11):
        m = gate + 1
        gate_racing = self.gate_location_cls.get_gate_racing(m)
        keyframe = []

        # keyframe with initial position keyframe = gate + 1
        init_pose = np.array([0.0, 0.0, 1.1, 0.0])
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
        #normal_vector = -1 * x_vector / y_vector
        gate_psi = np.arctan2( -x_vector, y_vector)

        gate_pose = np.array([gate_x, gate_y, gate_z, gate_psi])
        return gate_pose
