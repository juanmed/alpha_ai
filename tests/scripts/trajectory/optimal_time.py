import numpy as np


def compute_optimal_time(keyframe, waypoint, total_time):
    keyframe = np.transpose(keyframe)

    distance_array = np.array([])
    epsilon = 20
    psi_array = np.array([])
    for i in range(0, waypoint-1):
        p_vector = np.array([keyframe[i+1][0]-keyframe[i][0], keyframe[i+1][1]-keyframe[i][1], keyframe[i+1][2]-keyframe[i][2]])
        distance = np.linalg.norm(p_vector)
        distance_array = np.append(distance_array, distance)

        psi_vector = np.array(np.abs(keyframe[i+1][3] - keyframe[i][3]))
        psi_array = np.append(psi_array, psi_vector)


    full_distance = np.sum(distance_array)
    full_psi = np.sum(psi_array)
    t = np.array([0])
    for i in range(0, waypoint-1):
        distance_sum = 0
        psi_sum = 0
        for j in range(0, i+1):
            distance_sum = distance_sum + distance_array[j]
            psi_sum = psi_sum + psi_array[j]

        ratio = (distance_sum + psi_sum * epsilon) / (full_distance + full_psi * epsilon)
        time = ratio * total_time
        t = np.append(t, time)
    print t
    return t
