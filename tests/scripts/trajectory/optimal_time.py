import numpy as np


def compute_optimal_time(keyframe, gate_count, total_time):
    keyframe = np.transpose(keyframe)
    distance_array = np.array([])
    for i in range(0, gate_count):
        p_vector = np.array([keyframe[i+1][0]-keyframe[i][0], keyframe[i+1][1]-keyframe[i][1], keyframe[i+1][2]-keyframe[i][2]])
        distance = np.linalg.norm(p_vector)
        distance_array = np.append(distance_array, distance)

    full_distance = np.sum(distance_array)
    t = np.array([0])
    for i in range(0, gate_count):
        distance_sum = 0
        for j in range(0, i+1):
            distance_sum = distance_sum + distance_array[j]
        time = distance_sum / full_distance * total_time
        t = np.append(t, time)
    return t
