#!/usr/bin/env python
import rospy
import numpy as np


class GateLocation:

    def __init__(self):
        self.init_pose = np.array([0.0, 0.0, 1.0, 0.0])
        self.gate_1 = rospy.get_param("/uav/Gate1/nominal_location")
        self.gate_2 = rospy.get_param("/uav/Gate2/nominal_location")
        self.gate_3 = rospy.get_param("/uav/Gate3/nominal_location")
        self.gate_4 = rospy.get_param("/uav/Gate4/nominal_location")
        self.gate_5 = rospy.get_param("/uav/Gate5/nominal_location")
        self.gate_6 = rospy.get_param("/uav/Gate6/nominal_location")
        self.gate_7 = rospy.get_param("/uav/Gate7/nominal_location")
        self.gate_8 = rospy.get_param("/uav/Gate8/nominal_location")
        self.gate_9 = rospy.get_param("/uav/Gate9/nominal_location")
        self.gate_10 = rospy.get_param("/uav/Gate10/nominal_location")
        self.gate_11 = rospy.get_param("/uav/Gate11/nominal_location")
        self.gate_12 = rospy.get_param("/uav/Gate12/nominal_location")
        self.gate_13 = rospy.get_param("/uav/Gate13/nominal_location")
        self.gate_14 = rospy.get_param("/uav/Gate14/nominal_location")
        self.gate_15 = rospy.get_param("/uav/Gate15/nominal_location")
        self.gate_16 = rospy.get_param("/uav/Gate16/nominal_location")
        self.gate_17 = rospy.get_param("/uav/Gate17/nominal_location")
        self.gate_18 = rospy.get_param("/uav/Gate18/nominal_location")
        self.gate_19 = rospy.get_param("/uav/Gate19/nominal_location")
        self.gate_20 = rospy.get_param("/uav/Gate20/nominal_location")
        self.gate_21 = rospy.get_param("/uav/Gate21/nominal_location")
        self.gate_22 = rospy.get_param("/uav/Gate22/nominal_location")
        self.gate_23 = rospy.get_param("/uav/Gate23/nominal_location")
        self.gate_array = np.array([self.gate_1, self.gate_2, self.gate_3, self.gate_4, self.gate_5, self.gate_6,
                                    self.gate_7, self.gate_8, self.gate_9, self.gate_10, self.gate_11, self.gate_12,
                                    self.gate_13, self.gate_14, self.gate_15, self.gate_16, self.gate_17, self.gate_18,
                                    self.gate_19, self.gate_20, self.gate_21, self.gate_22, self.gate_23])

        # racing 10 -> 21 -> 2 -> 13 -> 9 -> 14 -> 1 -> 22 -> 15 -> 23 -> 6
        self.gate_racing = np.array([self.gate_10, self.gate_21, self.gate_2, self.gate_13, self.gate_9, self.gate_14,
                                     self.gate_1, self.gate_22, self.gate_15, self.gate_23, self.gate_6])

    def get_gate_racing(self, gate_count):
        return self.gate_racing[0:gate_count+1, :, :]

if __name__ == "__main__":

    gate_location_cls = GateLocation()
    print gate_location_cls.get_gate_racing(1)
