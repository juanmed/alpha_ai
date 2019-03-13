#!/usr/bin/env python
import rospy
import numpy as np


class GateLocation:

    def __init__(self):
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
        self.gate_full_location = np.array([self.gate_10, self.gate_21, self.gate_2, self.gate_13, self.gate_9, self.gate_14,
                                     self.gate_1, self.gate_22, self.gate_15, self.gate_23, self.gate_6])

        self.gate_dic = dict(Gate1=self.gate_1, Gate2=self.gate_2, Gate3=self.gate_3, Gate4=self.gate_4,
                             Gate5=self.gate_5, Gate6=self.gate_6, Gate7=self.gate_7, Gate8=self.gate_8,
                             Gate9=self.gate_9, Gate10=self.gate_10, Gate11=self.gate_11, Gate12=self.gate_12,
                             Gate13=self.gate_13, Gate14=self.gate_14, Gate15=self.gate_15, Gate16=self.gate_16,
                             Gate17=self.gate_17, Gate18=self.gate_18, Gate19=self.gate_19, Gate20=self.gate_20,
                             Gate21=self.gate_21, Gate22=self.gate_22, Gate23=self.gate_23)

    def get_gate_location(self, is_level, gate_count, gate_name=None):
        if is_level is True:
            gate_location = []
            for i in range(0, gate_count):
                gate_location = np.append(gate_location, self.gate_dic[gate_name[i]])
            gate_location = np.reshape(gate_location, (gate_count, 4, 3))
            return gate_location
        else:
            return self.gate_full_location[0:gate_count, :, :]
