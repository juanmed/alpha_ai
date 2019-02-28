#!/usr/bin/env python
import rospy
import numpy as np


level = False
gate = 0
t = []

order = 6               # order
n = 4                   # flatness output x y z psi
mass = rospy.get_param("/uav/flightgoggles_uav_dynamics/vehicle_mass")
# I = rospy.get_param("uav/~~/xx")   Inertia Matrix

# this is for trajectory with level(easy / medium / hard)
level_gate = len(rospy.get_param("/uav/gate_names"))    # gate count
time_interval = 1
level_t = np.linspace(0, time_interval*level_gate, level_gate+1)


# this is for trajectory with full gate ( maybe final test3 )
full_gate = 11    # keyframe (with returning)  final point = initial point
# set time interval depending on distance between gate
full_t = 1 * np.array([0, 1, 2, 3, 3.5, 4.5, 5, 5.5, 6, 6.5, 7.5, 8.5])


if level is True:
    gate = level_gate
    t = level_t
else:
    gate = full_gate
    t = full_t
