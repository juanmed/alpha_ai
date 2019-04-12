#!/usr/bin/env python
import rospy
import numpy as np


# Trajectory parameter
order = 10
flat_output = 4


# way point checking
# inflation means the scale of virtual cube including way point
inflation = 2
# tolerance is like threshold to decide whether drone pass or not
tolerance = 0.3


ros_isrunning = True

if ros_isrunning:
    # Waypoint
    init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
    gate_name = rospy.get_param("/uav/gate_names")

    # Drone physical property
    mass = rospy.get_param("/uav/flightgoggles_uav_dynamics/vehicle_mass")
    Ixx = rospy.get_param("/uav/flightgoggles_uav_dynamics/vehicle_inertia_xx")
    Iyy = rospy.get_param("/uav/flightgoggles_uav_dynamics/vehicle_inertia_yy")
    Izz = rospy.get_param("/uav/flightgoggles_uav_dynamics/vehicle_inertia_zz")
else:
    init_pose = np.array([0, 0, 1, 0])
