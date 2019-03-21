#!/usr/bin/env python
import rospy


init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
mass = rospy.get_param("/uav/flightgoggles_uav_dynamics/vehicle_mass")
Ixx = rospy.get_param("/uav/flightgoggles_uav_dynamics/vehicle_inertia_xx")
Iyy = rospy.get_param("/uav/flightgoggles_uav_dynamics/vehicle_inertia_yy")
Izz = rospy.get_param("/uav/flightgoggles_uav_dynamics/vehicle_inertia_zz")

gate_name = rospy.get_param("/uav/gate_names")
#tolerance = rospy.get_param("/uav/gate_width", 0.5)
#inflation = rospy.get_param("/uav/inflation", 1.0)
tolerance = 0.5
inflation = 1.0