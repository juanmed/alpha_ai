#!/usr/bin/env python

# This file is a test to search and print the uav parameters


import rospy


def search_params():

    # init node...


    # search for parameters
    mass = rospy.get_param("/uav/flightgoggles_uav_dynamics/vehicle_mass")
    Ixx = rospy.get_param(("/uav/flightgoggles_uav_dynamics/vehicle_inertia_xx"))
    Iyy = rospy.get_param(("/uav/flightgoggles_uav_dynamics/vehicle_inertia_yy"))
    Izz = rospy.get_param(("/uav/flightgoggles_uav_dynamics/vehicle_inertia_zz"))

    gate_names = rospy.get_param("/uav/gate_names")

    gate_locations = {}
    for name in gate_names:
        gate_locations[name] = rospy.get_param("/uav/"+name+"/location")

    """
    # search for param names
    uav_params = {'vehicle_mass': None,'inertia_xx': None,'inertia_yy': None,'inertia_zz': None}
    for key in uav_params:
        #search for params with name closest to the key
        param_name = rospy.search_param(key)
        if param_name is not None:
            # if found, save it dictionary
            names[key] = rospy.get_param(param_name)
        else:
            print("No se encontro "+key)

    mass = uav_params['vehicle_mass']
    Ixx = uav_params['inertia_xx']
    Iyy = uav_params['inertia_yy']
    Izz = uav_params['inertia_zz']
    """

    print("mass: {}, {}".format(mass,type(mass)))
    print("Ixx: {}, {}".format(Ixx,type(Ixx)))
    print("Iyy: {}, {}".format(Iyy,type(Iyy)))
    print("Izz: {}, {}".format(Izz,type(Izz)))
    for key in gate_locations:
        print("{}, {}, {}".format(key,gate_locations[key],type(gate_locations[key])))


if __name__ == '__main__':
    try:
        search_params()
    except rospy.ROSInterruptException:
        print("ROS has been terminated.")
        pass