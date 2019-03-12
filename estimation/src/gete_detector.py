#!/usr/bin/env python

import rospy
import cv2

from nav_msgs.msg import Odometry
from flightgoggles.msg import IRMarker, IRMarkerArray

class GateDetector():
    def ir_cb(self, array):
        
