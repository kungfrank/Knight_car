#!/usr/bin/env python
from __future__ import print_function
import rospy
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg
from duckietown_msgs.msg import ControlMode
import math
import numpy as np
import time


class VehicleCoordinator():
    """The Vehicle Coordination Module for Duckiebot"""

    def __init__(self, visualize=False):
        self.SHOW_VIS = visualize

        self.node = rospy.init_node('veh_coordinator', anonymous=True)
        self.mode_subscriber = rospy.Subscriber('mode', ControlMode, self.mode_callback)

        if self.SHOW_VIS:
            while not rospy.is_shutdown():
                self.loop()
                rospy.sleep(0.1)

    def loop(self):
        print('looping')
        pass

    def mode_callback(self):
        pass

if __name__ == '__main__':
    car = VehicleCoordinator(True)
    rospy.spin()
