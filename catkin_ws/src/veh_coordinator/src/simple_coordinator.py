#!/usr/bin/env python
from __future__ import print_function
import rospy
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg
from duckietown_msgs.msg import ControlMode, IntersectionDetection, VehicleDetection, TrafficLightDetection, \
    CoordinationClearance, RoofLight
import math
import numpy as np
import time

class VehicleCoordinator():
    """The Vehicle Coordination Module for Duckiebot"""

    def __init__(self, visualize=False):
        self.SHOW_VIS = visualize

        # Subscriptions
        self.mode = ControlMode.LANE_FOLLOWING
        rospy.Subscriber('mode', ControlMode, lambda msg: self.set('mode', msg.mode))

        self.intersection = IntersectionDetection.NONE
        rospy.Subscriber('intersection_detection', IntersectionDetection, lambda msg: self.set('intersection', msg.type))

        self.traffic_light = TrafficLightDetection.NA
        rospy.Subscriber('traffic_light_detection', TrafficLightDetection, lambda msg: self.set('traffic_light', msg.color))

        self.right_veh = VehicleDetection.NA
        rospy.Subscriber('right_vehicle_detection', VehicleDetection, lambda msg: self.set('right_veh', msg.roof_light))

        self.opposite_veh = VehicleDetection.NA
        rospy.Subscriber('opposite_vehicle_detection', VehicleDetection, lambda msg: self.set('opposite_veh', msg.roof_light))

        self.intersection_veh = VehicleDetection.NA
        rospy.Subscriber('intersection_vehicle_detection', VehicleDetection, lambda msg: self.set('intersection_veh', msg.roof_light))

        # Publishing
        self.clearance_to_go = CoordinationClearance.NA
        self.clearance_to_go_pub = rospy.Publisher('clearance_to_go', CoordinationClearance, queue_size=10)

        self.roof_light =  RoofLight.OFF
        self.roof_light_pub = rospy.Publisher('roof_light', RoofLight, queue_size=10)

        self.node = rospy.init_node('veh_coordinator', anonymous=True)

        if self.SHOW_VIS:
            while not rospy.is_shutdown():
                self.loop()
                rospy.sleep(0.1)

    def set(self, name, value):
        self.__dict__[name] = value
        self.reconsider()
        self.publish_topics()

    def publish_topics(self):
        self.clearance_to_go_pub.publish(CoordinationClearance(status=self.clearance_to_go))
        self.roof_light_pub.publish(RoofLight(color=self.roof_light))

    def loop(self):
        self.reconsider()
        self.publish_topics()

    def reconsider(self):
        if self.mode == ControlMode.COORDINATION_CONTROL:
            if self.intersection == IntersectionDetection.TRAFFIC_LIGHT:
                # traffic light
                if self.traffic_light == TrafficLightDetection.GREEN:
                    self.clearance_to_go = CoordinationClearance.GO
                    print('GO')
                else:
                    self.clearance_to_go = CoordinationClearance.WAIT
                    print('WAIT')

            elif self.intersection == IntersectionDetection.STOP:
                # stop-line
                if self.intersection_veh == VehicleDetection.NA and self.right_veh == VehicleDetection.NA:
                    self.clearance_to_go = CoordinationClearance.GO
                    print('GO')
                else:
                    self.clearance_to_go = CoordinationClearance.WAIT
                    print('WAIT')

            else:
                print('Coordination requested, but no intersection detected!')
        else:
            self.clearance_to_go = CoordinationClearance.NA

if __name__ == '__main__':
    car = VehicleCoordinator(True)
    rospy.spin()
