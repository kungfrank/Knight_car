#!/usr/bin/env python
from __future__ import print_function

from random import random
import rospy
from duckietown_msgs.msg import ControlMode, IntersectionDetection, VehicleDetection, TrafficLightDetection, \
    CoordinationClearance, RoofLight
from time import time

class State:
    LANE_FOLLOWING = 0
    AT_STOP = 1
    RESERVING = 2
    CLEARED = 3
    AT_TRAFFIC_LIGHT = 4
    INTERSECTION_NAVIGATION = 5
    RANDOM_DELAY = 6

    state_to_str = [ 'LANE_FOLLOWING', 'AT_STOP', 'RESERVING', 'CLEARED', 'AT_TRAFFIC_LIGHT',
                     'INTERSECTION_NAVIGATION', 'RANDOM_DELAY' ]


class VehicleCoordinator():
    """The Vehicle Coordination Module for Duckiebot"""

    RESERVE_DELAY = 2.0 # seconds
    MAX_RANDOM_DELAY = 5.0 # seconds

    def __init__(self):
        self.set_state(State.LANE_FOLLOWING)
        self.random_delay = 0

        self.node = rospy.init_node('veh_coordinator', anonymous=True)

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

        while not rospy.is_shutdown():
            self.loop()
            rospy.sleep(0.1)

    def get_state_str(self):
        return State.state_to_str[self.state]

    def set_state(self, state):
        self.state = state
        self.last_state_transition = time()

        print("Transitioned to state " + self.get_state_str())

    def time_at_current_state(self):
        return time() - self.last_state_transition

    def set(self, name, value):
        self.__dict__[name] = value

    def publish_topics(self):
        self.clearance_to_go_pub.publish(CoordinationClearance(status=self.clearance_to_go))
        self.roof_light_pub.publish(RoofLight(color=self.roof_light))

    def loop(self):
        self.reconsider()
        self.publish_topics()

    def reconsider(self):

        if self.state == State.LANE_FOLLOWING:
            if self.mode == ControlMode.COORDINATION_CONTROL:
                if self.intersection == IntersectionDetection.STOP:
                    self.clearance_to_go = CoordinationClearance.WAIT
                    self.set_state(State.AT_STOP)

                elif self.intersection == IntersectionDetection.TRAFFIC_LIGHT:
                    self.clearance_to_go = CoordinationClearance.WAIT
                    self.set_state(State.AT_TRAFFIC_LIGHT)

                else:
                    print('Coordination requested, but no intersection detected!')

        elif self.state == State.AT_STOP:
            if self.right_veh == VehicleDetection.NA and \
                    self.opposite_veh in {VehicleDetection.NA, VehicleDetection.OFF} and \
                    self.intersection_veh == VehicleDetection.NA:
                self.roof_light = RoofLight.YELLOW
                self.set_state(State.RESERVING)

        elif self.state == State.RESERVING:
            if self.time_at_current_state() > self.RESERVE_DELAY:
                if self.right_veh != VehicleDetection.NA:
                    self.set_state(State.AT_STOP)

                elif self.right_veh == VehicleDetection.NA and \
                        self.opposite_veh in {VehicleDetection.YELLOW, VehicleDetection.RED}:
                    self.roof_light = RoofLight.OFF
                    self.random_delay = random() * self.MAX_RANDOM_DELAY
                    print ("Other vehicle reserving as well. Will wait for %.2f s" % self.random_delay)
                    self.set_state(State.RANDOM_DELAY)

                elif self.right_veh == VehicleDetection.NA and \
                        self.opposite_veh in {VehicleDetection.NA, VehicleDetection.OFF}:
                    self.roof_light = RoofLight.RED
                    self.clearance_to_go = CoordinationClearance.GO
                    self.set_state(State.CLEARED)

        elif self.state == State.CLEARED:
            if self.mode == ControlMode.INTERSECTION_CONTROL:
                self.roof_light = RoofLight.OFF
                self.clearance_to_go = CoordinationClearance.NA
                self.set_state(State.INTERSECTION_NAVIGATION)

        elif self.state == State.INTERSECTION_NAVIGATION:
            if self.mode == ControlMode.LANE_FOLLOWING:
                self.set_state(State.LANE_FOLLOWING)

        elif self.state == State.AT_TRAFFIC_LIGHT:
            if self.traffic_light == TrafficLightDetection.GREEN and \
                    self.intersection_veh == VehicleDetection.NA:
                self.clearance_to_go = CoordinationClearance.GO
                self.set_state(State.CLEARED)

        elif self.state == State.RANDOM_DELAY:
            if self.time_at_current_state() > self.random_delay:
                self.set_state(State.AT_STOP)

if __name__ == '__main__':
    car = VehicleCoordinator()
    rospy.spin()
