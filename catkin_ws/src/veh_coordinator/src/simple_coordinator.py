#!/usr/bin/env python
from __future__ import print_function
from random import random
import rospy
from duckietown_msgs.msg import ControlMode, IntersectionDetection, VehicleDetection, TrafficLightDetection, \
    CoordinationClearance, CoordinationSignal
from time import time


class State:
    LANE_FOLLOWING = 0
    AT_STOP = 1
    AT_STOP_CLEARING = 2
    AT_STOP_CLEAR = 3
    RESERVING = 4
    CONFLICT = 5
    GO = 6
    AT_TRAFFIC_LIGHT = 7
    INTERSECTION_NAVIGATION = 8

    state_to_str = ['LANE_FOLLOWING',
                    'AT_STOP',
                    'AT_STOP_CLEARING',
                    'AT_STOP_CLEAR',
                    'RESERVING',
                    'CONFLICT',
                    'GO',
                    'AT_TRAFFIC_LIGHT',
                    'INTERSECTION_NAVIGATION']


class VehicleCoordinator():
    """The Vehicle Coordination Module for Duckiebot"""

    T_MAX_RANDOM = 2.0 # seconds
    T_CROSS = 5.0  # seconds
    T_S = 2.0      # seconds

    def __init__(self):
        self.state = State.LANE_FOLLOWING
        self.last_state_transition = time()
        self.random_delay = 0

        self.node = rospy.init_node('veh_coordinator', anonymous=True)

        # Subscriptions
        self.mode = ControlMode.LANE_FOLLOWING
        rospy.Subscriber('mode', ControlMode,
                         lambda msg: self.set('mode', msg.mode))

        self.intersection = IntersectionDetection.NONE
        rospy.Subscriber('intersection_detection', IntersectionDetection,
                         lambda msg: self.set('intersection', msg.type))

        self.traffic_light = TrafficLightDetection.NA
        rospy.Subscriber('traffic_light_detection', TrafficLightDetection,
                         lambda msg: self.set('traffic_light', msg.color))

        self.right_veh = VehicleDetection.NO_CAR
        rospy.Subscriber('right_vehicle_detection', VehicleDetection,
                         lambda msg: self.set('right_veh', msg.detection))

        self.opposite_veh = VehicleDetection.NO_CAR
        rospy.Subscriber('opposite_vehicle_detection', VehicleDetection,
                         lambda msg: self.set('opposite_veh', msg.detection))

        # Publishing
        self.clearance_to_go = CoordinationClearance.NA
        self.clearance_to_go_pub = rospy.Publisher('clearance_to_go', CoordinationClearance, queue_size=10)

        self.roof_light = CoordinationSignal.SIGNAL_A
        self.roof_light_pub = rospy.Publisher('coordination_signal', CoordinationSignal, queue_size=10)

        while not rospy.is_shutdown():
            self.loop()
            rospy.sleep(0.1)

    def get_state_str(self):
        return State.state_to_str[self.state]

    def set_state(self, state):
        self.state = state
        self.last_state_transition = time()

        if self.state == State.RESERVING:
            self.roof_light = CoordinationSignal.SIGNAL_B
        else:
            self.roof_light = CoordinationSignal.SIGNAL_A

        if self.state == State.GO:
            self.clearance_to_go = CoordinationClearance.GO
        else:
            self.clearance_to_go = CoordinationClearance.WAIT

        print("Transitioned to state " + self.get_state_str())

    def time_at_current_state(self):
        return time() - self.last_state_transition

    def set(self, name, value):
        self.__dict__[name] = value

    def publish_topics(self):
        self.clearance_to_go_pub.publish(CoordinationClearance(status=self.clearance_to_go))
        self.roof_light_pub.publish(CoordinationSignal(signal=self.roof_light))

    def loop(self):
        self.reconsider()
        self.publish_topics()

    def reconsider(self):

        if self.state == State.LANE_FOLLOWING:
            if self.mode == ControlMode.COORDINATION_CONTROL:
                if self.intersection == IntersectionDetection.STOP:
                    self.set_state(State.AT_STOP)
                elif self.intersection == IntersectionDetection.TRAFFIC_LIGHT:
                    self.set_state(State.AT_TRAFFIC_LIGHT)
                else:
                    print('Coordination requested, but no intersection detected!')

        elif self.state == State.AT_STOP:
            if self.right_veh == VehicleDetection.NO_CAR and self.opposite_veh != VehicleDetection.SIGNAL_B:
                self.set_state(State.AT_STOP_CLEARING)

        elif self.state == State.AT_STOP_CLEARING:
            if self.right_veh != VehicleDetection.NO_CAR or self.opposite_veh == VehicleDetection.SIGNAL_B:
                self.set_state(State.AT_STOP)
            elif self.time_at_current_state() > self.T_CROSS:
                self.set_state(State.AT_STOP_CLEAR)

        elif self.state == State.AT_STOP_CLEAR:
            if self.right_veh != VehicleDetection.NO_CAR or self.opposite_veh == VehicleDetection.SIGNAL_B:
                self.set_state(State.AT_STOP)
            else:
                self.set_state(State.RESERVING)

        elif self.state == State.RESERVING:
            if self.right_veh != VehicleDetection.NO_CAR:
                self.set_state(State.AT_STOP)
            elif self.time_at_current_state() > self.T_S:
                if self.opposite_veh == VehicleDetection.SIGNAL_B:
                    self.random_delay = random() * self.T_MAX_RANDOM
                    print ("Other vehicle reserving as well. Will wait for %.2f s" % self.random_delay)
                    self.set_state(State.CONFLICT)
                else:
                    self.set_state(State.GO)

        elif self.state == State.GO:
            if self.mode == ControlMode.LANE_FOLLOWING:
                self.set_state(State.LANE_FOLLOWING)

        elif self.state == State.CONFLICT:
            if self.right_veh != VehicleDetection.NO_CAR or self.opposite_veh == VehicleDetection.SIGNAL_B:
                self.set_state(State.AT_STOP)
            elif self.time_at_current_state() > self.random_delay:
                self.set_state(State.AT_STOP_CLEAR)

        elif self.state == State.AT_TRAFFIC_LIGHT:
            if self.traffic_light == TrafficLightDetection.GREEN:
                self.set_state(State.GO)

if __name__ == '__main__':
    car = VehicleCoordinator()
    rospy.spin()
